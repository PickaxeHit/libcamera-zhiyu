/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Autofocus: contrast-detect search over the lens travel
 *
 * The algorithm is deliberately small:
 *
 *   Init    wait for the stream to settle, look at the light level
 *   Seek    walk until the focus value shows a gradient, follow it to the
 *           peak, fit a parabola through the three points around the peak
 *   Locked  keep measuring; remember the best value, and when the picture
 *           stays clearly below it, throw the record away and seek again
 *
 * Everything is built on Measure(pos): move the lens (smoothly, a few counts
 * per frame), wait until the debayer holds a full window of frames taken at
 * that position, average a few of them and report the value with its own
 * standard deviation.  No decision is taken on a single sample.
 */

#include <linux/v4l2-controls.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "af.h"

using namespace libcamera::controls;

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftAf)

namespace ipa::soft::algorithms {

namespace {

/* ---- parameter set ---------------------------------------------------- */

/* Frames to wait after the stream starts before the light level is read. */
constexpr uint32_t kInitFrames = 10;
/*
 * Frames to wait after the exposure is frozen, before the first measurement of
 * a search: the sensor applies the last commanded exposure a frame or two
 * later, and a value taken while it still moves is not comparable with the
 * rest of the search.
 */
constexpr uint32_t kFreezeSettleFrames = 6;

/* Measure(): frames the debayer averages, then frames sampled. */
constexpr uint32_t kFvFrames = 5;
constexpr uint32_t kMeasureFramesBright = 4;
constexpr uint32_t kMeasureFramesDark = 8;
/* A measurement whose mean(Y) drifted this far is thrown away and redone. */
constexpr double kMeasureYDriftPercent = 5.0;
constexpr uint32_t kMeasureRetries = 2;
/* Below this mean luma the scene counts as dark and gets more averaging. */
constexpr uint32_t kDarkMeanY = 60;

/* A difference smaller than this many sigmas is not a difference. */
constexpr double kConfidenceSigmas = 2.0;

/* Ramp geometry: counts per frame, and the bounds on the number of frames. */
constexpr int32_t kRampMinDelta = 30;
constexpr int32_t kRampCountsPerFrame = 15;
constexpr uint32_t kRampMinFrames = 4;
constexpr uint32_t kRampMaxFrames = 8;
/* Frames to wait after the lens arrives, before the window is restarted. */
constexpr uint32_t kSettleFrames = 3;

/* Search steps, as a fraction of the lens travel. */
constexpr int kBigStepDiv = 5;
constexpr int kSmallStepDiv = 10;
constexpr int kMinStep = 2;

/* Locked: the picture is "gone" when it stays this far below the record. */
constexpr double kRestartPercent = 70.0;
/* Per frame decay of the record, so a transient peak does not stick. */
constexpr double kRecordDecay = 0.9995;
constexpr uint32_t kRestartFrames = 15;
/* After a search that found nothing, wait this long before trying again. */
constexpr uint64_t kRetryMs = 2000;

enum class Phase {
	Probe,		/* measure the starting position */
	ProbeUp,	/* one step above the start */
	ProbeDown,	/* one step below the start */
	Climb,		/* follow the gradient */
	Fit,		/* measure the position the parabola asked for */
};

uint64_t nowMs()
{
	using namespace std::chrono;

	return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

const char *stateName(AfState state)
{
	switch (state) {
	case AfState::Idle:
		return "idle";
	case AfState::Init:
		return "init";
	case AfState::Seek:
		return "seek";
	case AfState::Locked:
		return "locked";
	}

	return "?";
}

} /* namespace */

Af::Af()
{
	csv_ = std::fopen("/tmp/af.csv", "w");
	if (csv_) {
		std::fprintf(csv_, "t_ms,state,pos,fv,sigma,meanY,exposure,gain\n");
		std::fflush(csv_);
	}
}

Af::~Af()
{
	if (csv_)
		std::fclose(csv_);
}

int Af::init(IPAContext &context, [[maybe_unused]] const ValueNode &tuningData)
{
	context.activeState.af.state = AfState::Idle;
	return 0;
}

int Af::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	auto &af = context.activeState.af;
	const auto it = configInfo.lensControls.find(V4L2_CID_FOCUS_ABSOLUTE);

	context.configuration.freezeAe = false;
	af.state = AfState::Idle;
	af.request = true;
	af.measuring = false;
	af.haveMeasure = false;
	af.frames = 0;
	af.meanY = 0.0;
	af.yRef = 0.0;
	af.record = 0.0;
	af.lowFrames = 0;
	af.lastSearchMs = 0;
	af.retryAtMs = 0;

	if (it == configInfo.lensControls.end()) {
		LOG(IPASoftAf, Debug) << "No focus lens";
		context.configuration.lens.available = false;
		return 0;
	}

	context.configuration.lens.available = true;
	af.min = it->second.min().get<int32_t>();
	af.max = it->second.max().get<int32_t>();
	af.bigStep = std::max(kMinStep, (af.max - af.min) / kBigStepDiv);
	af.smallStep = std::max(kMinStep, (af.max - af.min) / kSmallStepDiv);
	af.minStep = kMinStep;
	af.step = af.bigStep;
	af.position = (af.min + af.max) / 2;	/* start from the middle */
	af.target = af.position;
	af.startPos = af.position;
	af.bestPos = af.position;

	LOG(IPASoftAf, Info)
		<< "Focus lens travel " << af.min << ".." << af.max
		<< ", steps " << af.bigStep << "/" << af.smallStep
		<< ", starting in the middle at " << af.position;

	return 0;
}

int32_t Af::clampPosition(const IPAContext &context, int32_t pos) const
{
	const auto &af = context.activeState.af;

	return std::clamp(pos, af.min, af.max);
}

/* ---- measurement ------------------------------------------------------ */

void Af::startMeasure(IPAContext &context, int32_t pos)
{
	auto &af = context.activeState.af;

	af.measuring = true;
	af.measurePos = clampPosition(context, pos);
	af.target = af.measurePos;
	af.measureNeed = 0;
	af.measureGot = 0;
	af.fvSum = 0;
	af.fvSumSq = 0;
	af.ySum = 0;
	af.measureRetries = 0;
	af.haveMeasure = false;
	af.resetPending = false;

	const int32_t delta = std::abs(af.target - af.position);
	af.moveDelta = delta;

	/* Walk to the position over a few frames instead of jumping. */
	const uint32_t travel = std::clamp<uint32_t>(delta / kRampCountsPerFrame,
						     kRampMinFrames, kRampMaxFrames);
	if (delta > kRampMinDelta)
		af.rampStep = std::max<uint32_t>(1, (delta + travel - 1) / travel);
	else
		af.rampStep = delta > 0 ? delta : 1;

	/*
	 * Already there: the arrival path will not run, so start the settle
	 * countdown here, or the window would never be restarted.
	 */
	af.measureDiscard = delta == 0 ? kSettleFrames : 0;
}

bool Af::feedMeasure(IPAContext &context, const SwIspStats *stats)
{
	auto &af = context.activeState.af;

	if (!af.measuring)
		return false;

	/* Nothing is measured while the lens is on its way. */
	if (af.position != af.target)
		return false;

	/*
	 * The window has to be full, and restarted after the lens settled, which
	 * the count of frames in it takes care of: it only reaches kFvFrames
	 * once that many frames were taken here.
	 */
	if (!stats->sharpnessValid || stats->fvFrames < kFvFrames)
		return false;

	if (!af.measureNeed)
		af.measureNeed = af.meanY > 0.0 && af.meanY < kDarkMeanY
					 ? kMeasureFramesDark
					 : kMeasureFramesBright;

	af.fvSum += stats->sharpness;
	af.fvSumSq += static_cast<uint64_t>(stats->sharpness) * stats->sharpness;
	af.ySum += stats->meanY;

	if (++af.measureGot < af.measureNeed)
		return false;

	const double n = af.measureGot;
	const double mean = af.fvSum / n;
	const double variance = std::max(0.0, af.fvSumSq / n - mean * mean);
	const double meanY = af.ySum / n;

	if (af.yRef > 0.0 &&
	    std::fabs(meanY - af.yRef) > af.yRef * kMeasureYDriftPercent / 100.0 &&
	    af.measureRetries < kMeasureRetries) {
		af.measureRetries++;
		LOG(IPASoftAf, Debug)
			<< "Measure at " << af.measurePos << " dropped, mean(Y) "
			<< meanY << " vs " << af.yRef;
		af.measureGot = 0;
		af.measureNeed = 0;
		af.fvSum = 0;
		af.fvSumSq = 0;
		af.ySum = 0;
		return false;
	}

	af.fv = mean;
	af.sigma = std::sqrt(variance / n);
	af.meanY = meanY;
	af.yRef = meanY;
	af.measuring = false;
	af.haveMeasure = true;
	af.resetPending = false;

	LOG(IPASoftAf, Debug)
		<< "measure pos " << af.measurePos << " fv " << mean
		<< " sigma " << af.sigma << " meanY " << meanY;

	return true;
}

void Af::logMeasure(IPAContext &context, IPAFrameContext &frameContext)
{
	const auto &af = context.activeState.af;

	if (!csv_)
		return;

	std::fprintf(csv_, "%llu,%s,%d,%.0f,%.0f,%.0f,%d,%.1f\n",
		     static_cast<unsigned long long>(nowMs()), stateName(af.state),
		     af.measurePos, af.fv, af.sigma, af.meanY,
		     frameContext.sensor.exposure, frameContext.sensor.gain);
	std::fflush(csv_);
}

/* ---- seek ------------------------------------------------------------- */

void Af::startSeek(IPAContext &context, int32_t start, int32_t step)
{
	auto &af = context.activeState.af;

	af.state = AfState::Seek;
	af.startPos = clampPosition(context, start);
	af.step = std::max(step, af.minStep);
	af.direction = 0;
	af.phase = static_cast<uint32_t>(Phase::Probe);
	af.bestFv = 0.0;
	af.bestPos = af.startPos;
	af.prevFv = 0.0;
	af.prevPrevFv = 0.0;
	af.lastSearchMs = nowMs();
	af.lowFrames = 0;
	/* Any measurement in flight belongs to the previous search. */
	af.measuring = false;
	af.haveMeasure = false;

	LOG(IPASoftAf, Info)
		<< "Seek from " << af.startPos << " in steps of " << af.step
		<< ", light " << af.meanY;

	/* The first measurement waits for the frozen exposure to take effect. */
	af.frames = 0;
}

/*
 * \brief Fit a parabola through the three points around the peak
 *
 * The neighbours were measured on the way up and on the way down, so this only
 * costs the measurement at the position the fit picks - which doubles as the
 * confirmation that the fit is worth locking on.
 */
void Af::fitPeak(IPAContext &context, double fLow, int32_t lowPos,
		 double fHigh, int32_t highPos)
{
	auto &af = context.activeState.af;

	const double fMid = af.bestFv;
	const int32_t midPos = af.bestPos;

	int32_t pos = midPos;
	const int32_t dLow = midPos - lowPos;
	const int32_t dHigh = highPos - midPos;

	/* The fit needs equal spacing; otherwise stay on the measured peak. */
	if (dLow > 0 && dLow == dHigh) {
		const double denom = fLow - 2.0 * fMid + fHigh;
		if (denom < 0.0) {
			double offset = 0.5 * (fLow - fHigh) / denom;
			offset = std::clamp(offset, -1.0, 1.0);
			pos = midPos + static_cast<int32_t>(std::lround(offset * dLow));
		}
	}

	af.fitPos = clampPosition(context, pos);

	const bool confident = fMid - std::max(fLow, fHigh) >
			       kConfidenceSigmas * af.sigma;

	LOG(IPASoftAf, Info)
		<< "Peak at " << midPos << " fv " << fMid << ", neighbours "
		<< fLow << "/" << fHigh << ", fit " << af.fitPos
		<< (confident ? "" : " (weak)");

	af.phase = static_cast<uint32_t>(Phase::Fit);
	startMeasure(context, af.fitPos);
}

void Af::seekStep(IPAContext &context)
{
	auto &af = context.activeState.af;

	const double fv = af.fv;
	const double confidence = kConfidenceSigmas * af.sigma;
	const int32_t lo = af.min;
	const int32_t hi = af.max;

	switch (static_cast<Phase>(af.phase)) {
	case Phase::Probe:
		af.bestFv = fv;
		af.bestPos = af.measurePos;
		af.phase = static_cast<uint32_t>(Phase::ProbeUp);
		startMeasure(context, clampPosition(context, af.startPos + af.step));
		return;

	case Phase::ProbeUp:
		af.prevFv = fv;
		af.prevPos = af.measurePos;
		if (fv > af.bestFv + confidence) {
			/* Going up helps: follow it. */
			af.bestFv = fv;
			af.bestPos = af.measurePos;
			af.direction = 1;
			af.phase = static_cast<uint32_t>(Phase::Climb);
			startMeasure(context, clampPosition(context, af.bestPos + af.step));
			return;
		}

		af.phase = static_cast<uint32_t>(Phase::ProbeDown);
		startMeasure(context, clampPosition(context, af.startPos - af.step));
		return;

	case Phase::ProbeDown:
		af.prevPrevFv = af.prevFv;
		af.prevPrevPos = af.prevPos;
		af.prevFv = fv;
		af.prevPos = af.measurePos;

		if (fv > af.bestFv + confidence) {
			af.bestFv = fv;
			af.bestPos = af.measurePos;
			af.direction = -1;
			af.phase = static_cast<uint32_t>(Phase::Climb);
			startMeasure(context, clampPosition(context, af.bestPos - af.step));
			return;
		}

		/*
		 * Both probes are below the start: if the start stands clear of
		 * them it is the peak, and the two probes are its neighbours.
		 */
		if (af.bestPos == af.startPos &&
		    af.bestFv > std::max(af.prevFv, af.prevPrevFv) + confidence) {
			fitPeak(context, af.prevPrevFv, af.prevPrevPos,
				af.prevFv, af.prevPos);
			return;
		}

		/*
		 * Flat in both directions at this step: look further out before
		 * giving up, then wait and try again.
		 */
		if (af.step < af.bigStep) {
			LOG(IPASoftAf, Info)
				<< "No gradient at step " << af.step
				<< ", retrying with " << af.bigStep;
			startSeek(context, af.startPos, af.bigStep);
			return;
		}

		af.retryAtMs = nowMs() + kRetryMs;
		af.state = AfState::Locked;
		af.record = 0.0;
		af.lockPos = af.position;
		context.configuration.freezeAe = false;
		LOG(IPASoftAf, Info)
			<< "No gradient anywhere, parked at " << af.position
			<< ", retrying in " << kRetryMs << " ms";
		return;

	case Phase::Climb:
		af.prevPrevFv = af.prevFv;
		af.prevPrevPos = af.prevPos;
		af.prevFv = fv;
		af.prevPos = af.measurePos;

		if (fv > af.bestFv) {
			/* Still climbing. */
			af.bestFv = fv;
			af.bestPos = af.measurePos;
			const int32_t next = af.bestPos + af.direction * af.step;
			if (next < lo || next > hi) {
				/* Reached the end of the travel. */
				af.step /= 2;
				if (af.step < af.minStep) {
					af.phase = static_cast<uint32_t>(Phase::Fit);
					startMeasure(context, af.bestPos);
					return;
				}
				af.direction = -af.direction;
				startMeasure(context, clampPosition(context, af.bestPos +
										   af.direction * af.step));
				return;
			}
			startMeasure(context, next);
			return;
		}

		/* Past the peak: the previous point was the top. */
		fitPeak(context, af.prevPrevFv, af.prevPrevPos, fv, af.measurePos);
		return;

	case Phase::Fit:
		/*
		 * Lock on whichever of the fitted position and the measured peak
		 * is better; the measurement just taken is at the fit position.
		 */
		if (fv > af.bestFv) {
			af.bestFv = fv;
			af.bestPos = af.measurePos;
		}
		enterLocked(context, af.bestPos, af.bestFv);
		return;
	}
}

void Af::enterLocked(IPAContext &context, int32_t pos, double fv)
{
	auto &af = context.activeState.af;

	af.position = clampPosition(context, pos);
	af.target = af.position;
	af.lockPos = af.position;
	af.state = AfState::Locked;
	/*
	 * The record is taken from the locked picture itself, not from the
	 * search: the search sees the value at its best moment, and a record
	 * that sits above what the locked lens actually delivers would restart
	 * the search forever.
	 */
	af.record = 0.0;
	af.lowFrames = 0;
	af.retryAtMs = 0;
	af.fvRingIdx = 0;
	std::fill(std::begin(af.fvRing), std::end(af.fvRing), 0.0);
	context.configuration.freezeAe = false;

	LOG(IPASoftAf, Info)
		<< "Locked at " << af.position << " (fv " << fv
		<< ", mean(Y) " << af.meanY << ")";
}

/* ---- locked ----------------------------------------------------------- */

/*
 * \brief Median of the recent focus values while locked
 *
 * A single frame is not trusted: the value swings on noise and on the scene,
 * and one spike would either poison the record or restart the search.
 */
double Af::ringMedian(const IPAContext &context) const
{
	const auto &af = context.activeState.af;
	double values[5];
	unsigned int n = 0;

	for (double v : af.fvRing)
		if (v > 0.0)
			values[n++] = v;

	if (!n)
		return 0.0;

	std::sort(values, values + n);
	return values[n / 2];
}

void Af::lockedStep(IPAContext &context, const SwIspStats *stats)
{
	auto &af = context.activeState.af;

	if (!stats->sharpnessValid)
		return;

	af.fvRing[af.fvRingIdx++ % 5] = stats->sharpness;

	const double fv = ringMedian(context);

	/* Start the record once the ring holds real values. */
	if (af.record == 0.0) {
		if (af.fvRingIdx >= 5)
			af.record = fv;
		return;
	}

	/*
	 * A better value than anything seen so far becomes the new record, and
	 * the record is allowed to fall slowly: without that, one bright moment
	 * raises the bar for the rest of the session and the search restarts
	 * every fifteen frames.
	 */
	af.record *= kRecordDecay;
	if (fv > af.record) {
		af.record = fv;
		af.lockPos = af.position;
		af.lowFrames = 0;
		return;
	}

	if (fv * 100.0 >= af.record * kRestartPercent) {
		af.lowFrames = 0;
		return;
	}

	if (++af.lowFrames < kRestartFrames)
		return;

	/*
	 * The picture has been clearly below the best seen for a while, so the
	 * subject moved or the lens drifted.  Throw the record away and look for
	 * the peak again, starting from where the lens is.
	 */
	LOG(IPASoftAf, Info)
		<< "Focus down to " << fv << " of " << af.record
		<< " for " << af.lowFrames << " frames, seeking again";

	af.record = 0.0;
	startSeek(context, af.position, af.smallStep);
}

/* ---- IPA callbacks ---------------------------------------------------- */

void Af::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 DebayerParams *params)
{
	auto &af = context.activeState.af;

	if (!context.configuration.lens.available) {
		params->wantSharpness = false;
		return;
	}

	params->wantSharpness = true;
	params->resetSharpness = af.resetPending;

	/* Hold the exposure while the lens is being moved around. */
	context.configuration.freezeAe = af.state == AfState::Seek;
}

void Af::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const SwIspStats *stats, ControlList &metadata)
{
	auto &af = context.activeState.af;

	if (!context.configuration.lens.available)
		return;

	metadata.set(controls::FocusFoM, static_cast<int32_t>(stats->sharpness));

	const bool completed = feedMeasure(context, stats);
	/* The flag is a one shot: prepare() has handed it to the debayer. */
	af.resetPending = false;
	if (completed)
		logMeasure(context, frameContext);

	/* Glide towards the position being measured. */
	if (af.position != af.target) {
		const int32_t remaining = af.target - af.position;
		const int32_t step = std::clamp<int32_t>(remaining,
							 -static_cast<int32_t>(af.rampStep),
							 static_cast<int32_t>(af.rampStep));
		af.position += step;

		/*
		 * Just arrived: give the pipeline and the coil a few frames, then
		 * restart the debayer window so that it only holds frames of this
		 * position.
		 */
		if (af.position == af.target)
			af.measureDiscard = kSettleFrames +
					    std::min<uint32_t>(6, af.moveDelta / 20);
	} else if (af.measureDiscard) {
		if (--af.measureDiscard == 0)
			af.resetPending = true;
	}

	switch (af.state) {
	case AfState::Idle:
		if (af.request && stats->sharpnessValid) {
			af.request = false;
			af.state = AfState::Init;
			af.frames = 0;
			af.meanY = stats->meanY;
		}
		break;

	case AfState::Init:
		af.frames++;
		af.meanY = stats->meanY;
		if (af.frames >= kInitFrames) {
			LOG(IPASoftAf, Info)
				<< "Light level " << stats->meanY << " after "
				<< af.frames << " frames"
				<< (stats->meanY < kDarkMeanY ? ", dark" : "");
			startSeek(context, af.position, af.bigStep);
		}
		break;

	case AfState::Seek:
		if (!af.measuring) {
			/* Start the first measurement once the exposure is steady. */
			if (af.frames < kFreezeSettleFrames) {
				af.frames++;
				break;
			}
			if (!af.haveMeasure)
				startMeasure(context, af.startPos);
		}
		if (completed)
			seekStep(context);
		break;

	case AfState::Locked:
		if (af.retryAtMs && nowMs() >= af.retryAtMs) {
			af.retryAtMs = 0;
			startSeek(context, af.position, af.bigStep);
			break;
		}
		lockedStep(context, stats);
		break;
	}

	const int32_t span = std::max(1, af.max - af.min);
	const float normalised = static_cast<float>(af.position - af.min) / span;
	metadata.set(controls::LensPosition, std::clamp(normalised, 0.0f, 1.0f));
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
