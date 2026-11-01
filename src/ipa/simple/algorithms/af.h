/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Autofocus: contrast-detect search over the lens travel
 *
 * The search is a sequence of measurements.  A measurement moves the lens,
 * throws away the frames that still show the old position, averages a few
 * frames of the focus value and reports the result together with its own
 * uncertainty, so that every decision is made against that uncertainty
 * instead of against a single noisy sample.
 */

#pragma once

#include <stdio.h>

#include <stdint.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Af : public Algorithm
{
public:
	Af();
	~Af();

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	/* Measurement */
	void startMeasure(IPAContext &context, int32_t pos);
	bool feedMeasure(IPAContext &context, const SwIspStats *stats);
	void logMeasure(IPAContext &context, IPAFrameContext &frameContext);

	/* Seek: gradient, peak, parabola */
	void startSeek(IPAContext &context, int32_t start, int32_t step);
	void seekStep(IPAContext &context);
	void fitPeak(IPAContext &context, double fLow, int32_t lowPos,
		     double fHigh, int32_t highPos);
	void enterLocked(IPAContext &context, int32_t pos, double fv);

	/* Locked monitoring */
	void lockedStep(IPAContext &context, const SwIspStats *stats);
	double ringMedian(const IPAContext &context) const;

	int32_t clampPosition(const IPAContext &context, int32_t pos) const;

	FILE *csv_ = nullptr;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
