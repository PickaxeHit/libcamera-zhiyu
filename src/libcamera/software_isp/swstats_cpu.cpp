/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * CPU based software statistics implementation
 */

#include "libcamera/internal/software_isp/swstats_cpu.h"

#include <libcamera/base/log.h>

#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

/**
 * \class SwStatsCpu
 * \brief Class for gathering statistics on the CPU
 *
 * CPU based software ISP statistics implementation.
 *
 * This class offers a configure function + functions to gather statistics on a
 * line by line basis. This allows CPU based software debayering to interleave
 * debayering and statistics gathering on a line by line basis while the input
 * data is still hot in the cache.
 *
 * It is also possible to specify a window over which to gather statistics
 * instead of processing the whole frame.
 */

/**
 * \fn SwStatsCpu::SwStatsCpu(const CameraManager &cm)
 * \brief Construct a SwStatsCpu object
 * \param[in] cm The camera manager
 *
 * Creates a SwStatsCpu object and initialises shared memory for statistics
 * exchange.
 */

/**
 * \fn bool SwStatsCpu::isValid() const
 * \brief Gets whether the statistics object is valid
 *
 * \return True if it's valid, false otherwise
 */

/**
 * \fn const SharedFD &SwStatsCpu::getStatsFD()
 * \brief Get the file descriptor for the statistics
 *
 * \return The file descriptor
 */

/**
 * \fn const Size &SwStatsCpu::patternSize()
 * \brief Get the pattern size
 *
 * For some input-formats, e.g. Bayer data, processing is done multiple lines
 * and/or columns at a time. Get width and height at which the (bayer) pattern
 * repeats. Window values are rounded down to a multiple of this and the height
 * also indicates if processLine2() should be called or not.
 * This may only be called after a successful configure() call.
 *
 * Valid sizes are: 1x1, 2x2, 4x2 or 4x4.
 *
 * \return The pattern size
 */

/**
 * \fn void SwStatsCpu::processLine0(uint32_t frame, unsigned int y, const uint8_t *src[], unsigned int statsBufferIndex = 0)
 * \brief Process line 0
 * \param[in] frame The frame number
 * \param[in] y The y coordinate
 * \param[in] src The input data
 * \param[in] statsBufferIndex Index of stats buffer to use for multi-threading
 *
 * This function processes line 0 for input formats with
 * patternSize height == 1.
 * It'll process line 0 and 1 for input formats with patternSize height >= 2.
 * This function may only be called after a successful setWindow() call.
 *
 * This function takes an array of src pointers each pointing to a line in
 * the source image.
 *
 * Bayer input data requires (patternSize_.height + 1) src pointers, with
 * the middle element of the array pointing to the actual line being processed.
 * Earlier element(s) will point to the previous line(s) and later element(s)
 * to the next line(s). See the DebayerCpu::debayerFn documentation for details.
 *
 * Planar input data requires a src pointer for each plane, with src[0] pointing
 * to the line in plane 0, etc.
 *
 * For non Bayer single plane input data only a single src pointer is required.
 *
 * The statsBufferIndex value must be less than the statsBufferCount value passed
 * to configure().
 */

/**
 * \fn void SwStatsCpu::processLine2(uint32_t frame, unsigned int y, const uint8_t *src[], unsigned int statsBufferIndex = 0)
 * \brief Process line 2 and 3
 * \param[in] frame The frame number
 * \param[in] y The y coordinate
 * \param[in] src The input data
 * \param[in] statsBufferIndex Index of stats buffer to use for multi-threading
 *
 * This function processes line 2 and 3 for input formats with
 * patternSize height == 4.
 * This function may only be called after a successful setWindow() call.
 */

/**
 * \var Signal<> SwStatsCpu::statsReady
 * \brief Signals that the statistics are ready
 */

/**
 * \var SwStatsCpu::kStatPerNumFrames
 * \brief Run stats once every kStatPerNumFrames frames
 */

/**
 * \typedef SwStatsCpu::statsProcessFn
 * \brief Called when there is data to get statistics from
 * \param[in] src The input data
 *
 * These functions take an array of (patternSize_.height + 1) src
 * pointers each pointing to a line in the source image. The middle
 * element of the array will point to the actual line being processed.
 * Earlier element(s) will point to the previous line(s) and later
 * element(s) to the next line(s).
 *
 * See the documentation of DebayerCpu::debayerFn for more details.
 */

/**
 * \var unsigned int SwStatsCpu::ySkipMask_
 * \brief Skip lines where this bitmask is set in y
 */

/**
 * \var Rectangle SwStatsCpu::window_
 * \brief Statistics window, set by setWindow(), used every line
 */

/**
 * \var unsigned int SwStatsCpu::xShift_
 * \brief The offset of x, applied to window_.x for bayer variants
 *
 * This can either be 0 or 1.
 */

LOG_DEFINE_CATEGORY(SwStatsCpu)

SwStatsCpu::SwStatsCpu(const CameraManager &cm)
	: sharedStats_("softIsp_stats"), bench_(cm, "CPU stats")
{
	if (!sharedStats_)
		LOG(SwStatsCpu, Error)
			<< "Failed to create shared memory for statistics";
}

static constexpr unsigned int kRedYMul = 77; /* 0.299 * 256 */
static constexpr unsigned int kGreenYMul = 150; /* 0.587 * 256 */
static constexpr unsigned int kBlueYMul = 29; /* 0.114 * 256 */

#define SWSTATS_START_LINE_STATS(pixel_t) \
	pixel_t r, g, g2, b;              \
	uint64_t yVal;                    \
                                          \
	uint64_t sumR = 0;                \
	uint64_t sumG = 0;                \
	uint64_t sumB = 0;

#define SWSTATS_ACCUMULATE_LINE_STATS(div) \
	sumR += r;                         \
	sumG += g;                         \
	sumB += b;                         \
                                           \
	yVal = r * kRedYMul;               \
	yVal += g * kGreenYMul;            \
	yVal += b * kBlueYMul;             \
	stats.yHistogram[yVal * SwIspStats::kYHistogramSize / (256 * 256 * (div))]++;

#define SWSTATS_FINISH_LINE_STATS() \
	stats.sum_.r() += sumR;     \
	stats.sum_.g() += sumG;     \
	stats.sum_.b() += sumB;

void SwStatsCpu::statsBGGR8Line0(const uint8_t *src[], SwIspStats &stats)
{
	const uint8_t *src0 = src[1] + window_.x;
	const uint8_t *src1 = src[2] + window_.x;

	SWSTATS_START_LINE_STATS(uint8_t)

	if (swapLines_)
		std::swap(src0, src1);

	/* x += 4 sample every other 2x2 block */
	for (unsigned int x = 0; x < window_.width; x += 4) {
		b = src0[x];
		g = src0[x + 1];
		g2 = src1[x];
		r = src1[x + 1];

		g = (g + g2) / 2;

		SWSTATS_ACCUMULATE_LINE_STATS(1)
	}

	SWSTATS_FINISH_LINE_STATS()
}

void SwStatsCpu::statsBGGR10Line0(const uint8_t *src[], SwIspStats &stats)
{
	const uint16_t *src0 = (const uint16_t *)src[1] + window_.x;
	const uint16_t *src1 = (const uint16_t *)src[2] + window_.x;

	SWSTATS_START_LINE_STATS(uint16_t)

	if (swapLines_)
		std::swap(src0, src1);

	/* x += 4 sample every other 2x2 block */
	for (unsigned int x = 0; x < window_.width; x += 4) {
		b = src0[x];
		g = src0[x + 1];
		g2 = src1[x];
		r = src1[x + 1];

		g = (g + g2) / 2;

		/* divide Y by 4 for 10 -> 8 bpp value */
		SWSTATS_ACCUMULATE_LINE_STATS(4)
	}

	SWSTATS_FINISH_LINE_STATS()
}

void SwStatsCpu::statsBGGR12Line0(const uint8_t *src[], SwIspStats &stats)
{
	const uint16_t *src0 = (const uint16_t *)src[1] + window_.x;
	const uint16_t *src1 = (const uint16_t *)src[2] + window_.x;

	SWSTATS_START_LINE_STATS(uint16_t)

	if (swapLines_)
		std::swap(src0, src1);

	/* x += 4 sample every other 2x2 block */
	for (unsigned int x = 0; x < window_.width; x += 4) {
		b = src0[x];
		g = src0[x + 1];
		g2 = src1[x];
		r = src1[x + 1];

		g = (g + g2) / 2;

		/* divide Y by 16 for 12 -> 8 bpp value */
		SWSTATS_ACCUMULATE_LINE_STATS(16)
	}

	SWSTATS_FINISH_LINE_STATS()
}

void SwStatsCpu::statsBGGR10PLine0(const uint8_t *src[], SwIspStats &stats)
{
	const uint8_t *src0 = src[1] + window_.x * 5 / 4;
	const uint8_t *src1 = src[2] + window_.x * 5 / 4;
	const unsigned int widthInBytes = window_.width * 5 / 4;

	if (swapLines_)
		std::swap(src0, src1);

	SWSTATS_START_LINE_STATS(uint8_t)

	/* x += 5 sample every other 2x2 block */
	for (unsigned int x = 0; x < widthInBytes; x += 5) {
		/* BGGR */
		b = src0[x];
		g = src0[x + 1];
		g2 = src1[x];
		r = src1[x + 1];
		g = (g + g2) / 2;
		/* Data is already 8 bits, divide by 1 */
		SWSTATS_ACCUMULATE_LINE_STATS(1)
	}

	SWSTATS_FINISH_LINE_STATS()
}

void SwStatsCpu::statsGBRG10PLine0(const uint8_t *src[], SwIspStats &stats)
{
	const uint8_t *src0 = src[1] + window_.x * 5 / 4;
	const uint8_t *src1 = src[2] + window_.x * 5 / 4;
	const unsigned int widthInBytes = window_.width * 5 / 4;

	if (swapLines_)
		std::swap(src0, src1);

	SWSTATS_START_LINE_STATS(uint8_t)

	/* x += 5 sample every other 2x2 block */
	for (unsigned int x = 0; x < widthInBytes; x += 5) {
		/* GBRG */
		g = src0[x];
		b = src0[x + 1];
		r = src1[x];
		g2 = src1[x + 1];
		g = (g + g2) / 2;
		/* Data is already 8 bits, divide by 1 */
		SWSTATS_ACCUMULATE_LINE_STATS(1)
	}

	SWSTATS_FINISH_LINE_STATS()
}

void SwStatsCpu::statsBGGR12PLine0(const uint8_t *src[], SwIspStats &stats)
{
	const uint8_t *src0 = src[1] + window_.x * 3 / 2;
	const uint8_t *src1 = src[2] + window_.x * 3 / 2;
	const unsigned int widthInBytes = window_.width * 3 / 2;

	SWSTATS_START_LINE_STATS(uint8_t)

	if (swapLines_)
		std::swap(src0, src1);

	/* x += 6 sample every other 2x2 block */
	for (unsigned int x = 0; x < widthInBytes; x += 6) {
		b = src0[x];
		g = src0[x + 1];
		g2 = src1[x];
		r = src1[x + 1];

		g = (g + g2) / 2;

		SWSTATS_ACCUMULATE_LINE_STATS(1)
	}

	SWSTATS_FINISH_LINE_STATS()
}

void SwStatsCpu::statsGBRG12PLine0(const uint8_t *src[], SwIspStats &stats)
{
	const uint8_t *src0 = src[1] + window_.x * 3 / 2;
	const uint8_t *src1 = src[2] + window_.x * 3 / 2;
	const unsigned int widthInBytes = window_.width * 3 / 2;

	SWSTATS_START_LINE_STATS(uint8_t)

	if (swapLines_)
		std::swap(src0, src1);

	/* x += 6 sample every other 2x2 block */
	for (unsigned int x = 0; x < widthInBytes; x += 6) {
		g = src0[x];
		b = src0[x + 1];
		r = src1[x];
		g2 = src1[x + 1];

		g = (g + g2) / 2;

		SWSTATS_ACCUMULATE_LINE_STATS(1)
	}

	SWSTATS_FINISH_LINE_STATS()
}

/**
 * \brief Reset state to start statistics gathering for a new frame
 * \param[in] frame The frame number
 *
 * This may only be called after a successful setWindow() call.
 */
void SwStatsCpu::startFrame(uint32_t frame)
{
	if (frame % kStatPerNumFrames)
		return;

	if (window_.width == 0)
		LOG(SwStatsCpu, Error) << "Calling startFrame() without setWindow()";

	for (auto &s : stats_) {
		s.sum_ = RGB<uint64_t>({ 0, 0, 0 });
		s.yHistogram.fill(0);
	}
}

/**
 * \brief Finish statistics calculation for the current frame
 * \param[in] frame The frame number
 * \param[in] bufferId ID of the statistics buffer
 *
 * This may only be called after a successful setWindow() call.
 */
void SwStatsCpu::finishFrame(uint32_t frame, uint32_t bufferId)
{
	bool valid = frame % kStatPerNumFrames == 0;

	if (valid) {
		sharedStats_->sum_ = RGB<uint64_t>({ 0, 0, 0 });
		sharedStats_->yHistogram.fill(0);
		for (const auto &s : stats_) {
			sharedStats_->sum_ += s.sum_;
			for (unsigned int j = 0; j < SwIspStats::kYHistogramSize; j++)
				sharedStats_->yHistogram[j] += s.yHistogram[j];
		}

		sharedStats_->sum_ >>= sumShift_;
		/* Keep the image measurement when the debayer produces one. */
		if (!imageSharpnessValid_)
			sharedStats_->sharpness = stats_.size() ? stats_[0].sharpness : 0;
	}

	sharedStats_->sharpnessValid = sharpnessThisFrame_ || imageSharpnessValid_;
	sharedStats_->valid = valid;
	sharpnessThisFrame_ = false;
	statsReady.emit(frame, bufferId);
}

/**
 * \brief Setup SwStatsCpu object for standard Bayer orders
 * \param[in] order The Bayer order
 *
 * Check if order is a standard Bayer order and setup xShift_ and swapLines_
 * so that a single BGGR stats function can be used for all 4 standard orders.
 */
int SwStatsCpu::setupStandardBayerOrder(BayerFormat::Order order)
{
	switch (order) {
	case BayerFormat::BGGR:
		xShift_ = 0;
		swapLines_ = false;
		break;
	case BayerFormat::GBRG:
		xShift_ = 1; /* BGGR -> GBRG */
		swapLines_ = false;
		break;
	case BayerFormat::GRBG:
		xShift_ = 0;
		swapLines_ = true; /* BGGR -> GRBG */
		break;
	case BayerFormat::RGGB:
		xShift_ = 1; /* BGGR -> GBRG */
		swapLines_ = true; /* GBRG -> RGGB */
		break;
	default:
		return -EINVAL;
	}

	patternSize_.height = 2;
	patternSize_.width = 2;
	ySkipMask_ = 0x02; /* Skip every 3th and 4th line */
	return 0;
}

/**
 * \brief Configure the statistics object for the passed in input format
 * \param[in] inputCfg The input format
 * \param[in] statsBufferCount number of internal stats buffers to use for multi-threading
 *
 * \return 0 on success, a negative errno value on failure
 */
int SwStatsCpu::configure(const StreamConfiguration &inputCfg, unsigned int statsBufferCount)
{
	stride_ = inputCfg.stride;
	stats_.resize(statsBufferCount);

	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputCfg.pixelFormat);

	if (bayerFormat.packing == BayerFormat::Packing::None &&
	    setupStandardBayerOrder(bayerFormat.order) == 0) {
		processFrame_ = &SwStatsCpu::processBayerFrame2;
		switch (bayerFormat.bitDepth) {
		case 8:
			stats0_ = &SwStatsCpu::statsBGGR8Line0;
			sumShift_ = 0;
			return 0;
		case 10:
			stats0_ = &SwStatsCpu::statsBGGR10Line0;
			sumShift_ = 2;
			return 0;
		case 12:
			stats0_ = &SwStatsCpu::statsBGGR12Line0;
			sumShift_ = 4;
			return 0;
		}
	}

	uint8_t bitDepth = bayerFormat.bitDepth;

	if ((bitDepth == 10 || bitDepth == 12) &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
		if (bitDepth == 10)
			patternSize_.width = 4; /* 5 bytes per *4* pixels */
		else
			patternSize_.width = 2; /* 3 bytes for *2* pixels */

		patternSize_.height = 2;

		/* Skip every 3th and 4th line, sample every other 2x2 block */
		ySkipMask_ = 0x02;
		xShift_ = 0;
		sumShift_ = 0;
		processFrame_ = &SwStatsCpu::processBayerFrame2;

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
		case BayerFormat::GRBG:
			stats0_ = (bitDepth == 10) ? &SwStatsCpu::statsBGGR10PLine0 : &SwStatsCpu::statsBGGR12PLine0;
			swapLines_ = bayerFormat.order == BayerFormat::GRBG;
			return 0;
		case BayerFormat::GBRG:
		case BayerFormat::RGGB:
			stats0_ = (bitDepth == 10) ? &SwStatsCpu::statsGBRG10PLine0 : &SwStatsCpu::statsGBRG12PLine0;
			swapLines_ = bayerFormat.order == BayerFormat::RGGB;
			return 0;
		default:
			break;
		}
	}

	LOG(SwStatsCpu, Info)
		<< "Unsupported input format " << inputCfg.pixelFormat.toString();
	return -EINVAL;
}

/**
 * \brief Specify window coordinates over which to gather statistics
 * \param[in] window The window object.
 *
 * This method specifies the image area over which to gather the statistics.
 * It must be called to set the area, otherwise the default zero-sized
 * \a Rectangle is used and no statistics is gathered.
 *
 * The specified \a window is relative to what is passed to the processLine*
 * methods. For example, if statistics are to be gathered from the entire
 * processed area, then \a window should be a rectangle with the top-left corner
 * of (0,0) and the same size as the processed area. If only a part of the
 * processed area (e.g. its centre) is to be considered for statistics, then
 * \a window should specify such a restriction accordingly.
 *
 * It is the responsibility of the callers to provide sensible \a window values,
 * most notably not exceeding the original image boundaries. This means, among
 * other, that neither coordinate of the top-left corner shall be negative.
 *
 * Due to limitations of the implementation, the method may adjust the window
 * slightly if it is not aligned according to the bayer pattern determined in
 * \a SwStatsCpu::configure(). In that case the window will be modified such that
 * the sides are no larger than the original, and that the new bottom-left
 * corner will be no further from (0,0) (along either axis) than the original
 * was.
 */
void SwStatsCpu::setWindow(const Rectangle &window)
{
	ASSERT(window.x >= 0 && window.y >= 0);

	window_ = window;

	window_.x &= ~(patternSize_.width - 1);
	window_.x += xShift_;
	window_.y &= ~(patternSize_.height - 1);

	/* width_ - xShift_ to make sure the window fits */
	window_.width = (window_.width > xShift_ ? window_.width - xShift_ : 0);
	window_.width &= ~(patternSize_.width - 1);
	window_.height &= ~(patternSize_.height - 1);
}

void SwStatsCpu::processBayerFrame2(MappedFrameBuffer &in)
{
	const uint8_t *src = in.planes()[0].data();
	const uint8_t *linePointers[3];

	/* Adjust src for starting at window_.y */
	src += window_.y * stride_;

	for (unsigned int y = 0; y < window_.height; y += 2) {
		if (y & ySkipMask_) {
			src += stride_ * 2;
			continue;
		}

		/* linePointers[0] is not used by any stats0_ functions */
		linePointers[1] = src;
		linePointers[2] = src + stride_;
		(this->*stats0_)(linePointers, stats_[0]);
		src += stride_ * 2;
	}
}

/**
 * \brief Measure how sharp the frame is, for the autofocus search
 * \param[in] in The frame to process
 *
 * Autofocus needs one number per frame that peaks when the lens is in focus.
 * This uses the Tenengrad function, the sum of the squared horizontal and
 * vertical Sobel responses, which is what hardware autofocus engines measure
 * too, and it reacts strongly to the couple of pixels of blur that defocus
 * produces on a 13 MP sensor.
 *
 * Each sample is the average of one 2x2 Bayer cell, which matters for two
 * reasons: it mixes the colours away (sampling the raw pattern directly turns
 * the colour filter array into a checkerboard of gradients that has nothing to
 * do with focus), and it averages the sensor noise down, so that a frame taken
 * at high gain does not look sharper than a well-exposed one.  The metric is
 * normalised by the square of the mean level so that the AGC changing the
 * exposure during a search does not look like focus either.
 *
 * The patch is measured at full resolution: defocus blurs detail by only a
 * pixel or two, so downsampling the frame first throws the signal away.
 */
void SwStatsCpu::calculateSharpness(MappedFrameBuffer &in)
{
	if (!stats_.size())
		return;

	SwIspStats &stats = stats_[0];
	const uint8_t *src = in.planes()[0].data();

	const unsigned int width = std::min(window_.width, 1600u) & ~3u;
	const unsigned int height = std::min(window_.height, 1200u) & ~1u;
	const unsigned int x0 = (window_.x + (window_.width - width) / 2) & ~3u;
	const unsigned int y0 = (window_.y + (window_.height - height) / 2) & ~1u;
	const unsigned int cols = width / 2;	/* cells */
	const unsigned int rows = height / 2;	/* cells */
	const unsigned int groups = width / 4;	/* four pixels per five bytes */

	if (cols < 3 || rows < 3 || groups < 2) {
		stats.sharpness = 0;
		return;
	}

	sharpness_.resize(rows * cols);

	uint64_t sum = 0;

	for (unsigned int row = 0; row < rows; row++) {
		const uint8_t *l0 = src + (y0 + row * 2) * stride_ + (x0 / 4) * 5;
		const uint8_t *l1 = l0 + stride_;

		for (unsigned int g = 0; g < groups; g++) {
			const uint8_t *a = l0 + g * 5;
			const uint8_t *b = l1 + g * 5;

			/* Two cells out of one five-byte group per row. */
			uint16_t c00 = a[0] | ((a[4] & 0x3) << 8);
			uint16_t c01 = a[1] | (((a[4] >> 2) & 0x3) << 8);
			uint16_t c10 = b[0] | ((b[4] & 0x3) << 8);
			uint16_t c11 = b[1] | (((b[4] >> 2) & 0x3) << 8);
			uint16_t value = (c00 + c01 + c10 + c11) / 4;

			sharpness_[row * cols + g * 2] = value;
			sum += value;

			c00 = a[2] | (((a[4] >> 4) & 0x3) << 8);
			c01 = a[3] | (((a[4] >> 6) & 0x3) << 8);
			c10 = b[2] | (((b[4] >> 4) & 0x3) << 8);
			c11 = b[3] | (((b[4] >> 6) & 0x3) << 8);
			value = (c00 + c01 + c10 + c11) / 4;

			sharpness_[row * cols + g * 2 + 1] = value;
			sum += value;
		}
	}

	const uint64_t n = static_cast<uint64_t>(rows) * cols;

	if (!n || !sum) {
		stats.sharpness = 0;
		return;
	}

	const uint64_t mean = sum / n;
	uint64_t tenengrad = 0;

	for (unsigned int row = 1; row + 1 < rows; row++) {
		for (unsigned int col = 1; col + 1 < cols; col++) {
			const unsigned int i = row * cols + col;
			const int tl = sharpness_[i - cols - 1], tc = sharpness_[i - cols];
			const int tr = sharpness_[i - cols + 1];
			const int ml = sharpness_[i - 1], mr = sharpness_[i + 1];
			const int bl = sharpness_[i + cols - 1], bc = sharpness_[i + cols];
			const int br = sharpness_[i + cols + 1];

			const int gx = (tr + 2 * mr + br) - (tl + 2 * ml + bl);
			const int gy = (bl + 2 * bc + br) - (tl + 2 * tc + tr);

			tenengrad += static_cast<uint64_t>(gx * gx) + gy * gy;
		}
	}

	const uint64_t count = (rows - 2) * (cols - 2);

	stats.sharpness = mean ? tenengrad * 1000 / (count * mean * mean) : 0;
	LOG(SwStatsCpu, Debug)
		<< "sharpness " << stats.sharpness << " (" << cols << "x" << rows
		<< " cells, mean " << mean << ")";
}

/**
 * \brief Publish a sharpness value measured by the debayer
 * \param[in] value Normalised sharpness, in the same units as the statistics
 *            pass produces
 */
void SwStatsCpu::setSharpness(uint64_t value, uint32_t meanY, uint32_t noise,
			      uint32_t frames)
{
	sharedStats_->sharpness = value;
	sharedStats_->meanY = meanY;
	sharedStats_->noise = noise;
	sharedStats_->fvFrames = frames;
	sharedStats_->sharpnessValid = true;
	imageSharpnessValid_ = true;
}

/**
 * \brief Calculate statistics for a frame in one go
 * \param[in] frame The frame number
 * \param[in] bufferId ID of the statistics buffer
 * \param[in] input The frame to process
 *
 * This may only be called after a successful setWindow() call.
 */
void SwStatsCpu::processFrame(uint32_t frame, uint32_t bufferId, MappedFrameBuffer &input)
{
	if (frame % kStatPerNumFrames) {
		/*
		 * Autofocus wants a measurement per frame, it cannot step the lens
		 * once per statistics interval or the search crawls.  The sharpness
		 * costs little next to the full statistics pass, so measure it here
		 * and leave everything else alone.
		 */
		if (wantSharpness_) {
			startFrame(frame);
			calculateSharpness(input);
			sharpnessThisFrame_ = true;
		}
		finishFrame(frame, bufferId);
		return;
	}

	bench_.startFrame();
	startFrame(frame);
	(this->*processFrame_)(input);
	if (wantSharpness_) {
		calculateSharpness(input);
		sharpnessThisFrame_ = true;
	}
	finishFrame(frame, bufferId);
	bench_.finishFrame();
}

} /* namespace libcamera */
