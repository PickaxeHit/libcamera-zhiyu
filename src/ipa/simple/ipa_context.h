/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026 Red Hat, Inc.
 *
 * Simple pipeline IPA Context
 */

#pragma once

#include <array>
#include <optional>
#include <stdint.h>

#include <libcamera/controls.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

#include <libipa/fc_queue.h>

#include "core_ipa_interface.h"

namespace libcamera {

namespace ipa::soft {

struct IPASessionConfiguration {
	struct {
		int32_t exposureMin, exposureMax;
		double againMin, againMax, again10, againMinStep;
		utils::Duration lineDuration;
	} agc;
	struct {
		std::optional<uint8_t> level;
	} black;

	struct {
		bool available;
		int32_t min, max;
	} lens;

	/* Set while an autofocus search runs, so the AE stops moving. */
	bool freezeAe;
};

/*
 * Autofocus states: the search itself is a sequence of measurements, so the
 * state only says what the lens is doing as a whole.
 */
enum class AfState { Idle, Init, Seek, Locked };

struct IPAActiveState {
	struct {
		int32_t exposure;
		double again;
		bool valid;
	} agc;

	struct {
		uint8_t level;
		int32_t lastExposure;
		double lastGain;
	} blc;

	struct {
		RGB<float> gains;
		unsigned int temperatureK;
	} awb;

	struct {
		AfState state = AfState::Idle;
		bool request = true;		/* search once the stream starts */

		/* Lens travel and the steps used to walk it */
		int32_t min, max;
		int32_t bigStep, smallStep, minStep;
		int32_t position;		/* position the lens is asked for */
		int32_t target;			/* where it is on its way to */
		uint32_t rampStep;		/* counts per frame while moving */
		int32_t moveDelta;		/* distance of the running move */
		bool resetPending;		/* the debayer must drop its window */

		/* Running measurement */
		bool measuring;
		int32_t measurePos;
		uint32_t measureDiscard;
		uint32_t measureNeed;
		uint32_t measureGot;
		uint64_t fvSum, fvSumSq, ySum;
		uint32_t measureRetries;
		double yRef;

		/* Last completed measurement */
		bool haveMeasure;
		double fv;
		double sigma;
		double meanY;

		/* Seek: find a gradient, then the peak, then fit a parabola */
		uint32_t phase;
		int32_t direction;
		int32_t step;
		int32_t startPos;
		int32_t bestPos;		/* position of the record */
		double bestFv;			/* the record itself */
		int32_t prevPos, prevPrevPos;
		double prevFv, prevPrevFv;
		int32_t fitPos;

		/* Init: wait for the stream to settle, then look at the light */
		uint32_t frames;
		double lightFv;			/* focus value the search started from */
		uint32_t samples;		/* frames per measurement */

		/* Locked: keep the record, restart when it drops */
		int32_t lockPos;
		double record;			/* highest focus value seen so far */
		double fvRing[5];		/* recent values, the median is used */
		uint32_t fvRingIdx;
		uint32_t lowFrames;
		uint64_t lastSearchMs;
		uint64_t retryAtMs;
	} af;

	Matrix<float, 3, 3> combinedMatrix;

	struct {
		float gamma;
		/* 0..2 range, 1.0 = normal */
		std::optional<float> contrast;
		std::optional<float> saturation;
	} knobs;
};

struct IPAFrameContext : public FrameContext {
	Matrix<float, 3, 3> ccm;

	struct {
		int32_t exposure;
		double gain;
	} sensor;

	RGB<float> gains;

	float gamma;
	std::optional<float> contrast;
	std::optional<float> saturation;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPACameraSensorInfo sensorInfo;
	IPASessionConfiguration configuration;
	IPAActiveState activeState;
	FCQueue<IPAFrameContext> frameContexts;
	ControlInfoMap::Map ctrlMap;
	bool ccmEnabled = false;
};

} /* namespace ipa::soft */

} /* namespace libcamera */
