/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023-2026 Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * DebayerParams header
 */

#pragma once

#include <array>
#include <stdint.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

namespace libcamera {

struct DebayerParams {
	/*
	 * Lens shading correction grid: per-colour gains sampled on a regular
	 * grid covering the frame, row major, origin at the top left.  The
	 * grid is interpolated bilinearly between samples.
	 */
	static constexpr unsigned int kLscCols = 17;
	static constexpr unsigned int kLscRows = 13;
	std::array<float, kLscCols * kLscRows * 3> lsc = {};
	bool lscEnabled = false;

	Matrix<float, 3, 3> combinedMatrix = { { 1.0, 0.0, 0.0,
						 0.0, 1.0, 0.0,
						 0.0, 0.0, 1.0 } };
	RGB<float> blackLevel = RGB<float>({ 0.0, 0.0, 0.0 });
	float gamma = 1.0;
	float contrastExp = 1.0;
	RGB<float> gains = RGB<float>({ 1.0, 1.0, 1.0 });
};

} /* namespace libcamera */
