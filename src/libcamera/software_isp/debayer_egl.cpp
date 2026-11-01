/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 */

#include "debayer_egl.h"

#include <algorithm>
#include <array>
#include <assert.h>
#include <memory>
#include <stdlib.h>
#include <string>
#include <tuple>
#include <vector>

#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

#include <libcamera/formats.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/framebuffer.h"

#include "../shaders/glsl_shaders.h"

namespace libcamera {

/**
 * \class DebayerEGL
 * \brief Class for debayering using an EGL Shader
 *
 * Implements an EGL shader based debayering solution.
 */

/* Full screen quad, shared by every pass */
static const GLfloat kVertexCoordinates[4][2] = {
	{ -1.0f, -1.0f },
	{ -1.0f, +1.0f },
	{ +1.0f, +1.0f },
	{ +1.0f, -1.0f },
};

static const GLfloat kTextureCoordinates[4][2] = {
	{ 0.0f, 0.0f },
	{ 0.0f, 1.0f },
	{ 1.0f, 1.0f },
	{ 1.0f, 0.0f },
};

static const GLfloat kIdentityMatrix[16] = {
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f,
};

/**
 * \brief Construct a DebayerEGL object
 * \param[in] stats Statistics processing object
 * \param[in] cm The camera manager
 */
DebayerEGL::DebayerEGL(std::unique_ptr<SwStatsCpu> stats, const CameraManager &cm)
	: Debayer(cm), stats_(std::move(stats))
{
}

DebayerEGL::~DebayerEGL()
{
}

int DebayerEGL::getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);

	/*
	 * YUYV is listed first because that is what video applications
	 * (browsers, video conferencing) consume; the RGB formats stay
	 * available for still capture and for applications that prefer them.
	 */
	std::vector<PixelFormat> outputFormats = { formats::YUYV,
						   formats::XRGB8888,
						   formats::ARGB8888,
						   formats::XBGR8888,
						   formats::ABGR8888 };

	if ((bayerFormat.bitDepth == 8 || bayerFormat.bitDepth == 10) &&
	    bayerFormat.packing == BayerFormat::Packing::None &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = (bayerFormat.bitDepth + 7) & ~7;
		config.patternSize.width = 2;
		config.patternSize.height = 2;
		config.outputFormats = outputFormats;
		return 0;
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2 &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = 10;
		config.patternSize.width = 4; /* 5 bytes per *4* pixels */
		config.patternSize.height = 2;
		config.outputFormats = outputFormats;
		return 0;
	}

	if (bayerFormat.bitDepth == 12 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2 &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = 12;
		config.patternSize.width = 2; /* 3 bytes per *2* pixels */
		config.patternSize.height = 2;
		config.outputFormats = outputFormats;
		return 0;
	}

	LOG(Debayer, Info)
		<< "Unsupported input format " << inputFormat;

	return -EINVAL;
}

int DebayerEGL::getOutputConfig(PixelFormat outputFormat, DebayerOutputConfig &config)
{
	if (outputFormat == formats::YUYV) {
		config.bpp = 16;
		return 0;
	}

	if (outputFormat == formats::XRGB8888 || outputFormat == formats::ARGB8888 ||
	    outputFormat == formats::XBGR8888 || outputFormat == formats::ABGR8888) {
		config.bpp = 32;
		return 0;
	}

	LOG(Debayer, Info)
		<< "Unsupported output format " << outputFormat;

	return -EINVAL;
}

int DebayerEGL::getShaderVariableLocations(void)
{
	attributeVertex_ = glGetAttribLocation(programId_, "vertexIn");
	attributeTexture_ = glGetAttribLocation(programId_, "textureIn");

	textureUniformBayerDataIn_ = glGetUniformLocation(programId_, "tex_y");
	awbUniformDataIn_ = glGetUniformLocation(programId_, "awb");
	ccmUniformDataIn_ = glGetUniformLocation(programId_, "ccm");
	blackLevelUniformDataIn_ = glGetUniformLocation(programId_, "blacklevel");
	gammaUniformDataIn_ = glGetUniformLocation(programId_, "gamma");
	contrastExpUniformDataIn_ = glGetUniformLocation(programId_, "contrastExp");
	lscUniformDataIn_ = glGetUniformLocation(programId_, "lsc");
	lscEnabledUniformDataIn_ = glGetUniformLocation(programId_, "lscEnabled");

	textureUniformStep_ = glGetUniformLocation(programId_, "tex_step");
	textureUniformSize_ = glGetUniformLocation(programId_, "tex_size");
	textureUniformStrideFactor_ = glGetUniformLocation(programId_, "stride_factor");
	textureUniformBayerFirstRed_ = glGetUniformLocation(programId_, "tex_bayer_first_red");
	textureUniformProjMatrix_ = glGetUniformLocation(programId_, "proj_matrix");

	LOG(Debayer, Debug) << "vertexIn " << attributeVertex_ << " textureIn " << attributeTexture_
			    << " tex_y " << textureUniformBayerDataIn_
			    << " awb " << awbUniformDataIn_
			    << " ccm " << ccmUniformDataIn_
			    << " blacklevel " << blackLevelUniformDataIn_
			    << " gamma " << gammaUniformDataIn_
			    << " contrastExp " << contrastExpUniformDataIn_
			    << " lsc " << lscUniformDataIn_
			    << " lscEnabled " << lscEnabledUniformDataIn_
			    << " tex_step " << textureUniformStep_
			    << " tex_size " << textureUniformSize_
			    << " stride_factor " << textureUniformStrideFactor_
			    << " tex_bayer_first_red " << textureUniformBayerFirstRed_
			    << " proj_matrix " << textureUniformProjMatrix_;
	return 0;
}

int DebayerEGL::initBayerShaders(PixelFormat inputFormat, PixelFormat outputFormat)
{
	std::vector<std::string> shaderEnv;
	Span<const unsigned char> fragmentShaderData;
	Span<const unsigned char> vertexShaderData;
	GLenum err;

	/* Target gles 100 glsl requires "#version x" as first directive in shader */
	egl_.pushEnv(shaderEnv, "#version 100");

	/* Specify GL_OES_EGL_image_external */
	egl_.pushEnv(shaderEnv, "#extension GL_OES_EGL_image_external: enable");

	/*
	 * Tell shaders how to re-order output taking account of how the pixels
	 * are actually stored by EGL.
	 */
	switch (outputFormat) {
	case formats::ARGB8888:
	case formats::XRGB8888:
		break;
	case formats::ABGR8888:
	case formats::XBGR8888:
		egl_.pushEnv(shaderEnv, "#define SWAP_BLUE");
		break;
	case formats::YUYV:
		/*
		 * The first pass renders into an ordinary RGBA texture, the
		 * packing into YUYV happens in a second pass, so no channel
		 * reordering is needed here.
		 */
		break;
	default:
		LOG(Debayer, Error) << "Unsupported output format";
		return -EINVAL;
	}

	/* Pixel location parameters */
	glFormat_ = GL_LUMINANCE;
	bytesPerPixel_ = 1;
	shaderStridePixels_ = inputConfig_.stride;

	switch (inputFormat) {
	case libcamera::formats::SBGGR8:
	case libcamera::formats::SBGGR10_CSI2P:
	case libcamera::formats::SBGGR12_CSI2P:
		firstRed_x_ = 1.0;
		firstRed_y_ = 1.0;
		break;
	case libcamera::formats::SGBRG8:
	case libcamera::formats::SGBRG10_CSI2P:
	case libcamera::formats::SGBRG12_CSI2P:
		firstRed_x_ = 0.0;
		firstRed_y_ = 1.0;
		break;
	case libcamera::formats::SGRBG8:
	case libcamera::formats::SGRBG10_CSI2P:
	case libcamera::formats::SGRBG12_CSI2P:
		firstRed_x_ = 1.0;
		firstRed_y_ = 0.0;
		break;
	case libcamera::formats::SRGGB8:
	case libcamera::formats::SRGGB10_CSI2P:
	case libcamera::formats::SRGGB12_CSI2P:
		firstRed_x_ = 0.0;
		firstRed_y_ = 0.0;
		break;
	default:
		LOG(Debayer, Error) << "Unsupported input format";
		return -EINVAL;
	};

	/* Shader selection */
	switch (inputFormat) {
	case libcamera::formats::SBGGR8:
	case libcamera::formats::SGBRG8:
	case libcamera::formats::SGRBG8:
	case libcamera::formats::SRGGB8:
		fragmentShaderData = bayer_unpacked_frag;
		vertexShaderData = bayer_unpacked_vert;
		break;
	case libcamera::formats::SBGGR10_CSI2P:
	case libcamera::formats::SGBRG10_CSI2P:
	case libcamera::formats::SGRBG10_CSI2P:
	case libcamera::formats::SRGGB10_CSI2P:
		egl_.pushEnv(shaderEnv, "#define RAW10P");
		if (BayerFormat::fromPixelFormat(inputFormat).packing == BayerFormat::Packing::None) {
			fragmentShaderData = bayer_unpacked_frag;
			vertexShaderData = bayer_unpacked_vert;
			glFormat_ = GL_RG;
			bytesPerPixel_ = 2;
		} else {
			fragmentShaderData = bayer_1x_packed_frag;
			vertexShaderData = identity_vert;
			shaderStridePixels_ = width_;
		}
		break;
	case libcamera::formats::SBGGR12_CSI2P:
	case libcamera::formats::SGBRG12_CSI2P:
	case libcamera::formats::SGRBG12_CSI2P:
	case libcamera::formats::SRGGB12_CSI2P:
		egl_.pushEnv(shaderEnv, "#define RAW12P");
		if (BayerFormat::fromPixelFormat(inputFormat).packing == BayerFormat::Packing::None) {
			fragmentShaderData = bayer_unpacked_frag;
			vertexShaderData = bayer_unpacked_vert;
			glFormat_ = GL_RG;
			bytesPerPixel_ = 2;
		} else {
			fragmentShaderData = bayer_1x_packed_frag;
			vertexShaderData = identity_vert;
			shaderStridePixels_ = width_;
		}
		break;
	};

	if (egl_.compileVertexShader(vertexShaderId_, vertexShaderData,
				     shaderEnv)) {
		LOG(Debayer, Error) << "Compile vertex shader fail";
		return -ENODEV;
	}
	utils::scope_exit vShaderGuard([&] { glDeleteShader(vertexShaderId_); });

	if (egl_.compileFragmentShader(fragmentShaderId_, fragmentShaderData,
				       shaderEnv)) {
		LOG(Debayer, Error) << "Compile fragment shader fail";
		return -ENODEV;
	}
	utils::scope_exit fShaderGuard([&] { glDeleteShader(fragmentShaderId_); });

	if (egl_.linkProgram(programId_, vertexShaderId_, fragmentShaderId_)) {
		LOG(Debayer, Error) << "Linking program fail";
		return -ENODEV;
	}

	egl_.dumpShaderSource(vertexShaderId_);
	egl_.dumpShaderSource(fragmentShaderId_);

	/* Ensure we set the programId_ */
	egl_.useProgram(programId_);
	err = glGetError();
	if (err != GL_NO_ERROR) {
		LOG(Debayer, Error) << "Use program error " << err;
		return -ENODEV;
	}

	int ret = getShaderVariableLocations();
	if (ret)
		return ret;

	if (outputFormat == formats::YUYV)
		return initYUYVShaders();

	return 0;
}

int DebayerEGL::initYUYVShaders()
{
	std::vector<std::string> shaderEnv;

	egl_.pushEnv(shaderEnv, "#version 100");

	if (egl_.compileVertexShader(yuyvVertexShaderId_, identity_vert,
				     shaderEnv)) {
		LOG(Debayer, Error) << "Compile YUYV vertex shader fail";
		return -ENODEV;
	}
	utils::scope_exit vShaderGuard([&] { glDeleteShader(yuyvVertexShaderId_); });

	if (egl_.compileFragmentShader(yuyvFragmentShaderId_, rgb_to_yuyv_frag,
				       shaderEnv)) {
		LOG(Debayer, Error) << "Compile YUYV fragment shader fail";
		return -ENODEV;
	}
	utils::scope_exit fShaderGuard([&] { glDeleteShader(yuyvFragmentShaderId_); });

	if (egl_.linkProgram(yuyvProgramId_, yuyvVertexShaderId_,
			     yuyvFragmentShaderId_)) {
		LOG(Debayer, Error) << "Linking YUYV program fail";
		return -ENODEV;
	}

	egl_.dumpShaderSource(yuyvVertexShaderId_);
	egl_.dumpShaderSource(yuyvFragmentShaderId_);

	yuyvAttributeVertex_ = glGetAttribLocation(yuyvProgramId_, "vertexIn");
	yuyvAttributeTexture_ = glGetAttribLocation(yuyvProgramId_, "textureIn");
	yuyvTextureUniformIn_ = glGetUniformLocation(yuyvProgramId_, "tex_rgb");
	yuyvTextureUniformStep_ = glGetUniformLocation(yuyvProgramId_, "tex_step");
	yuyvTextureUniformStrideFactor_ = glGetUniformLocation(yuyvProgramId_, "stride_factor");
	yuyvTextureUniformProjMatrix_ = glGetUniformLocation(yuyvProgramId_, "proj_matrix");

	LOG(Debayer, Debug) << "YUYV vertexIn " << yuyvAttributeVertex_
			    << " textureIn " << yuyvAttributeTexture_
			    << " tex_rgb " << yuyvTextureUniformIn_
			    << " tex_step " << yuyvTextureUniformStep_;

	/* Leave the debayer program active, the pass is selected per frame. */
	egl_.useProgram(programId_);

	return 0;
}

int DebayerEGL::configure(const StreamConfiguration &inputCfg,
			  const std::vector<std::reference_wrapper<const StreamConfiguration>> &outputCfgs,
			  [[maybe_unused]] bool ccmEnabled)
{
	if (getInputConfig(inputCfg.pixelFormat, inputConfig_) != 0)
		return -EINVAL;

	if (stats_->configure(inputCfg) != 0)
		return -EINVAL;

	const Size &stats_pattern_size = stats_->patternSize();
	if (inputConfig_.patternSize.width != stats_pattern_size.width ||
	    inputConfig_.patternSize.height != stats_pattern_size.height) {
		LOG(Debayer, Error)
			<< "mismatching stats and debayer pattern sizes for "
			<< inputCfg.pixelFormat;
		return -EINVAL;
	}

	inputConfig_.stride = inputCfg.stride;
	inputPixelFormat_ = inputCfg.pixelFormat;
	width_ = inputCfg.size.width;
	height_ = inputCfg.size.height;

	if (outputCfgs.size() != 1) {
		LOG(Debayer, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	const StreamConfiguration &outputCfg = outputCfgs[0];
	SizeRange outSizeRange = sizes(inputCfg.pixelFormat, inputCfg.size);
	std::tie(outputConfig_.stride, outputConfig_.frameSize) =
		strideAndFrameSize(outputCfg.pixelFormat, outputCfg.size);

	if (!outSizeRange.contains(outputCfg.size) || outputConfig_.stride != outputCfg.stride) {
		LOG(Debayer, Error)
			<< "Invalid output size/stride: "
			<< "\n  " << outputCfg.size << " (" << outSizeRange << ")"
			<< "\n  " << outputCfg.stride << " (" << outputConfig_.stride << ")";
		return -EINVAL;
	}

	outputPixelFormat_ = outputCfg.pixelFormat;
	outputSize_ = outputCfg.size;
	nativeOutputSize_ = outSizeRange.max;

	window_.x = ((inputCfg.size.width - outputCfg.size.width) / 2) &
		    ~(inputConfig_.patternSize.width - 1);
	window_.y = ((inputCfg.size.height - outputCfg.size.height) / 2) &
		    ~(inputConfig_.patternSize.height - 1);
	window_.width = outputCfg.size.width;
	window_.height = outputCfg.size.height;

	/*
	 * Don't pass x,y from window_ since process() already adjusts for it.
	 * But crop the window to 2/3 of its width and height for speedup.
	 */
	stats_->setWindow(Rectangle(window_.size()));

	inputBufferCount_ = inputCfg.bufferCount;
	outputBufferCount_ = outputCfg.bufferCount;

	/*
	 * A YUYV buffer is imported as a 32-bit image in which one texel holds
	 * two pixels, so the image is half as wide as the frame. The stride is
	 * padded, which only adds texels at the end of a row that are never
	 * rendered to.
	 */
	outputIsYUYV_ = outputPixelFormat_ == formats::YUYV;
	outputImageWidth_ = outputIsYUYV_ ? outputConfig_.stride / 4
					  : outputSize_.width;

	return 0;
}

Size DebayerEGL::patternSize(PixelFormat inputFormat)
{
	DebayerEGL::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return {};

	return config.patternSize;
}

std::vector<PixelFormat> DebayerEGL::formats(PixelFormat inputFormat)
{
	DebayerEGL::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return std::vector<PixelFormat>();

	return config.outputFormats;
}

std::tuple<unsigned int, unsigned int>
DebayerEGL::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	DebayerEGL::DebayerOutputConfig config;

	if (getOutputConfig(outputFormat, config) != 0)
		return std::make_tuple(0, 0);

	/* Align stride to 256 bytes as a generic GPU memory access alignment */
	unsigned int stride = libcamera::utils::alignUp(size.width * config.bpp / 8, 256);

	return std::make_tuple(stride, stride * size.height);
}

uint32_t DebayerEGL::preferredInputStride(const PixelFormat &inputFormat, const Size &size)
{
	const PixelFormatInfo &info = PixelFormatInfo::info(inputFormat);
	return info.stride(size.width, 0, 256);
}

void DebayerEGL::setShaderVariableValues(eGLImage &eglImageIn, const DebayerParams &params)
{
	/*
	 * Raw Bayer 8-bit, and packed raw Bayer 10-bit/12-bit formats
	 * are stored in a GL_LUMINANCE texture. The texture width is
	 * equal to the stride.
	 */
	GLfloat firstRed[] = { firstRed_x_, firstRed_y_ };
	GLfloat imgSize[] = { (GLfloat)width_,
			      (GLfloat)height_ };
	GLfloat Step[] = { static_cast<float>(bytesPerPixel_) / (inputConfig_.stride - 1),
			   1.0f / (height_ - 1) };
	GLfloat Stride = (GLfloat)width_ / (shaderStridePixels_ / bytesPerPixel_);
	/*
	 * Scale the output size from the native size the algorithm produces for
	 * the input size. Keep the aspect ratio and prefer cropping over black
	 * bars.
	 */
	GLfloat scale = std::max((GLfloat)outputSize_.width / nativeOutputSize_.width,
				 (GLfloat)outputSize_.height / nativeOutputSize_.height);
	GLfloat trans = -(1.0f - scale);
	GLfloat projMatrix[] = {
		scale, 0, 0, 0,
		0, scale, 0, 0,
		0, 0, 1, 0,
		trans, trans, 0, 1
	};
	/* vertexIn - bayer_8.vert */
	glEnableVertexAttribArray(attributeVertex_);
	glVertexAttribPointer(attributeVertex_, 2, GL_FLOAT, GL_TRUE,
			      2 * sizeof(GLfloat), kVertexCoordinates);

	/* textureIn - bayer_8.vert */
	glEnableVertexAttribArray(attributeTexture_);
	glVertexAttribPointer(attributeTexture_, 2, GL_FLOAT, GL_TRUE,
			      2 * sizeof(GLfloat), kTextureCoordinates);

	/*
	 * Set the sampler2D to the respective texture unit for each texutre
	 * To simultaneously sample multiple textures we need to use multiple
	 * texture units
	 */
	glUniform1i(textureUniformBayerDataIn_, eglImageIn.texture_unit_uniform_id_);

	/*
	 * These values are:
	 * firstRed = tex_bayer_first_red - bayer_8.vert
	 * imgSize = tex_size - bayer_8.vert
	 * step = tex_step - bayer_8.vert
	 * Stride = stride_factor identity.vert
	 * textureUniformProjMatri = No scaling
	 */
	glUniform2fv(textureUniformBayerFirstRed_, 1, firstRed);
	glUniform2fv(textureUniformSize_, 1, imgSize);
	glUniform2fv(textureUniformStep_, 1, Step);
	glUniform1f(textureUniformStrideFactor_, Stride);
	glUniformMatrix4fv(textureUniformProjMatrix_, 1, GL_FALSE, projMatrix);

	LOG(Debayer, Debug) << "vertexIn " << attributeVertex_ << " textureIn " << attributeTexture_
			    << " tex_y " << textureUniformBayerDataIn_
			    << " tex_step " << textureUniformStep_
			    << " tex_size " << textureUniformSize_
			    << " stride_factor " << textureUniformStrideFactor_
			    << " tex_bayer_first_red " << textureUniformBayerFirstRed_;

	LOG(Debayer, Debug) << "textureUniformY_ = 0 "
			    << " firstRed.x " << firstRed[0]
			    << " firstRed.y " << firstRed[1]
			    << " textureUniformSize_.width " << imgSize[0]
			    << " textureUniformSize_.height " << imgSize[1]
			    << " textureUniformStep_.x " << Step[0]
			    << " textureUniformStep_.y " << Step[1]
			    << " textureUniformStrideFactor_ " << Stride
			    << " textureUniformProjMatrix_ " << textureUniformProjMatrix_;

	glUniformMatrix3fv(ccmUniformDataIn_, 1, GL_TRUE, params.combinedMatrix.data().data());
	LOG(Debayer, Debug) << " ccmUniformDataIn_ " << ccmUniformDataIn_ << " data " << params.combinedMatrix;

	/*
	 * 0 = Red, 1 = Green, 2 = Blue
	 */
	glUniform3f(blackLevelUniformDataIn_, params.blackLevel[0], params.blackLevel[1], params.blackLevel[2]);
	LOG(Debayer, Debug) << " blackLevelUniformDataIn_ " << blackLevelUniformDataIn_ << " data " << params.blackLevel;

	glUniform3f(awbUniformDataIn_, params.gains[0], params.gains[1], params.gains[2]);
	LOG(Debayer, Debug) << " awbUniformDataIn_ " << awbUniformDataIn_ << " data " << params.gains;

	/*
	 * Gamma
	 */
	glUniform1f(gammaUniformDataIn_, params.gamma);
	LOG(Debayer, Debug) << " gammaUniformDataIn_ " << gammaUniformDataIn_ << " data " << params.gamma;

	/*
	 * Contrast
	 */
	glUniform1f(contrastExpUniformDataIn_, params.contrastExp);
	LOG(Debayer, Debug) << " contrastExpUniformDataIn_ " << contrastExpUniformDataIn_ << " data " << params.contrastExp;

	/*
	 * Lens shading correction.  The gains are stored as 8-bit values scaled
	 * by two, which covers the 0.5x..2.0x range that a flat-field
	 * calibration produces; linear filtering of float textures is not
	 * portable on GLES, so the grid is uploaded as integers and the shader
	 * interpolates between the samples.
	 */
	if (params.lscEnabled) {
		std::array<uint8_t, DebayerParams::kLscCols *
					    DebayerParams::kLscRows * 3> lut;

		for (unsigned int i = 0; i < lut.size(); i++) {
			int v = params.lsc[i] * 127.5f + 0.5f;
			lut[i] = static_cast<uint8_t>(std::clamp(v, 0, 255));
		}

		glActiveTexture(GL_TEXTURE2);
		if (!lscTextureId_) {
			glGenTextures(1, &lscTextureId_);
			glBindTexture(GL_TEXTURE_2D, lscTextureId_);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		}

		glBindTexture(GL_TEXTURE_2D, lscTextureId_);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8,
			     DebayerParams::kLscCols, DebayerParams::kLscRows, 0,
			     GL_RGB, GL_UNSIGNED_BYTE, lut.data());
		glUniform1i(lscUniformDataIn_, 2);
		glUniform1f(lscEnabledUniformDataIn_, 1.0f);
		glActiveTexture(GL_TEXTURE0);
	} else {
		glUniform1f(lscEnabledUniformDataIn_, 0.0f);
	}

	return;
}

/**
 * \brief Set up the state for the RGB to YUYV packing pass
 * \param[in] eglImageIn The RGB image produced by the demosaicing pass
 *
 * The packing pass covers the whole output buffer with the identity
 * projection, and samples the RGB image at the centre of each pixel pair.
 */
void DebayerEGL::setYUYVShaderVariableValues(eGLImage &eglImageIn)
{
	GLfloat step[] = { 1.0f / outputSize_.width, 1.0f / outputSize_.height };

	glEnableVertexAttribArray(yuyvAttributeVertex_);
	glVertexAttribPointer(yuyvAttributeVertex_, 2, GL_FLOAT, GL_TRUE,
			      2 * sizeof(GLfloat), kVertexCoordinates);

	glEnableVertexAttribArray(yuyvAttributeTexture_);
	glVertexAttribPointer(yuyvAttributeTexture_, 2, GL_FLOAT, GL_TRUE,
			      2 * sizeof(GLfloat), kTextureCoordinates);

	glActiveTexture(eglImageIn.texture_unit_);
	glBindTexture(GL_TEXTURE_2D, eglImageIn.texture_);
	glUniform1i(yuyvTextureUniformIn_, eglImageIn.texture_unit_uniform_id_);
	glActiveTexture(GL_TEXTURE0);

	glUniform2fv(yuyvTextureUniformStep_, 1, step);
	glUniform1f(yuyvTextureUniformStrideFactor_, 1.0f);
	glUniformMatrix4fv(yuyvTextureUniformProjMatrix_, 1, GL_FALSE,
			   kIdentityMatrix);
}

eGLImage *DebayerEGL::getCachedInputFrameBuffer(FrameBuffer *input, std::optional<MappedFrameBuffer> *inMapped, std::optional<DmaSyncer> *inDmaSyncer)
{
	const SharedFD &fd = input->planes()[0].fd;
	bool cache_miss = true;
	eGLImage *eglImageIn;

	for (auto &[ifd, img] : eglImageInCache_) {
		if (ifd == fd) {
			eglImageIn = img.get();
			cache_miss = false;
			break;
		}
	}

	if (cache_miss) {
		if (eglImageInCache_.size() >= inputBufferCount_) {
			eglImageInCache_.pop_front();
			LOG(Debayer, Debug) << "Input cache " << inputBufferCount_ << " exceeded evicted entry";
		}

		eglImageInCache_.emplace_back(fd, std::make_unique<eGLImage>(glFormat_, inputConfig_.stride / bytesPerPixel_, height_, inputConfig_.stride, GL_TEXTURE0, 0));
		eglImageIn = eglImageInCache_.back().second.get();

		if (egl_.createInputDMABufTexture2D(*eglImageIn, input->planes()[0].fd.get()) == 0)
			return eglImageIn;

	} else if (!eglImageIn->dmabuf_import_failed_) {
		egl_.activateBindTexture(*eglImageIn);
		return eglImageIn;
	}

	/* DMA mode fail create/update an existing texture using the slow path */
	inDmaSyncer->emplace(input->planes()[0].fd, DmaSyncer::SyncType::Read);
	inMapped->emplace(input, MappedFrameBuffer::MapFlag::Read);
	if (!inMapped->value().isValid()) {
		LOG(Debayer, Error) << "mmap-ing buffer(s) failed";
		if (cache_miss) {
			eglImageInCache_.pop_back();
		}
		return nullptr;
	}
	if (cache_miss)
		egl_.createTexture2D(*eglImageIn, inMapped->value().planes()[0].data());
	else
		egl_.updateTexture2D(*eglImageIn, inMapped->value().planes()[0].data());

	return eglImageIn;
}

eGLImage *DebayerEGL::getCachedOutputFrameBuffer(FrameBuffer *output)
{
	const SharedFD &fd = output->planes()[0].fd;
	bool cache_miss = true;
	eGLImage *eglImageOut;

	for (auto &[ifd, img] : eglImageOutCache_) {
		if (ifd == fd) {
			eglImageOut = img.get();
			cache_miss = false;
			break;
		}
	}

	if (cache_miss) {
		if (eglImageOutCache_.size() >= outputBufferCount_) {
			eglImageOutCache_.pop_front();
			LOG(Debayer, Debug) << "Output cache " << outputBufferCount_ << " exceeded evicted entry";
		}

		eglImageOutCache_.emplace_back(fd, std::make_unique<eGLImage>(GL_RGBA, outputImageWidth_,
									      outputSize_.height, outputConfig_.stride, GL_TEXTURE1, 1));
		eglImageOut = eglImageOutCache_.back().second.get();

		if (egl_.createOutputDMABufTexture2D(*eglImageOut, output->planes()[0].fd.get())) {
			eglImageOutCache_.pop_back();
			return nullptr;
		}
	}

	return eglImageOut;
}

int DebayerEGL::debayerGPU(FrameBuffer *input, FrameBuffer *output, const DebayerParams &params, std::optional<MappedFrameBuffer> *inMapped, std::optional<DmaSyncer> *inDmaSyncer)
{
	eGLImage *eglImageIn;
	eGLImage *eglImageOut;

	/* eGL context switch */
	egl_.makeCurrent();

	eglImageIn = getCachedInputFrameBuffer(input, inMapped, inDmaSyncer);
	if (!eglImageIn)
		return -ENOMEM;
	eglImageOut = getCachedOutputFrameBuffer(output);
	if (!eglImageOut)
		return -ENOMEM;

	if (outputIsYUYV_) {
		int ret = rgbToYUYV(*eglImageOut, *eglImageIn, params);
		if (ret)
			return ret;

		egl_.flushOutput();

		return 0;
	}

	egl_.attachTextureToFBO(*eglImageOut);
	setShaderVariableValues(*eglImageIn, params);

	glViewport(0, 0, width_, height_);
	glClear(GL_COLOR_BUFFER_BIT);
	glDrawArrays(GL_TRIANGLE_FAN, 0, DEBAYER_OPENGL_COORDS);

	GLenum err = glGetError();
	if (err != GL_NO_ERROR) {
		LOG(eGL, Error) << "Drawing scene fail " << err;
		return -ENODEV;
	} else {
		egl_.flushOutput();
	}

	return 0;
}

/**
 * \brief Debayer into the RGB intermediate and pack the result into YUYV
 *
 * The demosaicing pass runs exactly as it does for an RGB output, except that
 * it renders into an intermediate RGBA texture. A second, very cheap pass then
 * reads two horizontally adjacent pixels per output texel and writes
 * Y0 U Y1 V into the four bytes of that texel, which is the memory layout of
 * a packed YUYV pixel pair.
 *
 * Splitting the work in two passes keeps the expensive demosaicing shader
 * unchanged, at the cost of one extra full-frame write and read in GPU memory.
 */
int DebayerEGL::rgbToYUYV(eGLImage &eglImageOut, eGLImage &eglImageIn,
			  const DebayerParams &params)
{
	/*
	 * First pass: Bayer to RGB in the intermediate texture. The viewport
	 * is the input size, not the output size: the projection matrix places
	 * the scaled image in the bottom left corner of the viewport, which is
	 * the bottom left corner of the (smaller) output buffer, so that the
	 * rendered image covers exactly the output buffer.
	 */
	egl_.attachTextureToFBO(*rgbIntermediate_);
	egl_.useProgram(programId_);
	setShaderVariableValues(eglImageIn, params);

	glViewport(0, 0, width_, height_);
	glClear(GL_COLOR_BUFFER_BIT);
	glDrawArrays(GL_TRIANGLE_FAN, 0, DEBAYER_OPENGL_COORDS);

	GLenum err = glGetError();
	if (err != GL_NO_ERROR) {
		LOG(eGL, Error) << "Drawing RGB intermediate fail " << err;
		return -ENODEV;
	}

	/* Second pass: pack the RGB image into YUYV */
	egl_.attachTextureToFBO(eglImageOut);
	egl_.useProgram(yuyvProgramId_);
	setYUYVShaderVariableValues(*rgbIntermediate_);

	glViewport(0, 0, outputSize_.width / 2, outputSize_.height);
	glDrawArrays(GL_TRIANGLE_FAN, 0, DEBAYER_OPENGL_COORDS);

	err = glGetError();
	if (err != GL_NO_ERROR) {
		LOG(eGL, Error) << "Drawing YUYV fail " << err;
		return -ENODEV;
	}

	return 0;
}

void DebayerEGL::process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, const DebayerParams &params)
{
	bench_.startFrame();

	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	std::optional<MappedFrameBuffer> inMapped;
	std::optional<DmaSyncer> inDmaSyncer;

	if (debayerGPU(input, output, params, &inMapped, &inDmaSyncer)) {
		LOG(Debayer, Error) << "debayerGPU failed";
		goto error;
	}

	metadata.planes()[0].bytesused = output->planes()[0].length;

	/* Calculate stats for the whole frame */
	if (frame % SwStatsCpu::kStatPerNumFrames) {
		stats_->finishFrame(frame, 0);
	} else {
		if (!inMapped) {
			/*
			 * The buffer was directly imported into EGL and thus
			 * not mapped for texture upload. Do it now for the
			 * CPU-based stats calculation.
			 */
			assert(!inDmaSyncer);
			inDmaSyncer.emplace(input->planes()[0].fd, DmaSyncer::SyncType::Read);
			inMapped.emplace(input, MappedFrameBuffer::MapFlag::Read);
		}
		stats_->processFrame(frame, 0, inMapped.value());
	}
	inDmaSyncer.reset();

	egl_.syncOutput();
	bench_.finishFrame();

	outputBufferReady.emit(output);
	inputBufferReady.emit(input);

	return;

error:
	bench_.finishFrame();
	metadata.status = FrameMetadata::FrameError;
	return;
}

int DebayerEGL::start()
{
	GLint maxTextureImageUnits;

	if (egl_.initEGLContext())
		return -ENODEV;

	glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &maxTextureImageUnits);

	LOG(Debayer, Debug) << "Available fragment shader texture units " << maxTextureImageUnits;

	if (initBayerShaders(inputPixelFormat_, outputPixelFormat_))
		return -EINVAL;

	/*
	 * The intermediate the demosaicing pass renders into when a YUYV
	 * output is requested. It is a plain texture, so unlike the output
	 * buffer it needs no DMA-BUF import and can use the natural stride.
	 */
	if (outputIsYUYV_) {
		rgbIntermediate_ = std::make_unique<eGLImage>(GL_RGBA,
							     outputSize_.width,
							     outputSize_.height,
							     outputSize_.width * 4,
							     GL_TEXTURE0, 0);
		egl_.createOutputTexture2D(*rgbIntermediate_);
	}

	return 0;
}

void DebayerEGL::stop()
{
	eglImageOutCache_.clear();
	eglImageInCache_.clear();
	rgbIntermediate_.reset();

	if (programId_)
		glDeleteProgram(programId_);

	if (yuyvProgramId_)
		glDeleteProgram(yuyvProgramId_);
}

SizeRange DebayerEGL::sizes(PixelFormat inputFormat, const Size &inputSize)
{
	Size patternSize = this->patternSize(inputFormat);
	unsigned int borderHeight = patternSize.height;

	if (patternSize.isNull())
		return {};

	/* No need for top/bottom border with a pattern height of 2 */
	if (patternSize.height == 2)
		borderHeight = 0;

	/*
	 * For debayer interpolation a border is kept around the entire image
	 * and the minimum output size is pattern-height x pattern-width.
	 */
	if (inputSize.width < (3 * patternSize.width) ||
	    inputSize.height < (2 * borderHeight + patternSize.height)) {
		LOG(Debayer, Warning)
			<< "Input format size too small: " << inputSize;
		return {};
	}

	return SizeRange(Size(patternSize.width, patternSize.height),
			 Size((inputSize.width - 2 * patternSize.width) & ~(patternSize.width - 1),
			      (inputSize.height - 2 * borderHeight) & ~(patternSize.height - 1)),
			 patternSize.width, patternSize.height);
}

} /* namespace libcamera */
