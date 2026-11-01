/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * rgb_to_yuyv.frag - Pack an RGB image into YUYV 4:2:2
 *
 * There is no renderable YUYV format in OpenGL ES, so the output buffer is
 * imported as a 32-bit RGBA image in which one texel holds two YUYV pixels:
 *
 *   byte 0  byte 1  byte 2  byte 3
 *     Y0      U       Y1      V
 *
 * which is exactly the memory layout of a packed YUYV pixel pair. Every
 * fragment thus reads two horizontally adjacent RGB pixels from the debayered
 * image, converts both to YCbCr and averages the chroma.
 */

#ifdef GL_ES
precision highp float;
#endif

varying vec2 textureOut;

uniform sampler2D tex_rgb;
uniform vec2 tex_step;

/*
 * BT.601 with studio swing levels, which is what V4L2 devices report as
 * "default" encoding and quantisation for packed 4:2:2.
 */
vec3 rgb_to_yuv(vec3 rgb)
{
	float y = dot(rgb, vec3( 0.2568,  0.5041,  0.0979)) + 16.0 / 255.0;
	float u = dot(rgb, vec3(-0.1482, -0.2910,  0.4392)) + 0.5;
	float v = dot(rgb, vec3( 0.4392, -0.3678, -0.0714)) + 0.5;

	return vec3(y, u, v);
}

void main(void)
{
	/* Step from the centre of a pixel pair onto either pixel */
	vec2 offset = vec2(tex_step.x * 0.5, 0.0);

	vec3 yuv0 = rgb_to_yuv(texture2D(tex_rgb, textureOut - offset).rgb);
	vec3 yuv1 = rgb_to_yuv(texture2D(tex_rgb, textureOut + offset).rgb);

	gl_FragColor = vec4(yuv0.x, (yuv0.y + yuv1.y) * 0.5,
			    yuv1.x, (yuv0.z + yuv1.z) * 0.5);
}
