/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2019-2024 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#ifdef COEFF_TEX_FIXED_POINT_FACTOR
void project(layout(r32i) iimage2DArray coeffTex, int N, float depth, float val)
#else
void project(layout(size1x32) image2DArray coeffTex, int N, float depth, float val)
#endif
{
    if (depth < 0.0 || depth > 1.0) return;
    int bin = min(int(depth * N), N - 1);
    // pre integrate
    for (int i = bin; i <= N - 1; i++) {
        ivec3 coord = ivec3(gl_FragCoord.xy, i);
        #if defined(COEFF_TEX_FIXED_POINT_FACTOR)
            imageAtomicAdd(coeffTex, coord, int(val * COEFF_TEX_FIXED_POINT_FACTOR));
        #elif defined(COEFF_TEX_ATOMIC_FLOAT)
            imageAtomicAdd(coeffTex, coord, val);
        #else
            float currVal = imageLoad(coeffTex, coord).x;
            imageStore(coeffTex, coord, vec4(currVal + val));
        #endif
    }
}

#ifdef COEFF_TEX_FIXED_POINT_FACTOR
float approximate(layout(r32i) iimage2DArray coeffTex, int N, float depth)
#else
float approximate(layout(size1x32) image2DArray coeffTex, int N, float depth)
#endif
{
    int bin = min(int(depth * N), N - 1);
    ivec3 coord = ivec3(gl_FragCoord.xy, bin);
    ivec3 prevbincoord = ivec3(gl_FragCoord.xy, bin - 1);
    #ifdef COEFF_TEX_FIXED_POINT_FACTOR
        float binsum = float(imageLoad(coeffTex, coord).x) / COEFF_TEX_FIXED_POINT_FACTOR;
        float prevbinsum = 0.0;
        if (bin > 0)
            prevbinsum = float(imageLoad(coeffTex, prevbincoord).x) / COEFF_TEX_FIXED_POINT_FACTOR;
    #else
        float binsum = imageLoad(coeffTex, coord).x;
        float prevbinsum = 0.0;
        if (bin > 0)
            prevbinsum = imageLoad(coeffTex, prevbincoord).x;
    #endif
    return prevbinsum + fract(depth * N) * (binsum - prevbinsum);
}

#ifdef COEFF_TEX_FIXED_POINT_FACTOR
float total(layout(r32i) iimage2DArray coeffTex, int N)
#else
float total(layout(size1x32) image2DArray coeffTex, int N)
#endif
{
    #ifdef COEFF_TEX_FIXED_POINT_FACTOR
        return float(imageLoad(coeffTex, ivec3(gl_FragCoord.xy, N - 1)).x) / COEFF_TEX_FIXED_POINT_FACTOR;
    #else
        return imageLoad(coeffTex, ivec3(gl_FragCoord.xy, N - 1)).x;
    #endif
}