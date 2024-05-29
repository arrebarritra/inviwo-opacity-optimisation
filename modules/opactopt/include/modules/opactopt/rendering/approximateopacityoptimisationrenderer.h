/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2024 Inviwo Foundation
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
#pragma once

#include <modules/opactopt/opactoptmoduledefine.h>

#include <modules/opactopt/rendering/opacityoptimisationrenderer.h>
#include <modules/opactopt/rendering/approximation.h>

#include <modules/opengl/texture/texture2darray.h>
#include <modules/opengl/buffer/bufferobject.h>

namespace inviwo {

/**
 * \brief VERY_BRIEFLY_DESCRIBE_THE_CLASS
 * DESCRIBE_THE_CLASS_FROM_A_DEVELOPER_PERSPECTIVE
 */
class IVW_MODULE_OPACTOPT_API ApproximateOpacityOptimisationRenderer
    : public OpacityOptimisationRenderer {
public:
    ApproximateOpacityOptimisationRenderer(const Approximations::ApproximationProperties* p,
                                           CameraProperty* c, int isc, int odc,
                                           int gaussianRadius = 3, float sigma = 1.0f);

    virtual void prePass(const size2_t& screenSize) override;
    virtual bool postPass(bool useIllustration, const Image* background) override;

    void setDescriptor(const Approximations::ApproximationProperties* const p);
    void setImportanceSumCoeffs(int isc);
    void setOpticalDepthCoeffs(int odc);
    void generateAndUploadGaussianKernel(int radius, float sigma, bool force = false);

    bool smoothing;

private:
    virtual void buildShaders(bool hasBackground = false) override;
    virtual void setUniforms(Shader& shader, const TextureUnit& abuffUnit) const override;
    virtual void resizeBuffers(const size2_t& screenSize) override;
    virtual void process();
    virtual void render(const Image* background);

    Shader project_, smoothH_, smoothV_, blend_, clearaoo_;

    const Approximations::ApproximationProperties* ap_;
    int nImportanceSumCoefficients_;
    int nOpticalDepthCoefficients_;
    Texture2DArray importanceSumCoeffs_[2];
    Texture2DArray opticalDepthCoeffs_;

    TextureUnit *abuffUnit_, *importanceSumUnitMain_, *importanceSumUnitSmooth_, *opticalDepthUnit_;
    BufferObject gaussianKernel_;
    int gaussianRadius_;
    float gaussianSigma_;
};

}  // namespace inviwo
