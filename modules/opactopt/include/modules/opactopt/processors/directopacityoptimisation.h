/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2013-2024 Inviwo Foundation
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

#include <modules/opactopt/rendering/approximation.h>
#include <modules/opactopt/utils/graphicsexectimer.h>
#include <modules/opengl/texture/texture2darray.h>
#include <modules/opengl/texture/textureunit.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/buffer/bufferobject.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/core/properties/fileproperty.h>
#include <modules/opengl/volume/volumeutils.h>
#include <modules/opengl/openglcapabilities.h>

#include <inviwo/core/interaction/cameratrackball.h>  // for CameraTrackball
#include <inviwo/core/ports/imageport.h>              // for ImageInport, ImageOutport
#include <inviwo/core/ports/meshport.h>               // for MeshFlatMultiInport
#include <inviwo/core/processors/processor.h>         // for Processor
#include <inviwo/core/processors/processorinfo.h>     // for ProcessorInfo
#include <inviwo/core/properties/boolproperty.h>      // for BoolProperty
#include <inviwo/core/properties/buttonproperty.h>
#include <inviwo/core/properties/cameraproperty.h>           // for CameraProperty
#include <inviwo/core/properties/compositeproperty.h>        // for CompositeProperty
#include <inviwo/core/properties/optionproperty.h>           // for OptionPropertyInt
#include <inviwo/core/properties/ordinalproperty.h>          // for FloatVec4Property
#include <inviwo/core/properties/boolcompositeproperty.h>    // for BoolCompositeProperty
#include <inviwo/core/properties/simplelightingproperty.h>   // for SimpleLightingProperty
#include <modules/opengl/shader/shader.h>                    // for Shader
#include <modules/basegl/properties/linesettingsproperty.h>  // for LineSettingsProperty

namespace inviwo {

class IVW_MODULE_OPACTOPT_API DirectOpacityOptimisation : public Processor {
public:
    DirectOpacityOptimisation();

    DirectOpacityOptimisation(const DirectOpacityOptimisation&) = delete;
    DirectOpacityOptimisation& operator=(const DirectOpacityOptimisation&) = delete;

    virtual ~DirectOpacityOptimisation();

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    virtual void initializeResources() override;
    virtual void process() override;

protected:
    void setUniforms(Shader& shader);
    void renderGeometry(int pass);
    void resizeBuffers(size2_t screenSize);
    void generateAndUploadGaussianKernel();
    void generateAndUploadLegendreCoefficients();

    size2_t screenSize_;

    MeshFlatMultiInport inport_;
    ImageInport imageInport_;
    ImageOutport outport_;
    Image intermediateImage_;

    CameraProperty camera_;

    // Mesh properties
    CompositeProperty meshProperties_;
    BoolProperty overrideColorBuffer_;
    FloatVec4Property overrideColor_;

    // Line properties
    LineSettingsProperty lineSettings_;

    // Point properties
    CompositeProperty pointProperties_;
    FloatProperty pointSize_;
    FloatProperty borderWidth_;
    FloatVec4Property borderColor_;
    FloatProperty antialising_;

    // General properties
    SimpleLightingProperty lightingProperty_;
    CameraTrackball trackball_;

    CompositeProperty layers_;
    BoolProperty colorLayer_;
    BoolProperty texCoordLayer_;
    BoolProperty normalsLayer_;
    BoolProperty viewNormalsLayer_;

    // Shaders for each rendering pass and primitive type
    Shader meshShaders_[3];
    Shader lineShaders_[3];
    Shader lineAdjacencyShaders_[3];
    Shader pointShaders_[3];

    // Screen space shaders
    Shader smoothH_, smoothV_, clear_, normalise_;

    // Optional importance volume
    VolumeInport importanceVolume_;

    // Opacity optimisation settings
    FloatProperty q_, r_, lambda_;
    CompositeProperty approximationProperties_;
    OptionProperty<std::string> approximationMethod_;
    const Approximations::ApproximationProperties* ap_;
    IntProperty importanceSumCoefficients_, opticalDepthCoefficients_;
    BoolProperty normalisedBlending_;
    FloatProperty coeffTexFixedPointFactor_;

    TextureUnitContainer textureUnits_;
    GLFormat imageFormat_ = GLFormats::getGLFormat(
        OpenGLCapabilities::isExtensionSupported("GL_NV_shader_atomic_float") ? GL_FLOAT
                                                                              : GL_INT,
        1);
    Texture2DArray importanceSumTexture_[2];
    Texture2DArray opticalDepthTexture_;
    TextureUnit *importanceSumUnitMain_, *importanceSumUnitSmooth_, *opticalDepthUnit_;

    BufferObject gaussianKernel_;
    BoolCompositeProperty smoothing_;
    IntProperty gaussianRadius_;
    FloatProperty gaussianSigma_;

    BufferObject legendreCoefficients_;
    bool legendreCoefficientsGenerated_ = false;

    Approximations::MomentSettings ms;

    IntVec2Property debugCoords_;
    FileProperty debugFileName_;
    ButtonProperty debugApproximation_;

    util::GraphicsExecTimer execTimer;
    IntProperty timingMode_;  // 0 off, 1 total, 2 per pass
    Int64Property timeTotal_, timeSetup_, timeProjection_, timeSmoothing_, timeImportanceApprox_,
        timeBlending_, timeNormalisation_;

    bool debugOn_;
    Approximations::DebugBuffer db_;
};

}  // namespace inviwo