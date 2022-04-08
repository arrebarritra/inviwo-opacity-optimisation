/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2019-2021 Inviwo Foundation
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

#include <modules/basegl/shadercomponents/lightcomponent.h>
#include <modules/opengl/shader/shaderutils.h>

#include <string_view>

namespace inviwo {

LightComponent::LightComponent(CameraProperty* camera)
    : ShaderComponent(), lighting_("lighting", "Lighting", camera) {}

std::string_view LightComponent::getName() const { return lighting_.getIdentifier(); }

void LightComponent::initializeResources(Shader& shader) {
    utilgl::addShaderDefines(shader, lighting_);
}

void LightComponent::process(Shader& shader, TextureUnitContainer&) {
    if (lighting_.shadingMode_ != ShadingMode::None) {
        utilgl::setUniforms(shader, lighting_);
    }
}

std::vector<Property*> LightComponent::getProperties() { return {&lighting_}; }

auto LightComponent::getSegments() -> std::vector<Segment> {
    return {{"uniform LightParameters lighting;", placeholder::uniform, 500},
            {R"(#include "utils/shading.glsl")", placeholder::include, 500}};
}

}  // namespace inviwo