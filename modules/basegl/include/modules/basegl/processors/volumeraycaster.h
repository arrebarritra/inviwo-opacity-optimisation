/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2012-2022 Inviwo Foundation
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

#include <modules/basegl/baseglmoduledefine.h>
#include <inviwo/core/processors/poolprocessor.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/isotfproperty.h>
#include <inviwo/core/properties/raycastingproperty.h>
#include <inviwo/core/properties/simplelightingproperty.h>
#include <inviwo/core/properties/cameraproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/eventproperty.h>
#include <inviwo/core/properties/volumeindicatorproperty.h>
#include <inviwo/core/ports/imageport.h>
#include <inviwo/core/ports/volumeport.h>
#include <modules/opengl/shader/shader.h>

namespace inviwo {

/** \docpage{org.inviwo.VolumeRaycaster, Volume Raycaster}
 * ![](org.inviwo.VolumeRaycaster.png?classIdentifier=org.inviwo.VolumeRaycaster)
 * Processor for visualizing volumetric data by means of volume raycasting. Besides the
 * volume data, entry and exit point locations of the bounding box are required. These
 * can be created with the EntryExitPoints processor. The camera properties between these
 * two processors need to be linked.
 *
 * ### Inports
 *   * __volume__ input volume
 *   * __entry__  entry point locations of input volume (image generated by EntryExitPoints
 * processor)
 *   * __exit__   exit point positions of input volume (image generated by EntryExitPoints
 * processor)
 *   * __bg__     optional background image. The depth channel is used to terminated the raycasting.
 *
 * ### Outports
 *   * __outport__ output image containing volume rendering of the input
 *
 * ### Properties (from VolumeRaycaster)
 *   * __Render Channel__    selects which channel of the input volume is rendered
 *   * __Raycasting__        raycasting parameters including rendering type (DVR / isosurfaces),
 *                           compositing, sampling rate, etc.
 *   * __Transfer function__ property for both transfer function and isovalues
 *   * __Lighting__          lighting properties
 *   * __Position Indicator__  position indicator properties
 *   * __Camera__            camera properties (to be linked with EntryExitPoints processor)
 *   * __Toggle Shading__    boolean flag for enabling/disabling shading
 *
 */
class IVW_MODULE_BASEGL_API VolumeRaycaster : public PoolProcessor {
public:
    VolumeRaycaster();
    virtual ~VolumeRaycaster() = default;

    virtual void initializeResources() override;

    // override to do member renaming.
    virtual void deserialize(Deserializer& d) override;
    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

protected:
    virtual void process() override;
    void raycast(const Volume& volume);

    void toggleShading(Event*);

    Shader shader_;
    VolumeInport volumePort_;
    ImageInport entryPort_;
    ImageInport exitPort_;
    ImageInport backgroundPort_;
    ImageOutport outport_;

    OptionPropertyInt channel_;
    RaycastingProperty raycasting_;
    IsoTFProperty isotfComposite_;

    CameraProperty camera_;
    SimpleLightingProperty lighting_;
    VolumeIndicatorProperty positionIndicator_;
    EventProperty toggleShading_;
};

}  // namespace inviwo
