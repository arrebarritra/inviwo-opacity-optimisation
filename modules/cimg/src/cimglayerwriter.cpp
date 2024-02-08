/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2015-2024 Inviwo Foundation
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

#include <modules/cimg/cimglayerwriter.h>

#include <inviwo/core/datastructures/image/layer.h>  // for Layer (ptr only), DataWriterType
#include <inviwo/core/datastructures/image/layerram.h>
#include <inviwo/core/io/datawriter.h>       // for DataWriterType
#include <inviwo/core/util/fileextension.h>  // for FileExtension
#include <modules/cimg/cimgutils.h>          // for saveLayer, saveLayerToBuffer

namespace inviwo {

CImgLayerWriter::CImgLayerWriter() : DataWriterType<Layer>() {
#ifdef cimg_use_jpeg
    addExtension(FileExtension("jpg", "Joint Photographic Experts Group"));
    addExtension(FileExtension("jpeg", "Joint Photographic Experts Group"));
#endif
#ifdef cimg_use_tiff
    addExtension(FileExtension("tif", "Tagged Image File Format"));
    addExtension(FileExtension("tiff", "Tagged Image File Format"));
#endif
    addExtension(FileExtension("bmp", "Windows bitmap"));
#ifdef cimg_use_openexr
    addExtension(FileExtension("exr", "OpenEXR"));
#endif
    addExtension(FileExtension("hdr", "Analyze 7.5"));
    addExtension(FileExtension("raw", "RAW"));
}

CImgLayerWriter* CImgLayerWriter::clone() const { return new CImgLayerWriter(*this); }

void CImgLayerWriter::writeData(const Layer* data, const std::filesystem::path& filePath) const {
    if (auto ram = data->getRepresentation<LayerRAM>()) {
        cimgutil::saveLayer(*ram, filePath);
    }
}

std::unique_ptr<std::vector<unsigned char>> CImgLayerWriter::writeDataToBuffer(
    const Layer* data, const std::string_view fileExtension) const {
    auto buffer = std::make_unique<std::vector<unsigned char>>();
    if (auto ram = data->getRepresentation<LayerRAM>()) {
        cimgutil::saveLayer(*ram, *buffer, fileExtension);
    }
    return buffer;
}

}  // namespace inviwo
