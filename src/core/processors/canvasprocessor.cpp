/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2012-2016 Inviwo Foundation
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

#include <inviwo/core/common/inviwoapplication.h>
#include <inviwo/core/processors/canvasprocessor.h>
#include <inviwo/core/common/inviwoapplication.h>
#include <inviwo/core/util/canvas.h>
#include <inviwo/core/util/datetime.h>
#include <inviwo/core/util/stringconversion.h>
#include <inviwo/core/processors/canvasprocessorwidget.h>
#include <inviwo/core/processors/processorwidget.h>
#include <inviwo/core/io/datawriterfactory.h>
#include <inviwo/core/datastructures/image/layer.h>
#include <inviwo/core/util/filesystem.h>
#include <inviwo/core/util/fileextension.h>
#include <inviwo/core/util/filedialog.h>
#include <inviwo/core/util/dialogfactory.h>

namespace inviwo {

CanvasProcessor::CanvasProcessor()
    : Processor()
    , inport_("inport")
    , dimensions_("dimensions", "Canvas Size", ivec2(256, 256), ivec2(128, 128), ivec2(4096, 4096),
                  ivec2(1, 1), InvalidationLevel::Valid)
    , enableCustomInputDimensions_("enableCustomInputDimensions", "Separate Image Size", false,
                                   InvalidationLevel::Valid)
    , customInputDimensions_("customInputDimensions", "Image Size", ivec2(256, 256),
                             ivec2(128, 128), ivec2(4096, 4096), ivec2(1, 1),
                             InvalidationLevel::Valid)
    , keepAspectRatio_("keepAspectRatio", "Lock Aspect Ratio", true, InvalidationLevel::Valid)
    , aspectRatioScaling_("aspectRatioScaling", "Image Scale", 1.f, 0.1f, 4.f, 0.01f,
                          InvalidationLevel::Valid)
    , position_("position", "Canvas Position", ivec2(128, 128), ivec2(std::numeric_limits<int>::lowest() ), ivec2(std::numeric_limits<int>::max()),
        ivec2(1, 1), InvalidationLevel::Valid,PropertySemantics::Text)
    , visibleLayer_("visibleLayer", "Visible Layer")
    , colorLayer_("colorLayer_", "Color Layer ID", 0, 0, 0)
    , imageTypeExt_("fileExt", "Image Type")
    , saveLayerDirectory_("layerDir", "Output Directory", "", "image")
    , saveLayerButton_("saveLayer", "Save Image Layer", InvalidationLevel::Valid)
    , saveLayerToFileButton_("saveLayerToFile", "Save Image Layer to File...", InvalidationLevel::Valid)
    , inputSize_("inputSize", "Input Dimension Parameters")
    , toggleFullscreen_("toggleFullscreen", "Toggle Full Screen")
    , fullscreen_("fullscreen", "FullScreen",
                  [this](Event* event) { setFullScreen(!isFullScreen()); }, IvwKey::F,
                  KeyState::Press, KeyModifier::Shift)
    , previousImageSize_(customInputDimensions_)
    , widgetMetaData_{createMetaData<ProcessorWidgetMetaData>(
          ProcessorWidgetMetaData::CLASS_IDENTIFIER)}
    , canvasWidget_(nullptr)
    , queuedRequest_(false) {
    widgetMetaData_->addObserver(this);
    
    addPort(inport_);
    addProperty(inputSize_);
    
    dimensions_.setSerializationMode(PropertySerializationMode::None);
    dimensions_.onChange([this](){widgetMetaData_->setDimensions(dimensions_.get());});
    inputSize_.addProperty(dimensions_);

    enableCustomInputDimensions_.onChange(this, &CanvasProcessor::sizeChanged);
    inputSize_.addProperty(enableCustomInputDimensions_);

    customInputDimensions_.onChange(this, &CanvasProcessor::sizeChanged);
    customInputDimensions_.setVisible(false);
    inputSize_.addProperty(customInputDimensions_);

    keepAspectRatio_.onChange(this, &CanvasProcessor::sizeChanged);
    keepAspectRatio_.setVisible(false);
    inputSize_.addProperty(keepAspectRatio_);

    aspectRatioScaling_.onChange(this, &CanvasProcessor::sizeChanged);
    aspectRatioScaling_.setVisible(false);
    inputSize_.addProperty(aspectRatioScaling_);

    position_.onChange([this]() {widgetMetaData_->setPosition(position_.get()); });
    addProperty(position_);

    visibleLayer_.addOption("color", "Color layer", LayerType::Color);
    visibleLayer_.addOption("depth", "Depth layer", LayerType::Depth);
    visibleLayer_.addOption("picking", "Picking layer", LayerType::Picking);
    visibleLayer_.set(LayerType::Color);

    // add all supported image extensions to option property
    auto wf = InviwoApplication::getPtr()->getDataWriterFactory();
    // save first writer extension matching "png" to be used as default
    std::string defaultExt;
    for (auto ext : wf->getExtensionsForType<Layer>()) {
        imageTypeExt_.addOption(ext.toString(), ext.toString());
        if (defaultExt.empty() && ext.extension_ == "png") {
            defaultExt = ext.toString();
        }
    }
    if (!defaultExt.empty()) {
        imageTypeExt_.setSelectedIdentifier(defaultExt);
    }

    addProperty(visibleLayer_);
    addProperty(colorLayer_);
    addProperty(saveLayerDirectory_);
    addProperty(imageTypeExt_);

    saveLayerButton_.onChange(this, &CanvasProcessor::saveImageLayer);
    addProperty(saveLayerButton_);

    saveLayerToFileButton_.onChange([this]() {
        auto fileDialog = util::dynamic_unique_ptr_cast<FileDialog>(
            InviwoApplication::getPtr()->getDialogFactory()->create("FileDialog"));
        if (!fileDialog) {
            // no file dialog found, disable button
            saveLayerToFileButton_.setReadOnly(true);
            return;
        }
        fileDialog->setTitle("Save Layer to File...");
        fileDialog->setAcceptMode(AcceptMode::Save);
        fileDialog->setFileMode(FileMode::AnyFile);

        auto writerFactory = InviwoApplication::getPtr()->getDataWriterFactory();
        fileDialog->addExtensions(writerFactory->getExtensionsForType<Layer>());

        if (fileDialog->show()) {
            saveImageLayer(fileDialog->getSelectedFile(), fileDialog->getSelectedFileExtension());
        }
    });
    addProperty(saveLayerToFileButton_);

    colorLayer_.setSerializationMode(PropertySerializationMode::All);
    colorLayer_.setVisible(false);

    visibleLayer_.onChange([&]() {
        if (inport_.hasData()) {
            auto layers = inport_.getData()->getNumberOfColorLayers();
            colorLayer_.setVisible(layers > 1 && visibleLayer_.get() == LayerType::Color);
        }
    });

    toggleFullscreen_.onChange([this]() { setFullScreen(!isFullScreen()); });

    addProperty(toggleFullscreen_);
    addProperty(fullscreen_);

    inport_.onChange([&]() {
        int layers = static_cast<int>(inport_.getData()->getNumberOfColorLayers());
        colorLayer_.setVisible(layers > 1 && visibleLayer_.get() == LayerType::Color);
        colorLayer_.setMaxValue(layers - 1);
    });

    inport_.onConnect([&](){
       sizeChanged();
    });

    setAllPropertiesCurrentStateAsDefault();
}

CanvasProcessor::~CanvasProcessor() {
    if (processorWidget_) {
        processorWidget_->hide();
        canvasWidget_->getCanvas()->setEventPropagator(nullptr);
    }
}

void CanvasProcessor::setProcessorWidget(std::unique_ptr<ProcessorWidget> processorWidget) {
    if (auto cw = dynamic_cast<CanvasProcessorWidget*>(processorWidget.get())) {
        canvasWidget_ = cw;
    }
    Processor::setProcessorWidget(std::move(processorWidget));
}

void CanvasProcessor::onProcessorWidgetPositionChange(ProcessorWidgetMetaData*) {
    if (widgetMetaData_->getPosition() != position_.get()) {
        Property::OnChangeBlocker blocker{ position_ };
        position_.set(widgetMetaData_->getPosition());
    }
}

void CanvasProcessor::onProcessorWidgetDimensionChange(ProcessorWidgetMetaData*) {
    if (widgetMetaData_->getDimensions() != dimensions_.get()) {
        Property::OnChangeBlocker blocker{dimensions_};
        dimensions_.set(widgetMetaData_->getDimensions());
    }
}

void CanvasProcessor::setCanvasSize(ivec2 dim) {
    NetworkLock lock(this);
    dimensions_.set(dim);
    sizeChanged();
}

ivec2 CanvasProcessor::getCanvasSize() const { return dimensions_.get(); }

bool CanvasProcessor::getUseCustomDimensions() const { return enableCustomInputDimensions_; }
ivec2 CanvasProcessor::getCustomDimensions() const { return customInputDimensions_; }

void CanvasProcessor::sizeChanged() {
    NetworkLock lock(this);

    customInputDimensions_.setVisible(enableCustomInputDimensions_);
    customInputDimensions_.setReadOnly(keepAspectRatio_);
    keepAspectRatio_.setVisible(enableCustomInputDimensions_);
    aspectRatioScaling_.setVisible(enableCustomInputDimensions_ && keepAspectRatio_);

    if (keepAspectRatio_) customInputDimensions_.get() = calcSize(); // avoid triggering on change
    ResizeEvent resizeEvent(uvec2(0));
    if (enableCustomInputDimensions_) {
        resizeEvent.setSize(static_cast<uvec2>(customInputDimensions_.get()));
        resizeEvent.setPreviousSize(static_cast<uvec2>(previousImageSize_));
        previousImageSize_ = customInputDimensions_;
    } else {
        resizeEvent.setSize(static_cast<uvec2>(dimensions_.get()));
        resizeEvent.setPreviousSize(static_cast<uvec2>(previousImageSize_));
        previousImageSize_ = dimensions_;
    }

    inputSize_.invalidate(InvalidationLevel::Valid, &customInputDimensions_);
    inport_.propagateEvent(&resizeEvent);
}

ivec2 CanvasProcessor::calcSize() {
    ivec2 size = dimensions_;

    int maxDim, minDim;

    if (size.x >= size.y) {
        maxDim = 0;
        minDim = 1;
    } else {
        maxDim = 1;
        minDim = 0;
    }

    float ratio = static_cast<float>(size[minDim]) / static_cast<float>(size[maxDim]);
    size[maxDim] = static_cast<int>(static_cast<float>(size[maxDim]) * aspectRatioScaling_);
    size[minDim] = static_cast<int>(static_cast<float>(size[maxDim]) * ratio);

    return size;
}

void CanvasProcessor::saveImageLayer() {
    if (saveLayerDirectory_.get().empty()) saveLayerDirectory_.requestFile();

    auto ext = FileExtension::createFileExtensionFromString(imageTypeExt_.get());
    std::string snapshotPath(saveLayerDirectory_.get() + "/" + toLower(getIdentifier()) + "-" +
                             currentDateTime() + "." + ext.extension_);
    saveImageLayer(snapshotPath, ext);
}

void CanvasProcessor::saveImageLayer(std::string snapshotPath, const FileExtension &extension) {
    if (auto layer = getVisibleLayer()) {
        auto writer = std::shared_ptr<DataWriterType<Layer>>(
            InviwoApplication::getPtr()
            ->getDataWriterFactory()
            ->getWriterForTypeAndExtension<Layer>(extension));

        if (!writer) {
            // could not find a reader for the given extension, extension might be invalid
            // try to get reader for the extension extracted from the file name, i.e. snapshotPath
            const auto ext = filesystem::getFileExtension(snapshotPath);
            writer = std::shared_ptr<DataWriterType<Layer>>(
                InviwoApplication::getPtr()
                ->getDataWriterFactory()
                ->getWriterForTypeAndExtension<Layer>(ext));
            if (!writer) {
                LogError("Could not find a writer for the specified file extension (\""
                         << ext << "\")");
                return;
            }
        }

        try {
            writer->setOverwrite(true);
            writer->writeData(layer, snapshotPath);
            LogInfo("Canvas layer exported to disk: " << snapshotPath);
        }
        catch (DataWriterException const& e) {
            LogError(e.getMessage());
        }
    }
    else {
        LogError("Could not find visible layer");
    }
}

const Layer* CanvasProcessor::getVisibleLayer() const {
    if (auto image = inport_.getData()) {
        if (visibleLayer_.get() == LayerType::Color) {
            return image->getColorLayer(colorLayer_.get());
        } else {
            return image->getLayer(visibleLayer_.get());
        }
    } else {
        return nullptr;
    }
}

std::shared_ptr<const Image> CanvasProcessor::getImage() const {
    if (inport_.hasData()) {
        return inport_.getData();
    }
    else {
        return nullptr;
    }
}

void CanvasProcessor::process() {
    if (canvasWidget_ && canvasWidget_->getCanvas()) {
        LayerType layerType = visibleLayer_.get();
        if (visibleLayer_.get() == LayerType::Color) {
            canvasWidget_->getCanvas()->render(inport_.getData(), LayerType::Color,
                                               colorLayer_.get());
        } else {
            canvasWidget_->getCanvas()->render(inport_.getData(), layerType, 0);
        }
    }
}

void CanvasProcessor::doIfNotReady() {
    if (canvasWidget_ && canvasWidget_->getCanvas()) {
        canvasWidget_->getCanvas()->render(nullptr, visibleLayer_.get());
    }
}

void CanvasProcessor::triggerQueuedEvaluation() {
    if (queuedRequest_) {
        performEvaluateRequest();
        queuedRequest_ = false;
    }
}

void CanvasProcessor::performEvaluationAtNextShow() { queuedRequest_ = true; }

void CanvasProcessor::performEvaluateRequest() {
    if (processorWidget_) {
        if (processorWidget_->isVisible()) {
            notifyObserversRequestEvaluate(this);
        } else {
            performEvaluationAtNextShow();
        }
    } else {
        notifyObserversRequestEvaluate(this);
    }
}

bool CanvasProcessor::isReady() const {
    return Processor::isReady() && processorWidget_ && processorWidget_->isVisible();
}

void CanvasProcessor::propagateEvent(Event* event, Outport* source) {
    if (event->hasVisitedProcessor(this)) return;
    event->markAsVisited(this);

    invokeEvent(event);
    if (event->hasBeenUsed()) return;

    if (event->hash() == ResizeEvent::chash()) {
        auto resizeEvent = static_cast<ResizeEvent*>(event);

        // Avoid continues evaluation when port dimensions changes
        NetworkLock lock(this);
        dimensions_.set(resizeEvent->size());
        if (enableCustomInputDimensions_) {
            sizeChanged();
        } else {
            inport_.propagateEvent(resizeEvent, nullptr);
            // Make sure this processor is invalidated.
            invalidate(InvalidationLevel::InvalidOutput);
        }
    } else {
        bool used = event->hasBeenUsed();
        for (auto inport : getInports()) {
            if (event->shouldPropagateTo(inport, this, source)) {
                inport->propagateEvent(event);
                used |= event->hasBeenUsed();
                event->markAsUnused();
            }
        }
        if (used) event->markAsUsed();
    }
}

bool CanvasProcessor::isFullScreen() const {
    if(canvasWidget_) {
        return canvasWidget_->getCanvas()->isFullScreen();
    }
    return false;
}

void CanvasProcessor::setFullScreen(bool fullscreen) {
    if (canvasWidget_) {
        return canvasWidget_->getCanvas()->setFullScreen(fullscreen);
    }
}

}  // namespace
