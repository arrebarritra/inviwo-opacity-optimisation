/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2013-2022 Inviwo Foundation
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

#include <inviwo/core/util/clock.h>

namespace inviwo {

Clock::Clock() { start(); }

bool Clock::isRunning() const { return isRunning_; }

void Clock::start() {
    startTime_ = clock::now();
    isRunning_ = true;
}

void Clock::stop() {
    auto currentTime = clock::now();

    if (isRunning_) {
        accumulatedTime_ += currentTime - startTime_;
        isRunning_ = false;
    }
}

void Clock::reset() { accumulatedTime_ = static_cast<duration>(0); }

auto Clock::getElapsedTime() const -> duration {
    auto currentTime = clock::now();

    if (isRunning_) {
        return accumulatedTime_ + currentTime - startTime_;
    } else {
        return accumulatedTime_;
    }
}

double Clock::getElapsedMilliseconds() const {
    using duration_double = std::chrono::duration<double, std::milli>;
    return std::chrono::duration_cast<duration_double>(getElapsedTime()).count();
}

double Clock::getElapsedSeconds() const {
    using duration_double = std::chrono::duration<double, std::ratio<1>>;
    return std::chrono::duration_cast<duration_double>(getElapsedTime()).count();
}

}  // namespace inviwo