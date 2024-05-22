/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2023 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef FILTER_SRC_IIR_H_  // NOLINT
#define FILTER_SRC_IIR_H_

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cmath>
#include <cstddef>
#include <cstdint>
#endif

namespace bfs {

class Iir {
 public:
  void Init(const float cutoff_hz, const float samp_hz);
  void Init(const float cutoff_hz, const float samp_hz,
            const float initial_val);
  float Filter(const float val);

 private:
  float a_, b_;
  float fc_, ret_;
  float prev_output_ = 0.0f;
};

}  // namespace bfs

#endif  // FILTER_SRC_IIR_H_ NOLINT
