/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#pragma once

#include <event_detector/common.hpp>

namespace event_detector_db_recording_plugin {

namespace ed = event_detector;

// clang-format off
template <typename T>
bsoncxx::document::value toBson(const ed::Stamped<T>& s) {
  return bsoncxx::builder::stream::document{}
    << "stamp"  << toBson(s.stamp)  // time
    << "msg"    << toBson(*s.msg)   // ROS message
  << bsoncxx::builder::stream::finalize;
}
// clang-format on

template <typename T>
bsoncxx::builder::stream::array toBson(const T& array) {
  bsoncxx::builder::stream::array bson_array{};
  for (const auto& x : array) bson_array << toBson(x);
  return bson_array;
}

}  // namespace event_detector_db_recording_plugin