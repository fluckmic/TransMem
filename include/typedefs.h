#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <string>
#include <sstream>
#include <chrono>
#include <QtGui/QQuaternion>

#include <iostream>
#include <iomanip>

// NOTE: just  for developing needed
#include <cassert>

using FrameID = std::string;
using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;
using DurationSec = std::chrono::seconds;
using DurationNanoSec = std::chrono::nanoseconds;

#endif // TYPEDEFS_H
