#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <string>
#include <sstream>
#include <chrono>
#include <QtGui/QQuaternion>
#include <QtGui/QGenericMatrix>

#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonDocument>

#include <iostream>
#include <iomanip>

#include <cassert>

using FrameID = std::string;
using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;
using DurationSec = std::chrono::seconds;
using DurationMilliSec = std::chrono::milliseconds;
using DurationNanoSec = std::chrono::nanoseconds;

#endif // TYPEDEFS_H
