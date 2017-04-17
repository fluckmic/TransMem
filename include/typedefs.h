#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <string>
#include <chrono>
#include <QtGui/QQuaternion>

using FrameID = std::string;
using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;
using Duration = std::chrono::seconds;

/**
 * @brief The TransEntry struct
 */
struct TransEntry {
    QQuaternion rotation;
    QQuaternion translation;
    Timestamp time;
};

#endif // TYPEDEFS_H
