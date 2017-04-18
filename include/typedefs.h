#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <string>
#include <chrono>
#include <QtGui/QQuaternion>

#include <iostream>
#include <iomanip>

#include <cassert>

using FrameID = std::string;
using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;
using DurationSec = std::chrono::seconds;
using DurationNanoSec = std::chrono::nanoseconds;

/**
 * @brief The TransEntry struct
 */
struct TransEntry {
    Timestamp time;
    QQuaternion rotation;
    QQuaternion translation;
};

/**
 * @brief operator <<
 * @param os
 * @param te
 * @return
 */
inline std::ostream &operator<<(std::ostream &os, const TransEntry &te){

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(te.time.time_since_epoch());
    std::time_t yet = std::chrono::system_clock::to_time_t(te.time);

    std::cout << "\n++++++++++++++++++++++++++ \n";
    std::cout << "  timestamp:  " << std::put_time(std::localtime(&yet), "%T") << ":" << ms.count() % 1000 << "\n";
    std::cout << "   rotation: (" << te.rotation.scalar() << "," << te.rotation.x() << "," << te.rotation.y() << "," << te.rotation.z() << ")\n";
    std::cout << "translation: (" << te.translation.scalar() << "," << te.translation.x() << "," << te.translation.y() << "," << te.translation.z() << ")\n";
    std::cout << "++++++++++++++++++++++++++ \n";

    return os;
}

#endif // TYPEDEFS_H
