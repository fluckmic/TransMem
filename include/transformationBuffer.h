#ifndef TRANSFORMATIONBUFFER_H
#define TRANSFORMATIONBUFFER_H

#include "typedefs.h"
#include "stampedTransformation.h"
#include <list>

/**
 * @brief The TransformationBuffer class
 */
class TransformationBuffer
{

friend class GMLWriter;

public:

    TransformationBuffer(const DurationSec &d)
    : storageTime(d)
    {}

    bool oldestEntry(StampedTransformation &te);

    bool newestEntry(StampedTransformation &te);

    bool entryAt(StampedTransformation &te);

    bool addEntry(StampedTransformation &te);

    bool distanceToCloserEntry(const Timestamp &tStamp, std::chrono::milliseconds &distanceToNextEntry);

    void printCurrentBuffer();

    void writeJSON(QJsonObject &json) const;

protected:

    void interpolate(const StampedTransformation &el, const StampedTransformation &er, StampedTransformation &res);

    void pruneStorage();

    std::list<StampedTransformation> buffer;

    const DurationSec storageTime;

    const DurationNanoSec minDistForInterpolation{5};

    const unsigned int maxNumberOfEntries = 1000000;
    // TODO: avoid usage of more than maxNumberOfEntries in the buffer

};

#endif // TRANSFORMATIONBUFFER_H
