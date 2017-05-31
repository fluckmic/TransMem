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

    bool entryAt(StampedTransformation &te) const;

    bool addEntry(const StampedTransformation &te);

    bool oldestEntry(StampedTransformation &te) const;

    bool newestEntry(StampedTransformation &te) const;

    bool distanceToNextClosestEntry(const Timestamp &tStamp, std::chrono::milliseconds &distanceToCloserEntry) const;

    void printCurrentBuffer();

    void writeJSON(QJsonObject &json) const;

protected:

    void interpolate(const StampedTransformation &el, const StampedTransformation &er, StampedTransformation &res) const;

    void pruneStorage();

    std::list<StampedTransformation> buffer;

    const DurationSec storageTime;

    const DurationNanoSec minDistForInterpolation{5};

    const unsigned int maxNumberOfEntries = 1000000;
    // TODO: avoid usage of more than maxNumberOfEntries in the buffer

};

#endif // TRANSFORMATIONBUFFER_H
