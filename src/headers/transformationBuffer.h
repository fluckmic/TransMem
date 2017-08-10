#ifndef TRANSFORMATIONBUFFER_H
#define TRANSFORMATIONBUFFER_H

#include <list>

#include "typedefs.h"
#include "stampedTransformation.h"

/*************************
 * TRANSFORMATION BUFFER *
 *************************/

class TransformationBuffer {

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

    void writeJSON(QJsonObject &json) const;

protected:

    void interpolate(const StampedTransformation &el, const StampedTransformation &er, StampedTransformation &res) const;

    void pruneStorage();

    std::list<StampedTransformation> buffer;

    const DurationSec storageTime;

    const DurationNanoSec MIN_DISTANCE_FOR_INTERPOLATION {5};

    const unsigned int MAX_NUMBER_OF_ENTRIES {1000000};

};

#endif // TRANSFORMATIONBUFFER_H
