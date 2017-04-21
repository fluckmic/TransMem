#ifndef TRANSFORMATIONBUFFER_H
#define TRANSFORMATIONBUFFER_H

#include "typedefs.h"
#include "stampedtransformation.h"
#include <list>

/**
 * @brief The TransformationBuffer class
 */
class TransformationBuffer
{

friend class GMLWriter;

public:

    /**
     * @brief TransformationBuffer
     * @param d
     */
    TransformationBuffer(const DurationSec &d)
    : storageTime(d)
    {}

    /**
     * @brief oldestEntry
     * @param te
     */
    void oldestEntry(StampedTransformation &te);

    /**
     * @brief newestEntry
     * @param te
     */
    void newestEntry(StampedTransformation &te);

    /**
     * @brief entryAt
     * @param te
     */
    void entryAt(StampedTransformation &te);

    /**
     * @brief addEntry
     * @param te
     */
    void addEntry(StampedTransformation &te);

    /**
     * @brief printCurrentBuffer
     */
    void printCurrentBuffer();

    void writeJSON(QJsonObject &json) const;

protected:

    /**
     * @brief interpolate
     * @param t
     * @param el
     * @param er
     * @param res
     */
    void interpolate(const StampedTransformation &el, const StampedTransformation &er, StampedTransformation &res);

    /**
     * @brief pruneStorage
     */
    void pruneStorage();

    /**
     * @brief buffer
     */
    std::list<StampedTransformation> buffer;

    /**
     * @brief storageTime
     */
    const DurationSec storageTime;

    /**
     * @brief DurationNanoSec
     */
    const DurationNanoSec minDistForInterpolation{5};

    /**
     * @brief maxNumberOfEntries
     */
    const unsigned int maxNumberOfEntries = 1000000;
    // TODO: avoid usage of more than maxNumberOfEntries in the buffer

};

#endif // TRANSFORMATIONBUFFER_H
