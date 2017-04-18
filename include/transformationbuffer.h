#ifndef TRANSFORMATIONBUFFER_H
#define TRANSFORMATIONBUFFER_H

#include "typedefs.h"

#include <list>

/**
 * @brief The TransformationBuffer class
 */
class TransformationBuffer
{

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
    void oldestEntry(TransEntry &te);

    /**
     * @brief newestEntry
     * @param te
     */
    void newestEntry(TransEntry &te);

    /**
     * @brief entryAt
     * @param te
     */
    void entryAt(TransEntry &te);

    /**
     * @brief addEntry
     * @param te
     */
    void addEntry(TransEntry &te);

    /**
     * @brief printCurrentBuffer
     */
    void printCurrentBuffer();

protected:

    /**
     * @brief interpolate
     * @param t
     * @param el
     * @param er
     * @param res
     */
    void interpolate(const TransEntry &el, const TransEntry &er, TransEntry &res);

    /**
     * @brief pruneStorage
     */
    void pruneStorage();

    /**
     * @brief buffer
     */
    std::list<TransEntry> buffer;

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

};

#endif // TRANSFORMATIONBUFFER_H
