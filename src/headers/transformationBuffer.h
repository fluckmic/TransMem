#ifndef TRANSFORMATIONBUFFER_H
#define TRANSFORMATIONBUFFER_H

#include <list>

#include "typedefs.h"
#include "stampedTransformation.h"

class TransformationBuffer {

friend class GMLWriter;

public:

    TransformationBuffer(const DurationMilliSec &storageTimeInMS)
    : storageTimeInMS(storageTimeInMS)
    {}

    /*! \fn  bool entryAt(StampedTransformation &te) const
     * \brief Sets \a te to the transformation valid a the time specified already
     * in \a te. If no transformation is stored on the the link false is returned,
     * otherwise true.
     * For more information see the subsection Link-Query in TransMem.h.
     */
    bool entryAt(StampedTransformation &te) const;

    /*! \fn  bool addEntry(const StampedTransformation &te)
     * \brief Stores the StampedTransformation \a te on the buffer.
     * \return True if the entry was stored successfully, false otherwise (entry might be to old)
     */
    bool addEntry(const StampedTransformation &te);

    /*! \fn  bool oldestEntry(StampedTransformation &te) const
     * \brief Sets \a te to the oldest transformation stored in the buffer.
     * If no transformation is stored false is returned, otherwise true.
     */
    bool oldestEntry(StampedTransformation &te) const;

    /*! \fn bool newestEntry(StampedTransformation &te) const
     * \brief Sets \a te to the newest transformation stored in the buffer.
     * If no transformation is stored false is returned, otherwise true.
     */
    bool newestEntry(StampedTransformation &te) const;

    /*! \fn  bool distanceToNextClosestEntry(const Timestamp &tStamp, std::chrono::milliseconds &distanceToCloserEntry) const
     * \brief \see Link::distanceToNextClosestEntry(..).
     */
    bool distanceToNextClosestEntry(const Timestamp &tStamp, std::chrono::milliseconds &distanceToCloserEntry) const;

    void writeJSON(QJsonObject &json) const;

protected:

    /*! \fn  void interpolate(const StampedTransformation &el, const StampedTransformation &er, StampedTransformation &res) const
     * \brief  Interpolates the transformation between \a el and \a er and stores it into \a res. The rotation part is interpolated
     * using SLERP, the translation part of the transfomations is interpolated linearly. The two transformation are weighted with regard
     * to the ratio of the distance between the time specified in \a res and the time specified in \a er and \a res, respectively.
     */
    void interpolate(const StampedTransformation &el, const StampedTransformation &er, StampedTransformation &res) const;

    /*! \fn void pruneStorage()
     * \brief Removes all transformations which are too old.
     */
    void pruneStorage();

    std::list<StampedTransformation> buffer;

    /*! \var const DurationMilliSec storageTimeInMS
     * \brief Internal container which contains all the stored StampedTransformation objects.
     */
    const DurationMilliSec storageTimeInMS;

    /*! \var const DurationNanoSec MIN_DISTANCE_FOR_INTERPOLATION_IN_NS
     *  \brief Minimal distance between two stored entries required for
     * interpolation. If the distance is less the older entry is used.
     * (\see TransformationBuffer::entryAt(..)).
     */
    const DurationNanoSec MIN_DISTANCE_FOR_INTERPOLATION_IN_NS {5};

    /*! \var const unsigned int MAX_NUMBER_OF_ENTRIES
     * \brief Maximal number of transformations which can be stored in the buffer.
     */
    const unsigned int MAX_NUMBER_OF_ENTRIES {1000000};

};

#endif // TRANSFORMATIONBUFFER_H
