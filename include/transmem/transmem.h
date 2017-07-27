/*! @file   transmem.h
    @brief  header file of transmem.cpp
*/

#ifndef TRANSMEM_H
#define TRANSMEM_H

#include <QtGui/QMatrix4x4>
#include <QtGui/QQuaternion>
#include <unordered_map>
#include <deque>
#include <queue>
#include <mutex>
#include <functional>
#include <QDateTime>
#include <math.h>


#include "../../src/headers/typedefs.h"
#include "../../src/headers/frameAndLink.h"
#include "../../src/headers/stampedTransformation.h"
#include "../../src/headers/graphMLWriter.h"

/********************************
 * NO SUCH LINK FOUND EXCEPTION *
 ********************************/

class NoSuchLinkFoundException : public std::runtime_error {

public:
    NoSuchLinkFoundException(const FrameID &_srcFrame, const FrameID &_destFrame)
    : std::runtime_error("No such link found: " + _srcFrame + "-" + _destFrame)
    , srcFrame(_srcFrame)
    , destFrame(_destFrame)
    {}

    virtual const char* what() const throw();

private:
    FrameID srcFrame;
    FrameID destFrame;
};

/******************************
 * EMPTY LINK QUERY EXCEPTION *
 ******************************/

class EmptyLinkQueryException : public std::runtime_error {

public:
    EmptyLinkQueryException(const Link &emptyLink)
    : std::runtime_error("Queried an empty link: " + emptyLink.parent->frameID
                         + "-" + emptyLink.child->frameID)
    , emptyLink(emptyLink)
    {}

    virtual const char* what() const throw();

private:
    Link emptyLink;
};

/********
 * PATH *
 ********/

struct Path {

    FrameID src;
    FrameID dst;
    std::vector< std::reference_wrapper<Link> > links;

    void writeJSON(QJsonObject &json) const;
};

/************************************
 * STAMPED AND RATED TRANSFORAMTION *
 * **********************************/

/* REMARK:
   New return type of transmem. Encodes not just the transformation but also
   some information about the quality. Unnecessary convertion between matrix and quaternion
   are no longer needed and the return type is much more flexible, one can add additional
   stuff without changing interface to much. */

struct StampedAndRatedTransformation {

    // Quaternions encoding the transformation.
    QQuaternion qRot;
    QQuaternion qTra{0,0,0,0};

    // Average of the confidence of all links used to calculate the transformation.
    double avgLinkConfidence{0};

    // Distance to the most farthest update of all links used to calcualte the transformation.
    double maxDistanceToEntry{std::numeric_limits<float>::max()};

    Timestamp time;
};

/************
 * TRANSMEM *
 ************/

class TransMem
{

typedef double (*func_t)(double);

friend class GraphMLWriter;

public:

    /**
      * Default constructor.@n
      * Constructs a transmem object which bufferes the entries
      * for a default duration of 10 seconds.
      *
      * The default confidence of a link is set to 1.
      * The default mapping for distanceToEntry is f(x) = x.
      */
    TransMem() = default;

    /**
     * Alternative constructor.@n
     * Constructs a transmem object which bufferes the entries
     * for the duration specified in @a dur.
     * If the duration is smaller than one second, the duration
     * is set to one second.
     * @param storageTime Buffer duration in seconds.
     * @param defaultLinkConfidence Default value for a new link if confidence is not set via the update..
     * @param distanceToEntryMapping Custom mapping specified as function pointer.
     */
    TransMem(DurationSec storageTime, const double defaultLinkConfidence, func_t distanceToEntryMapping)
    : storageTime(storageTime)
    , defaultLinkConfidence(defaultLinkConfidence)
    , distanceToEntryMapping(distanceToEntryMapping)
    {
        // If the duration time is smaller than one second,
        // we set the duration time to one second.
        if(storageTime < DurationSec(1))
            storageTime = DurationSec(1);
    }

    /**
     * @fn void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans)
     *  @brief Registers a transformation @a trans between the source frame @a srcFrame and the destination frame @a destFrame at time @a tstamp.@n@n
     *  Does not complain if the added link leads to a cycle in the underlying datastructure. If there where no previous registrations of this Link
     *  the confidence of the link is set to the defaultValue. If there where previous registrations the confidence of the link does not change.
     *  @param srcFrame identifier of the source frame
     *  @param destFrame identifier of the destination frame
     *  @param tstamp timestamp at what time the transformation is valid
     *  @param trans transformation matrix
     */
    void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans);

    /**
     * @fn void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QQuaternion &qrot, const QQuaternion &qtrans)
     * @brief Registers a transformation consisting of a rotation @a qrot followed by a translation @a qtrans between the source frame @a srcFrame and the destination frame
     * @a destFrame at time @a tstamp.@n@n
     * Does not complain if the added link leads to a cycle in the underlying datastructure. If there where no previous registrations of this Link
     *  the confidence of the link is set to the defaultValue. If there where previous registrations the confidence of the link does not change.
     * @param srcFrame identifier of the source frame
     * @param destFrame identifier of the destination frame
     * @param tstamp timestamp at what time the transformation is valid
     * @param qrot rotation quaternion
     * @param qtrans transformation quaternion
     */
    void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QQuaternion &qrot, const QQuaternion &qtrans);

    /**
     * @fn void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans, const float &confidence)
     * @see void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans)
     * @brief Parameter @a confidence allows to set a confidence to the link between the source frame and the destination frame.
     * @param confidence Sets the confidence of the link between srcFrame and destFrame to @a confidence.
     */
    void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans, const double &confidence);

    /**
     * @fn void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QQuaternion &qrot, const QQuaternion &qtrans, const float &confidence)
     * @see void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans, const float &confidence)
     */
    void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QQuaternion &qrot, const QQuaternion &qtrans, const double &confidence);

    /**
     * @fn StampedAndRatedTransformation getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp) const
     * @brief Returns the transformation from @a srcFrame to @a destFrame at time @a tstamp encoded in a StampedAndRatedTransformation object.@n@n
     * The library searches for the shortest path from @a srcFrame to @a destFrame in the underlying graph datastructure with regard to the number of edges.@n
     * If the there is more than one shortest path, the one which is discovered first is used for the calculation of the transformation.@n@n@n
     * If @a tstamp is smaller than the oldest buffered transformation on a certain link, the library uses the oldest buffered transformation for the calculation
     * of the transformation.@n@n
     * If @a tstamp is larger than the newest buffered transformation on a certain link, the library uses the most recent transformation for the calculation.@n@n
     * If t1 < @a tstamp < t2 lies in between two entries bufered at time t1,t2 the library interpolates between the two entries. (SLERP for rotation, linear interpolation
     * for translation)@n and uses the interpolated transformation for the calculation.@n@n
     * If t1 < @a tstamp < t2 and (t2-t1) < 5 ns no interplation is done, the transformation at time t1 is used for the calculation.
     * @param srcFrame Identifier of the source frame.
     * @param destFrame Identiier of the destination frame.
     * @param tstamp Timestamp at what time the returned transformation is valid.
     * @return A StampedAndRatedTransformation object encoding the requested transformation and additional quality information.
     * @throws InvalidArgumentException
     * @throws NoSuchLinkFoundException
     */
    StampedAndRatedTransformation getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp) const;

    /**
     * @fn StampedAndRatedTransformation getBestLink(const FrameID &srcFrame, const FrameID &destFrame, Timestamp &tstamp) const
     * @brief Returns the best transformation betwen @a srcFrame and @a destFrame encoded in a StampedAndRatedTransformation object.@n@n
     * Best transformatio:@n
     * Let L1,..,Ln be the links required for the transformation from @a srcFrame to @a destFrame.@n
     * Let Eij be the j-th transformaton entry inserted at time tij in the link Li.@n
     * The best transformation is the transformation obtained at time t, were t minimizes the sum over all |t-tij| from 1 to n.@n@n
     * The resolution of t is 10 ms.
     * Transmem caches the entries. That is, if no link of a certain transformation was updated, the cached transformation is returned.
     * @param srcFrame Identifier of the source frame.
     * @param destFrame Identifier of the destination frame.
     * @param tstamp Returns at what time the best transformation was available.
     * @return A StampedAndRatedTransformation object encoding the requested transformation and additional quality information.
     * @throws InvalidArgumentException
     * @throws NoSuchLinkFoundException
     */
    StampedAndRatedTransformation getBestLink(const FrameID &srcFrame, const FrameID &destFrame);

    /**
     * @fn void updateLinkConfidence(const FrameID &srcFrame, const FrameID &destFrame, const double &confidence)
     * @brief Updates the link from @a srcFrame to @a destFrame with @a confidence. If there was never such a link
     * registered before, a NoSuchLinkFoundException is thrown.
     * @param srcFrame Identifier of the source frame.
     * @param destFrame Identifier of the destination frame.
     * @param confidence Confidence value.
     * @throws NoSuchLinkFoundException
     */
    void updateLinkConfidence(const FrameID &srcFrame, const FrameID &destFrame, const double &confidence);

    /**
     * @fn voidAsJSON() const
     * @brief Dumps the current state of the transmem as JSON to a file.
     */
    void dumpAsJSON() const;

    /**
     * @fn void dumpAsGraphML() const
     * @brief Dumps the current graph build by the different registered links as GraphML to a file.
     */
    void dumpAsGraphML() const;

protected:

    bool shortestPath(Path &p) const;
    bool bestLink(StampedAndRatedTransformation &stT, Path &p) const;
    void calculateBestPointInTime(Path &path, Timestamp &bestPoint) const;
    void calculateTransformation(const Path &path, StampedAndRatedTransformation &resultT) const;
    void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp,
                      const QQuaternion &qrot, const QQuaternion &qtrans,
                      const double &confidence, const bool &updateConfidence);

    std::unordered_map<FrameID, Frame> frameID2Frame;

    std::deque<Link> links;

    std::unordered_map< std::string, std::pair< Timestamp, Path > > cachedBestLinks;
    std::unordered_map< std::string, StampedAndRatedTransformation > cachedBestTransformations;

    DurationSec storageTime{10};

    const double defaultLinkConfidence = 1.;
    func_t distanceToEntryMapping = [](double x) {return x;};

    mutable std::recursive_mutex lock;

    // JSON output
    enum class OutputType { PATH, TRANSMEM };

    void writeJSON(QJsonObject &json) const;
    void dumpJSONfile(const QString &path, const QJsonObject &json, const OutputType& outputType) const;
    void dumpPathAsJSON(const Path &p) const;

};

#endif // TRANSMEM_H
