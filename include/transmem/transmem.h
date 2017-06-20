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

#include "src/headers/typedefs.h"
#include "src/headers/frameAndLink.h"
#include "src/headers/stampedTransformation.h"
#include "src/headers/graphMLWriter.h"
#include "src/headers/dijkstra.h"


/****************************
 * NOSUCHLINKFOUNDEXCEPTION *
 ****************************/

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

/***************************
 * EMPTYLINKQUERYEXCEPTION *
 ***************************/

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


/****************************
 * PATH                     *
 ****************************/

struct Path {
    FrameID src;
    FrameID dst;
    std::vector< std::reference_wrapper<Link> > links;

    void writeJSON(QJsonObject &json) const;
};


/****************************
 * TRANSMEM                 *
 ****************************/

class TransMem
{

friend class GraphMLWriter;

public:

    /**
      * Default constructor.@n
      * Constructs a transmem object which bufferes the entries
      * for a default duration of 10 seconds.
      */
    TransMem() = default;

    /**
     * Alternative constructor.@n
     * Constructs a transmem object which bufferes the entries
     * for the duration specified in @a dur.
     * If the duration is smaller than one second, the duration
     * is set to one second.
     * @param dur buffer duration in seconds
     *
     */
    TransMem(DurationSec storageTime)
    : storageTime(storageTime)
    {
        // if the duration time is smaller than one second,
        // we set the duration time to one second.
        if(storageTime < DurationSec(1))
            storageTime = DurationSec(1);
    }

    /**
     * @fn void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans)
     *  @brief Registers a transformation @a trans between the source frame @a srcFrame and the destination frame @a destFrame at time @a tstamp.@n@n
     *  Does not complain if the added link leads to a cycle in the underlying datastructure.
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
     * Does not complain if the added link leads to a cycle in the underlying datastructure.
     * @param srcFrame identifier of the source frame
     * @param destFrame identifier of the destination frame
     * @param tstamp timestamp at what time the transformation is valid
     * @param qrot rotation quaternion
     * @param qtrans transformation quaternion
     */

    void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QQuaternion &qrot, const QQuaternion &qtrans);

    /**
     * @fn QQuaternion getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp) const
     * @brief Returns the transformation from @a srcFrame to @a destFrame at time @a tstamp.@n@n
     * The library searches for the shortest path from @a srcFrame to @a destFrame in the underlying graph datastructure with regard to the number of edges.@n
     * If the there is more than one shortest path, the one which is discovered first is used for the calculation of the transformation.@n@n@n
     * If @a tstamp is smaller than the oldest buffered transformation on a certain link, the library uses the oldest buffered transformation for the calculation
     * of the transformation.@n@n
     * If @a tstamp is larger than the newest buffered transformation on a certain link, the library uses the most recent transformation for the calculation.@n@n
     * If t1 < @a tstamp < t2 lies in between two entries bufered at time t1,t2 the library interpolates between the two entries. (SLERP for rotation, linear interpolation
     * for translation)@n and uses the interpolated transformation for the calculation.@n@n
     * If t1 < @a tstamp < t2 and (t2-t1) < 5 ns no interplation is done, the transformation at time t1 is used for the calculation.
     * @param srcFrame identifier of the source frame
     * @param destFrame identiier of the destination frame
     * @param tstamp timestamp at what time the returned transformation is valid
     * @return matrix encoding the requested transformation
     * @throws InvalidArgumentException
     * @throws NoSuchLinkFoundException
     */
    QMatrix4x4 getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp) const;

    /**
     * @fn QQuaternion getLink(const FrameID &srcFrame, const FrameID &fixFrame, const FrameID &destFrame, const Timestamp &tstamp1, const Timestamp &tstamp2) const
     * @brief Returns the transformation between @a srcFrame at time @a tstamp1 to @a destFrame at time @a tstamp2.@n The query requires a third frame @a fixFrame which is
     * stationary relative to @a srcFrame and @a destFrame.
     * @see TransMem::getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp) const
     * @param srcFrame identifier of the source frame
     * @param fixFrame identifier of the fix frame
     * @param destFrame identifier of the destination frame
     * @param tstamp1 timestamp when the transformation from the source to the fix frame is valid
     * @param tstamp2 timestamp when the transformation from the fix frame to the destination frame is valid
     * @return matrix encoding the requested transformation
     * @throws InvalidArgumentException
     * @throws NoSuchLinkFoundException
     */
    QMatrix4x4 getLink(const FrameID &srcFrame, const FrameID &fixFrame, const FrameID &destFrame, const Timestamp &tstamp1, const Timestamp &tstamp2) const;

    /**
     * @fn QQuaternion getBestLink(const FrameID &srcFrame, const FrameID &destFrame, Timestamp &tstamp) const
     * @brief Returns the best transformation betwen @a srcFrame and @a destFrame.@n@n
     * Best transformation:@n
     * Let L1,..,Ln be the links required for the transformation from @a srcFrame to @a destFrame.@n
     * Let Eij be the j-th transformaton entry inserted at time tij in the link Li.@n
     * The best transformation is the transformation obtained at time t, were t minimizes the sum over all |t-tij| from 1 to n.@n@n
     * The resolution of t is 10 ms.
     * @param srcFrame identifier of the source frame
     * @param destFrame identifier of the destination frame
     * @param tstamp returns at what time the best transformation was available
     * @return matrix encoding the requested transformation
     * @throws InvalidArgumentException
     * @throws NoSuchLinkFoundException
     */
    QMatrix4x4 getBestLink(const FrameID &srcFrame, const FrameID &destFrame, Timestamp &tstamp) const;

    QMatrix4x4 getBestLinkCached(const FrameID &srcFrame, const FrameID &destFrame, Timestamp &tstamp);

    void dumpAsJSON() const;

    void dumpAsGraphML() const;

protected:

    bool shortestPath(Path &p) const;

    bool calculateBestPointInTime(Path &path, Timestamp &tStampBestPointInTime) const;

    bool calculateTransformation(const Path &path, StampedTransformation &e) const;

    std::unordered_map<FrameID, Frame> frameID2Frame;

    std::deque<Link> links;

    std::unordered_map< std::string, std::pair< Timestamp, Path > > cachedBestLinks;
    std::unordered_map< std::string, QMatrix4x4 > cachedBestTransformations;

    DurationSec storageTime{10};

    mutable std::recursive_mutex lock;

    // JSON output
    enum class OutputType { PATH, TRANSMEM };

    void writeJSON(QJsonObject &json) const;
    void dumpJSONfile(const QString &path, const QJsonObject &json, const OutputType& outputType) const;
    void dumpPathAsJSON(const Path &p) const;

    bool bestLink(QMatrix4x4 &trans, Timestamp &tstamp, Path &p) const;
};

/****************************
 * TRANSMEM QML INTERFACE   *
 ****************************/

class TransMemQMLInterface : public QObject, public TransMem {
    Q_OBJECT

public:
    Q_INVOKABLE QMatrix4x4 getLinkBestCached(const QString& srcFrame, const QString& dstFrame);
    Q_INVOKABLE void registerLinkNow(const QString& srcFrame, const QString& dstFrame, const QMatrix4x4& trans);

    Q_INVOKABLE QMatrix4x4 getLinkNow(const QString& srcFrame, const QString& dstFrame) const;
    Q_INVOKABLE QMatrix4x4 getLinkBest(const QString& srcFrame, const QString& dstFrame) const;

};

#endif // TRANSMEM_H
