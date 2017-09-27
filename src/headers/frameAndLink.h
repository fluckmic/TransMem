#ifndef FRAMEANDLINK_H
#define FRAMEANDLINK_H

#include "typedefs.h"
#include "transformationBuffer.h"
#include "stampedTransformation.h"

#include <string>
#include <vector>
#include <QtGui/QQuaternion>

class Frame;


/*! \class Link
 * \brief Stores all the transformations mapping between the two
 * frames connected through the link. Each link connects a parent frame
 * with a child frame. The transformations stored on the link map from the
 * parent frame to the child frame.
 * \see class Frame
 */
class Link
{

friend class GMLWriter;

public:

    /*! \fn Link(Frame *const parent, Frame *const child, const DurationMilliSec &storageTimeInMS, const double &confidence)
     * \param parent Pointer to the parent frame.
     * \param child Pointer to the child frame.
     * \param storageTimeinMS Duration how long the transformations are stored on the link. Upon every update all tranformations
     * which are older than the time of the newest stored entry minus this duration are removed.
     * \param confidence Intial confidence value of the link. (\see Link::confidence).
     */
    Link(Frame *const parent, Frame *const child, const DurationMilliSec &storageTimeInMS, const double &confidence)
    : parent{parent}
    , child(child)
    , confidence(confidence)
    , buf(storageTimeInMS)
    {}

    /*! \fn bool oldestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const
     * \brief Returns the oldest transformation stored on the link. If there is no transformation stored
     * on the link false is returned, otherwise true. The transformation is stored in \a stampedTransformation.
     * The FrameID \a srcFrame determines the direction of the returned transformation.
     */
    bool oldestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const;

    /*! \fn bool newestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const
     * \brief Returns the newest transformation stored on the link. If there is no transformation stored
     * on the link false is returned, otherwise true. The transformation is stored in \a stampedTransformation.
     * The FrameID \a srcFrame determines the direction of the returned transformation.
     */
    bool newestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const;

    /*! \fn  bool addTransformation(const FrameID &srcFrame, StampedTransformation stampedTransformation
     * \brief Adds the transformation together with its timestamp as StampedTransformation object to the
     * underlying data structure which stores all transformations on the link. If the transformation describes
     * a mapping to the srcFrame (i.e. the child frame of this link has the identifier \a srcFrame)
     * the transformation is inverted prior to the storing. If this link has the identifier \a dstFrame
     * then \a stampedTransformation is stored direct.
     * (A link stores the transformations mapping from its parent frame towards its child frame.)
     */
    bool addTransformation(const FrameID &srcFrame, StampedTransformation stampedTransformation);

    /*! \fn bool transformationAtTimeT(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const
     * \brief Adds the transformation valid at the time already specified in \a stampedTransformation to this
     * StampedTransformation object. For more information see the subsection Link-Query in TransMem.h.
     */
    bool transformationAtTimeT(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const;

    /*! \fn bool distanceToNextClosestEntry(const Timestamp &tStamp, std::chrono::milliseconds &distanceToCloserEntry) const
     * \brief Determines the distance to the closest transformation stored on the link relative to the time \a tStamp. If
     * no transformation is stored on the link false is returned, otherwise true. The actual distance is returned
     * in \a distanceToCloserEntry.
     */
    bool distanceToNextClosestEntry(const Timestamp &tStamp, std::chrono::milliseconds &distanceToCloserEntry) const;

    /*! \var Frame *const parent
     * \brief The link connects a parent frame with child frame.
     * This is the pointer to the parent frame.
     */
    Frame *const parent;

    /*! \var Frame* const child
     * \brief The link connects a parent frame with child frame.
     * This is the pointer to the child frame.
     */
    Frame *const child;

    /*! \var double weight
     * \brief Weight of this link considerd for the calculation
     * of the shortest path.
     */
    double weight{1.};

    /*! \var double confidence
     * \brief The confidence value can be specified by the user. The value is not used for any
     * internal calculations. It can for example be used to represent a quality measure of a link.
     */
    double confidence;

    /*! \var Timestamp lastTimeUpdated
     * \brief Point in time when the links was updated for the last time.
     * Only a successful addition (transformation is acutally stored in the underlying datastructure) of a transformation
     * is considered to be an update. The retrieval of a transformation is no update.
     */
    Timestamp lastTimeUpdated;

    void writeJSON(QJsonObject &json) const;

protected:

    enum class AccessType { TIME, OLDEST, NEWEST };

    /*! \fn  bool getTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation, AccessType accessType) const
     * \brief Allows to retrieve a transformation stored on the link.
     * \see bool oldestTransformation(..), bool newestTransformation(..) and bool transformationAtTimeT(..).
     */
    bool getTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation, AccessType accessType) const;

    /*! \fn void invertTransformation(StampedTransformation &stampedTransformation) const
     * \brief Inverts the trasnformation represented through \a stampedTransformation.
     */
    void invertTransformation(StampedTransformation &stampedTransformation) const;

    /*! \var TransformationBuffer buf
     * \brief Stores all the transformation on the link.
     */
    TransformationBuffer buf;
};

/*! \class Frame
 * \brief Node representing a coordinate system (i.e. a frame).
 */
class Frame
{
public:

    Frame(const FrameID &_frameID)
    : frameID(_frameID)
    {}

    /*! \fn void addLink(Link* const newLink)
     * \brief Adds a new link conneting this frame and the frame
     * specified as child frame in \a newLink. If there is already such a
     * link nothing happens. The parent of \a newLink has to be this frame.
     */
    void addLink(Link* const newLink);

    /*! \fn void connectionTo(const FrameID &destination, Link* &linkToDest)
     * \brief Returns a pointer to the link connecting this frame with the frame
     * with the identifier \a destination. If there is no such link a null pointer
     * is returned.
     */
    void connectionTo(const FrameID &destination, Link* &linkToDest);

    /*! \var const FrameID frameID
     * \brief Unique identifier of the frame.
     */
    const FrameID frameID;

    /*! \var std::vector<Link*> parents
     * \brief Stores the pointer to all links connecting a parent frame with
     * this frame. The direction of this link is therefore from the parent
     * frame towards this frame.
     */
    std::vector<Link*> parents;

    /*! \var std::vector<Link*> children
     * \brief Stores the pointer to all links connecting this frame with
     * child frame. The direction of this link is therefore from this frame
     * frame towards the child frame.
     */
    std::vector<Link*> children;

    void writeJSON(QJsonObject &json) const;

};

#endif // FRAMEANDLINK_H
