#ifndef FRAMEANDLINK_H
#define FRAMEANDLINK_H

#include "typedefs.h"
#include "transformationBuffer.h"
#include "stampedTransformation.h"

#include <string>
#include <vector>
#include <QtGui/QQuaternion>

class Frame;

/***************************
 * LINK                    *
 ***************************/

class Link
{

friend class GMLWriter;

public:

    Link(Frame *const parent, Frame *const child, const DurationSec &storageTime)
    : parent{parent}
    , child(child)
    , buf(storageTime)
    {}

    bool oldestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const;

    bool newestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const;

    bool addTransformation(const FrameID &srcFrame, StampedTransformation stampedTransformation);

    bool transformationAtTimeT(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const;

    bool distanceToNextClosestEntry(const Timestamp &tStamp, std::chrono::milliseconds &distanceToCloserEntry) const;

    Frame *const parent;

    Frame *const child;

    double weight = 1.;

    void writeJSON(QJsonObject &json) const;

protected:

    enum class AccessType { TIME, OLDEST, NEWEST };

    bool getTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation, AccessType accessType) const;

    void invertTransformation(StampedTransformation &stampedTransformation) const;

     TransformationBuffer buf;
};


/***************************
 * FRAME                   *
 ***************************/

class Frame
{
public:

    Frame(const FrameID &_frameID)
    : frameID(_frameID)
    {}

    void addLink(Link* const newLink);

    void connectionTo(const FrameID &destination, Link* &linkToDest);

    const FrameID frameID;

    std::vector<Link*> parents;

    std::vector<Link*> children;

    void writeJSON(QJsonObject &json) const;


};

#endif // FRAMEANDLINK_H
