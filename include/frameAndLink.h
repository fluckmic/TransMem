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

    Link(Frame *const _parent, Frame *const _child, const DurationSec &_storageTime)
    : parent{_parent}
    , child(_child)
    , buf(_storageTime)
    {}

    void oldestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation);

    void newestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation);

    void addTransformation(const FrameID &srcFrame, StampedTransformation stampedTransformation);

    void transformationAtTimeT(const FrameID &srcFrame, StampedTransformation &stampedTransformation);

    Frame *const parent;

    Frame *const child;

    double weight = 1.;

    void writeJSON(QJsonObject &json) const;

protected:

    enum AccessType { TIME, OLDEST, NEWEST };

    void getTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation, AccessType accessType);

    void invertTransformation(StampedTransformation &stampedTransformation);

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

    // needed for dikstra
    double distance{0};
    Frame* predecessor{nullptr};
    bool active{true};

};

#endif // FRAMEANDLINK_H
