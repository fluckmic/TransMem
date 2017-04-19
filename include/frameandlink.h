#ifndef FRAMEANDLINK_H
#define FRAMEANDLINK_H

#include "typedefs.h"
#include "transformationbuffer.h"
#include "stampedtransformation.h"

#include <string>
#include <vector>
#include <QtGui/QQuaternion>

class Frame;

/***************************
 * LINK                    *
 ***************************/

class Link
{

public:

    Link(Frame *const _parent, Frame *const _child, const DurationSec &_storageTime)
    : parent{_parent}
    , child(_child)
    , buf(_storageTime)
    {}

    void oldestTransformation(const FrameID &srcFrame, StampedTransformation &e);

    void newestTransformation(const FrameID &srcFrame, StampedTransformation &e);

    void addTransformation(const FrameID &srcFrame, StampedTransformation &e);

    void transformationAtTimeT(const FrameID &srcFrame, StampedTransformation &e);

    Frame *const parent;

    Frame *const child;

    double weight = 1.;

protected:

    enum accessType { TIME, OLDEST, NEWEST };

    void getTransformation(const FrameID &srcFrame, StampedTransformation &e, accessType at);

    void invertTransformation(StampedTransformation &e);

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

    void addLink(Link* const l);

    void connectionTo(const FrameID &f, Link* l);

    const FrameID frameID;

    std::vector<Link*> parents;

    std::vector<Link*> children;

};

#endif // FRAMEANDLINK_H
