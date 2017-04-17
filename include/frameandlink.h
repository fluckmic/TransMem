#ifndef FRAMEANDLINK_H
#define FRAMEANDLINK_H

#include "typedefs.h"
#include "transformationbuffer.h"

#include <string>
#include <vector>
#include <QtGui/QQuaternion>

class Frame;

/**
 * @brief The Link class
 */
class Link
{

public:

    /**
     * @brief Link
     * @param parent
     * @param child
     */
    Link(Frame *const _parent, Frame *const _child, const DurationSec &_storageTime)
    : parent{_parent}
    , child(_child)
    , buf(_storageTime)
    {}

    /**
     * @brief oldestTransformation
     * @param srcFrame
     * @param destFrame
     * @param e
     */
    void oldestTransformation(const FrameID &srcFrame, const FrameID &destFrame, TransEntry &e);

    /**
     * @brief newestTransformation
     * @param srcFrame
     * @param destFrame
     * @param e
     */
    void newestTransformation(const FrameID &srcFrame, const FrameID &destFrame, TransEntry &e);

    /**
     * @brief addTransformation
     * @param srcFrame
     * @param destFrame
     * @param e
     */
    void addTransformation(const FrameID &srcFrame, const FrameID &destFrame, TransEntry &e);

    /**
     * @brief transformationAtTimeT
     * @param srcFrame
     * @param destFrame
     * @param e
     */
    void transformationAtTimeT(const FrameID &srcFrame, const FrameID &destFrame, TransEntry &e);

    /**
     * @brief parent
     */
    Frame *const parent;

    /**
     * @brief child
     */
    Frame *const child;

    /**
     * @brief weight
     */
    double weight = 1.;

protected:

    /**
     * @brief buf
     */
    TransformationBuffer buf;
};

/**
 * @brief The Frame class
 */
class Frame
{
public:
    /**
     * @brief Frame
     * @param frameID
     */
    Frame(const FrameID &frameID);

    /**
     * @brief isConnectedTo
     * @param f
     * @return
     */
    bool isConnectedTo(const FrameID &f) const;

    /**
     * @brief addParent
     * @param p
     */
    void addParent(Link &p);

    /**
     * @brief addChild
     * @param c
     */
    void addChild(Link &c);

    /**
     * @brief frameID
     */
    const FrameID frameID;

    /**
     * @brief parents
     */
    std::vector<Link> parents;

    /**
     * @brief children
     */
    std::vector<Link> children;
};

#endif // FRAMEANDLINK_H
