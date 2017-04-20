#include "frameandlink.h"


/****************************
 * LINK                     *
 ****************************/

void Link::addTransformation(const FrameID &srcFrame, StampedTransformation &e){

    // NOTE: assertion just during development
    // caller should not call the function on a wrong link
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    // NOTE: assertion just during development
    // caller should make sure that the translation is pure
    assert(e.translation.scalar() == 0.);

    if(child->frameID == srcFrame)
        invertTransformation(e);

    buf.addEntry(e);

    return;
}

void Link::transformationAtTimeT(const FrameID &srcFrame, StampedTransformation &e){

    // NOTE: assertion just during development
    // caller should not call the function on a wrong link
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    getTransformation(srcFrame, e, TIME);

    return;
}

void Link::oldestTransformation(const FrameID &srcFrame, StampedTransformation &e){

    // NOTE: assertion just during development
    // caller should not call the function on a wrong link
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    getTransformation(srcFrame, e, OLDEST);

    return;
}

void Link::newestTransformation(const FrameID &srcFrame, StampedTransformation &e){

    // NOTE: assertion just during development
    // caller should not call the function on a wrong link
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    getTransformation(srcFrame, e, NEWEST);

    return;
}

void Link::getTransformation(const FrameID &srcFrame, StampedTransformation &e, accessType at){

    switch(at){

    case TIME:      buf.entryAt(e);     break;
    case OLDEST:    buf.oldestEntry(e); break;
    case NEWEST:    buf.newestEntry(e); break;
    }

    if(child->frameID == srcFrame)
        invertTransformation(e);
}

void Link::invertTransformation(StampedTransformation &e){

    e.rotation = e.rotation.inverted();
    e.translation = -(e.rotation*e.translation*e.rotation.conjugated());

}


/****************************
 * FRAME                    *
 ****************************/

void Frame::addLink(Link * const l){

    // NOTE: assertion just during development
    // caller passes a valid link object
    assert(l != nullptr);

    // link is connected to actual frames
    assert(l->parent && l->child != nullptr);

    //the link leads from this frame to another frame
    assert(frameID == l->parent->frameID);

    //check if there already exist such a link
    //in either direction
    Link* lnk = nullptr;
    connectionTo(l->child->frameID, lnk);
    if(lnk != nullptr)  return;

    l->child->connectionTo(frameID, lnk);
    if(lnk != nullptr)  return;

    //add link to this frame
    children.push_back(l);

    //add link to the destination
    l->child->parents.push_back(l);

    return;
}

void Frame::connectionTo(const FrameID &f, Link *&l) {

    // returns the link to the frame f if there is one
    // if no link exist, the null pointer is returned

    l = nullptr;

    for(Link* p : parents)
        if(p->parent->frameID == f)
           l = p;

    for(Link* c : children)
        if(c->child->frameID == f)
            l = c;

    return;
}
