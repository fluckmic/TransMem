#include "frameAndLink.h"


/****************************
 * LINK                     *
 ****************************/

void Link::addTransformation(const FrameID &srcFrame, StampedTransformation stampedTransformation){

    // NOTE: assertion just during development
    // caller should not call the function on a wrong link
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    // caller should make sure that the translation is pure
    assert(stampedTransformation.translation.scalar() == 0.);

    if(child->frameID == srcFrame)
        invertTransformation(stampedTransformation);

    buf.addEntry(stampedTransformation);

    return;
}

void Link::transformationAtTimeT(const FrameID &srcFrame, StampedTransformation &stampedTransformation){

    // NOTE: assertion just during development
    // caller should not call the function on a wrong link
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    getTransformation(srcFrame, stampedTransformation, TIME);

    return;
}

void Link::oldestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation){

    // NOTE: assertion just during development
    // caller should not call the function on a wrong link
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    getTransformation(srcFrame, stampedTransformation, OLDEST);

    return;
}

void Link::newestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation){

    // NOTE: assertion just during development
    // caller should not call the function on a wrong link
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    getTransformation(srcFrame, stampedTransformation, NEWEST);

    return;
}

void Link::getTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation, AccessType accessType){

    switch(accessType){

    case TIME:      buf.entryAt(stampedTransformation);
                    break;
    case OLDEST:    buf.oldestEntry(stampedTransformation);
                    break;
    case NEWEST:    buf.newestEntry(stampedTransformation);
                    break;
    default:        assert(false);
                    /* we should never reach this point but
                       it's always good to have a default plan. */
    }

    if(child->frameID == srcFrame)
        invertTransformation(stampedTransformation);
}

void Link::invertTransformation(StampedTransformation &stampedTransformation){

    stampedTransformation.rotation = stampedTransformation.rotation.inverted();
    stampedTransformation.translation = -(stampedTransformation.rotation*stampedTransformation.translation*stampedTransformation.rotation.conjugated());

}

void Link::writeJSON(QJsonObject &json) const {

    QJsonObject parentObject; parentObject.insert("frameID", QString::fromStdString(parent->frameID));
    QJsonObject childObject; childObject.insert("frameID", QString::fromStdString(child->frameID));
    QJsonObject bufferObject; buf.writeJSON(bufferObject);

    json.insert("1_parent", parentObject);
    json.insert("2_child", childObject);
    json.insert("3_bufferedEntries", bufferObject);
}

/****************************
 * FRAME                    *
 ****************************/

void Frame::addLink(Link * const newLink){

    // NOTE: assertion just during development
    // caller passes a valid link object
    assert(newLink != nullptr);

    // link is connected to actual frames
    assert(newLink->parent && newLink->child != nullptr);

    //the link leads from this frame to another frame
    assert(frameID == newLink->parent->frameID);

    //check if there already exist such a link
    //in either direction
    Link* excistingLink = nullptr;
    connectionTo(newLink->child->frameID, excistingLink);
    if(excistingLink != nullptr)  return;

    newLink->child->connectionTo(frameID, excistingLink);
    if(excistingLink != nullptr)  return;

    //add link to this frame
    children.push_back(newLink);

    //add link to the destination
    newLink->child->parents.push_back(newLink);

    return;
}

void Frame::connectionTo(const FrameID &destination, Link *&linkToDest) {

    // returns the link to the frame f if there is one
    // if no link exist, the null pointer is returned

    linkToDest = nullptr;

    for(Link* p : parents)
        if(p->parent->frameID == destination)
           linkToDest = p;

    for(Link* c : children)
        if(c->child->frameID == destination)
            linkToDest = c;

    return;
}

void Frame::writeJSON(QJsonObject &json) const{

    QJsonArray parentObjects;
    for(Link* p: parents){
        QJsonObject parentObject; parentObject.insert("frameID", QString::fromStdString(p->parent->frameID));
        parentObjects.append(parentObject);
    }

    QJsonArray childObjects;
    for(Link* c: children){
        QJsonObject childObject; childObject.insert("frameID", QString::fromStdString(c->child->frameID));
        childObjects.append(childObject);
    }

    json.insert("1_frameID", QString::fromStdString(frameID));
    json.insert("2_parents", parentObjects);
    json.insert("3_children", childObjects);

}
