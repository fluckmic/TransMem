#include "../src/headers/frameAndLink.h"

bool Link::distanceToNextClosestEntry(const Timestamp &tStamp, std::chrono::milliseconds &distanceToCloserEntry) const {

    return buf.distanceToNextClosestEntry(tStamp, distanceToCloserEntry);
}

bool Link::addTransformation(const FrameID &srcFrame, StampedTransformation stampedTransformation) {

    // NOTE: Assertion just during development.
    // Caller should not call the function on a wrong link.
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    // Caller should make sure that the translation is pure.
    assert(stampedTransformation.translation.scalar() == 0.);

    if(child->frameID == srcFrame)
        invertTransformation(stampedTransformation);

    if(buf.addEntry(stampedTransformation)){
        lastTimeUpdated = std::chrono::high_resolution_clock::now();
        return true;
    }
    else
        return false;
}

bool Link::transformationAtTimeT(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const {

    // NOTE: Assertion just during development.
    // Caller should not call the function on a wrong link.
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    return getTransformation(srcFrame, stampedTransformation, AccessType::TIME);
}

bool Link::oldestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const {

    // NOTE: assertion just during development
    // caller should not call the function on a wrong link
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    return getTransformation(srcFrame, stampedTransformation, AccessType::OLDEST);
}

bool Link::newestTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation) const {

    // NOTE: Assertion just during development.
    // Caller should not call the function on a wrong link.
    assert((parent->frameID == srcFrame) || ( child->frameID == srcFrame));

    return getTransformation(srcFrame, stampedTransformation, AccessType::NEWEST);
}

bool Link::getTransformation(const FrameID &srcFrame, StampedTransformation &stampedTransformation, AccessType accessType) const {

    bool ret = false;

    switch(accessType){

    case AccessType::TIME:      ret = buf.entryAt(stampedTransformation);
                                break;
    case AccessType::OLDEST:    ret = buf.oldestEntry(stampedTransformation);
                                break;
    case AccessType::NEWEST:    ret = buf.newestEntry(stampedTransformation);
                                break;
    default:        assert(false);
                    // We should never reach this point but
                    // it's always good to have a default plan.
    }

    if(!ret)
        return false;

    if(child->frameID == srcFrame)
        invertTransformation(stampedTransformation);

    return true;
}

void Link::invertTransformation(StampedTransformation &stampedTransformation) const {

    stampedTransformation.rotation = stampedTransformation.rotation.conjugated();
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

void Frame::addLink(Link * const newLink) {

    // Caller passes a valid link object.
    assert(newLink != nullptr);

    // Link is connected to actual frames.
    assert(newLink->parent && newLink->child != nullptr);

    // The link leads from this frame to another frame.
    assert(frameID == newLink->parent->frameID);

    // Check if there already exist such a link
    // in either direction.
    Link* excistingLink = nullptr;
    connectionTo(newLink->child->frameID, excistingLink);
    if(excistingLink != nullptr)  return;

    newLink->child->connectionTo(frameID, excistingLink);
    if(excistingLink != nullptr)  return;

    // Add link to this frame.
    children.push_back(newLink);

    // Add link to the destination.
    newLink->child->parents.push_back(newLink);
}

void Frame::connectionTo(const FrameID &destination, Link *&linkToDest) {

    // Returns the link to the frame f if there is one
    // if no link exist, the null pointer is returned.

    linkToDest = nullptr;

    for(Link* p : parents)
        if(p->parent->frameID == destination)
           linkToDest = p;

    for(Link* c : children)
        if(c->child->frameID == destination)
            linkToDest = c;

    return;
}

void Frame::writeJSON(QJsonObject &json) const {

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
