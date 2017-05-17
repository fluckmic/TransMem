/*! \file   transmem.cpp
    \brief  source file transmem.cpp
*/

#include "transmem.h"

/****************************
 * NOSUCHLINKFOUNDEXCEPTION *
 ****************************/

const char* NoSuchLinkFoundException::what() const throw(){

    return std::runtime_error::what();
}

/****************************
 * TRANSMEM                 *
 ****************************/

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QQuaternion &qrot, const QQuaternion &qtrans){

    // TODO: check if normalized rotation quaternion and pure translation quaternion?!

    // get the lock
    lock.lock();

    // check if frames already exist
    auto iter2SrcFrame = frameID2Frame.find(srcFrame);
    auto iter2DstFrame = frameID2Frame.find(destFrame);

    Frame* ptr2SrcFrame; Frame* ptr2DstFrame;

    // if a frame does not exist, create it
    if(iter2SrcFrame == frameID2Frame.end()){

        frames.emplace_back(Frame{srcFrame});
        ptr2SrcFrame = &frames.back();
        frameID2Frame.insert({srcFrame, ptr2SrcFrame});

    }
    else
        ptr2SrcFrame = (*iter2SrcFrame).second;

    if(iter2DstFrame == frameID2Frame.end()){

        frames.emplace_back(Frame{destFrame});
        ptr2DstFrame = &frames.back();
        frameID2Frame.insert({destFrame, ptr2DstFrame});

    }
    else
        ptr2DstFrame = (*iter2DstFrame).second;

    // check if a link between srcFrame and destFrame exists
    Link* ptr2Link = nullptr;
    ptr2SrcFrame->connectionTo(destFrame, ptr2Link);

    // if the link does not exist, create it
    if(ptr2Link == nullptr){

        links.emplace_back(Link{ptr2SrcFrame, ptr2DstFrame, storageTime});
        ptr2Link = &links.back();
        ptr2SrcFrame->addLink(ptr2Link);
    }

    // add the transformation to the link
    ptr2Link->addTransformation(srcFrame, StampedTransformation{tstamp, qrot, qtrans});

    // release the lock
    lock.unlock();

    return;
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans) {

   float data[]{trans(0,0),trans(0,1),trans(0,2),
                trans(1,0),trans(1,1),trans(1,2),
                trans(2,0),trans(2,1),trans(2,2)};

   QMatrix3x3 rM(data);

   // TODO: check if the rotation matrix is normal, emit a warning if not

   registerLink(srcFrame, destFrame, tstamp, QQuaternion::fromRotationMatrix(rM), QQuaternion(0, trans(0,3), trans(1,3), trans(2,3)));

}

void TransMem::writeJSON(QJsonObject &json) const {

    QJsonArray frameObjects;
    for(Frame f: frames){
        QJsonObject frameObject;
        f.writeJSON(frameObject);
        frameObjects.append(frameObject);
    }

    QJsonArray linkObjects;
    for(Link l: links){
        QJsonObject linkObject;
        l.writeJSON(linkObject);
        linkObjects.append(linkObject);
    }

    json.insert("frames", frameObjects);
    json.insert("links", linkObjects);

}

void TransMem::shortestPath(Path &path){

    Diijkstra diijkstra(frameID2Frame);
    diijkstra.calculateShortestPath(path);

}

void TransMem::dumpAsJSON() {

    // TODO: add date and time to filename of dump

    QJsonObject transmemObject;
    lock.lock(); writeJSON(transmemObject); lock.unlock();

    dumpJSONfile("TransMemDump", transmemObject);

    return;
}

void TransMem::dumpJSONfile(const QString &path, const QJsonObject &json) const {

    QFile file(path+".json");
    if(!file.open(QIODevice::WriteOnly)){
        // TODO: error handling
        return;
    }

    QJsonDocument saveJSON(json);
    file.write(saveJSON.toJson());

    file.close();
    if(file.error()){
        // TODO: error handling
        return;
    }

}

void TransMem::dumpPathAsJSON(const Path &p){

    // TODO:: add date and time to filname of dump

   QJsonObject pathObject;
   lock.lock(); p.writeJSON(pathObject); lock.unlock();

   dumpJSONfile("PathDump", pathObject);

   return;
}

void TransMem::dumpAsGraphML() {

   GMLWriter writer;

   lock.lock(); writer.write(this); lock.unlock();

}

QMatrix4x4 TransMem::getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp) {

    if(srcFrame == destFrame)
        throw std::invalid_argument("Not allowed to query for link if source frame is equal to destination frame.");

    // get the lock
    lock.lock();

    // search for shortest path between source frame and  destination frame
    Path p{srcFrame, destFrame, std::vector<Link*>()};
    shortestPath(p);

    // TODO: just for debugging
    this->dumpPathAsJSON(p);

    // calculate transformation along path
    StampedTransformation t{tstamp, QQuaternion(), QQuaternion(0,0,0,0)};
    calculateTransformation(p, t);

    // release the lock
    lock.unlock();

    // convert to QMatrix4x4
    QMatrix3x3 rot = t.rotation.toRotationMatrix();
    QMatrix4x4 ret(rot);
    ret(0,3) = t.translation.x(); ret(1,3) = t.translation.y(); ret(2,3) = t.translation.z();

    return ret;

}

QMatrix4x4 TransMem::getLink(const FrameID &srcFrame, const FrameID &fixFrame, const FrameID &destFrame, const Timestamp &tstamp1, const Timestamp &tstamp2){

    return getLink(fixFrame, destFrame, tstamp2) * getLink(srcFrame, fixFrame, tstamp1);

}

 void TransMem::calculateTransformation(const Path &path, StampedTransformation &stampedTransformation){

    FrameID currentSrcFrameID = path.src;
    StampedTransformation currentTrans;

    // calculate transformation along the path
    for(Link* l : path.links){
        // get the transformation of the current link
        l->transformationAtTimeT(currentSrcFrameID, currentTrans);

       stampedTransformation.rotation = currentTrans.rotation * stampedTransformation.rotation;
       stampedTransformation.translation = currentTrans.rotation * stampedTransformation.translation * currentTrans.rotation.inverted();
       stampedTransformation.translation = stampedTransformation.translation + currentTrans.translation;

       // choose new current frame depending on the direction of the link
       if(l->parent->frameID == currentSrcFrameID)
           currentSrcFrameID = l->child->frameID;
       else
           currentSrcFrameID = l->parent->frameID;
    }

 }

/****************************
 * PATH                     *
 ****************************/

void Path::writeJSON(QJsonObject &json) const {

    QJsonObject sourceObject; sourceObject.insert("frameID", QString::fromStdString(src));

    QJsonArray linkObjects;
    for(Link* l : links){
        QJsonObject linkObject;
        QJsonObject parentObject; parentObject.insert("frameID", QString::fromStdString(l->parent->frameID));
        linkObject.insert("01_parent", parentObject);
        QJsonObject chilObject; parentObject.insert("frameID", QString::fromStdString(l->child->frameID));
        linkObject.insert("02_child", parentObject);
        linkObjects.append(linkObject);
    }

    QJsonObject destinationObject; destinationObject.insert("frameID", QString::fromStdString(dst));


    json.insert("01_source", sourceObject);
    json.insert("02_links", linkObjects);
    json.insert("03_destination", destinationObject);

}
