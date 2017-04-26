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
    auto IterSrcF = frameID2Frame.find(srcFrame);
    auto IterDstF = frameID2Frame.find(destFrame);

    Frame* sFPtr; Frame* dFPtr;

    // if a frame does not exist, create it
    if(IterSrcF == frameID2Frame.end()){

        frames.emplace_back(Frame{srcFrame});
        sFPtr = &frames.back();
        frameID2Frame.insert({srcFrame, sFPtr});

    }
    else
        sFPtr = (*IterSrcF).second;

    if(IterDstF == frameID2Frame.end()){

        frames.emplace_back(Frame{destFrame});
        dFPtr = &frames.back();
        frameID2Frame.insert({destFrame, dFPtr});

    }
    else
        dFPtr = (*IterDstF).second;

    // check if the link exists
    Link* lnkPtr = nullptr;
    sFPtr->connectionTo(destFrame, lnkPtr);

    // if the link does not exist, create it
    if(lnkPtr == nullptr){

        links.emplace_back(Link{sFPtr, dFPtr, storageTime});
        lnkPtr = &links.back();
        sFPtr->addLink(lnkPtr);
    }

    // add the transformation to the link
    lnkPtr->addTransformation(srcFrame, StampedTransformation{tstamp, qrot, qtrans});

    // release the lock
    lock.unlock();

    return;
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans) {

   float data[]{trans(0,0),trans(0,1),trans(0,2),trans(1,0),trans(1,1),trans(1,2),trans(2,0),trans(2,1),trans(2,2)};
   QMatrix3x3 rM(data);

   /*
   auto det = [](const QMatrix3x3 &m){
      return m(0,0)*(m(1,1)*m(2,2)-m(1,2)*m(2,1)) -
             m(0,1)*(m(1,0)*m(2,2)-m(1,2)*m(2,0)) +
             m(0,2)*(m(1,0)*m(2,1)-m(1,1)*m(2,0));
   };

   if( std::fabs(det(rM)-1.) > 1e-06)
       std::cout << "warning: rotation matrix not normal\n";

   */
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

void TransMem::shortestPath(Path &p){

    // check if the source frame exists
    if(frameID2Frame.find(p.src) == frameID2Frame.end())
      throw NoSuchLinkFoundException(p.src, p.dst);

    // check if the dest frame exists
    auto iterD = frameID2Frame.find(p.dst);
    if(iterD == frameID2Frame.end())
        throw NoSuchLinkFoundException(p.src, p.dst);

    Frame* currFrame = (*iterD).second;

    // initialize for dikstra
    // set the distance of all frames to infinity and the predecessor to null
    auto iter = frameID2Frame.begin();
    while(iter != frameID2Frame.end()){
        Frame* f = (*iter).second;
        f->distance = std::numeric_limits<double>::infinity();
        f->predecessor = nullptr;
        f->active = true;
        iter++;
    }
    // set the distance of the src frame to zero
        currFrame->distance = 0.;
        currFrame->active = false;

    // helper lambda's
    auto updateDistance = [](Frame* cu, Frame* ne, double w){
        double alternativeDist = cu->distance + w;
        if(alternativeDist < ne->distance){
            ne->distance = alternativeDist;
            ne->predecessor = cu;
            return;
        }
    };
    auto getShortest = [this](){
        auto iter = frameID2Frame.begin();
        double minDist = std::numeric_limits<double>::infinity();
        Frame* ret = nullptr;
        while(iter != frameID2Frame.end()){
            Frame* cur = (*iter).second;
            if(cur->distance < minDist && cur->active)
                ret = cur;
            iter++;
        }
        return ret;
    };

    // run dikstra
    while(currFrame->frameID != p.src){
        for(Link* l: currFrame->parents)
            if(l->parent->active)
                updateDistance(currFrame, l->parent, l->weight);
        for(Link* l: currFrame->children)
            if(l->child->active)
                updateDistance(currFrame, l->child, l->weight);
    currFrame = getShortest();

    // no path exists
    if(currFrame == nullptr)
        throw NoSuchLinkFoundException(p.src, p.dst);

    currFrame->active = false;
    }

    // create shortest path
    Link* currL;

    // NOTE: assertion just during development
    // there should always be at least one predecessor
    // since if there is a path, the path is at least of length one
    assert(currFrame->predecessor != nullptr);

    currFrame->connectionTo(currFrame->predecessor->frameID, currL); p.links.push_back(currL);
    currFrame = currFrame->predecessor;
    while(currFrame->predecessor != nullptr){
        currFrame->connectionTo(currFrame->predecessor->frameID, currL); p.links.push_back(currL);
        currFrame = currFrame->predecessor;
    }

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

    // NOTE: debugging just during development
    // debugging, dump path as json file
    dumpPathAsJSON(p);

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

 void TransMem::calculateTransformation(const Path &p, StampedTransformation &e){

    FrameID currentSrcFrameID = p.src;
    StampedTransformation currentTrans;
    for(Link* l : p.links){
        l->transformationAtTimeT(currentSrcFrameID, currentTrans);
       e.rotation = currentTrans.rotation * e.rotation;
       e.translation = e.rotation * e.translation * e.rotation.inverted();
       e.translation = e.translation + currentTrans.translation;
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
