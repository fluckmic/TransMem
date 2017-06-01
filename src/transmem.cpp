/*! \file   transmem.cpp
    \brief  source file transmem.cpp
*/

#include "transmem/transmem.h"

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

    std::lock_guard<std::recursive_mutex> guard(lock);

    // check if frames already exist
    auto iter2SrcFrame = frameID2Frame.find(srcFrame);
    auto iter2DstFrame = frameID2Frame.find(destFrame);

    Frame* ptr2SrcFrame; Frame* ptr2DstFrame;

    // if a frame does not exist, create it
    if(iter2SrcFrame == frameID2Frame.end()){

        frameID2Frame.insert({srcFrame, Frame{srcFrame}});
        ptr2SrcFrame = &(*frameID2Frame.find(srcFrame)).second;

    }
    else
        ptr2SrcFrame = &(*iter2SrcFrame).second;

    if(iter2DstFrame == frameID2Frame.end()){

        frameID2Frame.insert({destFrame, Frame{destFrame}});
        ptr2DstFrame = &(*frameID2Frame.find(destFrame)).second;

    }
    else
        ptr2DstFrame = &(*iter2DstFrame).second;

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
    if(!ptr2Link->addTransformation(srcFrame, StampedTransformation{tstamp, qrot, qtrans})){
        // TODO debug message: entry not stored since entry is to old
    }

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
    for(auto f: frameID2Frame){
        QJsonObject frameObject;
        f.second.writeJSON(frameObject);
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

bool TransMem::shortestPath(Path &path) const {

    // TODO:
    // rework calculation of shortest path.
    return false;

}

void TransMem::dumpAsJSON() const {

    // TODO: add date and time to filename of dump

    QJsonObject transmemObject;

    std::lock_guard<std::recursive_mutex> guard(lock);

    writeJSON(transmemObject);

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

void TransMem::dumpPathAsJSON(const Path &p) const{

    // TODO:: add date and time to filname of dump

   QJsonObject pathObject;

   std::lock_guard<std::recursive_mutex> guard(lock);
   p.writeJSON(pathObject);

   dumpJSONfile("PathDump", pathObject);

}

void TransMem::dumpAsGraphML() const {

   GraphMLWriter writer;

   std::lock_guard<std::recursive_mutex> guard(lock);
   writer.write(*this);

}

QMatrix4x4 TransMem::getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp) const {

    if(srcFrame == destFrame)
        throw std::invalid_argument("Not allowed to query for link if source frame is equal to destination frame.");

    std::lock_guard<std::recursive_mutex> guard(lock);

    // search for shortest path between source frame and  destination frame
    Path p{srcFrame, destFrame};

    if(!shortestPath(p))
        throw NoSuchLinkFoundException(srcFrame, destFrame);

    dumpPathAsJSON(p);

    dumpAsGraphML();

    // calculate transformation along path
    StampedTransformation t{tstamp, QQuaternion(), QQuaternion(0,0,0,0)};
    calculateTransformation(p, t);

    // convert to QMatrix4x4
    QMatrix3x3 rot = t.rotation.toRotationMatrix();
    QMatrix4x4 ret(rot);
    ret(0,3) = t.translation.x(); ret(1,3) = t.translation.y(); ret(2,3) = t.translation.z();

    return ret;

}

QMatrix4x4 TransMem::getLink(const FrameID &srcFrame, const FrameID &fixFrame, const FrameID &destFrame, const Timestamp &tstamp1, const Timestamp &tstamp2) const{

    return getLink(fixFrame, destFrame, tstamp2) * getLink(srcFrame, fixFrame, tstamp1);

}

QMatrix4x4 TransMem::getBestLink(const FrameID &srcFrame, const FrameID &destFrame, Timestamp &tstamp) const {

    if(srcFrame == destFrame)
        throw std::invalid_argument("Not allowed to query for link if source frame is equal to destination frame.");

    std::lock_guard<std::recursive_mutex> guard(lock);

    // search for shortest path between source frame and  destination frame
    Path p{srcFrame, destFrame};
    if(!shortestPath(p))
        throw NoSuchLinkFoundException(srcFrame, destFrame);

    // evaluate best point in time
    calculateBestPointInTime(p, tstamp);

    // calculate transformation along path
    StampedTransformation t{tstamp, QQuaternion(), QQuaternion(0,0,0,0)};
    calculateTransformation(p, t);

    // convert to QMatrix4x4
    QMatrix3x3 rot = t.rotation.toRotationMatrix();
    QMatrix4x4 ret(rot);
    ret(0,3) = t.translation.x(); ret(1,3) = t.translation.y(); ret(2,3) = t.translation.z();

    return ret;

}

 bool TransMem::calculateTransformation(const Path &path, StampedTransformation &stampedTransformation) const {

    // we asume the thread already holds the lock.

    FrameID currentSrcFrameID = path.src;
    StampedTransformation currentTrans;

    currentTrans.time = stampedTransformation.time;

    // calculate transformation along the path
    for(Link& l : path.links){
        // get the transformation of the current link
        l.transformationAtTimeT(currentSrcFrameID, currentTrans);

       stampedTransformation.rotation = currentTrans.rotation * stampedTransformation.rotation;
       stampedTransformation.translation = currentTrans.rotation * stampedTransformation.translation * currentTrans.rotation.inverted();
       stampedTransformation.translation = stampedTransformation.translation + currentTrans.translation;

       // choose new current frame depending on the direction of the link
       if(l.parent->frameID == currentSrcFrameID)
           currentSrcFrameID = l.child->frameID;
       else
           currentSrcFrameID = l.parent->frameID;
    }
     return true;
 }

 bool TransMem::calculateBestPointInTime(Path &path, Timestamp &tStampBestPoinInTime) const{

     // we search for the best transformation in the timespan between the time when the
     // newest entry was inserted and when the oldest entry was inserted of all the links in the path

     Timestamp tStampOldest = std::chrono::time_point<std::chrono::high_resolution_clock>::max();
     StampedTransformation stampedTrans;

     for(Link& l : path.links){

        l.oldestTransformation(l.parent->frameID, stampedTrans);
        if(stampedTrans.time < tStampOldest)
            tStampOldest = stampedTrans.time;

        l.newestTransformation(l.parent->frameID, stampedTrans);
        if(stampedTrans.time > tStampBestPoinInTime)
            tStampBestPoinInTime = stampedTrans.time;
     }

     unsigned long best = std::numeric_limits<unsigned long>::max();

     for(Timestamp tStampCurr = tStampBestPoinInTime; tStampCurr > tStampOldest; tStampCurr = tStampCurr - std::chrono::milliseconds(5)){

         std::chrono::milliseconds temp(0);
         unsigned long sum = 0;
         for(Link &l: path.links){
            l.distanceToNextClosestEntry(tStampCurr, temp);
            sum += temp.count() * temp.count();
         }

         if(sum < best){
             best = sum;
             tStampBestPoinInTime = tStampCurr;
         }
     }

    return true;
 }

/****************************
 * PATH                     *
 ****************************/

void Path::writeJSON(QJsonObject &json) const {

    QJsonObject sourceObject; sourceObject.insert("frameID", QString::fromStdString(src));

    QJsonArray linkObjects;
    for(Link l : links){
        QJsonObject linkObject;
        QJsonObject parentObject; parentObject.insert("frameID", QString::fromStdString(l.parent->frameID));
        linkObject.insert("01_parent", parentObject);
        QJsonObject chilObject; parentObject.insert("frameID", QString::fromStdString(l.child->frameID));
        linkObject.insert("02_child", parentObject);
        linkObjects.append(linkObject);
    }

    QJsonObject destinationObject; destinationObject.insert("frameID", QString::fromStdString(dst));


    json.insert("01_source", sourceObject);
    json.insert("02_links", linkObjects);
    json.insert("03_destination", destinationObject);

}
