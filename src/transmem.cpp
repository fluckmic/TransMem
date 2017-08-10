/*! \file   transmem.cpp
    \brief  source file transmem.cpp
*/

#include "transmem/transmem.h"

/********************************
 * NO SUCH LINK FOUND EXCEPTION *
 ********************************/

const char* NoSuchLinkFoundException::what() const throw(){

    return std::runtime_error::what();
}

/************
 * TRANSMEM *
 ************/

// Public main functions

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QQuaternion &qrot, const QQuaternion &qtrans,
                            const double &confidence){

    // Check if rotation quaternion is normalized.
    if( qrot.length() < 0.995 || qrot.length() > 1.005)
        qWarning() << "Rotation quaternion is not normalized.\n";

    // Check if translation quaternion is pure.
    if(qtrans.scalar() != 0.)
        qWarning() << "Translation quaternion is not pure.\n";

    registerLink(srcFrame, destFrame, tstamp, qrot, qtrans, confidence, true);
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QQuaternion &qrot, const QQuaternion &qtrans){

    // Check if rotation quaternion is normalized
    if( qrot.length() < 0.995 || qrot.length() > 1.005)
        qWarning() << "Rotation quaternion is not normalized.\n";

    // Check if translation quaternion is pure
    if(qtrans.scalar() != 0.)
        qWarning() << "Translation quaternion is not pure.\n";

    registerLink(srcFrame, destFrame, tstamp, qrot, qtrans, 0, false);
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans, const double &confidence){

    float data[]{trans(0,0),trans(0,1),trans(0,2),
                 trans(1,0),trans(1,1),trans(1,2),
                 trans(2,0),trans(2,1),trans(2,2)};

    QMatrix3x3 rM(data);

    // Check if the rotation matrix is normal (det = 1/-1).
    double det = fabs(data[0]*data[4]*data[8]+data[1]*data[5]*data[6]+data[2]*data[3]*data[7]-
                      data[2]*data[4]*data[6]-data[1]*data[3]*data[8]-data[0]*data[5]*data[7]);

    if(det < 0.995 || det > 1.005)
        qWarning() << "Rotation Matrix is not normal.\n";

    registerLink(srcFrame, destFrame, tstamp, QQuaternion::fromRotationMatrix(rM), QQuaternion(0, trans(0,3), trans(1,3), trans(2,3)), confidence, true);
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans) {

   float data[]{trans(0,0),trans(0,1),trans(0,2),
                trans(1,0),trans(1,1),trans(1,2),
                trans(2,0),trans(2,1),trans(2,2)};

   QMatrix3x3 rM(data);

   // Check if the rotation matrix is normal (det = 1/-1).
   double det = fabs(data[0]*data[4]*data[8]+data[1]*data[5]*data[6]+data[2]*data[3]*data[7]-
                     data[2]*data[4]*data[6]-data[1]*data[3]*data[8]-data[0]*data[5]*data[7]);

   if(det < 0.995 || det > 1.005)
       qWarning() << "Rotation Matrix is not normal.\n";

   registerLink(srcFrame, destFrame, tstamp, QQuaternion::fromRotationMatrix(rM), QQuaternion(0, trans(0,3), trans(1,3), trans(2,3)), 0, false);

}

void TransMem::updateLinkConfidence(const FrameID &srcFrame, const FrameID &destFrame, const double &confidence){

    if(srcFrame == destFrame)
        throw std::invalid_argument("Cannot update a link where srcFrame == destFrame.");

    std::lock_guard<std::recursive_mutex> guard(lock);

    // Check if frames already exist.
    auto iter2SrcFrame = frameID2Frame.find(srcFrame);
    auto iter2DstFrame = frameID2Frame.find(destFrame);

    Frame* ptr2SrcFrame;

    // Just can update links for which already a transformation was registered.
    if(iter2SrcFrame == frameID2Frame.end() || iter2DstFrame == frameID2Frame.end())
        throw NoSuchLinkFoundException(srcFrame, destFrame);

    ptr2SrcFrame = &(*iter2SrcFrame).second;

    // Check if a link between srcFrame and destFrame exists.
    Link* ptr2Link = nullptr;
    ptr2SrcFrame->connectionTo(destFrame, ptr2Link);

    // This would actually be very strange..
    if(ptr2Link == nullptr)
        throw NoSuchLinkFoundException(srcFrame, destFrame);

    ptr2Link->confidence = confidence;
}

StampedAndRatedTransformation TransMem::getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp) const {

    if(srcFrame == destFrame)
        throw std::invalid_argument("Not allowed to query for link if source frame is equal to destination frame.");

    // Search for shortest path between source frame and  destination frame.
    Path p{srcFrame, destFrame};

    if(!shortestPath(p))
        throw NoSuchLinkFoundException(srcFrame, destFrame);

    // Calculate transformation along path
    StampedAndRatedTransformation resultingTransformation;
    resultingTransformation.time = tstamp;
    calculateTransformation(p, resultingTransformation);

    return resultingTransformation;
}

StampedAndRatedTransformation TransMem::getBestLink(const FrameID &srcFrame, const FrameID &destFrame) {

    if(srcFrame == destFrame)
        throw std::invalid_argument("Not allowed to insert a link with srcFrame == destFrame.");

    bool recalculation = true;
    std::string linkID = srcFrame+destFrame;

    std::lock_guard<std::recursive_mutex> guard(lock);

    // Check if link is already cached.
    auto itr2CachedBestLink = cachedBestLinks.find(linkID);
    if(itr2CachedBestLink != cachedBestLinks.end()){

    Timestamp tstampBestLinkCalc = (*itr2CachedBestLink).second.first;
    Path pathBestLink = (*itr2CachedBestLink).second.second;

    // The link is just recalculated if every link was updated in the mean time.
       for(Link& l : pathBestLink.links)
           recalculation = recalculation && ( tstampBestLinkCalc < l.lastTimeUpdated);
   }

    // Do the recalculation if either the link was not cached before or every link along
    // the path was updated since the link was added to the cache.
    if(recalculation){

        StampedAndRatedTransformation stT;
        Path path{srcFrame, destFrame};

        if(!bestLink(stT, path))
               throw NoSuchLinkFoundException(srcFrame, destFrame);

        // Remove old entries.
        cachedBestLinks.erase(linkID);
        cachedBestTransformations.erase(linkID);

        cachedBestLinks.insert({linkID,{std::chrono::high_resolution_clock::now(), path}});
        cachedBestTransformations.insert({linkID, stT});

        return stT;
    }
    else{
        return cachedBestTransformations.at(linkID);
    }
}

// Public debug functions

void TransMem::dumpAsJSON() const {

    QString path = "";
    QJsonObject transmemObject;

    std::lock_guard<std::recursive_mutex> guard(lock);

    writeJSON(transmemObject);

    dumpJSONfile(path, transmemObject, OutputType::TRANSMEM);

    return;
}

void TransMem::dumpAsGraphML() const {

   GraphMLWriter writer;
   QString path = "";

   std::lock_guard<std::recursive_mutex> guard(lock);
   writer.write(path, *this);

}

// Protected functions

bool TransMem::bestLink(StampedAndRatedTransformation &stT, Path &p) const {

    // Asume the lock is already aquired.

    // Search for shortest path between source frame and  destination frame.
    if(!shortestPath(p))
        return false;

    // Evaluate best point in time.
    calculateBestPointInTime(p, stT.time);

    // Calculate transformation along path.
    calculateTransformation(p, stT);

    return true;
}

void TransMem::calculateTransformation(const Path &path, StampedAndRatedTransformation &resultT) const {

    // Asume the lock is already aquired.

    FrameID currentSrcFrameID = path.src;

    StampedTransformation currentTrans;

    resultT.qRot = QQuaternion();
    resultT.qTra = QQuaternion(0,0,0,0);

    double confidenceSum = 0;
    double maxTimeDiff = 0;

    // Calculate transformation along the path.
    for(Link& l : path.links){

        currentTrans.time = resultT.time;

        // Get the transformation of the current link.
        l.transformationAtTimeT(currentSrcFrameID, currentTrans);

       resultT.qRot = currentTrans.rotation * resultT.qRot;
       resultT.qTra = currentTrans.rotation * resultT.qTra * currentTrans.rotation.conjugated();
       resultT.qTra = resultT.qTra + currentTrans.translation;

       // Sum up the confidence of all the links.
       confidenceSum += l.confidence;

       // Sum up the mapped time difference of all the links.
       maxTimeDiff = std::max(maxTimeDiff,
                              distanceToEntryMapping(
                                fabs(((std::chrono::milliseconds)
                                   std::chrono::duration_cast<std::chrono::milliseconds>(resultT.time - currentTrans.time)).count())));

       // Choose new current frame depending on the direction of the link.
       if(l.parent->frameID == currentSrcFrameID)
           currentSrcFrameID = l.child->frameID;
       else
           currentSrcFrameID = l.parent->frameID;
    }

    resultT.avgLinkConfidence = confidenceSum / path.links.size();
    resultT.maxDistanceToEntry = maxTimeDiff;
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp,
                            const QQuaternion &qrot, const QQuaternion &qtrans, const double &confidence, const bool &updateConfidence){

    if(srcFrame == destFrame)
        throw std::invalid_argument("Not allowed to insert a link with srcFrame == destFrame.");

    std::lock_guard<std::recursive_mutex> guard(lock);

    // Check if frames already exist.
    auto iter2SrcFrame = frameID2Frame.find(srcFrame);
    auto iter2DstFrame = frameID2Frame.find(destFrame);

    Frame* ptr2SrcFrame; Frame* ptr2DstFrame;

    // If a frame does not exist, create it.
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

    // Check if a link between srcFrame and destFrame exists.
    Link* ptr2Link = nullptr;
    ptr2SrcFrame->connectionTo(destFrame, ptr2Link);

    // If the link does not exist, create it.
    if(ptr2Link == nullptr){

        links.emplace_back(Link{ptr2SrcFrame, ptr2DstFrame, storageTime, (updateConfidence ? confidence : defaultLinkConfidence)});
        ptr2Link = &links.back();
        ptr2SrcFrame->addLink(ptr2Link);
    }

    // Add the transformation to the link.
    if(!ptr2Link->addTransformation(srcFrame, StampedTransformation{qrot, qtrans, tstamp})){
        qWarning() << "Entry not stored since entry is to old.\n";
        return;
    }

    // Maybe update confidence of the link.
    if(updateConfidence)
        ptr2Link->confidence = confidence;
}

void TransMem::calculateBestPointInTime(Path &path, Timestamp &bestPoint) const{

     Timestamp tStampOldest = std::chrono::time_point<std::chrono::high_resolution_clock>::max();
     StampedTransformation stampedTrans;

     /* We search for the best point in time in the timespan between the time when the
      * newest entry was inserted and when the oldest entry was inserted of all the links in the path.

      * We therefore first search for this points and store them in tStampOldest and bestPoint. */
    for(Link& l : path.links){

        l.oldestTransformation(l.parent->frameID, stampedTrans);
        if(stampedTrans.time < tStampOldest)
            tStampOldest = stampedTrans.time;

        l.newestTransformation(l.parent->frameID, stampedTrans);
        if(stampedTrans.time > bestPoint)
            bestPoint = stampedTrans.time;
     }

     unsigned long best = std::numeric_limits<unsigned long>::max();

     // We then search for the point in time which minimizes the sum of the quadratic distance to the next entry over all links.
     for(Timestamp tStampCurr = bestPoint; tStampCurr > tStampOldest; tStampCurr = tStampCurr - std::chrono::milliseconds(5)){

         std::chrono::milliseconds temp(0);
         unsigned long sum = 0;
         for(Link &l: path.links){
            l.distanceToNextClosestEntry(tStampCurr, temp);
            sum += temp.count() * temp.count();
         }

         if(sum < best){
             best = sum;
             bestPoint = tStampCurr;
         }
     }
 }

bool TransMem::shortestPath(Path &path) const {

    // Check if the source frame exists.
    if(frameID2Frame.find(path.src) == frameID2Frame.end())
        return false;

    // Check if the destination frame exists.
    auto iter2DstFrame = frameID2Frame.find(path.dst);
    if(iter2DstFrame == frameID2Frame.end())
        return false;

   typedef std::pair<double, Frame*> distAndFramePtrPair;

   std::priority_queue< distAndFramePtrPair, std::vector<distAndFramePtrPair>, std::greater<distAndFramePtrPair> > prQ;

   std::unordered_map< FrameID, double > distances;
   std::unordered_map< FrameID, Frame* > predecessors;

   // Initialize temporary datastructures.
   for(std::pair<FrameID, Frame> f : frameID2Frame){
       distances.insert({f.first, std::numeric_limits<double>::infinity()});
       predecessors.insert({f.first, nullptr});
   }

   // Insert destination into priority queue and set distance to zero and predecessor to null.
   prQ.emplace(distAndFramePtrPair{0, (Frame*) (&(*frameID2Frame.find(path.dst)).second)});
   distances.at(path.dst) = 0;

   // Search shortest path
   while(!prQ.empty()){

    Frame* currPtr2Frame = prQ.top().second;
    double distanceViaCurr = prQ.top().first;

    // We found the shortest path.
    if(currPtr2Frame->frameID == path.src){

        // Path has at least one link, since it is not possible to query
        // for a transformation between the same frame
        FrameID frameIDPred = predecessors.at(path.src)->frameID;
        Link* link2Pred = nullptr;
        currPtr2Frame->connectionTo(frameIDPred, link2Pred);
        path.links.push_back(std::ref(*link2Pred));

        while(true){

            currPtr2Frame = predecessors.at(currPtr2Frame->frameID);

            if(!predecessors.at(currPtr2Frame->frameID))
                return true;    // path complete

            frameIDPred = predecessors.at(currPtr2Frame->frameID)->frameID;
            currPtr2Frame->connectionTo(frameIDPred, link2Pred);
            path.links.push_back(std::ref(*link2Pred));

        }

        return true;
    }

    prQ.pop();

    // Helper lambda
    auto updateDistance = [this, &prQ, &distances, &predecessors, &currPtr2Frame](FrameID adjFrameID, double alternativeDist){
        if(alternativeDist < distances.at(adjFrameID)){
            distances.at(adjFrameID) = alternativeDist;
            predecessors.at(adjFrameID) = currPtr2Frame;
            prQ.emplace(distAndFramePtrPair{alternativeDist, (Frame*) (&(*frameID2Frame.find(adjFrameID)).second)});
        }
    };

    // Update distances for each link.
    for(Link* l : currPtr2Frame->parents)
        updateDistance(l->parent->frameID, distanceViaCurr + l->weight);

    for(Link* l : currPtr2Frame->children)
        updateDistance(l->child->frameID, distanceViaCurr + l->weight);

   }

    // No path found.
    return false;
}

// Protected functions for debugging

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

void TransMem::dumpPathAsJSON(const Path &p) const{

   QString path = "";
   QJsonObject pathObject;

   std::lock_guard<std::recursive_mutex> guard(lock);
   p.writeJSON(pathObject);

   dumpJSONfile(path, pathObject, OutputType::PATH);

}

void TransMem::dumpJSONfile(const QString &path, const QJsonObject &json, const OutputType &outputType) const {

    QDateTime currentTime = QDateTime::currentDateTime();
    QString suffixFilename;

    switch(outputType){
        case OutputType::PATH:          suffixFilename = "_path_dump.json"; break;
        case OutputType::TRANSMEM:      suffixFilename = "_transmem_dump.json"; break;
    }

    QFile file( path + currentTime.toString("ddMMyy_HHmmss") + suffixFilename);
    if(!file.open(QIODevice::WriteOnly)){
        qDebug() << file.errorString();
        return;
    }

    QJsonDocument saveJSON(json);
    file.write(saveJSON.toJson());

    file.close();
    if(file.error()){
        qDebug() << file.errorString();
        return;
    }
}

/********
 * PATH *
 ********/

void Path::writeJSON(QJsonObject &json) const {

    QJsonObject sourceObject; sourceObject.insert("frameID", QString::fromStdString(src));

    QJsonArray linkObjects;
    for(Link& l : links){
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
