#include "transmem/transmem.h"

const char* NoSuchLinkFoundException::what() const throw(){

    return std::runtime_error::what();
}

// Public main functions

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QQuaternion &rotation, const QVector3D &translation,
                            const double confidence){

    // Check if rotation quaternion is normalized.
    if( rotation.length() <= 1. - TOLERATED_DEVIATION_ROTATION_NORMAL_CHECK ||
        rotation.length() >= 1. + TOLERATED_DEVIATION_ROTATION_NORMAL_CHECK )
        qWarning() << "Rotation quaternion is not normalized.\n";

    registerLink(srcFrame, destFrame, validTime, rotation, QQuaternion(0, translation.x(), translation.y(), translation.z()), confidence, true);
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QQuaternion &rotation, const QVector3D &translation){

    // Check if rotation quaternion is normalized
    if( rotation.length() <= 1. - TOLERATED_DEVIATION_ROTATION_NORMAL_CHECK ||
        rotation.length() >= 1. + TOLERATED_DEVIATION_ROTATION_NORMAL_CHECK )
        qWarning() << "Rotation quaternion is not normalized.\n";

    registerLink(srcFrame, destFrame, validTime, rotation, QQuaternion(0, translation.x(), translation.y(), translation.z()), 0, false);
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QMatrix4x4 &trans, const double confidence){

    float data[]{trans(0,0),trans(0,1),trans(0,2),
                 trans(1,0),trans(1,1),trans(1,2),
                 trans(2,0),trans(2,1),trans(2,2)};

    QMatrix3x3 rM(data);

    // Check if the rotation matrix is normal (det = 1/-1).
    double det = fabs(data[0]*data[4]*data[8]+data[1]*data[5]*data[6]+data[2]*data[3]*data[7]-
                      data[2]*data[4]*data[6]-data[1]*data[3]*data[8]-data[0]*data[5]*data[7]);

    if(det <= 1 - TOLERATED_DEVIATION_ROTATION_NORMAL_CHECK || det >= 1. + TOLERATED_DEVIATION_ROTATION_NORMAL_CHECK)
        qWarning() << "Rotation Matrix is not normal.\n";

    registerLink(srcFrame, destFrame, validTime, QQuaternion::fromRotationMatrix(rM), QQuaternion(0, trans(0,3), trans(1,3), trans(2,3)), confidence, true);
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QMatrix4x4 &trans) {

   float data[]{trans(0,0),trans(0,1),trans(0,2),
                trans(1,0),trans(1,1),trans(1,2),
                trans(2,0),trans(2,1),trans(2,2)};

   QMatrix3x3 rM(data);

   // Check if the rotation matrix is normal (det = 1/-1).
   double det = fabs(data[0]*data[4]*data[8]+data[1]*data[5]*data[6]+data[2]*data[3]*data[7]-
                     data[2]*data[4]*data[6]-data[1]*data[3]*data[8]-data[0]*data[5]*data[7]);

   if(det <= 1. - TOLERATED_DEVIATION_ROTATION_NORMAL_CHECK || det >= 1. + TOLERATED_DEVIATION_ROTATION_NORMAL_CHECK)
       qWarning() << "Rotation Matrix is not normal.\n";

   registerLink(srcFrame, destFrame, validTime, QQuaternion::fromRotationMatrix(rM), QQuaternion(0, trans(0,3), trans(1,3), trans(2,3)), 0, false);

}

void TransMem::updateLinkConfidence(const FrameID &srcFrame, const FrameID &destFrame, const double confidence){

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

StampedTransformationWithConfidence TransMem::getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &queryTime) const {

    if(srcFrame == destFrame)
        throw std::invalid_argument("Not allowed to query for link if source frame is equal to destination frame.");

    std::string linkID = srcFrame+destFrame;

    Path p{srcFrame, destFrame};

    // Check if the path is already cached.

    auto itr2CachedPaths = cachedPaths.find(linkID);
    // If yes, we load the cached path
    if(itr2CachedPaths != cachedPaths.end())
        p = (*itr2CachedPaths).second;
    // If not, we search for a shortest path between source frame and destination frame and cached it.
    else {
        if(!shortestPath(p))
            throw NoSuchLinkFoundException(srcFrame, destFrame);
        cachedPaths.insert({linkID, p});
    }

    // Calculate transformation along path
    StampedTransformationWithConfidence resultingTransformation;
    resultingTransformation.time = queryTime;
    calculateTransformation(p, resultingTransformation);

    return resultingTransformation;
}

StampedTransformationWithConfidence TransMem::getBestLink(const FrameID &srcFrame, const FrameID &destFrame) const {

    if(srcFrame == destFrame)
        throw std::invalid_argument("Not allowed to insert a link with srcFrame == destFrame.");

    bool recalculation = true;
    std::string linkID = srcFrame+destFrame;

    std::lock_guard<std::recursive_mutex> guard(lock);

    Path p{srcFrame, destFrame};

    // Check if the path is already cached.

    auto itr2BestTransformations = cachedBestTransformations.find(linkID);

    // If yes we check if a recalculation is necessary
    if(itr2BestTransformations != cachedBestTransformations.end()){

        // The link is just recalculated if every link was updated in the mean time
        // within a certain timespan

        Timestamp tstampBestLinkCalc = (*itr2BestTransformations).second.first;

        // Path has at least length one so there always exists a first entry
        Link link = ((Link) p.links.at(0));

        Timestamp earliestUpdate = link.lastTimeUpdated;
        Timestamp latestUpdate = link.lastTimeUpdated;

        for(int indx = 1; indx < p.links.size(); indx++){

            Link link = ((Link) p.links.at(indx));

            if(link.lastTimeUpdated < earliestUpdate)
                earliestUpdate = link.lastTimeUpdated;
            else if(latestUpdate < link.lastTimeUpdated)
                latestUpdate = link.lastTimeUpdated;
        }

        recalculation = (tstampBestLinkCalc < earliestUpdate) && (latestUpdate - earliestUpdate) < std::chrono::milliseconds(250);

    }
    // If not a recalculation is necessary for sure

    // Do the recalculation if either the path was not cached before or every link along
    // the path was updated since the link was added to the cache.
    if(recalculation){

        StampedTransformationWithConfidence stT;
        Path path{srcFrame, destFrame};

        if(!bestLink(path, stT))
               throw NoSuchLinkFoundException(srcFrame, destFrame);

        // Remove old best transformation and insert new one
        cachedBestTransformations.erase(linkID);

        cachedBestTransformations.insert({linkID,{std::chrono::high_resolution_clock::now(), stT}});

        return stT;
    }
    else{
        return (*itr2BestTransformations).second.second;
    }
}

// Public debug functions

void TransMem::dumpAsJSON(const QString &path) const {

    QJsonObject transmemJSONObject;

    std::lock_guard<std::recursive_mutex> guard(lock);

    // pack the transmem object to a JSON object
    writeJSON(transmemJSONObject);

    QDateTime currentTime = QDateTime::currentDateTime();
    QString suffixFilename = "_transmem_dump.json";

    QFile file( path + currentTime.toString("ddMMyy_HHmmss") + suffixFilename);
    if(!file.open(QIODevice::WriteOnly)){
        qDebug() << file.errorString();
        return;
    }

    QJsonDocument saveJSON(transmemJSONObject);
    file.write(saveJSON.toJson());

    file.close();
    if(file.error()){
        qDebug() << file.errorString();
        return;
    }

    return;
}

void TransMem::dumpAsGraphML(const QString &path) const {

   GraphMLWriter writer;

   std::lock_guard<std::recursive_mutex> guard(lock);
   writer.write(path, *this);

}

// Protected functions

bool TransMem::bestLink(Path &path, StampedTransformationWithConfidence &stT) const {

    // Asume the lock is already aquired.

    // Search for shortest path between source frame and  destination frame.
    if(!shortestPath(path))
        return false;

    // Evaluate best point in time.
    calculateBestPointInTime(path, stT.time);

    // Calculate transformation along path.
    calculateTransformation(path, stT);

    return true;
}

void TransMem::calculateTransformation(const Path &path, StampedTransformationWithConfidence &resultT) const {

    // Asume the lock is already aquired.

    FrameID currentSrcFrameID = path.src;

    StampedTransformation currentTrans;

    QQuaternion rotationResult = QQuaternion();
    QQuaternion translationResult = QQuaternion(0,0,0,0);

    double confidenceSum = 0;
    double maxTimeDiff = 0;

    // Calculate transformation along the path.
    for(Link& l : path.links){

        currentTrans.time = resultT.time;

        // Get the transformation of the current link.
        l.transformationAtTimeT(currentSrcFrameID, currentTrans);

       rotationResult = currentTrans.rotation * rotationResult;
       translationResult = currentTrans.rotation * translationResult * currentTrans.rotation.conjugated();
       translationResult = translationResult + currentTrans.translation;

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

    resultT.rotation = rotationResult;
    resultT.translation = QVector3D(translationResult.x(), translationResult.y(), translationResult.z());

    resultT.averageLinkConfidence = confidenceSum / path.links.size();
    resultT.maxDistanceToEntry = maxTimeDiff;
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime,
                            const QQuaternion &rotation, const QQuaternion &translation, const double confidence, const bool updateConfidence){

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

        // Clear the cached paths and best transformations
        cachedPaths.clear();
        cachedBestTransformations.clear();

        links.emplace_back(Link{ptr2SrcFrame, ptr2DstFrame, storageTime, (updateConfidence ? confidence : defaultLinkConfidence)});
        ptr2Link = &links.back();
        ptr2SrcFrame->addLink(ptr2Link);
    }

    // Add the transformation to the link.
    if(!ptr2Link->addTransformation(srcFrame, StampedTransformation{rotation, translation, validTime})){
        qWarning() << "Entry not stored since entry is to old.\n";
        return;
    }

    // Maybe update confidence of the link.
    if(updateConfidence)
        ptr2Link->confidence = confidence;
}

void TransMem::calculateBestPointInTime(Path &path, Timestamp &bestTime) const{

     Timestamp tStampOldest = std::chrono::time_point<std::chrono::high_resolution_clock>::max();
     StampedTransformation stampedTrans;

     /* We search for the best point in time in the timespan between the time when the
      * newest entry was inserted and when the oldest entry was inserted of all the links in the path.

      * We therefore first search for this points and store them in tStampOldest and bestPoint. */
    for(Link& link : path.links){

        link.oldestTransformation(link.parent->frameID, stampedTrans);
        if(stampedTrans.time < tStampOldest)
            tStampOldest = stampedTrans.time;

        link.newestTransformation(link.parent->frameID, stampedTrans);
        if(stampedTrans.time > bestTime)
            bestTime = stampedTrans.time;
     }

     unsigned long best = std::numeric_limits<unsigned long>::max();

     // We then search for the point in time which minimizes the sum of the quadratic distance to the next entry over all links.
     for(Timestamp tStampCurr = bestTime; tStampCurr > tStampOldest; tStampCurr = tStampCurr - std::chrono::milliseconds(RESOLUTION_BEST_TIME_CALCULATION_IN_MS)){

         std::chrono::milliseconds temp(0);
         unsigned long sum = 0;
         for(Link& link: path.links){
            link.distanceToNextClosestEntry(tStampCurr, temp);
            sum += temp.count() * temp.count();
         }

         if(sum < best){
             best = sum;
             bestTime = tStampCurr;
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
    for(Link* link : currPtr2Frame->parents)
        updateDistance(link->parent->frameID, distanceViaCurr + link->weight);

    for(Link* link : currPtr2Frame->children)
        updateDistance(link->child->frameID, distanceViaCurr + link->weight);

   }

    // No path found.
    return false;
}

// Protected functions for debugging

void TransMem::writeJSON(QJsonObject &json) const {

    QJsonArray frameObjects;
    for(std::pair<FrameID, Frame> frameIDframePair: frameID2Frame){
        QJsonObject frameObject;
        frameIDframePair.second.writeJSON(frameObject);
        frameObjects.append(frameObject);
    }

    QJsonArray linkObjects;
    for(Link link: links){
        QJsonObject linkObject;
        link.writeJSON(linkObject);
        linkObjects.append(linkObject);
    }

    json.insert("frames", frameObjects);
    json.insert("links", linkObjects);

}

void Path::writeJSON(QJsonObject &json) const {

    QJsonObject sourceObject; sourceObject.insert("frameID", QString::fromStdString(src));

    QJsonArray linkObjects;
    for(Link& link : links){
        QJsonObject linkObject;
        QJsonObject parentObject; parentObject.insert("frameID", QString::fromStdString(link.parent->frameID));
        linkObject.insert("01_parent", parentObject);
        QJsonObject chilObject; parentObject.insert("frameID", QString::fromStdString(link.child->frameID));
        linkObject.insert("02_child", parentObject);
        linkObjects.append(linkObject);
    }

    QJsonObject destinationObject; destinationObject.insert("frameID", QString::fromStdString(dst));

    json.insert("01_source", sourceObject);
    json.insert("02_links", linkObjects);
    json.insert("03_destination", destinationObject);

}
