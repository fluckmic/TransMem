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

    // check if rotation quaternion is normalized
    if( qrot.length() < 0.995 || qrot.length() > 1.005)
        qWarning() << "Rotation quaternion is not normalized.\n";

    // check if translation quaternion is pure
    if(qtrans.scalar() != 0.)
        qWarning() << "Translation quaternion is not pure.\n";

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
        qWarning() << "Entry not stored since entry is to old.\n";
    }

    //dumpAsJSON();

    return;
}

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans) {

   float data[]{trans(0,0),trans(0,1),trans(0,2),
                trans(1,0),trans(1,1),trans(1,2),
                trans(2,0),trans(2,1),trans(2,2)};

   QMatrix3x3 rM(data);

   // check if the rotation matrix is normal (det = 1/-1)
   double det = fabs(data[0]*data[4]*data[8]+data[1]*data[5]*data[6]+data[2]*data[3]*data[7]-
                     data[2]*data[4]*data[6]-data[1]*data[3]*data[8]-data[0]*data[5]*data[7]);

   if(det < 0.995 || det > 1.005)
       qWarning() << "Rotation Matrix is not normal.\n";

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

    // check if the source frame exists
    if(frameID2Frame.find(path.src) == frameID2Frame.end())
        return false;

    // check if the destination frame exists
    auto iter2DstFrame = frameID2Frame.find(path.dst);
    if(iter2DstFrame == frameID2Frame.end())
        return false;

   typedef std::pair<double, Frame*> distAndFramePtrPair;

   std::priority_queue< distAndFramePtrPair, std::vector<distAndFramePtrPair>, std::greater<distAndFramePtrPair> > prQ;

   std::unordered_map< FrameID, double > distances;
   std::unordered_map< FrameID, Frame* > predecessors;

   // initialize temporary datastructures
   for(std::pair<FrameID, Frame> f : frameID2Frame){
       distances.insert({f.first, std::numeric_limits<double>::infinity()});
       predecessors.insert({f.first, nullptr});
   }

   // insert destination into priority queue and set distance to zero / predecessor to null
   prQ.emplace(distAndFramePtrPair{0, (Frame*) (&(*frameID2Frame.find(path.dst)).second)});
   distances.at(path.dst) = 0;

   // search shortest path
   while(!prQ.empty()){

    Frame* currPtr2Frame = prQ.top().second;
    double distanceViaCurr = prQ.top().first;

    // we found the shortest path
    if(currPtr2Frame->frameID == path.src){

        // path has at least one link, since it is not possible to query
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

    /*******************/
    // helper lambda

    auto updateDistance = [this, &prQ, &distances, &predecessors, &currPtr2Frame](FrameID adjFrameID, double alternativeDist){
        if(alternativeDist < distances.at(adjFrameID)){
            distances.at(adjFrameID) = alternativeDist;
            predecessors.at(adjFrameID) = currPtr2Frame;
            prQ.emplace(distAndFramePtrPair{alternativeDist, (Frame*) (&(*frameID2Frame.find(adjFrameID)).second)});
        }
    };
    /*******************/

    // update distances
    for(Link* l : currPtr2Frame->parents)
        updateDistance(l->parent->frameID, distanceViaCurr + l->weight);

    for(Link* l : currPtr2Frame->children)
        updateDistance(l->child->frameID, distanceViaCurr + l->weight);

   }

    // no path found
    return false;

}

void TransMem::dumpAsJSON() const {

    QString path = "";
    QJsonObject transmemObject;

    std::lock_guard<std::recursive_mutex> guard(lock);

    writeJSON(transmemObject);

    dumpJSONfile(path, transmemObject, OutputType::TRANSMEM);

    return;
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

void TransMem::dumpPathAsJSON(const Path &p) const{

   QString path = "";
   QJsonObject pathObject;

   std::lock_guard<std::recursive_mutex> guard(lock);
   p.writeJSON(pathObject);

   dumpJSONfile(path, pathObject, OutputType::PATH);

}

void TransMem::dumpAsGraphML() const {

   GraphMLWriter writer;
   QString path = "";

   std::lock_guard<std::recursive_mutex> guard(lock);
   writer.write(path, *this);

}

QMatrix4x4 TransMem::getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp) const {

    if(srcFrame == destFrame)
        throw std::invalid_argument("Not allowed to query for link if source frame is equal to destination frame.");

    // search for shortest path between source frame and  destination frame
    Path p{srcFrame, destFrame};

    if(!shortestPath(p))
        throw NoSuchLinkFoundException(srcFrame, destFrame);

    //dumpPathAsJSON(p);
    //dumpAsGraphML();

    // calculate transformation along path
    StampedTransformation t{tstamp, QQuaternion(), QQuaternion(0,0,0,0)};
    calculateTransformation(p, t);

    // convert to QMatrix4x4
    QMatrix3x3 rot = t.rotation.toRotationMatrix();
    QMatrix4x4 ret(rot);
    ret(0,3) = t.translation.x(); ret(1,3) = t.translation.y(); ret(2,3) = t.translation.z();

    return ret;

}

// WARNING: TEST ME!
QMatrix4x4 TransMem::getLink(const FrameID &srcFrame, const FrameID &fixFrame, const FrameID &destFrame, const Timestamp &tstamp1, const Timestamp &tstamp2) const{

    return getLink(fixFrame, destFrame, tstamp2) * getLink(srcFrame, fixFrame, tstamp1);

}

QMatrix4x4 TransMem::getBestLink(const FrameID &srcFrame, const FrameID &destFrame, Timestamp &tstamp) const {

    if(srcFrame == destFrame)
        throw std::invalid_argument("Not allowed to query for link if source frame is equal to destination frame.");

    QMatrix4x4 trans;
    Path p{srcFrame, destFrame};

    std::lock_guard<std::recursive_mutex> guard(lock);

    if(!bestLink(trans, tstamp, p))
        throw NoSuchLinkFoundException(srcFrame, destFrame);

    return trans;
}

bool TransMem::bestLink(QMatrix4x4 &trans, Timestamp &tstamp, Path &p) const {

    // we asume the lock is already aquired

    // search for shortest path between source frame and  destination frame
    if(!shortestPath(p))
        return false;

    // evaluate best point in time
    calculateBestPointInTime(p, tstamp);

    // calculate transformation along path
    StampedTransformation t{tstamp, QQuaternion(), QQuaternion(0,0,0,0)};
    calculateTransformation(p, t);

    // convert to QMatrix4x4
    QMatrix3x3 rot = t.rotation.toRotationMatrix();
    QMatrix4x4 ret(rot);
    ret(0,3) = t.translation.x(); ret(1,3) = t.translation.y(); ret(2,3) = t.translation.z();

    trans = ret;

    return true;
}

QMatrix4x4 TransMem::getBestLinkCached(const FrameID &srcFrame, const FrameID &destFrame, Timestamp &tstamp) {

    std::string linkID = srcFrame+destFrame;

    std::lock_guard<std::recursive_mutex> guard(lock);

    auto itr2CachedBestLink = cachedBestLinks.find(linkID);

    auto itr2CachedBestLinkRev = cachedBestLinks.find(destFrame+srcFrame);

    // if the best link is cached but in the other direction we call the method again and invert its result
    if(itr2CachedBestLinkRev != cachedBestLinks.end())
        return getBestLinkCached(destFrame, srcFrame, tstamp).inverted();

    // otherwise we check if recalculation is necessary
    bool recalculation = true;

    // the link is already cached, check if an update is necessary
    if(itr2CachedBestLink != cachedBestLinks.end()){

    Timestamp tstampBestLinkCalc = (*itr2CachedBestLink).second.first;
    Path pathBestLink = (*itr2CachedBestLink).second.second;

    // the link is just recalculated if every link was updated in the mean time
    for(Link& l : pathBestLink.links)
        recalculation = recalculation && (l.lastTimeUpdated > tstampBestLinkCalc);
    }

    // do the recalculation
    if(recalculation){
        Path path{srcFrame, destFrame};
        QMatrix4x4 newTransformation;

        if(!bestLink(newTransformation, tstamp, path))
               throw NoSuchLinkFoundException(srcFrame, destFrame);

        cachedBestLinks.erase(linkID);
        cachedBestTransformations.erase(linkID);

        cachedBestLinks.insert({linkID,{std::chrono::high_resolution_clock::now(), path}});
        cachedBestTransformations.insert({linkID, newTransformation});

        return newTransformation;
    }
    else{
        return cachedBestTransformations.at(linkID);
    }
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

/****************************
 * TRANSMEM QML INTERFACE   *
 ****************************/

void TransMemQMLInterface::registerLinkNow(const QString& srcFrame, const QString& dstFrame, const QMatrix4x4& trans) {
    registerLink(srcFrame.toStdString(), dstFrame.toStdString(), std::chrono::high_resolution_clock::now(), trans);
}

QMatrix4x4 TransMemQMLInterface::getLinkNow(const QString& srcFrame, const QString& dstFrame) const {
    try {
        return getLink(srcFrame.toStdString(), dstFrame.toStdString(), std::chrono::high_resolution_clock::now());
    }
    catch(NoSuchLinkFoundException e){}
    return QMatrix4x4();
}

QMatrix4x4 TransMemQMLInterface::getLinkBest(const QString &srcFrame, const QString &dstFrame) const {
    try {
        Timestamp ts;
        return getBestLink(srcFrame.toStdString(), dstFrame.toStdString(), ts);
    }
    catch (NoSuchLinkFoundException e){}
    return QMatrix4x4();
}

QMatrix4x4 TransMemQMLInterface::getInterpolation(const QMatrix4x4 &m1, const QMatrix4x4 &m2, const double ratio) const {

    if(ratio <= 0) return m1;
    if(ratio >= 1) return m2;

    float dataM1[]{m1(0,0),m1(0,1),m1(0,2),m1(1,0),m1(1,1),m1(1,2),m1(2,0),m1(2,1),m1(2,2)}; QMatrix3x3 rM1(dataM1);
    float dataM2[]{m2(0,0),m2(0,1),m2(0,2),m2(1,0),m2(1,1),m2(1,2),m2(2,0),m2(2,1),m2(2,2)}; QMatrix3x3 rM2(dataM2);

    // spherical interpolation for the rotation
    QQuaternion rot = QQuaternion::slerp(QQuaternion::fromRotationMatrix(rM1), QQuaternion::fromRotationMatrix(rM2), ratio);
    // linear interpolation for the translation
    QQuaternion trans = QQuaternion(0, m1(0,3), m1(1,3), m1(2,3))*(1.-ratio) + QQuaternion(0, m2(0,3), m2(1,3), m2(2,3))*ratio;

    // convert to QMatrix4x4
    QMatrix4x4 ret(rot.toRotationMatrix()); ret(0,3) = trans.x(); ret(1,3) = trans.y(); ret(2,3) = trans.z();

    return ret;
}

QQuaternion TransMemQMLInterface::getLargestEigenVecAsQuaternion(const QMatrix4x4& m){

    Eigen::Matrix4f A;
    A << m(0,0) , m(0,1) , m(0,2) , m(0,3)
        , m(1,0) , m(1,1) , m(1,2) , m(1,3)
        , m(2,0) , m(2,1) , m(2,2) , m(2,3)
        , m(3,0) , m(3,1) , m(3,2) , m(3,3);

    Eigen::EigenSolver<Eigen::Matrix4f> ces;
    ces.compute(A);


    Eigen::Vector4cf eigenVec = ces.eigenvectors().col(0);
    double eigenVal = ((std::complex<float>) ces.eigenvalues()[0]).real();
    for(unsigned int i = 1; i < 4; i++)
        if(eigenVal < ((std::complex<float>) ces.eigenvalues()[i]).real()){
            eigenVal = ((std::complex<float>) ces.eigenvalues()[i]).real();
            eigenVec = ces.eigenvectors().col(i);
        }

    return QQuaternion( ((std::complex<float>) eigenVec(0)).real(),
                        ((std::complex<float>) eigenVec(1)).real(),
                        ((std::complex<float>) eigenVec(2)).real(),
                        ((std::complex<float>) eigenVec(3)).real() );
}

QMatrix4x4 TransMemQMLInterface::toRotQuatProduct(const QMatrix4x4& m) const {

    float dataM1[]{m(0,0),m(0,1),m(0,2),m(1,0),m(1,1),m(1,2),m(2,0),m(2,1),m(2,2)}; QMatrix3x3 rM1(dataM1);

    QQuaternion q = QQuaternion::fromRotationMatrix(rM1);

    return QMatrix4x4(  q.scalar()*q.scalar(),  q.scalar()*q.x(),   q.scalar()*q.y(),   q.scalar()*q.x(),
                             q.x()*q.scalar(),       q.x()*q.x(),        q.x()*q.y(),        q.x()*q.z(),
                             q.y()*q.scalar(),       q.y()*q.x(),        q.y()*q.y(),        q.y()*q.z(),
                             q.z()*q.scalar(),       q.z()*q.x(),        q.z()*q.y(),        q.z()*q.z()    );
}

QMatrix4x4 TransMemQMLInterface::toTransformationMatrix(const QQuaternion& rot, const QVector3D& trans){
    QMatrix4x4 m = QMatrix4x4(rot.toRotationMatrix());
    m(0,3) = trans.x(); m(1,3) = trans.y(); m(2,3) = trans.z();
    return m;
}

QVector3D TransMemQMLInterface::toTransVect(const QMatrix4x4& m) const {
    return QVector3D(m(0,3),m(1,3),m(2,3));
}

QMatrix4x4 TransMemQMLInterface::getLinkBestCached(const QString &srcFrame, const QString &dstFrame) {
    try {
        Timestamp ts;
        return getBestLinkCached(srcFrame.toStdString(), dstFrame.toStdString(), ts);
    }
    catch (NoSuchLinkFoundException e){}
    return QMatrix4x4();
}
