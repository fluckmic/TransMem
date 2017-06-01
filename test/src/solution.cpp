#include "test/include/solution.h"

 bool Solution::checkTransformation(const FrameID &src, const FrameID &dst, Timestamp tStamp, const QMatrix4x4 &result){

     QMatrix4x4 refSol = QMatrix4x4();

     Timestamp tTemp = tStamp;

     std::vector<std::string> links;

    // get the correct links from src to dst
    try {links = path2links.at(toLinkString(src, dst));
    }
    catch (const std::out_of_range &e){
        /* Should not happen, otherwise the solution is incomplete. */
         return false;
    }

    // calculate the reference solution
    for( std::string link : links){

        // adapt the timestamp if necessary
        try {adaptTimestamp(tTemp, link);
        }
        catch (const std::out_of_range &e){
            /* There is no entry on  given link. */
             return false;
        }

        ptr2paramTransMat nxtMatFunc;

        try {nxtMatFunc = link2paramTransMat.at(link);
        }
        catch (const std::out_of_range &e){
            /* Should not happen, otherwise the solution is incomplete. */
             return false;
        }

        refSol = ((*nxtMatFunc)(tTemp)) * refSol;
    }

     return matComparator(refSol, result);
 }

 void Solution::updateSolution(const FrameID &src, const FrameID &dst, const Timestamp &tStamp){

     std::string link = toLinkString(src,dst);

     std::set<Timestamp>* ptr2Timestamps = (*(link2TimestampSet.find(link))).second;

     // there was no update until now on this link..
     if(ptr2Timestamps->size() < 1){

         ptr2Timestamps->insert(tStamp);
         return;
     }

    Timestamp newestTstamp = getNewestStamp(link);

    // timestamp is to old to be stored
    if( tStamp < newestTstamp - bufferTime)
        return;

    if( newestTstamp < tStamp)
        newestTstamp = tStamp;

    ptr2Timestamps->insert(tStamp);

    // remove timestamps which are to old

    ptr2Timestamps->erase(ptr2Timestamps->begin(), ptr2Timestamps->upper_bound(newestTstamp - bufferTime));

    }

 std::string Solution::toLinkString(const FrameID &src, const FrameID &dst){
     return (src + "-" + dst);
 }

 void Solution::adaptTimestamp(Timestamp &tStamp, const std::string &link){

   Timestamp oldestTstamp = getOldestStamp(link);

    if(tStamp < oldestTstamp)
        tStamp = oldestTstamp;

    Timestamp newestTstamp = getNewestStamp(link);

    if(newestTstamp < tStamp)
        tStamp = newestTstamp;

 }

 bool Solution::matComparator(const QMatrix4x4 &ref, const QMatrix4x4 &oth){

     for(unsigned int ii = 0; ii < 4; ii++)
         for(unsigned int jj = 0; jj < 4; jj++)
             if(std::fabs(ref(ii,jj)-oth(ii,jj)) > precision)
                 return false;
     return true;
 }


 Timestamp Solution::getNewestStamp(const std::string &link){

     std::set<Timestamp>* ptr2Timestamps = (*(link2TimestampSet.find(link))).second;

     if(ptr2Timestamps->size() < 1)
         throw std::out_of_range(link + "contains no transformation.");

     return *(--ptr2Timestamps->end());
     // get the newest timestamp for a given link


 }

 Timestamp Solution::getOldestStamp(const std::string &link){

     std::set<Timestamp>* ptr2Timestamps = (*(link2TimestampSet.find(link))).second;

     if(ptr2Timestamps->size() < 1)
         throw std::out_of_range(link + "contains no transformation.");

     return *(ptr2Timestamps->begin());
     // get the oldest timestamp for a given link
 }
