#ifndef SOLUTION_H
#define SOLUTION_H

#include <QtGui/QMatrix4x4>
#include "typedefs.h"
#include <unordered_map>
#include <deque>
#include <set>

typedef QMatrix4x4 (*ptr2paramTransMat) (Timestamp);

class Solution {

public:

    Solution(std::vector< std::pair <FrameID, FrameID> > frameIDPairs,
             std::unordered_map<std::string, ptr2paramTransMat> link2paramTransMat,
             std::unordered_map<std::string, std::vector<std::string> > path2links)
        : link2paramTransMat(link2paramTransMat)
        , path2links(path2links)
    {
        std::string linkName;
        std::set<Timestamp>* ptr2TimestampSet;

        for(std::pair<FrameID, FrameID> frameIDPair : frameIDPairs){

            timestampSets.emplace_back(std::set<Timestamp>());
            ptr2TimestampSet = &timestampSets.back();

            linkName = toLinkString(frameIDPair.first, frameIDPair.second);
            link2TimestampSet.insert({linkName, ptr2TimestampSet});
            linkName = toLinkString(frameIDPair.second, frameIDPair.first);
            link2TimestampSet.insert({linkName, ptr2TimestampSet});
        }
    }

    void updateSolution(const FrameID &src, const FrameID &dst, const Timestamp &tStamp);

    bool checkTransformation(const FrameID &src, const FrameID &dst,Timestamp tStamp, const QMatrix4x4 &result);

private:

    const double precision = 1e-06;

    std::unordered_map<std::string, std::set<Timestamp>* > link2TimestampSet;
    std::deque< std::set<Timestamp> > timestampSets;

    std::unordered_map<std::string, ptr2paramTransMat> link2paramTransMat;

    std::unordered_map<std::string, std::vector<std::string> > path2links;

    std::chrono::seconds bufferTime{10};

    std::string toLinkString(const FrameID &src, const FrameID &dst);

    void adaptTimestamp(Timestamp &tStamp, const std::string &link);

    Timestamp getOldestStamp(const std::string &link);
    Timestamp getNewestStamp(const std::string &link);

    bool matComparator(const QMatrix4x4 &ref, const QMatrix4x4 &oth);
};


#endif // SOLUTION_H
