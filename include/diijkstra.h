#ifndef DIIJKSTRA_H
#define DIIJKSTRA_H

#include "frameAndLink.h"
#include "transmem.h"
#include <unordered_map>

struct Path;

class Diijkstra
{
public:
    Diijkstra(std::unordered_map<FrameID, Frame*> frameID2Frame)
    : frameID2Frame(frameID2Frame)
    {}

    void calculateShortestPath(Path &path);

protected:

    void initializeGraph();
    void searchPath(Path &path);
    void getPath(Path &path);

    void set2ShortestRemaining(Path &path);
    void updateDistance(Frame *adjFrame, double w);

    std::unordered_map<FrameID, Frame*> frameID2Frame;
    Frame *ptr2CurrFrame{nullptr};

};

#endif // DIIJKSTRA_H
