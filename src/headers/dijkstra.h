#ifndef DIIJKSTRA_H
#define DIIJKSTRA_H

#include "frameAndLink.h"
#include "transmem/transmem.h"
#include <unordered_map>

struct Path;

class Diijkstra
{
public:
    Diijkstra(std::unordered_map<FrameID, Frame*> frameID2Frame)
    : frameID2Frame(frameID2Frame)
    {}

    bool calculateShortestPath(Path &path);

protected:

    void initializeGraph();
    bool searchPath(Path &path);
    void getPath(Path &path);

    bool set2ShortestRemaining();
    void updateDistance(Frame *adjFrame, double w);

    std::unordered_map<FrameID, Frame*> frameID2Frame;
    Frame *ptr2CurrFrame{nullptr};

};

#endif // DIIJKSTRA_H
