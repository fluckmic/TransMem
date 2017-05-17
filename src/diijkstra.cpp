#include "diijkstra.h"

void Diijkstra::calculateShortestPath(Path &path){

    // check if the source frame exists
    if(frameID2Frame.find(path.src) == frameID2Frame.end())
      throw NoSuchLinkFoundException(path.src, path.dst);

    // check if the destination frame exists
    auto iter2DstFrame = frameID2Frame.find(path.dst);
    if(iter2DstFrame == frameID2Frame.end())
        throw NoSuchLinkFoundException(path.src, path.dst);

    // we start from the destination frame and search a path
    // to the source frame, that allows us to create the path from
    // source frame to destination frame later on
    ptr2CurrFrame = (*iter2DstFrame).second;

    initializeGraph();

    searchPath(path);

    getPath(path);
}

void Diijkstra::initializeGraph(){

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
        ptr2CurrFrame->distance = 0.;
        ptr2CurrFrame->active = false;

}

void Diijkstra::searchPath(Path &path){

    while(ptr2CurrFrame->frameID != path.src){
        // update distance between current node and its adjascent nodes
        for(Link* l: ptr2CurrFrame->parents)
            if(l->parent->active)
                updateDistance(l->parent, l->weight);
        for(Link* l: ptr2CurrFrame->children)
            if(l->child->active)
                updateDistance(l->child, l->weight);

        set2ShortestRemaining(path);
    }

}

void Diijkstra::getPath(Path &path){

    Link* currLink;

    // NOTE: assertion just during development
    // there should always be at least one predecessor
    // since if there is a path, the path is at least of length one
    assert(ptr2CurrFrame->predecessor != nullptr);

    // get the link to the predecessor
    ptr2CurrFrame->connectionTo(ptr2CurrFrame->predecessor->frameID, currLink);
    // add the link to the path
    path.links.push_back(currLink);
    // set the ptr to the predecessor
    ptr2CurrFrame = ptr2CurrFrame->predecessor;

    // repeat this until there is no longer a predecessor
    while(ptr2CurrFrame->predecessor != nullptr){
        ptr2CurrFrame->connectionTo(ptr2CurrFrame->predecessor->frameID, currLink);
        path.links.push_back(currLink);
        ptr2CurrFrame = ptr2CurrFrame->predecessor;
    }
}

void Diijkstra::set2ShortestRemaining(Path &path){

    // set ptr2CurrFrame to the shortest active frame
    // one could improve the performance by using a priority queue..
    ptr2CurrFrame = nullptr;

    auto iter = frameID2Frame.begin();
    double minDist = std::numeric_limits<double>::infinity();

    while(iter != frameID2Frame.end()){
        Frame* cur = (*iter).second;
        if(cur->distance < minDist && cur->active){
            ptr2CurrFrame = cur;
            minDist = cur->distance;
        }
        iter++;
    }

    // no path exists
    if(ptr2CurrFrame == nullptr)
        throw NoSuchLinkFoundException(path.src, path.dst);

    ptr2CurrFrame->active = false;
}

void Diijkstra::updateDistance(Frame *ptr2adjFrame, double w){

    double alternativeDist = ptr2CurrFrame->distance + w;

    if(alternativeDist < ptr2adjFrame->distance){
        ptr2adjFrame->distance = alternativeDist;
        ptr2adjFrame->predecessor = ptr2CurrFrame;
    }

}