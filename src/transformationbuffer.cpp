#include "transformationbuffer.h"

using namespace std;
using namespace std::chrono;

void TransformationBuffer::oldestEntry(StampedTransformation &te){

    if(buffer.empty())
        return;

    te = buffer.front();
    return;
}

void TransformationBuffer::newestEntry(StampedTransformation &te){

    if(buffer.empty())
        return;

    te = buffer.back();
    return;
}

void TransformationBuffer::addEntry(StampedTransformation &te){

    // buffer is empty, insert new entry and return
    if( buffer.empty() ){
        buffer.push_back(te);
        return;
    }

    // new entry is to old to be stored
    if( te.time + storageTime < ((StampedTransformation)buffer.back()).time ) {
        return;
    }

    // if new entry is newer than every entry, insert it direct in
    // front of the list (should be the case most of the time)
    if( te.time > ((StampedTransformation)buffer.back()).time ){
        buffer.push_back(te);
        pruneStorage();
        return;
    }

    // if new entry is older than every entry, insert it direct at
    // the back of the list
    if( te.time < ((StampedTransformation)buffer.front()).time ){
        buffer.push_front(te);
        pruneStorage();
        return;
    }

    // if entry already exists, just update it properly
    auto iterF = find_if(buffer.begin(), buffer.end(), [&te](const StampedTransformation &e){ return te.time == e.time; });
    if( iterF != buffer.end() ){
        *iterF = te;
        return;
    }

    // insert new entry in between two existing entries
    auto iterA = adjacent_find(buffer.begin(), buffer.end(), [&te](const StampedTransformation &el, const StampedTransformation &er){ return (el.time < te.time && te.time < er.time); });
    buffer.insert(++iterA, te);
    pruneStorage();
    return;
}

void TransformationBuffer::pruneStorage(){

    // NOTE: assertion just during development
    // pruneStorage should never be called when the buffer is empty
    assert(!buffer.empty());

    Timestamp mostRecent = ((StampedTransformation)buffer.back()).time;
    buffer.remove_if([&mostRecent, this](const StampedTransformation &te){return te.time + storageTime < mostRecent;});

    return;
}

void TransformationBuffer::printCurrentBuffer(){
    for(StampedTransformation te : buffer )
        cout << te;
}

void TransformationBuffer::entryAt(StampedTransformation &te) {

    // just return if buffer is empty
    if(buffer.empty())
        return;

    // return the newest entry if queried for en even newer one
    if(te.time >= ((StampedTransformation)buffer.back()).time){
        te = buffer.back();
        return;
    }

    // return the oldest entry queried for an even older one
    if(te.time <= ((StampedTransformation)buffer.front()).time){
        te = buffer.front();
        return;
    }

    // search the two closest entries
    auto iterA = adjacent_find(buffer.begin(), buffer.end(), [&te](const StampedTransformation &el, const StampedTransformation &er){ return (el.time < te.time && te.time < er.time); });
    StampedTransformation tl = *iterA; StampedTransformation tr = *(++iterA);

    // return left entry if distance between entries is to small
    if(tr.time - tl.time < minDistForInterpolation){
        te = tl;
        return;
    }

    // interpolate
    interpolate(tl, tr, te);
    return;
}

void TransformationBuffer::interpolate(const StampedTransformation &el, const StampedTransformation &er, StampedTransformation &res){

    milliseconds abs = duration_cast<milliseconds>(er.time-el.time);
    milliseconds lt = duration_cast<milliseconds>(res.time-el.time);

    float ratio = (float) lt.count()/abs.count();

    // NOTE: assertion just during development
    // ratio should not be outside this range
    assert(ratio > 0 && ratio < 1);

    // spherical interpolation for the rotation
    res.rotation = QQuaternion::slerp(el.rotation, er.rotation, ratio);

    // linear interpolation for the translation
    res.translation = el.translation*(1.-ratio) + er.translation*ratio;

    return;
}
