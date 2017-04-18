#include "transformationbuffer.h"

void TransformationBuffer::oldestEntry(TransEntry &te){

    if(buffer.empty())
        return;

    te = buffer.front();
    return;
}

void TransformationBuffer::newestEntry(TransEntry &te){

    if(buffer.empty())
        return;

    te = buffer.back();
    return;
}

void TransformationBuffer::addEntry(TransEntry &te){

    // buffer is empty, insert new entry and return
    if( buffer.empty() ){
        buffer.push_back(te);
        return;
    }

    // new entry is to old to be stored
    if( te.time + storageTime < ((TransEntry)buffer.back()).time ) {
        return;
    }

    // if new entry is newer than every entry, insert it direct in
    // front of the list (should be the case most of the time)
    if( te.time > ((TransEntry)buffer.back()).time ){
        buffer.push_back(te);
        pruneStorage();
        return;
    }

    // if new entry is older than every entry, insert it direct at
    // the back of the list
    if( te.time < ((TransEntry)buffer.front()).time ){
        buffer.push_front(te);
        pruneStorage();
        return;
    }

    // if entry already exists, just update it properly
    auto iterF = std::find_if(buffer.begin(), buffer.end(), [&te](const TransEntry &e){ return te.time == e.time; });
    if( iterF != buffer.end() ){
        *iterF = te;
        return;
    }

    // insert new entry in between two existing entries
    auto iterA = std::adjacent_find(buffer.begin(), buffer.end(), [&te](const TransEntry &el, const TransEntry &er){ return (el.time < te.time && te.time < er.time); });
    buffer.insert(++iterA, te);
    pruneStorage();
    return;
}

void TransformationBuffer::pruneStorage(){

    // pruneStorage should never be called when the buffer is empty
    assert(!buffer.empty());

    Timestamp mostRecent = ((TransEntry)buffer.back()).time;
    buffer.remove_if([&mostRecent, this](const TransEntry &te){return te.time + storageTime < mostRecent;});

    return;
}

void TransformationBuffer::printCurrentBuffer(){
    for(TransEntry te : buffer )
        std::cout << te;
}

void TransformationBuffer::entryAt(TransEntry &te) {

    // just return if buffer is empty
    if(buffer.empty())
        return;

    // return the newest entry if queried for en even newer one
    if(te.time >= ((TransEntry)buffer.back()).time){
        te = buffer.back();
        return;
    }

    // return the oldest entry queried for an even older one
    if(te.time <= ((TransEntry)buffer.front()).time){
        te = buffer.front();
        return;
    }

    // search the two closest entries
    auto iterA = std::adjacent_find(buffer.begin(), buffer.end(), [&te](const TransEntry &el, const TransEntry &er){ return (el.time < te.time && te.time < er.time); });
    TransEntry tl = *iterA; TransEntry tr = *(++iterA);

    // return left entry if distance between entries is to small
    if(tr.time - tl.time < minDistForInterpolation){
        te = tl;
        return;
    }

    // interpolate
    interpolate(tl, tr, te);
    return;
}

void TransformationBuffer::interpolate(const TransEntry &el, const TransEntry &er, TransEntry &res){

    std::chrono::milliseconds abs = std::chrono::duration_cast<std::chrono::milliseconds>(er.time-el.time);
    std::chrono::milliseconds lt = std::chrono::duration_cast<std::chrono::milliseconds>(res.time-el.time);

    float ratio = (float) lt.count()/abs.count();

    // spherical interpolation for the rotation
    res.rotation = QQuaternion::slerp(el.rotation,er.rotation,ratio);

    // linear interpolation for the translation
    // TODO!
    return;
}
