#include "src/headers/transformationBuffer.h"

using namespace std;
using namespace std::chrono;

bool TransformationBuffer::distanceToNextClosestEntry(const Timestamp &tStamp, milliseconds &distanceToCloserEntry) const {

    milliseconds tStampMS = chrono::duration_cast<milliseconds>(tStamp.time_since_epoch());

    if(buffer.empty()){
        return false;
    }

    // if t is newer than every entry, we just need to know the distLeft
    // to the newest entry in the buffer
    if( tStamp > ((StampedTransformation)buffer.back()).time ){

        distanceToCloserEntry = (tStampMS - duration_cast<milliseconds>(((StampedTransformation)buffer.back()).time.time_since_epoch()));
        return true;
    }

    // if t is older than every entry, we just need to know the distLeft
    // to the oldest entry in the buffer
    if( tStamp < ((StampedTransformation)buffer.front()).time ){

        distanceToCloserEntry = (duration_cast<milliseconds>(((StampedTransformation)buffer.front()).time.time_since_epoch()) - tStampMS);
        return true;
    }

    // return zero if we hit an entry
    auto iterF = find_if(buffer.begin(), buffer.end(), [&tStamp](const StampedTransformation &e){ return tStamp == e.time; });
    if( iterF != buffer.end() ){

        distanceToCloserEntry = milliseconds(0);
        return true;
    }

    // otherwise we have to choose the minima
    auto iterA = adjacent_find(buffer.begin(), buffer.end(),
                               [&tStamp](const StampedTransformation &el, const StampedTransformation &er)
                               {return (el.time < tStamp && tStamp < er.time); });

    milliseconds distRight;

    distanceToCloserEntry = tStampMS - duration_cast<milliseconds>(((StampedTransformation)*iterA).time.time_since_epoch());
    distRight = duration_cast<milliseconds>(((StampedTransformation)*(++iterA)).time.time_since_epoch()) - tStampMS;

    if(distRight < distanceToCloserEntry)
        distanceToCloserEntry = distRight;

    return true;
}

bool TransformationBuffer::oldestEntry(StampedTransformation &te) const {

    if(buffer.empty())
        return false;

    te = buffer.front();
    return true;
}

bool TransformationBuffer::newestEntry(StampedTransformation &te) const {

    if(buffer.empty())
        return false;

    te = buffer.back();
    return true;
}

bool TransformationBuffer::addEntry(const StampedTransformation &te){

    // buffer is empty, insert new entry and return
    if( buffer.empty() ){
        buffer.push_back(te);
        return true;
    }

    // new entry is too old to be stored
    if( te.time + storageTime < ((StampedTransformation)buffer.back()).time ) {
        return false;
    }

    // if new entry is newer than every entry, insert it direct in
    // front of the list (should be the case most of the time)
    if( te.time > ((StampedTransformation)buffer.back()).time ){

        buffer.push_back(te);
        pruneStorage();
        return true;
    }

    // if new entry is older than every entry, insert it direct at
    // the back of the list
    if( te.time < ((StampedTransformation)buffer.front()).time ){

        buffer.push_front(te);
        pruneStorage();
        return true;
    }

    // if entry already exists, just update it properly
    auto iterF = find_if(buffer.begin(), buffer.end(), [&te](const StampedTransformation &e){ return te.time == e.time; });
    if( iterF != buffer.end() ){
        *iterF = te;
        return true;
    }

    // insert new entry in between two existing entries
    auto iterA = adjacent_find(buffer.begin(), buffer.end(), [&te](const StampedTransformation &el, const StampedTransformation &er){ return (el.time < te.time && te.time < er.time); });
    buffer.insert(++iterA, te);
    pruneStorage();
    return true;
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

bool TransformationBuffer::entryAt(StampedTransformation &te) const {

    // return false if buffer is empty
    if(buffer.empty())
        return false;

    // return the newest entry if queried for en even newer one
    if(((StampedTransformation)buffer.back()).time <= te.time){
        te = buffer.back();
        return true;
    }

    // return the oldest entry queried for an even older one
    if(te.time <= ((StampedTransformation)buffer.front()).time){
        te = buffer.front();
        return true;
    }

    // search the two closest entries
    auto iterA = adjacent_find(buffer.begin(), buffer.end(), [&te](const StampedTransformation &el, const StampedTransformation &er){ return (el.time < te.time && te.time < er.time); });
    StampedTransformation tl = *iterA; StampedTransformation tr = *(++iterA);

    // return left entry if distance between entries is to small
    if(tr.time - tl.time < minDistForInterpolation){
        te = tl;
        return true;
    }

    // interpolate
    interpolate(tl, tr, te);

    return true;
}

void TransformationBuffer::interpolate(const StampedTransformation &el, const StampedTransformation &er, StampedTransformation &res) const{

    milliseconds abs = duration_cast<milliseconds>(er.time-el.time);
    milliseconds lt = duration_cast<milliseconds>(res.time-el.time);

    float ratio = (float) lt.count()/abs.count();

    // NOTE: assertion just during development
    // ratio should not be outside this range
    assert(ratio > 0 && ratio <= 1);

    // spherical interpolation for the rotation
    res.rotation = QQuaternion::slerp(el.rotation, er.rotation, ratio);

    // linear interpolation for the translation
    res.translation = el.translation*(1.-ratio) + er.translation*ratio;

    return;
}

void TransformationBuffer::writeJSON(QJsonObject &json) const{

    QJsonArray bufferedEntries;

    for(StampedTransformation st : buffer){
        QJsonObject entryObject;
        st.writeJSON(entryObject);
        bufferedEntries.append(entryObject);
    }

    json["bufferedEntries"] = bufferedEntries;
}
