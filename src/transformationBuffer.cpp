#include "../src/headers/transformationBuffer.h"

using namespace std;
using namespace std::chrono;

bool TransformationBuffer::distanceToNextClosestEntry(const Timestamp &tStamp, milliseconds &distanceToCloserEntry) const {

    milliseconds tStampMS = chrono::duration_cast<milliseconds>(tStamp.time_since_epoch());

    if(buffer.empty()){
        return false;
    }

    // If tStamp is newer than every entry, we just need to know the distLeft
    // to the newest entry in the buffer.
    if( tStamp > ((StampedTransformation)buffer.back()).time ){

        distanceToCloserEntry = (tStampMS - duration_cast<milliseconds>(((StampedTransformation)buffer.back()).time.time_since_epoch()));
        return true;
    }

    // If tStamp is older than every entry, we just need to know the distLeft
    // to the oldest entry in the buffer.
    if( tStamp < ((StampedTransformation)buffer.front()).time ){

        distanceToCloserEntry = (duration_cast<milliseconds>(((StampedTransformation)buffer.front()).time.time_since_epoch()) - tStampMS);
        return true;
    }

    // Return zero if we hit an entry.
    auto iterF = find_if(buffer.begin(), buffer.end(), [&tStamp](const StampedTransformation &e){ return tStamp == e.time; });
    if( iterF != buffer.end() ){

        distanceToCloserEntry = milliseconds(0);
        return true;
    }

    // Otherwise we have to choose the minima.
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

bool TransformationBuffer::addEntry(const StampedTransformation &te) {

    // Buffer is empty, insert new entry and return.
    if( buffer.empty() ){
        buffer.push_back(te);
        return true;
    }

    // New entry is too old to be stored.
    if( te.time + storageTimeInMS < ((StampedTransformation)buffer.back()).time ) {
        return false;
    }

    // If new entry is newer than every entry, insert it direct in
    // front of the list (should be the case most of the time).
    if( te.time > ((StampedTransformation)buffer.back()).time ){

        buffer.push_back(te);
        pruneStorage();
        return true;
    }

    // If new entry is older than every entry, insert it direct at
    // the back of the list.
    if( te.time < ((StampedTransformation)buffer.front()).time ){

        buffer.push_front(te);
        pruneStorage();
        return true;
    }

    // If entry already exists, just update it properly.
    auto iterF = find_if(buffer.begin(), buffer.end(), [&te](const StampedTransformation &e){ return te.time == e.time; });
    if( iterF != buffer.end() ){
        *iterF = te;
        return true;
    }

    // Insert new entry in between two existing entries
    auto iterA = adjacent_find(buffer.begin(), buffer.end(), [&te](const StampedTransformation &el, const StampedTransformation &er){ return (el.time < te.time && te.time < er.time); });
    buffer.insert(++iterA, te);
    pruneStorage();
    return true;
}

void TransformationBuffer::pruneStorage() {

    // Should never be called when the buffer is empty.
    assert(!buffer.empty());

    Timestamp mostRecent = ((StampedTransformation)buffer.back()).time;
    buffer.remove_if([&mostRecent, this](const StampedTransformation &te){return te.time + storageTimeInMS < mostRecent;});

    // To many entries, remove the oldest ones.
    if(buffer.size() > MAX_NUMBER_OF_ENTRIES){
        while(buffer.size() > MAX_NUMBER_OF_ENTRIES)
            buffer.pop_front();
    }
}

bool TransformationBuffer::entryAt(StampedTransformation &te) const {

    // Return false if buffer is empty.
    if(buffer.empty())
        return false;

    // Return the newest entry if queried for en even newer one.
    if(((StampedTransformation)buffer.back()).time <= te.time){
        te = buffer.back();
        return true;
    }

    // Return the oldest entry queried for an even older one.
    if(te.time <= ((StampedTransformation)buffer.front()).time){
        te = buffer.front();
        return true;
    }

    // Search the two closest entries.
    auto iterA = adjacent_find(buffer.begin(), buffer.end(), [&te](const StampedTransformation &el, const StampedTransformation &er){ return (el.time < te.time && te.time < er.time); });
    StampedTransformation tl = *iterA; StampedTransformation tr = *(++iterA);

    // Return left entry if distance between entries is to small.
    if(tr.time - tl.time < MIN_DISTANCE_FOR_INTERPOLATION_IN_NS){
        te = tl;
        return true;
    }

    interpolate(tl, tr, te);

    return true;
}

void TransformationBuffer::interpolate(const StampedTransformation &el, const StampedTransformation &er, StampedTransformation &res) const {

    milliseconds abs = duration_cast<milliseconds>(er.time-el.time);
    milliseconds lt = duration_cast<milliseconds>(res.time-el.time);

    // abs.count() cant be zero since interpolate is just called if the two entries are a minimum distance apart.
    double ratio = (double) lt.count()/abs.count();

    // Protection against strange cases.
    if(ratio > 1) ratio = 1.;
    if(ratio < 0) ratio = 0.;

    // Spherical interpolation for the rotation.
    res.rotation = QQuaternion::slerp(el.rotation, er.rotation, ratio);

    // Linear interpolation for the translation.
    res.translation = el.translation*(1.-ratio) + er.translation*ratio;

    // We set time of the result to the time of the transformation entry which is farther away.
    res.time = (ratio > 0.5 ? el.time : er.time);
}

void TransformationBuffer::writeJSON(QJsonObject &json) const {

    QJsonArray bufferedEntries;

    for(StampedTransformation st : buffer){
        QJsonObject entryObject;
        st.writeJSON(entryObject);
        bufferedEntries.append(entryObject);
    }

    json["bufferedEntries"] = bufferedEntries;
}
