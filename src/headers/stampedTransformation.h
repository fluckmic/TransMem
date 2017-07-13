#ifndef STAMPEDTRANSFORMATION_H
#define STAMPEDTRANSFORMATION_H

#include "typedefs.h"

struct StampedTransformation {

    // timestamp as additional information
    Timestamp time;

    // Quaternions encoding the transformation
    QQuaternion rotation;
    QQuaternion translation;

    // Information about the "quality" of the transformation

    // One measurement for the quality of a transformation is the is the average of all link qualities along the path. For all
    // links for which the quality was not set explicitly the default quality is used.
    float avgLinkQuality;

    // Another measurement for the quality of a transformation is the average of the the time distances to the saved transformation entry
    // which is used for the calculation of a single links transformation and which is further away.
    //
    // Link 1:              |****x**********|
    // Link 2:      |************x*****|
    //
    // avgDistanceToEntry:  10+12 / 2 = 11

    // The value is in ms. One can change the mapping via a function f which can be passed to transmem constructor
    float avgDistanceToEntry;

    // NOTE: for both qualities are different calculation methods than the averaging thinkable. Maybe even setable from outside via function pointers..

    std::string timeAsString() const;
    std::string rotationAsString() const;
    std::string translationAsString() const;

    void writeJSON(QJsonObject &json) const;
};

std::ostream &operator<<(std::ostream &os, const StampedTransformation &te);

#endif // STAMPEDTRANSFORMATION_H
