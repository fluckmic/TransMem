#ifndef STAMPEDTRANSFORMATION_H
#define STAMPEDTRANSFORMATION_H

#include "typedefs.h"

struct StampedTransformation {
    Timestamp time;
    QQuaternion rotation;
    QQuaternion translation;

    std::string timeAsString() const;
    std::string rotationAsString() const;
    std::string translationAsString() const;
};

std::ostream &operator<<(std::ostream &os, const StampedTransformation &te);

#endif // STAMPEDTRANSFORMATION_H
