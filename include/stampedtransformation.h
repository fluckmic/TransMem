#ifndef STAMPEDTRANSFORMATION_H
#define STAMPEDTRANSFORMATION_H

#include "typedefs.h"

struct StampedTransformation {
    Timestamp time;
    QQuaternion rotation;
    QQuaternion translation;
};

std::ostream &operator<<(std::ostream &os, const StampedTransformation &te);

#endif // STAMPEDTRANSFORMATION_H
