#ifndef STAMPEDTRANSFORMATION_H
#define STAMPEDTRANSFORMATION_H

#include "typedefs.h"

/**************************
 * STAMPED TRANSFORMATION *
 **************************/

struct StampedTransformation {

    // Quaternions encoding the transformation
    QQuaternion rotation;
    QQuaternion translation;

    Timestamp time;

    void writeJSON(QJsonObject &json) const;
};

#endif // STAMPEDTRANSFORMATION_H
