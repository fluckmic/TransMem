#ifndef STAMPEDTRANSFORMATION_H
#define STAMPEDTRANSFORMATION_H

#include "typedefs.h"

/*! \struct StampedTransformation
 * \brief Represents a single transformation entry stored on a link.
 */
struct StampedTransformation {

    /*! Rotational part of the transformation. */
    QQuaternion rotation;
    /*! Translational part of the transformation. */
    QQuaternion translation;

    /*! Stores the time when the transfomation is valid. */
    Timestamp time;

    void writeJSON(QJsonObject &json) const;
};

#endif // STAMPEDTRANSFORMATION_H
