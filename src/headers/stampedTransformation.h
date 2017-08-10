#ifndef STAMPEDTRANSFORMATION_H
#define STAMPEDTRANSFORMATION_H

#include "typedefs.h"

/*! \struct StampedTransformation
 *
 * Represents internally a single transformation entry. Provides common fields to
 * StampedTransformationWithConfidence as a superclass. */
struct StampedTransformation {

    /*! Rotational part of the transformation. */
    QQuaternion rotation;
    /*! Translational part of the transformation. */
    QQuaternion translation;

    /*! Stores the valid time if internally used as StampedTransformation.\n\n
     *
     * In StampedTransformationWithConfidence the field represents the query time if the object is
     * returned as a result of a link query and the best time in the case when the object is returned
     * as a result of a best link query. */
    Timestamp time;

    void writeJSON(QJsonObject &json) const;
};

#endif // STAMPEDTRANSFORMATION_H
