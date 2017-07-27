#ifndef MATHELPER_H
#define MATHELPER_H

#include <QtGui/QQuaternion>
#include <QtGui/QGenericMatrix>
#include <QtGui/QMatrix4x4>
#include <QtGui/QVector3D>

#include "src/headers/typedefs.h"
#include "include/transmem/transmem.h"

#endif // MATHELPER_H

/**************
 * MAT HELPER *
 **************/

// Funct

class MatHelper {

public:

   static QMatrix4x4 getRandTransMatrix();

   static QMatrix4x4 getTransMatrix(double x, double y, double z);
   static QMatrix4x4 getTransMatrix(QVector3D t);

   static QMatrix4x4 getXRotMatrix(double angle);
   static QMatrix4x4 getYRotMatrix(double angle);
   static QMatrix4x4 getZRotMatrix(double angle);

   static QMatrix4x4 getRotMatrix(double angle, QVector3D axis);

   static bool matrixComparator(const QMatrix4x4 &ref, const QMatrix4x4 &oth);

   static QMatrix4x4 toMatrix4x4(const StampedAndRatedTransformation &t);

   /* allows the comparision of two translation matrices. The implementation is not
      used in transmem but i want to use the testing environment to test the implementation
      of the comparator. */
    static QVector4D transformationComparator(const QMatrix4x4 &m1, const QMatrix4x4 &m2);

};
