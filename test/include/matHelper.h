#ifndef MATHELPER_H
#define MATHELPER_H

#include <QtGui/QQuaternion>
#include <QtGui/QGenericMatrix>
#include <QtGui/QMatrix4x4>
#include <QtGui/QVector3D>
#include "src/headers/typedefs.h"

#endif // MATHELPER_H

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

};
