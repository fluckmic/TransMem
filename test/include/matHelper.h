#ifndef MATHELPER_H
#define MATHELPER_H

#include <QtGui/QQuaternion>
#include <QtGui/QGenericMatrix>
#include <QtGui/QMatrix4x4>
#include <QtGui/QVector3D>
#include "typedefs.h"

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

   static QMatrix4x4 simpleParam1(Timestamp t);
   static QMatrix4x4 simpleParam1Inv(Timestamp t);

   static QMatrix4x4 simpleParam2(Timestamp t);
   static QMatrix4x4 simpleParam2Inv(Timestamp t);


};
