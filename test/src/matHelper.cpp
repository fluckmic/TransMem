#include "test/include/matHelper.h"

QMatrix4x4 MatHelper::getRandTransMatrix(){

    std::uniform_real_distribution<double> unif1(0, 1);
    std::uniform_real_distribution<double> unif2(0, 1);
    std::uniform_real_distribution<double> unif3(0, 2 * M_PI);
    std::uniform_real_distribution<double> unif4(-1, 1);

    std::default_random_engine re(time(NULL));

    double theta = acos(2 * unif1(re) - 1.);
    double phi = 2 * M_PI * unif2(re);

    QVector3D axis = QVector3D(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));

    double angle = unif3(re);

    QMatrix4x4 ret =  MatHelper::getRotMatrix(angle, axis) *
                      MatHelper::getTransMatrix(unif4(re), unif4(re), unif4(re));

    return ret;
}

QMatrix4x4 MatHelper::getZRotMatrix(double a){

    a = a * (M_PI/180);

    return QMatrix4x4(   cos(a),   -sin(a),    0,  0,
                         sin(a),    cos(a),    0,  0,
                              0,         0,    1,  0,
                              0,         0,    0,  1  );
    }

QMatrix4x4 MatHelper::getYRotMatrix(double a){

    a = a * (M_PI/180);

    return QMatrix4x4(  cos(a),     0,  sin(a),     0,
                             0,     1,       0,     0,
                       -sin(a),     0,  cos(a),     0,
                             0,     0,       0,     1   );
    }

QMatrix4x4 MatHelper::getXRotMatrix(double a){

    a = a * (M_PI/180);

    return QMatrix4x4(   1,         0,       0,  0,
                         0,    cos(a), -sin(a),  0,
                         0,    sin(a),  cos(a),  0,
                         0,         0,       0,  1  );
    }

QMatrix4x4 MatHelper::getRotMatrix(double angle, QVector3D axis){

    angle = angle * (M_PI/180);

    double c = cos(angle); double s = sin(angle); double t = 1. - cos(angle);
    double x = axis.x();   double y = axis.y();   double z = axis.z();

    return QMatrix4x4(    t*x*x+c,  t*x*y-s*z,  t*x*z+s*y,   0,
                        t*x*y+s*z,    t*y*y+c,  t*y*z-s*x,   0,
                        t*x*z-s*y,  t*y*z+s*x,    t*z*z+c,   0,
                                0,          0,          0,   1  );
    }

QMatrix4x4 MatHelper::getTransMatrix(double x, double y, double z) {

    return QMatrix4x4(   1, 0, 0, x,
                         0, 1, 0, y,
                         0, 0, 1, z,
                         0, 0, 0, 1 );
    }

bool MatHelper::matrixComparator(const QMatrix4x4 &ref, const QMatrix4x4 &oth){

         for(unsigned int ii = 0; ii < 4; ii++)
             for(unsigned int jj = 0; jj < 4; jj++)
                 if(std::fabs(ref(ii,jj)-oth(ii,jj)) > 1e-05)
                     return false;
         return true;
}

QMatrix4x4 MatHelper::toMatrix4x4(const StampedAndRatedTransformation &t){

    QMatrix3x3 rotM = t.qRot.toRotationMatrix();

    QMatrix4x4 ret = QMatrix4x4(rotM);
    ret(0,3) = t.qTra.x(); ret(1,3) = t.qTra.y(); ret(2,3) = t.qTra.z();

    return ret;
}

QVector4D MatHelper::transformationComparator(const QMatrix4x4 &m1, const QMatrix4x4 &m2){

    double toRad = M_PI/180;

    /* split each transformation matrix in a translation vector vTra,
       a rotation axis vRotA and an angle fAngl. */

    float dataM1[]{m1(0,0),m1(0,1),m1(0,2),m1(1,0),m1(1,1),m1(1,2),m1(2,0),m1(2,1),m1(2,2)},
          dataM2[]{m2(0,0),m2(0,1),m2(0,2),m2(1,0),m2(1,1),m2(1,2),m2(2,0),m2(2,1),m2(2,2)};

    QMatrix3x3 rM1(dataM1), rM2(dataM2);

    QVector3D vTra1 = QVector3D(m1(0,3), m1(1,3), m1(2,3)),
              vTra2 = QVector3D(m2(0,3), m2(1,3), m2(2,3));

    QVector3D vRotA1, vRotA2; float fAngl1, fAngl2;

    QQuaternion::fromRotationMatrix(rM1).getAxisAndAngle(&vRotA1, &fAngl1);
    QQuaternion::fromRotationMatrix(rM2).getAxisAndAngle(&vRotA2, &fAngl2);

    if(vRotA1.z() < 0 ){ vRotA1 *= -1; fAngl1 = 360 - fAngl1; }
    if(vRotA2.z() < 0 ){ vRotA2 *= -1; fAngl2 = 360 - fAngl2; }

    // calculate the distance between the two normalized translation vectors
    double diffRotTrans = fabs((vTra1.normalized() - vTra2.normalized()).length());

    // calculate the difference of the length of the to translation vectors
    double diffLenTra1 = fabs(vTra1.length() - vTra2.length());

    // calculate the distance between a point perpendicular to the rotation axis
    // and this point rotated by the angle difference
    double diffAnglRot = sqrt(2*(1-cos((fAngl1-fAngl2)*toRad)));

    // calculate the distance between the two rotation vectors
    vRotA1.normalize(); vRotA2.normalize();
    double diffRotRot = fabs((vRotA1 - vRotA2).length());

    return QVector4D(diffRotTrans, diffLenTra1, diffAnglRot, diffRotRot);
}



