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

QMatrix4x4 MatHelper::getXRotMatrix(double a){

    return QMatrix4x4(   cos(a),   -sin(a),    0,  0,
                         sin(a),    cos(a),    0,  0,
                              0,         0,    1,  0,
                              0,         0,    0,  1  );
    }

QMatrix4x4 MatHelper::getYRotMatrix(double a){

    return QMatrix4x4(  cos(a),     0,  sin(a),     0,
                             0,     1,       0,     0,
                       -sin(a),     0,  cos(a),     0,
                             0,     0,       0,     1   );
    }

QMatrix4x4 MatHelper::getZRotMatrix(double a){

    return QMatrix4x4(   1,         0,       0,  0,
                         0,    cos(a), -sin(a),  0,
                         0,    sin(a),  cos(a),  0,
                         0,         0,       0,  1  );
    }

QMatrix4x4 MatHelper::getRotMatrix(double angle, QVector3D axis){

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

QMatrix4x4 MatHelper::simpleParam1(Timestamp t) {

    std::chrono::milliseconds tMS = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
    return getZRotMatrix(tMS.count() % 1000);
}

QMatrix4x4 MatHelper::simpleParam1Inv(Timestamp t) {

    return simpleParam1(t).inverted();
}

QMatrix4x4 MatHelper::simpleParam2(Timestamp t) {

    std::chrono::milliseconds tMS = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
    return getYRotMatrix(tMS.count() % 500) * getTransMatrix(tMS.count() % 8, tMS.count() % 3, tMS.count() % 12);
}

QMatrix4x4 MatHelper::simpleParam2Inv(Timestamp t) {

    return simpleParam2(t).inverted();
}
