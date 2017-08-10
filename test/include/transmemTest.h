#include <QtTest/QTest>

#include <QtGui/QQuaternion>
#include <QtGui/QGenericMatrix>
#include <QtGui/QMatrix4x4>
#include <QtGui/QVector3D>
#include <math.h>

#include "transmem/transmem.h"

class transmemTest : public QObject {

    Q_OBJECT

 private slots:

    void throwsExceptionTest();
    void simpleQueriesTest();
    void simpleMultiStepQueriesTest();
    void inversionTestSimple();
    void inversionTestwithAmbigousPath();
    void pruningTest();
    void bestPointInTimeTest();
    void cachedBestLinksTest();
    void linkConfidenceTest();
    void timeDiffTest();

    void transComparatorTest();
};




