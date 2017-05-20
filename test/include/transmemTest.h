#include <QtTest/QTest>

#include "transmem.h"
#include "matHelper.h"
#include <math.h>

#include "ctime"

class transmemTest : public QObject {
    Q_OBJECT

 private slots:

    void simpleQueriesTest();
    void simpleMultiStepQueriesTest();
    void throwsExceptionTest();
    void inversionTestSimple();
    void inversionTestwithAmbigousPath();
    void pruningTest();
    void bestPointInTimeTest();
};




