#include <QtTest/QTest>

#include "transmem/transmem.h"
#include "matHelper.h"
#include <math.h>

#include "ctime"

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
    void linkQualityTest();
};




