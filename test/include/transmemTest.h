#include <QtTest/QTest>

#include "transmem.h"
#include "matHelper.h"
#include "solution.h"
#include <math.h>

#include "ctime"
#include "random"

Q_DECLARE_METATYPE(FrameID)

using PrequelSequence = std::vector< std::pair<FrameID, FrameID> > ;
using PrequelSequenceWithT = std::vector < std::pair < std::pair < FrameID, FrameID > , QMatrix4x4> > ;

class transmemTest : public QObject {
    Q_OBJECT

 private slots:

    /*
    void throwsException_data();
    void throwsException();

    void simpleQueries_data();
    void simpleQueries();

    void simpleNStepQueries_data();
    void simpleNStepQueries();

    void bestPointInTime_data();
    void bestPointInTime();
    */

    void dynamicTest();
    void dynamicTest_data();


 public:
    enum ExceptionType {InvalidArgument, NoSuchLinkFound};

 protected:
    bool compareHelper(const QMatrix4x4 &ref, const QMatrix4x4 &oth, const double eps);
    std::string toLinkString(const FrameID &src, const FrameID &dst);
    const double precision = 1e-06;

    QMatrix4x4 createRandTMatrix();

    QMatrix4x4 tRz(double a);
    QMatrix4x4 tRx(double a);
    QMatrix4x4 tTr(double x, double y, double z);

};

struct updateSequence {
    FrameID src;
    FrameID dest;
    std::vector<Timestamp> timeStamps;
    std::vector<QMatrix4x4> transformations;
};

struct query {
    FrameID src;
    FrameID dest;
    Timestamp tStamp;
    QMatrix4x4 transfomation;
};

Q_DECLARE_METATYPE(query)
Q_DECLARE_METATYPE(updateSequence)
Q_DECLARE_METATYPE(transmemTest::ExceptionType)



