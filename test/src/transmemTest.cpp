#include "test/include/transmemTest.h"


void transmemTest::throwsException_data() {

    // initialize
    QTest::addColumn<FrameID>("srcFrame");
    QTest::addColumn<FrameID>("destFrame");
    QTest::addColumn<PrequelSequence>("prequelSequence");
    QTest::addColumn<ExceptionType>("exceptionType");

    // test set 1
    QTest::newRow("throws exception test 1 - empty transmem, first query between identical frames")
        << (FrameID)"f1"
        << (FrameID)"f1"
        << PrequelSequence()
        << InvalidArgument;

    // test set 2
    QTest::newRow("throws exception test 2 - empty transmem, query between two frames")
        << (FrameID)"f1"
        << (FrameID)"f2"
        << PrequelSequence()
        << NoSuchLinkFound;

    // test set 3
    QTest::newRow("throws exception test 3 - simple network, query for transformation between frames which are not connected")
        << (FrameID)"f1"
        << (FrameID)"F3"
        << PrequelSequence{{"F2","f3"},{"f1","f2"},{"F1","F3"}}
        << NoSuchLinkFound;

    // test set 4
    QTest::newRow("throws exception test 4 - more complicated network, query for transformation between two frames which are not connected")
         << (FrameID)"f4"
         << (FrameID)"f7"
         << PrequelSequence{{"f5","f6"},{"f4","f5"},{"f4","f6"},{"f6","f1"},{"f6","f8"},{"f10","f11"},{"f10","f9"},
                            {"f9","f7"},{"f7","f11"},{"f2","f1"},{"f2","f3"},{"f3","f6"}}
         << NoSuchLinkFound;
}

void transmemTest::throwsException() {

    QMatrix4x4 test = this->createRandTMatrix();

    // fetch test data
    QFETCH(FrameID,srcFrame);
    QFETCH(FrameID, destFrame);
    QFETCH(PrequelSequence, prequelSequence);
    QFETCH(ExceptionType, exceptionType);

    // create dummy timestamp and transformation matrix
    QMatrix4x4 m; Timestamp tStamp = std::chrono::high_resolution_clock::now();

    // create transmem object
    TransMem t;

    // run prequel
    for(std::pair<FrameID,FrameID> p : prequelSequence)
        t.registerLink((FrameID)p.first, (FrameID)p.second, tStamp, m);

    switch(exceptionType){
        case InvalidArgument: QVERIFY_EXCEPTION_THROWN(t.getLink(srcFrame, destFrame, tStamp), std::invalid_argument);
                              break;
        case NoSuchLinkFound:
                              QVERIFY_EXCEPTION_THROWN(t.getLink(srcFrame, destFrame, tStamp), NoSuchLinkFoundException);
                              break;
    };

}

void transmemTest::simpleQueries_data(){

    // initialize
    QTest::addColumn<QMatrix4x4>("transformation");

    // test set 1
    QTest::newRow("simple queries test 1 - just a rotation")
        << tRz(M_PI/4);

    // test set 2
    QTest::newRow("simple queries test 2 - just a translation")
        << tTr(-1.,0,3.4);

    // test set 3
    QTest::newRow("simple queries test 3 - two rotations")
        << tRz(9*M_PI/16)*tRz(M_PI/3);

    // test set 4
    QTest::newRow("simple queries test 4 - two translations")
        << tTr(4.4,-2.5,0.)*tTr(-1.,0,3.4);

    // test set 5
    QTest::newRow("simple queries test 5 - ")
       << tRz(M_PI/4)*tRz(M_PI/18)*tTr(-1.,0,3.4);

    // test set 6
    QTest::newRow(("simple queries test 6 - "))
        << tRz(M_PI/4)*tRz(M_PI/18)*tTr(-1.,0,3.4);

    // test set 7
    QTest::newRow(("simple queries test 7 - "))
        << tRx(M_PI)*tRz(123*M_PI/239)*tTr(-1.,0,0)*tRz(56*M_PI/57)*tTr(-1,-1,-1);
}

void transmemTest::simpleQueries(){

    // fetch test data
    QFETCH(QMatrix4x4, transformation);

    // create transmem object
    TransMem t;

    // timestamp & duration
    Timestamp tStamp = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds dt = std::chrono::milliseconds(100);

    t.registerLink("f1", "f2", tStamp, transformation);

    // get same transformation back if queried at the exact time in the same direction
    QMatrix4x4 ret = t.getLink("f1", "f2", tStamp);
    QVERIFY(compareHelper(transformation, ret, precision));

    // get the inverse transformation back if queried at the exact time in the opposite direction
    ret = t.getLink("f2", "f1", tStamp);
    QVERIFY(compareHelper(transformation.inverted(), ret, precision));

    // get the same transformation back if queried a little later in the same direction (just one entry)
    ret = t.getLink("f1", "f2", tStamp + dt);
    QVERIFY(compareHelper(transformation, ret, precision));

    // get the same transformation back if queried a little later in the opposite direction (just one entry)
    ret = t.getLink("f2", "f1", tStamp + dt);
    QVERIFY(compareHelper(transformation.inverted(), ret, precision));

    // get the same transformation back if queried a little earlier in the same direction (just one entry)
    ret = t.getLink("f1", "f2", tStamp - dt);
    QVERIFY(compareHelper(transformation, ret, precision));

    // get the same transformation back if queried a little earlier in the opposite direction (just one entry)
    ret = t.getLink("f2", "f1", tStamp - dt);
    QVERIFY(compareHelper(transformation.inverted(), ret, precision));


}

void transmemTest::simpleNStepQueries_data(){

    // initialize
    QTest::addColumn<FrameID>("srcFrame");
    QTest::addColumn<FrameID>("destFrame");
    QTest::addColumn<PrequelSequenceWithT>("prequelSequence");
    QTest::addColumn<QMatrix4x4>("refTransformation");

    // test set 1
    QMatrix4x4 tM21 = tRz(13*M_PI/89);
    QMatrix4x4 tM23 = tRx(M_PI/12);
    QMatrix4x4 tM43 = tRz(27*M_PI/4);

    QTest::newRow("simple n step queries test 1 - ")
        << (FrameID) "f4"
        << (FrameID) "f1"
        << PrequelSequenceWithT{{{"f2","f3"},{tM23}},{{"f2","f1"},{tM21}},{{"f4","f3"},{tM43}}}
        << tM21*tM23.inverted()*tM43;

}

void transmemTest::simpleNStepQueries(){

    // fetch test data
    QFETCH(FrameID,srcFrame);
    QFETCH(FrameID, destFrame);
    QFETCH(PrequelSequenceWithT, prequelSequence);
    QFETCH(QMatrix4x4, refTransformation);

    // create dummy timestamp
    Timestamp tStamp = std::chrono::high_resolution_clock::now();

    // create transmem object
    TransMem t;

    // run prequel
    for(std::pair < std::pair < FrameID, FrameID > , QMatrix4x4> p : prequelSequence){
        std::pair<FrameID, FrameID> fP = p.first;
        t.registerLink((FrameID)fP.first, (FrameID)fP.second, tStamp, (QMatrix4x4) p.second);
    }

    QMatrix4x4 ret = t.getLink(srcFrame,destFrame,tStamp);
    QVERIFY(compareHelper(refTransformation, ret, precision));

    t.dumpAsGraphML();
    t.dumpAsJSON();

}

bool transmemTest::compareHelper(const QMatrix4x4 &ref, const QMatrix4x4 &oth, double eps){

    for(unsigned int ii = 0; ii < 4; ii++)
        for(unsigned int jj = 0; jj < 4; jj++)
            if(std::fabs(ref(ii,jj)-oth(ii,jj)) > eps)
                return false;
    return true;
}

QMatrix4x4 transmemTest::createRandTMatrix(){

    double lowerBoundFloat = -1000;
    double upperBoundFloat =  1000;

    unsigned int maxNumbOfMatrices = 10;

    std::uniform_real_distribution<double> unif(lowerBoundFloat, upperBoundFloat);
    std::default_random_engine re;

    std::srand(std::time(0)); int numbOfMat = ( std::rand() % maxNumbOfMatrices ) + 5;

    QMatrix4x4 ret;

    for(int matCount = 0; matCount < numbOfMat; matCount++){

        // rotation or translation matrix
        std::srand(std::time(0)); int transOrRot = std::rand() % 2;

        switch(transOrRot){

        // translation
        case 0:
            ret = ret * tTr(unif(re), unif(re), unif(re));
            break;

        // rotation
        case 1:
            // x, y or z rotation
            std::srand(std::time(0)); int xyzRotation = std::rand() % 3;
            switch(xyzRotation){
            //x
            case 0:
                ret = ret * tRx(unif(re));
                break;
            //y
            case 1:
                ret = ret * tRx(unif(re));
                break;
            //z
            case 2:
                ret = ret * tRz(unif(re));
                break;
            }
            break;

        }
    }

    return ret;
}

QMatrix4x4  transmemTest::tRz(double a){    return QMatrix4x4(   1,              0,            0,  0,
                                                                 0,    std::cos(a), -std::sin(a),  0,
                                                                 0,    std::sin(a),       cos(a),  0,
                                                                 0,              0,            0,  1  );
                                       }

QMatrix4x4 transmemTest::tRx(double a){     return QMatrix4x4(   std::cos(a),   -std::sin(a),            0,  0,
                                                                 std::sin(a),    std::cos(a),            0,  0,
                                                                           0,              0,            1,  0,
                                                                           0,              0,            0,  1  );
                                      }

QMatrix4x4 transmemTest::tTr(double x, double y, double z) {return QMatrix4x4(   1, 0, 0, x,
                                                                                 0, 1, 0, y,
                                                                                 0, 0, 1, z,
                                                                                 0, 0, 0, 1 );
                                                           }

QTEST_MAIN(transmemTest)
