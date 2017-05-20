#include "test/include/transmemTest.h"

/*
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
        << MatHelper::getYRotMatrix(M_PI/4);

    // test set 2
    QTest::newRow("simple queries test 2 - just a translation")
        << MatHelper::getTransMatrix(-1.,0,3.4);

    // test set 3
    QTest::newRow("simple queries test 3 - two rotations")
        << MatHelper::getZRotMatrix(9*M_PI/16) * MatHelper::getXRotMatrix(M_PI/3);

    // test set 4
    QTest::newRow("simple queries test 4 - two translations")
        << MatHelper::getTransMatrix(4.4,-2.5,0.) * MatHelper::getTransMatrix(-1.,0,3.4);

    // test set 5
    QTest::newRow("simple queries test 5 - combination of a translation followed by two rotation")
       << MatHelper::getZRotMatrix(M_PI/4) * MatHelper::getYRotMatrix(M_PI/18) * MatHelper::getTransMatrix(-1.,0,3.4);

    // test set 6
    QTest::newRow(("simple queries test 6 - combination of some rotation and translation matrices"))
        << MatHelper::getXRotMatrix(M_PI) * MatHelper::getYRotMatrix(123*M_PI/239) * MatHelper::getTransMatrix(-1.,0,0)
                                          * MatHelper::getYRotMatrix(56*M_PI/57) * MatHelper::getTransMatrix(-1,-1,-1);
    // test set 7
    QTest::newRow(("simple queries test 7 - random matrices"))
        << MatHelper::getRandTransMatrix() * MatHelper::getRandTransMatrix();
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
    QMatrix4x4 tM21 = MatHelper::getXRotMatrix(13*M_PI/89);
    QMatrix4x4 tM23 = MatHelper::getYRotMatrix(M_PI/12);
    QMatrix4x4 tM43 = MatHelper::getZRotMatrix(27*M_PI/4);

    QTest::newRow("simple n step queries test 1 - just a small graph with 3 frame nodes only rotation")
        << (FrameID) "f4"
        << (FrameID) "f1"
        << PrequelSequenceWithT{{{"f2","f3"},{tM23}},{{"f2","f1"},{tM21}},{{"f4","f3"},{tM43}}}
        << tM21*tM23.inverted()*tM43;

    // test set 2
    tM21 = MatHelper::getTransMatrix(0.2,1.4,-0.22);
    tM23 = MatHelper::getTransMatrix(-0.9, 0.221, -0.707);
    tM43 = MatHelper::getTransMatrix(1.01, 4, 0.49);

    QTest::newRow("simple n step queries test 2 - just a small graph with 3 frame nodes and only translation ")
        << (FrameID) "f4"
        << (FrameID) "f1"
        << PrequelSequenceWithT{{{"f2","f3"},{tM23}},{{"f2","f1"},{tM21}},{{"f4","f3"},{tM43}}}
        << tM21*tM23.inverted()*tM43;

    // test set 3
    tM21 = MatHelper::getTransMatrix(3.2,-1.41,-1.22);
    tM23 = MatHelper::getYRotMatrix(11*M_PI/12);
    tM43 = MatHelper::getTransMatrix(-3.41, 0.1, 1);

    QTest::newRow("simple n step queries test 3 - just a small graph with 3 frame nodes, translation and rotation ")
        << (FrameID) "f4"
        << (FrameID) "f1"
        << PrequelSequenceWithT{{{"f2","f3"},{tM23}},{{"f2","f1"},{tM21}},{{"f4","f3"},{tM43}}}
        << tM21*tM23.inverted()*tM43;

    // prepare larger graph and prequel for further tests..
    std::unordered_map<unsigned int, QMatrix4x4> trans;
    for(unsigned int i = 1; i < 20; i++)
        trans.insert({i, MatHelper::getRandTransMatrix()});

    PrequelSequenceWithT prequel = PrequelSequenceWithT{

        {{"f1","f2"},{trans.at(1)}},{{"f1","f3"},{trans.at(2)}},{{"f3","f4"},{trans.at(4)}},
        {{"f4","f5"},{trans.at(5)}},{{"f5","f6"},{trans.at(6)}},{{"f5","f3"},{trans.at(3)}},
        {{"f5","f7"},{trans.at(7)}},{{"f7","f17"},{trans.at(17)}},{{"f17","f8"},{trans.at(8)}},
        {{"f17","f13"},{trans.at(13)}},{{"f12","f17"},{trans.at(12)}},{{"f11","f12"},{trans.at(11)}},
        {{"f10","f11"},{trans.at(10)}},{{"f10","f15"},{trans.at(16)}},{{"f11","f9"},{trans.at(9)}},
        {{"f16","f9"},{trans.at(18)}},{{"f15","f16"},{trans.at(19)}},{{"f15","f14"},{trans.at(14)}},
        {{"f15","f1"},{trans.at(15)}}

    };

    // test set 4
    QTest::newRow("simple n step queries test 4 - larger graph with random transformation matrices ")
        << (FrameID) "f1"
        << (FrameID) "f17"
        <<  prequel
        << trans.at(17)*trans.at(7)*trans.at(3).inverted()*trans.at(2);

    // test set 5
    QTest::newRow("simple n step queries test 5 - larger graph with random transformation matrices ")
        << (FrameID) "f11"
        << (FrameID) "f6"
        <<  prequel
        << trans.at(6)*trans.at(7).inverted()*trans.at(17).inverted()*trans.at(12)*trans.at(12);

    // test set 6
    QTest::newRow("simple n step queries test 6 - larger graph with random transformation matrices ")
        << (FrameID) "f5"
        << (FrameID) "f9"
        <<  prequel
        << trans.at(9)*(trans.at(11).inverted()*(trans.at(12).inverted()*(trans.at(17)*trans.at(7))));

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


void transmemTest::bestPointInTime_data(){

    // initialize
    QTest::addColumn<std::vector<updateSequence> >("upSeq");
    QTest::addColumn<query>("quer");

    Timestamp tStampNow = std::chrono::high_resolution_clock::now();

    // create data for first test
    // create update seq 1
    std::vector<Timestamp> ts1 = std::vector<Timestamp> {
            tStampNow + std::chrono::milliseconds(35),
            tStampNow + std::chrono::milliseconds(75),
            tStampNow + std::chrono::milliseconds(105),
            tStampNow + std::chrono::milliseconds(140),
    };

    updateSequence us1 = updateSequence { (FrameID)"f1", (FrameID)"f2", ts1, std::vector<QMatrix4x4>() };

    // create update seq 2
    std::vector<Timestamp> ts2 = std::vector<Timestamp> {
            tStampNow + std::chrono::milliseconds(10), tStampNow + std::chrono::milliseconds(30),
            tStampNow + std::chrono::milliseconds(50), tStampNow + std::chrono::milliseconds(65),
            tStampNow + std::chrono::milliseconds(75), tStampNow + std::chrono::milliseconds(95),
            tStampNow + std::chrono::milliseconds(105), tStampNow + std::chrono::milliseconds(125),
            tStampNow + std::chrono::milliseconds(140), tStampNow + std::chrono::milliseconds(160)
    };

    updateSequence us2 = updateSequence { (FrameID)"f3", (FrameID)"f2", ts2, std::vector<QMatrix4x4>() };

    // create vector of update sequences
    std::vector< updateSequence > uss = std::vector<updateSequence>{us1, us2};

    // create query
    query q = query{ (FrameID)"f1", (FrameID)"f3", tStampNow + std::chrono::milliseconds(140), QMatrix4x4()};

    // test set 1
    QTest::newRow("best point in time test 1")
        << uss
        << q;

    // create data for the second test
    tStampNow = std::chrono::high_resolution_clock::now();

    // create data for second test
    // create update seq 1
     ts1 = std::vector<Timestamp> {
            tStampNow + std::chrono::milliseconds(0),   tStampNow + std::chrono::milliseconds(10),
            tStampNow + std::chrono::milliseconds(20),  tStampNow + std::chrono::milliseconds(35),
            tStampNow + std::chrono::milliseconds(40),  tStampNow + std::chrono::milliseconds(50)
    };

    us1 = updateSequence { (FrameID)"f1", (FrameID)"f2", ts1, std::vector<QMatrix4x4>() };

    ts2 = std::vector<Timestamp> {
            tStampNow + std::chrono::milliseconds(85), tStampNow + std::chrono::milliseconds(95),
            tStampNow + std::chrono::milliseconds(100), tStampNow + std::chrono::milliseconds(110),
            tStampNow + std::chrono::milliseconds(120), tStampNow + std::chrono::milliseconds(130),
            tStampNow + std::chrono::milliseconds(140), tStampNow + std::chrono::milliseconds(145)
    };

    us2 = updateSequence { (FrameID)"f3", (FrameID)"f2", ts2, std::vector<QMatrix4x4>() };

    // create vector of update sequences
    uss = std::vector<updateSequence>{us1, us2};

    // create query
    q = query{ (FrameID)"f1", (FrameID)"f3", tStampNow + std::chrono::milliseconds(67), QMatrix4x4()};

    // test set 2
    QTest::newRow("best point in time test 2")
        << uss
        << q;

}

void transmemTest::bestPointInTime(){

    // fetch test data
    QFETCH(std::vector<updateSequence>,upSeq);
    QFETCH(query, quer);

    // create transmem object
    TransMem t;

    // run prequel
    for(updateSequence us : upSeq){
        for(unsigned int i = 0; i < us.timeStamps.size(); i++)
            t.registerLink(us.src, us.dest, us.timeStamps.at(i), QMatrix4x4());
    }

    std::chrono::milliseconds tStampRefSolMS = std::chrono::duration_cast<std::chrono::milliseconds>(quer.tStamp.time_since_epoch());
    quer.transfomation = t.getBestLink(quer.src, quer.dest, quer.tStamp);
    std::chrono::milliseconds tStampRetMS = std::chrono::duration_cast<std::chrono::milliseconds>(quer.tStamp.time_since_epoch());

    t.dumpAsJSON();

    QVERIFY( (tStampRefSolMS - std::chrono::milliseconds(5)).count() < tStampRetMS.count() &&
             tStampRetMS.count() < (tStampRefSolMS + std::chrono::milliseconds(5)).count());
}

*/

void transmemTest::dynamicTest_data(){


}


void transmemTest::dynamicTest(){

    TransMem transMem;

    std::vector< std::pair<FrameID, FrameID> > frameIDPairs({{"f1","f2"},{"f2","f4"}});

    std::unordered_map<std::string, std::vector<std::string> > path2Links;
    path2Links.insert({
                      {"f1-f4", std::vector<std::string>{"f1-f2", "f2-f4"}},
                      {"f4-f1", std::vector<std::string>{"f4-f2", "f2-f1"}},
                      {"f1-f2", std::vector<std::string>{"f1-f2"}},
                      {"f2-f4", std::vector<std::string>{"f2-f4"}},
                      {"f2-f1", std::vector<std::string>{"f2-f1"}},
                      {"f4-f2", std::vector<std::string>{"f4-f2"}}
                      });

    std::unordered_map<std::string, ptr2paramTransMat> link2paramTransMat;

    link2paramTransMat.insert({
                              {"f1-f2", &MatHelper::simpleParam1},
                              {"f2-f1", &MatHelper::simpleParam1Inv},
                              {"f2-f4", &MatHelper::simpleParam2},
                              {"f4-f2", &MatHelper::simpleParam2Inv}
                              });

    Solution sol(frameIDPairs, link2paramTransMat, path2Links);

    Timestamp tStamp = std::chrono::high_resolution_clock::now();
    FrameID src, dst;
    QMatrix4x4 tMat, res;

    src = "f1"; dst = "f2";

    // test case
    // **2**A**1**B**3**C**4**
    // B
    tMat = (link2paramTransMat.at(toLinkString(src,dst)))(tStamp);
    transMem.registerLink(src, dst, tStamp, tMat);
    sol.updateSolution(src, dst, tStamp);
    // C
    tStamp = tStamp + std::chrono::milliseconds(100);
    tMat = (link2paramTransMat.at(toLinkString(src,dst)))(tStamp);
    transMem.registerLink(src, dst, tStamp, tMat);
    sol.updateSolution(src, dst, tStamp);
    // A
    tStamp = tStamp - std::chrono::milliseconds(200);
    tMat = (link2paramTransMat.at(toLinkString(src,dst)))(tStamp);
    transMem.registerLink(src, dst, tStamp, tMat);
    sol.updateSolution(src, dst, tStamp);
    // 1
    tStamp = tStamp + std::chrono::milliseconds(50);
    res = transMem.getLink(src, dst, tStamp);
    QVERIFY(sol.checkTransformation(src, dst, tStamp, res));
    // 2
    tStamp = tStamp - std::chrono::milliseconds(100);
    res = transMem.getLink(src, dst, tStamp);
    QVERIFY(sol.checkTransformation(src, dst, tStamp, res));
    // 3
    tStamp = tStamp + std::chrono::milliseconds(200);
    res = transMem.getLink(src, dst, tStamp);
    QVERIFY(sol.checkTransformation(src, dst, tStamp, res));
    // 4
    tStamp = tStamp + std::chrono::milliseconds(100);
    res = transMem.getLink(src, dst, tStamp);
    QVERIFY(sol.checkTransformation(src, dst, tStamp, res));

    // test case
    // **2**D**1**E**3
    // D
    src = "f4"; dst = "f2";
    tStamp = tStamp + std::chrono::milliseconds(100);
    tMat = (link2paramTransMat.at(toLinkString(src,dst)))(tStamp);
    transMem.registerLink(src, dst, tStamp, tMat);
    sol.updateSolution(src, dst, tStamp);
    // E
    tStamp = tStamp + std::chrono::milliseconds(60);
    tMat = (link2paramTransMat.at(toLinkString(src,dst)))(tStamp);
    transMem.registerLink(src, dst, tStamp, tMat);
    sol.updateSolution(src, dst, tStamp);
    // 3
    tStamp = tStamp - std::chrono::milliseconds(10);
    res = transMem.getLink(src, dst, tStamp);
    QVERIFY(sol.checkTransformation(src, dst, tStamp, res));
    // 2
    tStamp = tStamp + std::chrono::milliseconds(20);
    res = transMem.getLink(src, dst, tStamp);
    QVERIFY(sol.checkTransformation(src, dst, tStamp, res));
    // 1
    tStamp = tStamp - std::chrono::milliseconds(30);
    res = transMem.getLink(src, dst, tStamp);
    QVERIFY(sol.checkTransformation(src, dst, tStamp, res));

}

bool transmemTest::compareHelper(const QMatrix4x4 &ref, const QMatrix4x4 &oth, double eps){

    for(unsigned int ii = 0; ii < 4; ii++)
        for(unsigned int jj = 0; jj < 4; jj++)
            if(std::fabs(ref(ii,jj)-oth(ii,jj)) > eps)
                return false;
    return true;
}

std::string transmemTest::toLinkString(const FrameID &src, const FrameID &dst){
    return (src + "-" + dst);
}

QTEST_MAIN(transmemTest)
