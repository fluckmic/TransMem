#include "test/include/transmemTest.h"

 typedef std::pair<FrameID, FrameID> linkPair;
 typedef std::pair< linkPair , QMatrix4x4 > linkTransPair;

void transmemTest::throwsExceptionTest() {

    // Simple instruction which all should cause an exception.

    TransMem transMem; FrameID src, dst;
    Timestamp tStamp = std::chrono::high_resolution_clock::now();

    // Test 1
    // empty transmem, all queries cause an exception
    src = "f2"; dst = "f3";
    QVERIFY_EXCEPTION_THROWN(MatHelper::toMatrix4x4(transMem.getLink(src, dst, tStamp)), NoSuchLinkFoundException);
    qInfo() << "PASS   : Test 1";

    // Test 2
    // emptry transmem, query between two identical frames
    src = "fa"; dst = "fa";
    QVERIFY_EXCEPTION_THROWN(MatHelper::toMatrix4x4(transMem.getLink(src, dst, tStamp)), std::invalid_argument);
    qInfo() << "PASS   : Test 2";

    // Test 3
    // transmem containing some links, query between two frames which are not connected
    src = "f4"; dst = "f7";
    std::vector<linkPair> links = {
        {"f5","f6"},{"f4","f5"},{"f4","f6"},{"f6","f1"},{"f6","f8"},{"f3","f6"},
        {"f10","f11"},{"f10","f9"},{"f9","f7"},{"f7","f11"},{"f2","f1"},{"f2","f13"},
    };
    for(linkPair l : links)
        transMem.registerLink(l.first, l.second, tStamp, MatHelper::getRandTransMatrix());
    QVERIFY_EXCEPTION_THROWN(MatHelper::toMatrix4x4(transMem.getLink(src, dst, tStamp)), NoSuchLinkFoundException);
    qInfo() << "PASS   : Test 3";
}

void transmemTest::simpleQueriesTest(){

    /* Simple queries against the datastructure which stores just
       one transformation entry on one single link.

       Test if this simple mechanisms works correct. */

    TransMem transMem;

    Timestamp tStamp = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds dt = std::chrono::milliseconds(100);

    FrameID src; FrameID dst;
    src = "f1"; dst = "f2";

    //          f1 --------> f2

    QMatrix4x4 ret, insrt = MatHelper::getRandTransMatrix();
    transMem.registerLink(src, dst, tStamp, insrt);

    // Test 1
    // get same transformation back if queried at the exact time in the same direction
    ret = MatHelper::toMatrix4x4(transMem.getLink(src, dst, tStamp));
    QVERIFY(MatHelper::matrixComparator(insrt, ret));
    qInfo() << "PASS   : Test 1";

    // Test 2
    // get the inverse transformation back if queried at the exact time in the opposite direction
    ret = MatHelper::toMatrix4x4(transMem.getLink(dst, src, tStamp));
    QVERIFY(MatHelper::matrixComparator(insrt.inverted(), ret));
    qInfo() << "PASS   : Test 2";

    // Test 3
    // get the same transformation back if queried a little later
    ret = MatHelper::toMatrix4x4(transMem.getLink(src, dst, tStamp + dt));
    QVERIFY(MatHelper::matrixComparator(insrt, ret));
    qInfo() << "PASS   : Test 3";

    // Test 4
    // get the inverse transformation back if queried a little earlier in the opposite directio
    ret = MatHelper::toMatrix4x4(transMem.getLink(dst, src, tStamp - dt));
    QVERIFY(MatHelper::matrixComparator(insrt.inverted(), ret));
    qInfo() << "PASS   : Test 4";

}

void transmemTest::simpleMultiStepQueriesTest(){

    /* Queries against the datastructure which stores only one
       transformation on a link.

       We test if calculation along a multistep path works correct. */

    Timestamp tStamp = std::chrono::high_resolution_clock::now();
    TransMem transMem;

    // create a container save the transformation stored on each link
    QMatrix4x4 d;
    std::vector<linkTransPair> links = {
        /*0*/ {{"f1","f3"},d},  /*1*/ {{"f2","f1"},d},   /*2*/ {{"f2","f6"},d},  /*3*/ {{"f2","f4"},d},  /*4*/ {{"f3","f5"},d},
        /*5*/ {{"f20","f3"},d}, /*6*/ {{"f5","f6"},d},   /*7*/ {{"f5","f8"},d},  /*8*/ {{"f7","f5"},d},  /*9*/ {{"f8","f12"},d},
        /*10*/{{"f14","f8"},d}, /*11*/{{"f14","f15"},d}, /*12*/{{"f13","f14"},d},/*13*/{{"f12","f13"},d},/*14*/{{"f11","f12"},d},
        /*15*/{{"f20","f10"},d},/*16*/{{"f20","f9"},d},  /*17*/{{"f23","f20"},d},/*18*/{{"f23","f19"},d},/*19*/{{"f21","f9"},d},
        /*20*/ {{"f10","f4"},d},/*21*/{{"f5","f10"},d},  /*22*/{{"f22","f10"},d},/*23*/{{"f22","f17"},d},/*24*/{{"f17", "f18"},d},
        /*25*/{{"f17","f21"},d}
    };

    for(unsigned int indx = 0; indx < links.size(); indx++)
        links.at(indx).second = MatHelper::getRandTransMatrix();

    for(linkTransPair lp : links)
        transMem.registerLink(lp.first.first, lp.first.second, tStamp, lp.second);

    FrameID src, dst; QMatrix4x4 res, sol1, sol2;

    // Test 1
    // "f6" - "f23"
    src = "f6"; dst = "f23";

    sol1 = links.at(17).second.inverted()*links.at(5).second.inverted()*
           links.at(4).second.inverted()*links.at(6).second.inverted();
    sol2 = links.at(17).second.inverted()*links.at(15).second.inverted()*
           links.at(21).second*links.at(6).second.inverted();

    res = MatHelper::toMatrix4x4(transMem.getLink(src, dst, tStamp));
    QVERIFY(MatHelper::matrixComparator(sol1,res) || MatHelper::matrixComparator(sol2,res));
    qInfo() << "PASS   : Test 1";

    // Test 2
    // "f17" - "f15"
    src = "f17"; dst = "f15";

    sol1 = links.at(23).second*links.at(22).second.inverted()*
           links.at(21).second*links.at(7).second.inverted()*
           links.at(10).second*links.at(11).second.inverted();

    res = MatHelper::toMatrix4x4(transMem.getLink(src, dst, tStamp));
    QVERIFY(MatHelper::matrixComparator(sol1,res));
    qInfo() << "PASS   : Test 2";
}

void transmemTest::bestPointInTimeTest(){

    /* Test whether the Timestamp returned from getBestLink(..) is correct. */

    TransMem transMem1, transMem2; FrameID src, dst; QMatrix4x4 res, resInv;
    std::vector<TransMem*> transMems = {&transMem1, &transMem2};

    Timestamp tStampZ = std::chrono::high_resolution_clock::now();

    // offset for updates
    std::vector <std::vector<int> > offsets = {
        { -1,35,75,105,140,
          -1,10,30,50,65,75,95,105,125,140,160},
        { -1,0,10,20,35,40,50,
          -1,85,95,100,110,120,130,140,145},
    };

    std::vector<linkPair> links = {{"f1","f2"},{"f3","f2"}};

    unsigned int transMemIndx = 0;
    for(std::vector<int> offset : offsets){
        unsigned int linkIndx = 0;
        for(int o : offset){

            if(o < 0){
                src = links.at(linkIndx).first;
                dst = links.at(linkIndx).second;
                linkIndx++;
            }
            else{
                transMems.at(transMemIndx)->registerLink(src,dst,tStampZ + std::chrono::milliseconds(o), MatHelper::getRandTransMatrix());
            }
        }
        transMemIndx++;
    }

    // Test 1
    // get time when the "best" transformation betwen f1 and f3 is available for transMem1
    src = "f1"; dst = "f3";

    StampedAndRatedTransformation tr = transMems.at(0)->getBestLink(src, dst);
    Timestamp bestTimestamp = tr.time;

    Timestamp solBestTimestamp = tStampZ +  std::chrono::milliseconds(140);

    auto tStampRetMS =
            (std::chrono::duration_cast<std::chrono::milliseconds>(bestTimestamp.time_since_epoch())).count();
    auto tStampRefSolMS =
            (std::chrono::duration_cast<std::chrono::milliseconds>(solBestTimestamp.time_since_epoch())).count();

    QVERIFY(tStampRefSolMS - 5 < tStampRetMS && tStampRetMS < tStampRefSolMS + 5);
    qInfo() << "PASS   : Test 1";


    // Test 2
    // get time when the "best" transformation betwen f1 and f3 is available for the transMem2
    src = "f1"; dst = "f3";

    tr = transMems.at(1)->getBestLink(src, dst);
    bestTimestamp = tr.time;

    solBestTimestamp = tStampZ +  std::chrono::milliseconds(67);

    tStampRetMS =
            (std::chrono::duration_cast<std::chrono::milliseconds>(bestTimestamp.time_since_epoch())).count();
    tStampRefSolMS =
            (std::chrono::duration_cast<std::chrono::milliseconds>(solBestTimestamp.time_since_epoch())).count();

    QVERIFY(tStampRefSolMS - 5 < tStampRetMS && tStampRetMS < tStampRefSolMS + 5);
    qInfo() << "PASS   : Test 2";
}

void transmemTest::cachedBestLinksTest(){

    /* Simple test to check the main functionality of the caching function */

    TransMem tm; FrameID src, dst; QMatrix4x4 res; Timestamp ts;

    QMatrix4x4 trans1 =  MatHelper::getRandTransMatrix();
    QMatrix4x4 trans2 =  MatHelper::getRandTransMatrix();

    src = "f1"; dst = "f27";

    Timestamp tsInsertion = std::chrono::high_resolution_clock::now();

    tm.registerLink(src, dst, tsInsertion, trans1);

    for(int i = 0; i < 5; i++)
       QVERIFY(MatHelper::matrixComparator(trans1, MatHelper::toMatrix4x4(tm.getBestLink(src, dst))));

    tm.registerLink(src, dst, tsInsertion - std::chrono::milliseconds(13), trans2);

    QVERIFY(MatHelper::matrixComparator(trans2, MatHelper::toMatrix4x4(tm.getBestLink(src, dst))));
    qInfo() << "PASS   : Test 1";

    dst = "f11";

    tm.registerLink(src, dst, tsInsertion + std::chrono::milliseconds(32), trans1);

    QVERIFY(MatHelper::matrixComparator(trans1, MatHelper::toMatrix4x4(tm.getBestLink(src,dst))));

    for(int i = 0; i < 5; i++)
       QVERIFY(MatHelper::matrixComparator(trans1, MatHelper::toMatrix4x4(tm.getBestLink(src, dst))));

    qInfo() << "PASS   : Test 1";
}

void transmemTest::linkQualityTest() {

    typedef std::pair< linkPair, double > linkQualityPair;

    TransMem tm1; FrameID src, dst;
    Timestamp tStamp = std::chrono::high_resolution_clock::now();

    std::vector<linkQualityPair> linkQualityPairs = {
        {{"f1","f2"},1},{{"f2","f3"},11},{{"f3","f4"},10},{{"f5","f4"},9},{{"f6","f5"},12},
        {{"f6","f7"},13},{{"f10","f7"},15},{{"f10","f8"},16},{{"f9","f10"},14},{{"f9","f11"},8},
        {{"f11","f5"},7},{{"f11","f2"},6},{{"f11","f12"},2},{{"f2","f12"},5},{{"f13","f12"},7},
        {{"f13","f11"},8},{{"f1","f14"},2},{{"f14","f12"},4},{{"f14","f15"},3}
    };
    for(linkQualityPair lq : linkQualityPairs)
        tm1.registerLink(lq.first.first, lq.first.second, tStamp, MatHelper::getRandTransMatrix(), lq.second);

    // Test 1
    src = "f2"; dst = "f4";
    StampedAndRatedTransformation res = tm1.getLink(src, dst, tStamp);
    QVERIFY(res.avgLinkQuality == 10.5);
    qInfo() << "PASS   : Test 1";

    // Test 2
    src = "f8"; dst = "f1";
    res = tm1.getLink(src, dst, tStamp);
    QVERIFY(res.avgLinkQuality == 9);
    qInfo() << "PASS   : Test 2";

    // Test 3
    // updates without quality do not change the quality of the transformation
    tm1.registerLink("f2", "f1", tStamp, MatHelper::getRandTransMatrix());
    res = tm1.getLink(src, dst, tStamp);
    QVERIFY(res.avgLinkQuality == 9);
    qInfo() << "PASS   : Test 3";

    // Test 4
    // updates with quality do change the quality of the transformation
    tm1.registerLink("f2","f11", tStamp, MatHelper::getRandTransMatrix(), 11);
    res = tm1.getLink(src, dst, tStamp);
    QVERIFY(res.avgLinkQuality == 10);
    qInfo() << "PASS   : Test 4";

    // Test 5
    // quality of a single link
    src = "f5"; dst = "f11";
    res = tm1.getLink(src, dst, tStamp);
    QVERIFY(res.avgLinkQuality == 7);
    qInfo() << "PASS   : Test 5";
}

void transmemTest::timeDiffTest(){

    TransMem tm; FrameID src, dst; StampedAndRatedTransformation res;

    Timestamp tStampZ = std::chrono::high_resolution_clock::now();

    // offset for updates
    std::vector<int> offset = { -1,10,50,
                                -1,20,100,
                                -1,30,
                                -1,10,40,100,
                                -1,20,50,130,
                                -1,30,110,
                                -1,10,50,70,100,130
                               };

    std::vector<linkPair> links = {{"f1","f2"},{"f3","f2"},{"f3","f4"},{"f2","f4"},
                                   {"f4","f5"},{"f6","f5"},{"f6","f7"}};

    unsigned int linkIndx = 0;
    for(int o : offset){

        if(o < 0){
            src = links.at(linkIndx).first;
            dst = links.at(linkIndx).second;
            linkIndx++;
        }
        else{
            tm.registerLink(src,dst,tStampZ + std::chrono::milliseconds(o), MatHelper::getRandTransMatrix());
        }
    }

    // Test 1
    src = "f1"; dst = "f2";
    res = tm.getLink(src, dst, tStampZ + std::chrono::milliseconds(20));
    QVERIFY(res.avgDistanceToEntry == 30);
    qInfo() << "PASS   : Test 1";

    // Test 2
    res = tm.getLink(src, dst, tStampZ + std::chrono::milliseconds(70));
    QVERIFY(res.avgDistanceToEntry == 20);
    qInfo() << "PASS   : Test 2";

    // Test 3
    src = "f3";
    res = tm.getLink(src, dst, tStampZ + std::chrono::milliseconds(10));
    QVERIFY(res.avgDistanceToEntry == 10);
    qInfo() << "PASS   : Test 3";

    // Test 4
    src = "f1"; dst = "f4";
    res = tm.getLink(src, dst, tStampZ + std::chrono::milliseconds(30));
    QVERIFY(res.avgDistanceToEntry == 20);
    qInfo() << "PASS   : Test 4";

    // Test 5
    src = "f7"; dst = "f1";
    res = tm.getLink(src, dst, tStampZ + std::chrono::milliseconds(60));
    QVERIFY(res.avgDistanceToEntry == 36);
    qInfo() << "PASS   : Test 5";
}


void transmemTest::pruningTest(){

    /* A sequence of updates and queries on a single
       link to test the internal pruning. */

    Timestamp tStampZ = std::chrono::high_resolution_clock::now(),
              tStampQ = tStampZ - std::chrono::seconds(2);

     QMatrix4x4 res, sol;

    TransMem transMem; FrameID src = "f1", dst = "f2";

    typedef std::pair < int, QMatrix4x4 > offsetTransPair;

    QMatrix4x4 d;
    std::vector<offsetTransPair> offsetTransPairs = {
         /*0*/{0,d},  /*1*/{4,d},  /*2*/{6,d},  /*3*/{8,d},  /*4*/{10,d}, /*5*/{12,d}, /*6*/{14,d},
         /*7*/{16,d}, /*8*/{18,d}, /*9*/{22,d},/*10*/{25,d},/*11*/{28,d},/*12*/{30,d},/*13*/{33,d},
        /*14*/{36,d},/*15*/{39,d},/*16*/{42,d},/*17*/{44,d},/*18*/{46,d},/*18*/{48,d},/*19*/{50,d},
        /*20*/{53,d}
    };

    std::vector<unsigned int> queryTime = {3,5,7,9,12,13,18,22};
    std::vector<unsigned int> solutionNr = {0,0,1,3,8,9,14,17};

    unsigned int innerIndx = 0;
    for(unsigned int outerIndx = 0; outerIndx < queryTime.size(); outerIndx++){
        unsigned int nextQuery = queryTime.at(outerIndx);

        for(; innerIndx < nextQuery; innerIndx++)
            transMem.registerLink(src, dst, tStampZ + std::chrono::seconds(offsetTransPairs.at(innerIndx).first),
                                  offsetTransPairs.at(innerIndx).second);

        sol = offsetTransPairs.at(solutionNr.at(outerIndx)).second;
        res = MatHelper::toMatrix4x4(transMem.getLink(src, dst, tStampQ));
        QVERIFY(MatHelper::matrixComparator(sol,res));
        qInfo() << "PASS   : Test " + QString::number(outerIndx + 1);
    }

}

void transmemTest::inversionTestSimple(){

    /*  After registering some links in the datastructure different queries
        are made and the result is compared with the inverse query.  */

    TransMem transMem; FrameID src, dst; QMatrix4x4 res, resInv;

    Timestamp tStampQu, tStampA = std::chrono::high_resolution_clock::now();

    //  Test 1
    //  f1 - f2      1*****A*****2*3***B*****4

     src = "f1"; dst = "f2";
     Timestamp ts1 = tStampA - std::chrono::milliseconds(30);

     //A
     transMem.registerLink(src, dst, ts1 + std::chrono::milliseconds(30), MatHelper::getRandTransMatrix());
     //B
     transMem.registerLink(src, dst, ts1 + std::chrono::milliseconds(90), MatHelper::getRandTransMatrix());

     std::vector<int> offset = {0, 60, 70, 120};

     for(int o: offset){

         tStampQu = ts1 + std::chrono::milliseconds(o);
         res = MatHelper::toMatrix4x4(transMem.getLink(src,dst, tStampQu));
         resInv = MatHelper::toMatrix4x4(transMem.getLink(dst, src, tStampQu));

         QVERIFY(MatHelper::matrixComparator(res.inverted(), resInv));
     }
     qInfo() << "PASS   : Test 1";

     //  Test 2
     //  f2 - f4     5*C*********6*******D***7

     src = "f2"; dst = "f4";
     Timestamp ts5 = tStampA + std::chrono::milliseconds(10);

     //C
     transMem.registerLink(src, dst, ts5 + std::chrono::milliseconds(10), MatHelper::getRandTransMatrix());
     //D
     transMem.registerLink(src, dst, ts5 + std::chrono::milliseconds(100), MatHelper::getRandTransMatrix());

     offset = {0, 60, 120};    // 5,6,7
     for(int o : offset){

         tStampQu = ts5 + std::chrono::milliseconds(o);
         res = MatHelper::toMatrix4x4(transMem.getLink(src,dst, tStampQu));
         resInv = MatHelper::toMatrix4x4(transMem.getLink(dst, src, tStampQu));

         QVERIFY(MatHelper::matrixComparator(res.inverted(), resInv));

     }
     qInfo() << "PASS   : Test 2";

     //  Test 3
     //  f1 - f4     1*****A*****2*3***B*****4
     //                      5*C*********6*******D***7
     //                  8****9******1*****1*******1
     //                              0     1       2

     src = "f1"; dst = "f4";
     Timestamp ts8 = tStampA - std::chrono::milliseconds(10);

     offset = {0,15,50,80,120};     // 8,9,10,11,12
     for(int o : offset){

         tStampQu = ts8 + std::chrono::milliseconds(o);
         res = MatHelper::toMatrix4x4(transMem.getLink(src,dst, tStampQu));
         resInv = MatHelper::toMatrix4x4(transMem.getLink(dst, src, tStampQu));

         QVERIFY(MatHelper::matrixComparator(res.inverted(), resInv));

     }
     qInfo() << "PASS   : Test 3";
}

void transmemTest::inversionTestwithAmbigousPath(){

    /* After the registration of some links most of the frames
       are connected through multiple paths. Different queries are made
       and compared with the inverse query to see if the datastructure
       handles the ambiguouity due to multiple paths right. */

    TransMem transMem; FrameID src, dst; QMatrix4x4 res, resInv;

    Timestamp tStampQu, tStampZ = std::chrono::high_resolution_clock::now();

    // offset for updates
    std::vector<int> offset = {
        -1,30,70,85,110,135,
        -1,10,45,100,140,
        -1,20,75,120,
        -1,5,25,50,65,100,135,
        -1,10,20,40,60,85,110,130,
        -1,5,30,75,105,135,
        -1,10,35,90,120,
        -1,15,60,80,110,140,
        -1,25,40,70,85,130,
    };

    std::vector<linkPair> links = {
        {"f1","f3"},{"f1","f2"},{"f3","f5"},{"f3","f4"},{"f2","f4"},
        {"f2","f5"},{"f5","f6"},{"f4","f6"},{"f6","f7"}
    };

    unsigned int linkIndx = 0;
    for(int o : offset){

        if(o < 0){
            src = links.at(linkIndx).first;
            dst = links.at(linkIndx).second;
            linkIndx++;
        }
        else{
            transMem.registerLink(src,dst,tStampZ + std::chrono::milliseconds(o), MatHelper::getRandTransMatrix());
        }
    }

    //  Test 1
    //  f1 - f7

    src = "f1"; dst = "f7";

    // offset for queries
    offset = {0,15,55,72,115,142};
    for(int o: offset){

        tStampQu = tStampZ + std::chrono::milliseconds(o);
        res = MatHelper::toMatrix4x4(transMem.getLink(src,dst, tStampQu));
        resInv = MatHelper::toMatrix4x4(transMem.getLink(dst, src, tStampQu));

        QVERIFY(MatHelper::matrixComparator(res.inverted(), resInv));
    }
    qInfo() << "PASS   : Test 1";
}


QTEST_MAIN(transmemTest)
