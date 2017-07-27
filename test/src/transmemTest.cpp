#include "test/include/transmemTest.h"

 typedef std::pair<FrameID, FrameID> linkPair;
 typedef std::pair< linkPair , QMatrix4x4 > linkTransPair;

/********************
 * HELPER FUNCTIONS *
 ********************/

QMatrix4x4 getZRotMatrix(double a){

    a = a * (M_PI/180);

    return QMatrix4x4(   cos(a),   -sin(a),    0,  0,
                         sin(a),    cos(a),    0,  0,
                              0,         0,    1,  0,
                              0,         0,    0,  1  );
    }

QMatrix4x4 getYRotMatrix(double a){

    a = a * (M_PI/180);

    return QMatrix4x4(  cos(a),     0,  sin(a),     0,
                             0,     1,       0,     0,
                       -sin(a),     0,  cos(a),     0,
                             0,     0,       0,     1   );
    }

QMatrix4x4 getXRotMatrix(double a){

    a = a * (M_PI/180);

    return QMatrix4x4(   1,         0,       0,  0,
                         0,    cos(a), -sin(a),  0,
                         0,    sin(a),  cos(a),  0,
                         0,         0,       0,  1  );
    }

QMatrix4x4 getRotMatrix(double angle, QVector3D axis){

    angle = angle * (M_PI/180);

    double c = cos(angle); double s = sin(angle); double t = 1. - cos(angle);
    double x = axis.x();   double y = axis.y();   double z = axis.z();

    return QMatrix4x4(    t*x*x+c,  t*x*y-s*z,  t*x*z+s*y,   0,
                        t*x*y+s*z,    t*y*y+c,  t*y*z-s*x,   0,
                        t*x*z-s*y,  t*y*z+s*x,    t*z*z+c,   0,
                                0,          0,          0,   1  );
    }

QMatrix4x4 getTransMatrix(double x, double y, double z) {

    return QMatrix4x4(   1, 0, 0, x,
                         0, 1, 0, y,
                         0, 0, 1, z,
                         0, 0, 0, 1 );
    }

QMatrix4x4 getRandTransMatrix(){

    std::uniform_real_distribution<double> unif1(0, 1);
    std::uniform_real_distribution<double> unif2(0, 1);
    std::uniform_real_distribution<double> unif3(0, 2 * M_PI);
    std::uniform_real_distribution<double> unif4(-1, 1);

    std::default_random_engine re(time(NULL));

    double theta = acos(2 * unif1(re) - 1.);
    double phi = 2 * M_PI * unif2(re);

    QVector3D axis = QVector3D(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));

    double angle = unif3(re);

    QMatrix4x4 ret =  getRotMatrix(angle, axis) *
                      getTransMatrix(unif4(re), unif4(re), unif4(re));

    return ret;
}

bool matrixComparator(const QMatrix4x4 &ref, const QMatrix4x4 &oth){

         for(unsigned int ii = 0; ii < 4; ii++)
             for(unsigned int jj = 0; jj < 4; jj++)
                 if(std::fabs(ref(ii,jj)-oth(ii,jj)) > 1e-05)
                     return false;
         return true;
}

QMatrix4x4 toMatrix4x4(const StampedAndRatedTransformation &t){

    QMatrix3x3 rotM = t.qRot.toRotationMatrix();

    QMatrix4x4 ret = QMatrix4x4(rotM);
    ret(0,3) = t.qTra.x(); ret(1,3) = t.qTra.y(); ret(2,3) = t.qTra.z();

    return ret;
}

QVector4D transformationComparator(const QMatrix4x4 &m1, const QMatrix4x4 &m2){

    double toRad = M_PI/180;

    // Split each transformation matrix in a translation vector vTra,
    // a rotation axis vRotA and an angle fAngl.

    float dataM1[]{m1(0,0),m1(0,1),m1(0,2),m1(1,0),m1(1,1),m1(1,2),m1(2,0),m1(2,1),m1(2,2)},
          dataM2[]{m2(0,0),m2(0,1),m2(0,2),m2(1,0),m2(1,1),m2(1,2),m2(2,0),m2(2,1),m2(2,2)};

    QMatrix3x3 rM1(dataM1), rM2(dataM2);
    QQuaternion qr1 = QQuaternion::fromRotationMatrix(rM1),
                qr2 = QQuaternion::fromRotationMatrix(rM2);

    // We first split each transformation in a translation vector vTra,
    // a rotation axis vRotA and an angle fAngl.
    QVector3D vTra1 = QVector3D(m1(0,3), m1(1,3), m1(2,3)),
              vTra2 = QVector3D(m2(0,3), m2(1,3), m2(2,3));

    QVector3D vRotA1, vRotA2; float fAngl1, fAngl2;

    qr1.getAxisAndAngle(&vRotA1, &fAngl1);
    qr2.getAxisAndAngle(&vRotA2, &fAngl2);

    // Make sure the two rotation axes are comparable.
    if(vRotA1.z() < 0 ){ vRotA1 *= -1; fAngl1 = 360 - fAngl1; }
    if(vRotA2.z() < 0 ){ vRotA2 *= -1; fAngl2 = 360 - fAngl2; }

    // Calculate ratio between the length of the two translation vectors and map them to the interval [0,2].
    double ratioLenT;
    double lenT1 = vTra1.length(), lenT2 = vTra2.length();
    if(lenT1 == 0 && lenT2 == 0)
        ratioLenT = 0;
    else if(lenT1 == 0 || lenT2 == 0)
        ratioLenT = 2;
    else
        ratioLenT = lenT2 > lenT1 ? (1-(lenT1/lenT2))*2. : (1-(lenT2/lenT1))*2.;

    // Calculate the distance between the two normalized translation vectors.
    vTra1.normalize(); vTra2.normalize();
    double distanceNormalT = fabs((vTra1 - vTra2).length());

    // Calculate the distance between a point perpendicular to the rotation axis
    // with length one and this point rotated by the angle difference.
    double distanceNormalPointR = sqrt(2*(1-cos((fAngl1-fAngl2)*toRad)));

    // Calculate the distance between the two rotation vectors.
    vRotA1.normalize(); vRotA2.normalize();
    double distanceNormalR = fabs((vRotA1 - vRotA2).length());

    // Return a vector encoding the "difference" between two transformations.
    // For equal transformation the method returns QVector4D(0,0,0,0).
                        //x             //y                 //z                     //w
    return QVector4D(   ratioLenT,     distanceNormalT,    distanceNormalPointR,   distanceNormalR);
}

/*****************
 * TRANSMEM TEST *
 *****************/

void transmemTest::throwsExceptionTest() {

    // Simple instruction which all should cause an exception.

    TransMem transMem; FrameID src, dst;
    Timestamp tStamp = std::chrono::high_resolution_clock::now();

    // Test 1
    // Empty transmem, all queries cause an exception.
    src = "f2"; dst = "f3";
    QVERIFY_EXCEPTION_THROWN(toMatrix4x4(transMem.getLink(src, dst, tStamp)), NoSuchLinkFoundException);
    qInfo() << "PASS   : Test 1";

    // Test 2
    // Emptry transmem, query between two identical frames.
    src = "fa"; dst = "fa";
    QVERIFY_EXCEPTION_THROWN(toMatrix4x4(transMem.getLink(src, dst, tStamp)), std::invalid_argument);
    qInfo() << "PASS   : Test 2";

    // Test 3
    // Transmem containing some links, query between two frames which are not connected.
    src = "f4"; dst = "f7";
    std::vector<linkPair> links = {
        {"f5","f6"},{"f4","f5"},{"f4","f6"},{"f6","f1"},{"f6","f8"},{"f3","f6"},
        {"f10","f11"},{"f10","f9"},{"f9","f7"},{"f7","f11"},{"f2","f1"},{"f2","f13"},
    };
    for(linkPair l : links)
        transMem.registerLink(l.first, l.second, tStamp, getRandTransMatrix());
    QVERIFY_EXCEPTION_THROWN(toMatrix4x4(transMem.getLink(src, dst, tStamp)), NoSuchLinkFoundException);
    qInfo() << "PASS   : Test 3";
}

void transmemTest::simpleQueriesTest() {

    /* Simple queries against the datastructure which stores just
     * one transformation entry on one single link.
     * Test if this simple mechanisms works correct. */

    TransMem transMem;

    Timestamp tStamp = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds dt = std::chrono::milliseconds(100);

    FrameID src; FrameID dst;
    src = "f1"; dst = "f2";

    //          f1 --------> f2

    QMatrix4x4 ret, insrt = getRandTransMatrix();
    transMem.registerLink(src, dst, tStamp, insrt);

    // Test 1
    // Get same transformation back if queried at the exact time in the same direction.
    ret = toMatrix4x4(transMem.getLink(src, dst, tStamp));
    QVERIFY(matrixComparator(insrt, ret));
    qInfo() << "PASS   : Test 1";

    // Test 2
    // Get the inverse transformation back if queried at the exact time in the opposite direction.
    ret = toMatrix4x4(transMem.getLink(dst, src, tStamp));
    QVERIFY(matrixComparator(insrt.inverted(), ret));
    qInfo() << "PASS   : Test 2";

    // Test 3
    // Get the same transformation back if queried a little later.
    ret = toMatrix4x4(transMem.getLink(src, dst, tStamp + dt));
    QVERIFY(matrixComparator(insrt, ret));
    qInfo() << "PASS   : Test 3";

    // Test 4
    // Get the inverse transformation back if queried a little earlier in the opposite direction.
    ret = toMatrix4x4(transMem.getLink(dst, src, tStamp - dt));
    QVERIFY(matrixComparator(insrt.inverted(), ret));
    qInfo() << "PASS   : Test 4";
}

void transmemTest::simpleMultiStepQueriesTest() {

    /* Queries against the datastructure which stores only one
     * transformation on a link.
     * We test if calculation along a multistep path works correct. */

    Timestamp tStamp = std::chrono::high_resolution_clock::now();
    TransMem transMem;

    // Create a container save the transformation stored on each link.
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
        links.at(indx).second = getRandTransMatrix();

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

    res = toMatrix4x4(transMem.getLink(src, dst, tStamp));
    QVERIFY(matrixComparator(sol1,res) || matrixComparator(sol2,res));
    qInfo() << "PASS   : Test 1";

    // Test 2
    // "f17" - "f15"
    src = "f17"; dst = "f15";

    sol1 = links.at(23).second*links.at(22).second.inverted()*
           links.at(21).second*links.at(7).second.inverted()*
           links.at(10).second*links.at(11).second.inverted();

    res = toMatrix4x4(transMem.getLink(src, dst, tStamp));
    QVERIFY(matrixComparator(sol1,res));
    qInfo() << "PASS   : Test 2";
}

void transmemTest::bestPointInTimeTest() {

    // Test whether the Timestamp returned from getBestLink(..) is correct.

    TransMem transMem1, transMem2; FrameID src, dst; QMatrix4x4 res, resInv;
    std::vector<TransMem*> transMems = {&transMem1, &transMem2};

    Timestamp tStampZ = std::chrono::high_resolution_clock::now();

    // Offset for updates
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
                transMems.at(transMemIndx)->registerLink(src,dst,tStampZ + std::chrono::milliseconds(o), getRandTransMatrix());
            }
        }
        transMemIndx++;
    }

    // Test 1
    // Get time when the "best" transformation betwen f1 and f3 is available for transMem1.
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
    // Get time when the "best" transformation betwen f1 and f3 is available for the transMem2.
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

void transmemTest::cachedBestLinksTest() {

    // Simple test to check the main functionality of the caching function.

    TransMem tm; FrameID src, dst; QMatrix4x4 res; Timestamp ts;

    QMatrix4x4 trans1 =  getRandTransMatrix();
    QMatrix4x4 trans2 =  getRandTransMatrix();

    src = "f1"; dst = "f27";

    Timestamp tsInsertion = std::chrono::high_resolution_clock::now();

    tm.registerLink(src, dst, tsInsertion, trans1);

    for(int i = 0; i < 5; i++)
       QVERIFY(matrixComparator(trans1, toMatrix4x4(tm.getBestLink(src, dst))));

    tm.registerLink(src, dst, tsInsertion - std::chrono::milliseconds(13), trans2);

    QVERIFY(matrixComparator(trans2, toMatrix4x4(tm.getBestLink(src, dst))));
    qInfo() << "PASS   : Test 1";

    dst = "f11";

    tm.registerLink(src, dst, tsInsertion + std::chrono::milliseconds(32), trans1);

    QVERIFY(matrixComparator(trans1, toMatrix4x4(tm.getBestLink(src,dst))));

    for(int i = 0; i < 5; i++)
       QVERIFY(matrixComparator(trans1, toMatrix4x4(tm.getBestLink(src, dst))));

    qInfo() << "PASS   : Test 1";
}

void transmemTest::linkConfidenceTest() {

    // A few tests to check if the averaging of the confidence works correct.

    typedef std::pair< linkPair, double > linkConfidencePair;

    TransMem tm1; FrameID src, dst;
    Timestamp tStamp = std::chrono::high_resolution_clock::now();

    std::vector<linkConfidencePair> linkConfidencePairs = {
        {{"f1","f2"},1},{{"f2","f3"},11},{{"f3","f4"},10},{{"f5","f4"},9},{{"f6","f5"},12},
        {{"f6","f7"},13},{{"f10","f7"},15},{{"f10","f8"},16},{{"f9","f10"},14},{{"f9","f11"},8},
        {{"f11","f5"},7},{{"f11","f2"},6},{{"f11","f12"},2},{{"f2","f12"},5},{{"f13","f12"},7},
        {{"f13","f11"},8},{{"f1","f14"},2},{{"f14","f12"},4},{{"f14","f15"},3}
    };
    for(linkConfidencePair lq : linkConfidencePairs)
        tm1.registerLink(lq.first.first, lq.first.second, tStamp, getRandTransMatrix(), lq.second);

    // Test 1
    src = "f2"; dst = "f4";
    StampedAndRatedTransformation res = tm1.getLink(src, dst, tStamp);
    QVERIFY(res.avgLinkConfidence == 10.5);
    qInfo() << "PASS   : Test 1";

    // Test 2
    src = "f8"; dst = "f1";
    res = tm1.getLink(src, dst, tStamp);
    QVERIFY(res.avgLinkConfidence == 9);
    qInfo() << "PASS   : Test 2";

    // Test 3
    // Updates without quality do not change the quality of the transformation.
    tm1.registerLink("f2", "f1", tStamp, getRandTransMatrix());
    res = tm1.getLink(src, dst, tStamp);
    QVERIFY(res.avgLinkConfidence == 9);
    qInfo() << "PASS   : Test 3";

    // Test 4
    // Updates with quality do change the quality of the transformation.
    tm1.registerLink("f2","f11", tStamp, getRandTransMatrix(), 11);
    res = tm1.getLink(src, dst, tStamp);
    QVERIFY(res.avgLinkConfidence == 10);
    qInfo() << "PASS   : Test 4";

    // Test 5
    // Quality of a single link.
    src = "f5"; dst = "f11";
    res = tm1.getLink(src, dst, tStamp);
    QVERIFY(res.avgLinkConfidence == 7);
    qInfo() << "PASS   : Test 5";
}

void transmemTest::timeDiffTest() {

    // Simple tests to check if the maxDistanceToEntry is calculculated rigth.

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
            tm.registerLink(src,dst,tStampZ + std::chrono::milliseconds(o), getRandTransMatrix());
        }
    }

    // Test 1
    src = "f1"; dst = "f2";
    res = tm.getLink(src, dst, tStampZ + std::chrono::milliseconds(20));
    QVERIFY(res.maxDistanceToEntry == 30);
    qInfo() << "PASS   : Test 1";

    // Test 2
    res = tm.getLink(src, dst, tStampZ + std::chrono::milliseconds(70));
    QVERIFY(res.maxDistanceToEntry == 20);
    qInfo() << "PASS   : Test 2";

    // Test 3
    src = "f3";
    res = tm.getLink(src, dst, tStampZ + std::chrono::milliseconds(10));
    QVERIFY(res.maxDistanceToEntry == 10);
    qInfo() << "PASS   : Test 3";

    // Test 4
    src = "f1"; dst = "f4";
    res = tm.getLink(src, dst, tStampZ + std::chrono::milliseconds(30));
    QVERIFY(res.maxDistanceToEntry == 20);
    qInfo() << "PASS   : Test 4";

    // Test 5
    src = "f7"; dst = "f1";
    res = tm.getLink(src, dst, tStampZ + std::chrono::milliseconds(60));
    QVERIFY(res.maxDistanceToEntry == 70);
    qInfo() << "PASS   : Test 5";
}

void transmemTest::pruningTest() {

    // A sequence of updates and queries on a single link to test the internal pruning.

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
        res = toMatrix4x4(transMem.getLink(src, dst, tStampQ));
        QVERIFY(matrixComparator(sol,res));
        qInfo() << "PASS   : Test " + QString::number(outerIndx + 1);
    }
}

void transmemTest::inversionTestSimple(){

    // After registering some links in the datastructure different queries
    // are made and the result is compared with the inverse query.

    TransMem transMem; FrameID src, dst; QMatrix4x4 res, resInv;

    Timestamp tStampQu, tStampA = std::chrono::high_resolution_clock::now();

    //  Test 1
    //  f1 - f2      1*****A*****2*3***B*****4

     src = "f1"; dst = "f2";
     Timestamp ts1 = tStampA - std::chrono::milliseconds(30);

     //A
     transMem.registerLink(src, dst, ts1 + std::chrono::milliseconds(30), getRandTransMatrix());
     //B
     transMem.registerLink(src, dst, ts1 + std::chrono::milliseconds(90), getRandTransMatrix());

     std::vector<int> offset = {0, 60, 70, 120};

     for(int o: offset){

         tStampQu = ts1 + std::chrono::milliseconds(o);
         res = toMatrix4x4(transMem.getLink(src,dst, tStampQu));
         resInv = toMatrix4x4(transMem.getLink(dst, src, tStampQu));

         QVERIFY(matrixComparator(res.inverted(), resInv));
     }
     qInfo() << "PASS   : Test 1";

     //  Test 2
     //  f2 - f4     5*C*********6*******D***7

     src = "f2"; dst = "f4";
     Timestamp ts5 = tStampA + std::chrono::milliseconds(10);

     //C
     transMem.registerLink(src, dst, ts5 + std::chrono::milliseconds(10), getRandTransMatrix());
     //D
     transMem.registerLink(src, dst, ts5 + std::chrono::milliseconds(100), getRandTransMatrix());

     offset = {0, 60, 120};    // 5,6,7
     for(int o : offset){

         tStampQu = ts5 + std::chrono::milliseconds(o);
         res = toMatrix4x4(transMem.getLink(src,dst, tStampQu));
         resInv = toMatrix4x4(transMem.getLink(dst, src, tStampQu));

         QVERIFY(matrixComparator(res.inverted(), resInv));

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
         res = toMatrix4x4(transMem.getLink(src,dst, tStampQu));
         resInv = toMatrix4x4(transMem.getLink(dst, src, tStampQu));

         QVERIFY(matrixComparator(res.inverted(), resInv));

     }
     qInfo() << "PASS   : Test 3";
}

void transmemTest::inversionTestwithAmbigousPath() {

    /* After the registration of some links most of the frames
     * are connected through multiple paths. Different queries are made
     * and compared with the inverse query to see if the datastructure
     * handles the ambiguouity due to multiple paths right. */

    TransMem transMem; FrameID src, dst; QMatrix4x4 res, resInv;

    Timestamp tStampQu, tStampZ = std::chrono::high_resolution_clock::now();

    // Offset for updates.
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
            transMem.registerLink(src,dst,tStampZ + std::chrono::milliseconds(o), getRandTransMatrix());
        }
    }

    //  Test 1
    //  f1 - f7

    src = "f1"; dst = "f7";

    // offset for queries
    offset = {0,15,55,72,115,142};
    for(int o: offset){

        tStampQu = tStampZ + std::chrono::milliseconds(o);
        res = toMatrix4x4(transMem.getLink(src,dst, tStampQu));
        resInv = toMatrix4x4(transMem.getLink(dst, src, tStampQu));

        QVERIFY(matrixComparator(res.inverted(), resInv));
    }
    qInfo() << "PASS   : Test 1";
}

void transmemTest::transComparatorTest() {

    // A few simple test to test the functionality of the transformation comparator.

    QVector4D diff; QMatrix4x4 m1, m2;

    // Test 1
    // Comparison of a random matrix with itself should yield zero.
    QMatrix4x4 m = getRandTransMatrix();
    diff = transformationComparator(m,m);
    QVERIFY(diff == QVector4D(0,0,0,0));
    qInfo() << "PASS   : Test 1";

    // Test 2
    // Rotation around different axis by equal angles.
    m1 = getXRotMatrix(90);
    m2 = getYRotMatrix(90);
    diff = transformationComparator(m1,m2);
    QVERIFY( fabs(diff.w() - sqrt(2.)) < 1e-05);
    qInfo() << "PASS   : Test 2";

    // Test 3
    // Rotation around different axis by equal angles.
    m1 = getYRotMatrix(21);
    m2 = getXRotMatrix(21);
    diff = transformationComparator(m1,m2);
    QVERIFY( fabs(diff.w() - sqrt(2.)) < 1e-05);
    qInfo() << "PASS   : Test 3";

    // Test 4
    // No rotation at all.
    m1 = getXRotMatrix(0);
    m2 = getYRotMatrix(0);
    diff = transformationComparator(m1,m2);
    QVERIFY(diff == QVector4D(0,0,0,0));
    qInfo() << "PASS   : Test 4";

    // Test 5
    // Rotation around the same axis by the same angle.
    m1 = getRotMatrix(156, QVector3D(3.3,1.3,-3).normalized());
    m2 = getRotMatrix(204, QVector3D(3.3,1.3,-3).normalized()*(-1.));
    diff = transformationComparator(m1,m2);
    QVERIFY(diff == QVector4D(0,0,0,0));
    qInfo() << "PASS   : Test 5";

    // Test 6
    // Rotation around same axes by different angles.
    m1 = getRotMatrix(35, QVector3D(0.4,3.1,0.4).normalized());
    m2 = getRotMatrix(215, QVector3D(0.4,3.1,0.4).normalized());
    diff = transformationComparator(m1,m2);
    QVERIFY(diff.z() == 2);
    qInfo() << "PASS   : Test 6";

    // Test 7
    m1 = getRotMatrix(300, QVector3D(1.2,-3,12).normalized());
    m2 = getRotMatrix(210, QVector3D(1.2,-3,12).normalized());
    diff = transformationComparator(m1,m2);
    QVERIFY( fabs(diff.z() - sqrt(2.)) < 1e-05);
    qInfo() << "PASS   : Test 7";

    // Test8
    m1 = getRotMatrix(12.34, QVector3D(1.2,-4,0.32).normalized());
    m2 = getRotMatrix(12.325, QVector3D(1.2,-4,0.32).normalized());
    diff = transformationComparator(m1,m2);
    QVERIFY( fabs(diff.z()) < 1e-03 && fabs(diff.w()) < 1e-03);
    qInfo() << "PASS   : Test 8";

}

QTEST_MAIN(transmemTest)
