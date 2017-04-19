/*! \file   transmem.cpp
    \brief  source file transmem.cpp
*/

#include "transmem.h"

/****************************
 * NOSUCHLINKFOUNDEXCEPTION *
 ****************************/

/****************************
 * TRANSMEM                 *
 ****************************/

void TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QQuaternion &qrot, const QQuaternion &qtrans){

    // TODO: check if normalized rotation quaternion and pure translation quaternion?!

    // check if link already exists

    // check if frames already exist
    auto IterSrcF = frameID2Frame.find(srcFrame);
    auto IterDstF = frameID2Frame.find(destFrame);

    Frame* sFPtr; Frame* dFPtr;

    // if a frame does not exist, create it
    if(IterSrcF == frameID2Frame.end()){
        Frame sF{srcFrame};
        frameID2Frame.insert({srcFrame, &sF});
        frames.push_back(sF);

        sFPtr = &sF;
    }
    else{
        sFPtr = (*IterSrcF).second;
    }

    if(IterDstF == frameID2Frame.end()){
        Frame dF{destFrame};
        frameID2Frame.insert({destFrame, &dF});
        frames.push_back(dF);

        dFPtr = &dF;
    }
    else{
        dFPtr = (*IterDstF).second;
    }

    // check if the link exists
    Link* lnkPtr = nullptr;
    sFPtr->connectionTo(destFrame, lnkPtr);

    // if the link does not exist, create it
    if(lnkPtr == nullptr){
        Link lnk = {sFPtr, dFPtr, storageTime};
        links.push_back(lnk);

        lnkPtr = &lnk;
        sFPtr->addLink(lnkPtr);
    }

    StampedTransformation ne = StampedTransformation{tstamp, qrot, qtrans};
    lnkPtr->addTransformation(srcFrame, ne);

}

/****************************
 * PATH                     *
 ****************************/
