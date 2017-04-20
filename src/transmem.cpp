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
        frames.emplace_back(Frame{srcFrame});
        frameID2Frame.insert({srcFrame, &frames.back()});

        sFPtr = &frames.back();
    }
    else{
        sFPtr = (*IterSrcF).second;
    }

    if(IterDstF == frameID2Frame.end()){
        frames.emplace_back(Frame{destFrame});
        frameID2Frame.insert({destFrame, &frames.back()});

        dFPtr = &frames.back();
    }
    else{
        dFPtr = (*IterDstF).second;
    }

    // check if the link exists
    Link* lnkPtr = nullptr;
    sFPtr->connectionTo(destFrame, lnkPtr);

    // if the link does not exist, create it
    if(lnkPtr == nullptr){
        links.emplace_back(Link{sFPtr, dFPtr, storageTime});

        lnkPtr = &links.back();
        sFPtr->addLink(lnkPtr);
    }

    StampedTransformation ne = StampedTransformation{tstamp, qrot, qtrans};
    lnkPtr->addTransformation(srcFrame, ne);

}

void TransMem::dumpAsGraphML(){

   GMLWriter writer;

   writer.write(this);

}

/****************************
 * PATH                     *
 ****************************/
