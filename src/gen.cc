//////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2004 Mellanox Technologies LTD. All rights reserved.
//
// This software is available to you under a choice of one of two
// licenses.  You may choose to be licensed under the terms of the GNU
// General Public License (GPL) Version 2, available from the file
// COPYING in the main directory of this source tree, or the
// OpenIB.org BSD license below:
//
//     Redistribution and use in source and binary forms, with or
//     without modification, are permitted provided that the following
//     conditions are met:
//
//      - Redistributions of source code must retain the above
//        copyright notice, this list of conditions and the following
//        disclaimer.
//
//      - Redistributions in binary form must reproduce the above
//        copyright notice, this list of conditions and the following
//        disclaimer in the documentation and/or other materials
//        provided with the distribution.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//////////////////////////////////////////////////////////////////////////////
//
// IB Packets Generator:
// Send IB credits one at a time. 
// 
// Internal Messages: 
// push - inject a new data credit into the Queues
//
// External Messages:
// sent - a vHoQ was sent by the VLA
//
// MODES:
// Normal mode - every push a new data credit will be inejcted  
// Flood mode - on init and "sent" event the Q depth will be checked and filled.
//

#include "ib_m.h"
#include "gen.h"
#include "vlarb.h"
#include <vec_file.h>

Define_Module(IBGenerator);

// use static to get a new packet id which is globaly unique
int IBGenerator::getPacketId() 
{
    static int id = 0;
    return(++id);
}

void IBGenerator::parseIntListParam(char *parName, std::vector<int> &out) 
{
    int cnt = 0;
    const char *str = par(parName);
    char *tmpBuf = new char[strlen(str)+1];
    strcpy(tmpBuf, str);
    char *entStr = strtok(tmpBuf, " ,");
    if (out.size()) out.clear();
    while (entStr) {
        cnt++;
        out.push_back(atoi(entStr));
        entStr = strtok(NULL, " ,");
    }
}


// allocate and init a new data credit
IBDataMsg *IBGenerator::getNewDataMsg()
{
    IBDataMsg *p_cred;

    char name[128];
    sprintf(name, "data-%d-%d", packetId, creditCounter);
    p_cred = new IBDataMsg(name, IB_DATA_MSG);
    p_cred->setSL(SL);
    p_cred->setVL(packetVL);
    if (RemainPktLength>=creditSize) {
        p_cred->setLength(creditSize*8); // length is in bits
        RemainPktLength -= creditSize;
    } else {
        p_cred->setLength(RemainPktLength*8); // length is in bits
        RemainPktLength = 0;
    }
    p_cred->setCreditSn(creditCounter);
    p_cred->setPacketId(packetId);
    p_cred->setSrcLid(srcLid);
    p_cred->setDstLid(dstLid);
    p_cred->setPacketLength(packetLength);
    p_cred->setPacketLengthBytes(packetLengthBytes);
    return p_cred;
}

// initialize the parameters for a new data packet.
// optionally being given a VL to use.
void IBGenerator::initIBPacketParams(int vl) 
{
    int rand = -1;
    // Current packet parameters
    packetId = getPacketId();

    // In case we are in flood mode, generate packet as part of message.
    // Else Generate each packet independantly with length packetLength

    if (firstPacket == 1) {
        if (GenModel == 0) {
            MsgLength = par("msgLengthInMTUs");
            MsgLength = MsgLength*mtu*creditSize;

        } else {
            if (trafficDist != TRF_APP) {
                MsgLength = par("msgLength");  
            } else {
                MsgLength = (*sizeSeq)[dstSeqIdx];
            }
        }
        RemainMsgLength = MsgLength;
    }

    // Generate packet Length
    if (RemainMsgLength == 0 ) {
        opp_error(" pktid =%d", packetId);
    }
    packetLengthBytes = getPacketLen();


    // Make sure the minimal size of packet is always 8 
    // Packet is at least 20B: LRH(8B) + BTH(12B)
    if (packetLengthBytes%creditSize<8 && packetLengthBytes%creditSize!=0) {
        packetLengthBytes += (8-packetLengthBytes%creditSize) ;
    }
    if (packetLengthBytes%creditSize == 0 ) {
        packetLength = packetLengthBytes/creditSize ;
    } else {
        packetLength = packetLengthBytes/creditSize +1;
    }

    creditCounter = 0; // count from 0 up.
    if (vl < 0)
        packetVL = par("VL");
    else
        packetVL = vl;

    // CHOSING DSTLID
    if (dstSeqMode != DST_BY_PARAM) {

        // only change dstLid if we crossed a message
        if (firstPacket ||  
            (GenModel == 1 && (RemainMsgLength == MsgLength || (trafficDist != TRF_FLOOD && trafficDist!=TRF_APP))) ||
            (GenModel == 0 && (mtuInMsgIdx + 1 == MsgLength/(creditSize*mtu)))) {

            if (dstBHS) {
                if (dstSeqMode == DST_HOT_SPOT && ((simTime() - dstBHS_start) > dstBurstHSDel)) {
                    if (!ev.disabled()) {
                        ev << "-I- " << fullPath() << " HS->SL simTime():"<< simTime()<< " dstBHS_start:" << dstBHS_start<< " diff:" << simTime() - dstBHS_start << " dstBurstHSDel:" << dstBurstHSDel << endl;
                    }
                    dstSeqMode = DST_SEQ_LOOP;
                    dstBHS_start = simTime();
                } else if (dstSeqMode == DST_SEQ_LOOP && ((simTime() - dstBHS_start) > dstBurstHSInterDel)) {

                    if (!ev.disabled()) {
                        ev << "-I- " << fullPath() << " SL->HS simTime():"<< simTime()<< " dstBHS_start:" << dstBHS_start<< " diff:" << simTime() - dstBHS_start << " dstBurstHSDel:" << dstBurstHSDel << endl;
                    }
                    dstSeqMode = DST_HOT_SPOT;
                    dstBHS_start = simTime();
                }
            }
            if (dstSeqMode == DST_HOT_SPOT) {
                rand = bernoulli(dstHotSpotPerc,0);
            } else {
                rand = 0;
            }

            if (dstSeqMode == DST_HOT_SPOT && rand ==1) {
                dstLid = parDstLid;
            } else if (dstSeqMode == DST_RANDOM) {
                dstLid = (*dstSeq)[intuniform(0,dstSeq->size()-1)];
            } else {
                dstLid = (*dstSeq)[dstSeqIdx];
                dstSeqIdx = ++dstSeqIdx;
                if (dstSeqIdx == dstSeq->size()) {
                    dstSeqIdx = 0;
                }
                // In case we are in bursty HS, we do not want to get the 
                // parameter Lid outside the window . 
                if (dstSeqMode == DST_SEQ_LOOP && dstBHS && dstLid == parDstLid) {
                    dstLid = (*dstSeq)[dstSeqIdx];
                    dstSeqIdx = ++dstSeqIdx;
                    if (dstSeqIdx == dstSeq->size()) {
                        dstSeqIdx = 0;
                    }
                }
                if ((dstSeqMode == DST_SEQ_ONCE) && (dstSeqIdx == 0)) {
                    dstSeqDone = 1;
                }
            }
            if (GenModel == 0) {
                mtuInMsgIdx = 0;
            }
        } else {
            if (GenModel == 0) {
                mtuInMsgIdx++;
            }
        }
    } else {
        dstLid = parDstLid;
    }

    if (dstLid == 0) {
        opp_error("Error: Packet %d has destination Lid = 0", packetId);
    }
    if (RemainMsgLength <= mtu*creditSize) {
        if (GenModel == 0) {
            MsgLength = par("msgLengthInMTUs");
            MsgLength = MsgLength*mtu*creditSize;
        } else { 
            if (trafficDist != TRF_APP) {
                MsgLength = par("msgLength");
            } else {
                MsgLength = (*sizeSeq)[dstSeqIdx];
            }   
        }
        RemainMsgLength = MsgLength;
    } else {
        RemainMsgLength = RemainMsgLength - packetLengthBytes;    
    }
    RemainPktLength = packetLengthBytes;
    firstPacket = 0; 
}


int IBGenerator::getPacketLen(){
    int PktLen = 0;
    if (GenModel == 0) {
        PktLen = par("packetlength");
        PktLen = PktLen*creditSize;
    } else {
        if (trafficDist== TRF_MIXLEN) {
            int TotProb = PktLenTotProb;
            int prob;
            int res; 
            for (int i = 0; i<PktLenDist.size();i++) {
                float k = (float) PktLenProb[i]/TotProb;
                res = bernoulli(k,990) ;
                if (res == 1) {
                    PktLen = PktLenDist[i];
                    return PktLen;
                } else {
                    TotProb -= PktLenProb[i];
                }
            }
        } else if (trafficDist == TRF_UNI) {
            PktLen = par("packetlengthbytes");
            if (GenModel ==0) {  // for ib_credits, generate packet length only multiple of 64
                if (PktLen%creditSize !=0 ) {
                    PktLen += (creditSize- (PktLen%creditSize));
                }
            }
        } else if (trafficDist == TRF_FLOOD || trafficDist == TRF_APP) {
            if (RemainMsgLength >= mtu*creditSize) {
                PktLen = mtu*creditSize;
            } else {
                PktLen = RemainMsgLength;
            }
        }
    }
    if (PktLen == 0) {
        opp_error(" PktLen is generated to 0" );
    }
    return PktLen;
}

void IBGenerator::initialize(){
    // init parameters and state variables
    // In non flood mode, parameters:
    burstLength = par("burstLength");
    burstNumber = par("burstNumber");
    burstCounter = burstLength;
    // General parameters
    width = par("width");
    speed = par("speed");
    SL = par("SL");
    srcLid = par("srcLid");
    creditSize = par("creditSize"); 
    dstHotSpotPerc = par("dstHotSpotPerc");
    GenModel = par("GenModel");    
    mtu = par("mtu");
    parDstLid = par("dstLid");

    firstPacket = 1;
    firstPacketSent = 1;
    mtuInMsgIdx = 0;
    dstSeqIdx = 0;
    dstSeqDone = 0;
    packetId = 0;
    dstLid = 0;
    TimeLastSent = 0;
    AccBytesSent = 0;

    // Destination Sequence Mode
    dstBHS = false;
    const char *dstSeqModePar = par("dstSeqMode");
    if (!strcmp(dstSeqModePar, "dstLid")) {
        dstSeqMode = DST_BY_PARAM;
    } else if (!strcmp(dstSeqModePar, "dstSeqOnce")) {
        dstSeqMode = DST_SEQ_ONCE;  
    } else if (!strcmp(dstSeqModePar, "dstSeqLoop")) {
        dstSeqMode = DST_SEQ_LOOP;
    } else if (!strcmp(dstSeqModePar, "dstRandom")) {
        dstSeqMode = DST_RANDOM;
    } else if (!strcmp(dstSeqModePar, "dstHotSpot")) {
        dstSeqMode = DST_HOT_SPOT;
    } else if (!strcmp(dstSeqModePar, "dstBurstHotSpot")) {
        dstSeqMode = DST_HOT_SPOT;
        dstBHS = true;
        dstBHS_start = simTime();
    } else {
        dstSeqMode = DST_BY_PARAM;      
    }
    // Traffic Distribution Mode
    const char *trafficDistPar = par("trafficDist");
    if (!strcmp(trafficDistPar,"trfFlood")) {
        trafficDist =  TRF_FLOOD;
    } else if (!strcmp(trafficDistPar,"trfUniform")) {
        trafficDist = TRF_UNI;
    } else if (!strcmp(trafficDistPar,"trfLengthMix")) {
        trafficDist = TRF_MIXLEN;
    } else if (!strcmp(trafficDistPar,"trfApp")) {
        trafficDist = TRF_APP;
        dstSeqMode = DST_SEQ_ONCE;
        dstBHS = false;
        GenModel = 1;
    } else {
        opp_error(" Unknown traffic Distribution " );
    }

    // destination related parameters
    if (dstSeqMode != DST_BY_PARAM) {
        const char *dstSeqVecFile = par("dstSeqVecFile");
        const int   dstSeqIndex = par("dstSeqIndex");
        vecFiles   *vecMgr = vecFiles::get();
        dstSeq = vecMgr->getIntVec(dstSeqVecFile, dstSeqIndex);
        if (dstSeq == NULL) {
            opp_error("fail to obtain dstSeq vector");
        }
        if (!ev.disabled())
            ev << "-I- Defined sequence of " << dstSeq->size() << " LIDs" << endl;
    }
    if (dstBHS) {
        dstBurstHSDel = par("dstBurstHSDel"); 
        dstBurstHSDel = dstBurstHSDel *1e-3; // (convert to s)
        dstBurstHSInterDel = par("dstBurstHSInterDel"); 
        dstBurstHSInterDel = dstBurstHSInterDel *1e-3; // (convert to s)
    }

    // Traffic related parameters
    PktLenTotProb = 0;
    if (trafficDist == TRF_MIXLEN) {
        parseIntListParam("PktLenDist",PktLenDist);
        parseIntListParam("PktLenProb",PktLenProb);
        for (int i = 0; i<PktLenProb.size(); i++) {
            PktLenTotProb += PktLenProb[i];
        }
        if (PktLenDist.size() !=PktLenProb.size() || PktLenDist.empty()) {
            opp_error("Error in ini file: PktLenDist has %d elements, PktLenProb has %d",PktLenDist.size(),PktLenProb.size()); 
        }
        if (PktLenTotProb == 0) {
            opp_error("PktLen prob all defined with priority 0");
        }
    }

    
    if (trafficDist == TRF_APP) {
        dstSeqMode = DST_SEQ_ONCE;
        const char *sizeSeqVecFile = par("sizeSeqVecFile");
        const int   sizeSeqIndex =par("dstSeqIndex");
        vecFiles   *vecMgr = vecFiles::get();
        sizeSeq = vecMgr->getIntVec(sizeSeqVecFile, sizeSeqIndex);
        if (sizeSeq == NULL) {
            opp_error("fail to obtain sizeSeq vector");
        }
        if (!ev.disabled())
            ev << "-I- Defined sequence of " << sizeSeq->size() << " message size" << endl;
    }

    // Queue first packets to send
    if (trafficDist == TRF_FLOOD || trafficDist == TRF_APP) {
        ev << "-I- " << fullPath() << " working in flood/application mode" << endl;

        const char *floodVLsStr = par("floodVLs");
        char *tmpBuf = new char[strlen(floodVLsStr)+1];
        strcpy(tmpBuf, floodVLsStr);
        char *vlStr = strtok(tmpBuf, " ,");
        while (vlStr) {
            floodVLQ(atoi(vlStr),1);
            vlStr = strtok(NULL, " ,");
        }
    } else {
        // The delay from one credit to the other in usec is defined as
        // 1/1gbps * 1usec * 10bits/byte * numBytes / ifc-width / ifc-speed-gbps
        genDelay_us = 1.0e-3 * 10 * creditSize / width / speed;

        // set starting packet parameters
        initIBPacketParams();

        // schedule first packet of first burst
        if (burstNumber>0 || (trafficDist == TRF_FLOOD) || (trafficDist == TRF_APP)) {
            scheduleAt(simTime(), new cMessage);
        }
    }
}

// find the VLA and check it HoQ is free...
int IBGenerator::isRemoteHoQFree(int vl){
    // find the VLA connected to the given port and
    // call its method for checking and setting HoQ
    cGate *p_gate = gate("out")->destinationGate();
    IBVLArb *p_vla = dynamic_cast<IBVLArb *>(p_gate->ownerModule());
    if ((p_vla == NULL) || strcmp(p_vla->name(), "vlarb")) {
        ev << "-E- " << fullPath() << " cannot get VLA for out port" << endl;
        ev.flush();
        exit(3);
    }

    int remotePortNum = p_gate->index();
    return(p_vla->isHoQFree(remotePortNum, vl));
}

void IBGenerator::sendDataOut(IBDataMsg *p_msg){

    double delay = 0;
    // In case we are in flood mode, we want to be able 
    // to send packet with delay between them. So far the intraburst delay
    // was not use in flood mode. This is our wayt to use it
    if (trafficDist == TRF_FLOOD || trafficDist == TRF_APP) {
        delay = par("intraBurstDelay");   // in nsec
        if (simTime() < TimeLastSent + delay*1e-9) {
            delay = TimeLastSent*1e+9 + delay - simTime()*1e+9;
        }
        TimeLastSent = simTime() + delay*1e-9;
    }
    // time stamp to enable tracking time in Fabric
    p_msg->setTimestamp(TimeLastSent);
    sendDelayed(p_msg, delay*1e-9, "out");

    if (!ev.disabled())
        ev << "-I- " << fullPath() 
        << " sending " << p_msg->name() 
        << " packetlength(B):" << p_msg->getPacketLengthBytes() 
        << " creditsn:" << p_msg->getCreditSn() 
        << " dstLid:" << p_msg->getDstLid() 
        << " @time(ns):" << simtimeToStr((simTime()+delay*1e-9)) 
        << endl;

    // For oBW calculations
    if (firstPacketSent) {
        firstPacketSent = 0;
        FirstPktSendTime = simTime();
    }
    AccBytesSent += p_msg->length()/8;
}

// push is an internal event for generating a new packet
// should only be happening if floodMode == 0
void IBGenerator::handlePush(cMessage *p_msg){

    if (trafficDist == TRF_FLOOD || trafficDist == TRF_APP) {
        opp_error("handlePush called with TRF_FLOOD / TRF_APP");
    }
    // get the new credit
    IBDataMsg *p_cred = getNewDataMsg();


    double delay = 1.0e-3 * 10 * p_cred->length()/ 8 / width / speed;
    // is the Q free?
    if (Q[packetVL].empty() && isRemoteHoQFree(packetVL)) {
        sendDataOut(p_cred);
    } else {
        // we can not send so we Q
        if (!ev.disabled())
            ev << "-I- " << fullPath() 
            << " queue packet:" << p_cred->name() 
            << " Time:" << simTime() 
            << endl;

        Q[packetVL].insert(p_cred);
    }

    // Do we need a new IB packet?
    if (++creditCounter == packetLength) {
        initIBPacketParams();

        //  Is it last packet of burst ?
        if (--burstCounter == 0) {
            if ((trafficDist != TRF_FLOOD || trafficDist == TRF_APP)) {
                burstNumber--;
            }
            // we need to send more burst?
            if ((trafficDist == TRF_FLOOD) || trafficDist == TRF_APP || burstNumber>0) {
                burstCounter = burstLength;
                delay = par("interBurstDelay");
                delay = delay*1e-3;
                delay += 1.0e-3 * 10 * p_cred->length()/ 8 / width / speed;
                scheduleAt(simTime()+delay*1e-6, p_msg);
            }
        } else {
            // schedule next sending within burst
            delay = par("intraBurstDelay");  \
            delay = delay*1e-3;
            delay += 1.0e-3 * 10 * p_cred->length()/ 8 / width / speed;
            scheduleAt(simTime()+delay*1e-6, p_msg);
        }
    } else {
        // but not in addition to the current data being sent
        scheduleAt(simTime()+delay*1e-6, p_msg);
    }
}

// keep at least 10 credits on this vL Q
// send the first one if HoQ is empty.
void IBGenerator::floodVLQ(int vl, int duringInit){
    IBDataMsg *p_cred;
    int wasEmpty;
    int doneMakingAll = 0;

    wasEmpty = Q[vl].empty();

    if ((dstSeqMode == DST_SEQ_ONCE) && (dstSeqDone != 0))
        doneMakingAll = 1;
    else
        doneMakingAll = 0;      

    while (!doneMakingAll && (Q[vl].length() < 10)) {
        // generate a new packet
        initIBPacketParams(vl);

        for (; creditCounter < packetLength; creditCounter++ ) {
            p_cred = getNewDataMsg();
            Q[vl].insert(p_cred);
        }
    }

    if (! duringInit) {
        if (wasEmpty && isRemoteHoQFree(vl) && !Q[vl].empty() ) {
            p_cred = (IBDataMsg *)Q[vl].pop();
            sendDataOut(p_cred);
        }
    } else {
        // we need to schedule somehow a late "sent"
        IBSentMsg *p_sent = new IBSentMsg("self-sent-on-init", IB_SENT_MSG);
        p_sent->setVL(vl);
        scheduleAt(simTime()+10e-9, p_sent);
    }
}

void IBGenerator::handleSent(IBSentMsg *p_sent){
    int vl = p_sent->getVL();
    // We can not just send - need to see if the HoQ is free...
    // NOTE : since we LOCK the HoQ when asking if HoQ is free we 
    // must make sure we have something to send before we ask about it
    if (!Q[vl].empty()) {
        if (isRemoteHoQFree(vl)) {
            IBDataMsg *p_msg = (IBDataMsg *)Q[vl].pop();
            if (!ev.disabled())
                ev << "-I- " << fullPath() << " de-queue packet:" << p_msg->name()<< " at time " << simTime() << endl;
            sendDataOut(p_msg);
        } else {
            if (!ev.disabled())
                ev << "-I- " << fullPath() << " HoQ not free for vl:" << vl << endl;
        }
    } else {
        if (!ev.disabled())
            ev << "-I- " << fullPath() << " nothing to send on vl:" << vl << endl;
    }
    if (trafficDist == TRF_FLOOD || trafficDist == TRF_APP) floodVLQ(vl);

    delete p_sent;
}

void IBGenerator::handleMessage(cMessage *p_msg){
    int msgType = p_msg->kind();
    if ( msgType == IB_SENT_MSG ) {
        handleSent((IBSentMsg *)p_msg);
    } else {
        handlePush(p_msg);
    }
}


void IBGenerator::finish()
{
    double oBW = AccBytesSent / (simTime() - FirstPktSendTime);
    ev << "STAT: " << fullPath() << " Gen Output BW (B/s):" << oBW  << endl; 
}
