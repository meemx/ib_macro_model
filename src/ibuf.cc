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
//
// The IBInBuf implements an IB Input Buffer
// See functional description in the header file.
//
#include "ib_m.h"
#include "ibuf.h"
#include "vlarb.h"
#include <vec_file.h>

Define_Module( IBInBuf );

long IBInBuf::getDoneMsgId()
{
    static long id = 0;
    return(id++);
}

void IBInBuf::parseIntListParam(char *parName, int numEntries, std::vector<int> &out) 
{
    int cnt = 0;
    const char *str = par(parName);
    char *tmpBuf = new char[strlen(str)+1];
    strcpy(tmpBuf, str);
    char *entStr = strtok(tmpBuf, " ,");
    while (entStr) {
        cnt++;
        out.push_back(atoi(entStr));
        entStr = strtok(NULL, " ,");
    }
    for (; cnt < numEntries; cnt++)
        out.push_back(0);
}

void IBInBuf::initialize()
{
    maxVL = par("maxVL");

    Q    = new cQueue*[gate("out")->size()];
    for (int pn = 0; pn < gate("out")->size(); pn++) {
        Q[pn] = new cQueue[maxVL+1];
    }
    recordVectors = par("recordVectors");
    maxBeingSent = par("maxBeingSent");
    numPorts = par("numPorts");
    totalBufferSize = par("totalBufferSize");      

    width = par("width");
    totalBufferSize = totalBufferSize*width/4;

    hcaIBuf = par("isHcaIBuf");
    if (hcaIBuf) {
        ev << "-I- " << fullPath() << " is HCA IBuf" << endl;
        FDB = NULL;
    } else {
        ev << "-I- " << fullPath() << " is Switch IBuf " << id() <<  endl;
        Switch = parentModule()->parentModule();
        if (Switch == NULL) {
            opp_error("Could not find grand parent Switch module");
        }

        ISWDelay = Switch->par("ISWDelay");
        const char *fdbsFile = Switch->par("fdbsVecFile"); 
        int         fdbIdx =   Switch->par("fdbIndex");
        vecFiles   *vecMgr = vecFiles::get();

        FDB = vecMgr->getIntVec(fdbsFile, fdbIdx);
        if (FDB == NULL) {
            opp_error("-E- Failed to obtain an FDB %s, %d", fdbsFile, fdbIdx);
        } else {
			 ev << "-I- " << fullPath() << " Obtained FDB of size:" << FDB->size() << endl;
		  }
    }

    // track how many parallel sends the IBUF do:
    numBeingSent = 0;
    WATCH(numBeingSent);

    // read Max Static parameters
    int totStatic = 0;
    int val;
    for ( int vl = 0; vl < maxVL+1; vl++ ) {
        char parName[12];
        sprintf(parName,"maxStatic%d", vl);
        val = par(parName);
        val = val*width/4;
        maxStatic.push_back(val);
        totStatic += val;
    }

    if (totStatic > totalBufferSize) {
        ev << "-E- " << fullPath() << " can not define total static (" << totStatic
        << ") > totalBufferSize (" << totalBufferSize << ")" << endl;
        ev.flush();
        exit(1);
    }

    // Initiazlize the statistical collection elements
    for ( int vl = 0; vl < maxVL+1; vl++ ) {
        char histName[40];
        sprintf(histName,"Used Static Credits for VL:%d", vl);
        staticUsageHist[vl].setName(histName);
        staticUsageHist[vl].setRangeAutoUpper(0, 10, 1);
    }

    // Initialize the data structures
    for ( int vl = 0; vl < maxVL+1; vl++ ) {
        ABR.push_back(0);
        staticFree.push_back(maxStatic[vl]);
        sendRxCred(vl, 1e-9);
    }

    WATCH_VECTOR(ABR);
    WATCH_VECTOR(staticFree);
    usedStaticCredits.setName("static credits used");

    curPacketId  = 0;  
    curPacketCredits = 0;
    curPacketVL = -1;
    curPacketOutPort = -1;
}

int IBInBuf::incrBusyUsedPorts() {
    if (numBeingSent < maxBeingSent) {
        numBeingSent++;
        if (!ev.disabled())
            ev << "-I- " << fullPath() << " increase numBeingSent to:"<< numBeingSent<< endl;
        return 1;
    }
    if (!ev.disabled())
        ev << "-I- " << fullPath() << " already sending:"<< numBeingSent<< endl;

    return 0;
};

// calculate FCCL and send to the OBUF
void IBInBuf::sendRxCred(int vl, double delay = 0)
{
    IBRxCredMsg *p_msg = new IBRxCredMsg("rxCred", IB_RXCRED_MSG);
    p_msg->setVL(vl);
    p_msg->setFCCL(ABR[vl] + staticFree[vl]);

    if (delay)
        sendDelayed(p_msg, delay, "rxCred");
    else
        send(p_msg, "rxCred");
}

// Forward the FCCL received in flow control packet to the VLA
void IBInBuf::sendTxCred(int vl, long FCCL)
{
    IBTxCredMsg *p_msg = new IBTxCredMsg("txCred", IB_TXCRED_MSG);
    p_msg->setVL(vl);
    p_msg->setFCCL(FCCL);
    send(p_msg, "txCred"); 
}

// Try to send the HoQ to the VLA
void IBInBuf::updateVLAHoQ(short int portNum, short vl)
{

    if (Q[portNum][vl].empty()) return;

    // find the VLA connected to the given port and
    // call its method for checking and setting HoQ
    cGate *p_gate = gate("out", portNum)->destinationGate();
    if (! hcaIBuf) {
        int remotePortNum = p_gate->index();
        IBVLArb *p_vla = dynamic_cast<IBVLArb *>(p_gate->ownerModule());
        if ((p_vla == NULL) || strcmp(p_vla->name(), "vlarb")) {
            ev << "-E- " << fullPath() << " fail to get VLA from out port:" << portNum << endl;
            ev.flush();
            exit(3);
        }
        if (! p_vla->isHoQFree(remotePortNum, vl))
            return;

        if (!ev.disabled())
            ev << "-I- " << fullPath() << " free HoQ on VLA:" << p_vla->fullPath() << " port:"
            << remotePortNum << " vl:" << vl << endl;
    }

    IBDataMsg *p_msg = (IBDataMsg *)Q[portNum][vl].pop();

    if (!hcaIBuf) {
        sendDelayed(p_msg,ISWDelay*1e-9, "out", portNum);
    } else {
        send(p_msg, "out", portNum);
    }
}

// Handle Push message
void IBInBuf::handlePush(IBWireMsg *p_msg)
{
    int msgType = p_msg->kind();
    if (msgType == IB_FLOWCTRL_MSG) {
        // FlowControl:
        // * The FCCL is delivered through the TxCred to the VLA
        // * ABR is overwritten with FCTBS (can cause RxCred send to OBUF
        //   FCCL = ABR + FREE is provided to the OBUF through the RxCred)
        IBFlowControl *p_flowMsg = (IBFlowControl *)p_msg;
        int vl = p_flowMsg->getVL();

        if (!ev.disabled())
            ev << "-I- " << fullPath() << " received flow control message:"
            << p_flowMsg->name() << " vl:" << vl
            << " FCTBS:" << p_flowMsg->getFCTBS()
            << " FCCL:" << p_flowMsg->getFCCL() << endl;

        sendTxCred(vl, p_flowMsg->getFCCL());

        // update ABR and send RxCred
        if (ABR[vl] > p_flowMsg->getFCTBS()) {
            ev << "-E- " << fullPath() << " how come we have ABR:" << ABR[vl] 
            << " > wire FCTBS" << p_flowMsg->getFCTBS() << "?" << endl;
        } else if (ABR[vl] < p_flowMsg->getFCTBS()) {
            ev << "-W- " << fullPath() << " how come we have ABR:" << ABR[vl] 
            << " < wire FCTBS" << p_flowMsg->getFCTBS() << " in lossles wires?" << endl;
            ABR[vl] = p_flowMsg->getFCTBS();
        }

        sendRxCred(vl);
        cancelAndDelete(p_msg);
    } else if (msgType == IB_DATA_MSG) {
        // Data Packet:
        IBDataMsg *p_dataMsg = (IBDataMsg *)p_msg;

        if (p_dataMsg->getCreditSn() == 0) {
            curPacketId = p_dataMsg->getPacketId();
            curPacketCredits = p_dataMsg->getPacketLength();
            curPacketVL = p_dataMsg->getVL();
            short dLid = p_dataMsg->getDstLid();

            if (dLid == 0) {
                opp_error("Error: dLid should not be 0 for %s", p_dataMsg->name());
            }

            if ((curPacketVL < 0) || (curPacketVL > maxVL+1)) {
                ev << "-E- " << fullPath() << " VL out of range:" << curPacketVL << endl;
                ev.flush();
                exit(1);
            }

            // do we have enough credits?
            if (curPacketCredits > staticFree[curPacketVL]) {
                ev << "-E- " << fullPath() << " Credits overflow. Required:" << curPacketCredits
                << " available:" << staticFree[curPacketVL] << endl;
                ev.flush();
                exit(1);
            }

            // lookup out port  on the first credit of a packet
            if (numPorts > 1) {
                if (dLid < FDB->size()) {
						curPacketOutPort = (*FDB)[dLid];
					 } else {
						// this is an error flow we need to pass the current message
						// to /dev/null
						curPacketOutPort = -1;
					 }
            } else {
				  curPacketOutPort = 0;
            }
        } else {
			 // Continuation Credit 
			 
			 // check the packet is the expected one:
            if (curPacketId != p_dataMsg->getPacketId()) {
                ev << "-E- " << fullPath() << " got unexpected packet:"
                << p_dataMsg->name() << " id:" << p_dataMsg->getPacketId()
                << " during packet:" << curPacketId << endl;
                ev.flush();
                exit(2);
            }
        }

        // check out port is valid
        if ((curPacketOutPort < 0) ||  (curPacketOutPort >= numPorts) ) {
            ev << "-E- " << fullPath() << " dropping packet:"
            << p_dataMsg->name() << " by FDB mapping to port:" << curPacketOutPort << endl;
            cancelAndDelete(p_dataMsg);
            return;
        }

        // Now consume a credit
		  staticFree[curPacketVL]--;
		  staticUsageHist[curPacketVL].collect(staticFree[curPacketVL]);
		  ABR[curPacketVL]++;
		  if (!ev.disabled()) {
			 ev << "-I- " << fullPath() << " New Static ABR[" 
				 << curPacketVL << "]:" << ABR[curPacketVL] << endl;
			 ev << "-I- " << fullPath() << " static queued msg:" << p_dataMsg->name() 
				 << " vl:" << curPacketVL
				 << ". still free:" << staticFree[curPacketVL] << endl;
        }

        // For every DATA "credit" (not only first one)
        // - Queue the Data in the Q[V]
        Q[curPacketOutPort][curPacketVL].insert(p_dataMsg);

        // - Send RxCred with updated ABR[VL] and FREE[VL] - only if the sum has
        //   changed which becomes the FCCL of the sent flow control
        sendRxCred(curPacketVL);

        // - If HoQ in the target VLA is empty - send the push event out.
        //   when the last packet is sent the "done" event has to be sent to all output
        //   ports, Note this also dequeue and send
        updateVLAHoQ(curPacketOutPort, curPacketVL);
    } else {
		ev << "-E- " << fullPath() << " push does not know how to handle message:"
			<< msgType << endl;
		cancelAndDelete(p_msg);
    }
}

// simple free static credits as reqired
void IBInBuf::simpleCredFree(int vl)
{
  // simply return the static credit first
  if (staticFree[vl] < maxStatic[vl]) {
	 staticFree[vl]++;
	 // need to update the OBUF we have one free... 
	 sendRxCred(vl);
  } else {
	 opp_error("Error: got a credit leak? trying to add credits to full buffer on vl  %d", vl);
  }
}

// Handle Sent Message
// A HoQ was sent by the VLA
void IBInBuf::handleSent(IBSentMsg *p_msg)
{
    // first calculate the total used static
    int totalUsedStatics = 0;
    for (int vli = 0; vli < maxVL+1; vli++) {
        totalUsedStatics += maxStatic[vli] - staticFree[vli]; 
    }

    if (!ev.disabled())
        recordVectors = par("recordVectors");

    if ( recordVectors ) {
        usedStaticCredits.record( totalUsedStatics );
    }

    // update the free credits acordingly:
    int vl = p_msg->getVL();
	 simpleCredFree(vl);

    // Only on switch ibuf we need to do the following...
    if (! hcaIBuf) {
        // if this was the last message we need to schedule a "done"
        // on each of the output ports
        if (p_msg->getWasLast()) {
            // first we decrement the number of outstanding sends
            if (numBeingSent <= 0) {
                ev << "-E- " << fullPath() << " got last message when numBeingSent:" 
                << numBeingSent << endl;
                ev.flush();
                exit(1);
            }

            numBeingSent--;
            if (!ev.disabled())
                ev << "-I- " << fullPath() << " completed send. down to:" 
                << numBeingSent << " sends" << endl;
            // inform all arbiters we drive
            int numOutPorts = gate("out")->size();
            for (int pn = 0; pn < numOutPorts; pn++) {
                char name[32];
                sprintf(name,"done-%d",getDoneMsgId());
                IBDoneMsg *p_doneMsg = new IBDoneMsg(name, IB_DONE_MSG);
                send(p_doneMsg, "out", pn);
            }
        }

        // if the data was sent we can expect the HoQ to be empty...
        updateVLAHoQ(p_msg->arrivalGate()->index(), p_msg->getVL());
    }

    cancelAndDelete(p_msg);
}

void IBInBuf::handleMessage(cMessage *p_msg)
{
    int msgType = p_msg->kind();
    if ( msgType == IB_SENT_MSG ) {
        handleSent((IBSentMsg *)p_msg);
    } else if ( (msgType == IB_DATA_MSG) || (msgType == IB_FLOWCTRL_MSG) ) {
        handlePush((IBWireMsg*)p_msg);
    } else {
        ev << "-E- " << fullPath() << " does not know how to handle message:" << msgType << endl;
        if (p_msg->isSelfMessage())
			 cancelAndDelete(p_msg);
        else
			 delete p_msg;
    }
}

void IBInBuf::finish()
{
  for ( int vl = 0; vl < maxVL+1; vl++ ) {
	 ev << "STAT: " << fullPath() << " VL:" << vl;
	 char histName[40];
	 ev << " Used Static Credits num/avg/max/std:"
		 << staticUsageHist[vl].samples()
		 << " / " << staticUsageHist[vl].mean()
		 << " / " << staticUsageHist[vl].max()
		 << " / " << staticUsageHist[vl].stddev()
		 << endl;
  } 
}
