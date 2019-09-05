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
// IB Device Port Input Buffer
// ============================
// Ports:
// * in     - where data arrives into the buffer
// * out[]  - through which they leave to each target out port VLArb
// * sent[] - gets message from the VLArb to inform HOQ completed transmission
// * rxCred - forward ABR+FREE of the local buffer to the OBUF
// * txCred - provide update of FCCL from received flow control to the
//            VLA of this port
// * done   - When done sending a packet provide signal to all the VLAs
//            connected to the IBUF about the change in number of busy ports.
//
// Parameters:
// MaxStatic[vl]  - max static credits allocated for each VL
// BufferSize     - the input buffer size in bytes
//
// External Events:
// * push - data is available on the input (either flow control or credit)
// * sent - the hoq was sent from the VLArb
//
// Functionality:
// * On Push the packet is classified to be flow control or data:
//   If it is flow control info:
//     The FCCL is delivered through the TxCred to the VLA
//     ARB is overwritten with FCTBS (and cause a RxCred send to OBUF)
//   If it is DATA Packet:
//     If the first credit of the packet need to decide which buffers
//     should be used for storing the packet:
//     - Lookup the target port of this pakcet by consulting the LFT
//       track it in CurOutPort
//     - Make sure enough credits exists for this VL (inspecting the
//       FREE[VL]). If there are not enough credits ASSERT.
//       FREE[VL] = MaxStatic[VL] - UsedStatic[VL]
//
//   For every DATA "credit" (not only first one)
//     - Queue the Data in the Q[V]
//          staticFree[VL]++
//          ABR[VL]++
//     - Send RxCred with updated ARB[VL] and FREE[VL] - only if sum changed
//       which becomes the FCCL of the sent flow control
//     - If HoQ in the target VLA is empty - send the push event out.
//       when the last packet is sent the "done" event is sent to all output
//       ports
//
// * On Sent:
//   - Decrease UsedStatic[VL] and update FREE[VL]
//   - If the message that was sent is the last in the packet we need to
//     send the "done" message to all connected ports.
// 
// WHEN DO WE SEND DATA OUT?
// * At any given time not more the maxBeingSent packets can be sent.
//   The VLAs track this. Each VLA that wants to send the HOQ
//   looks at the numBeingSent and if < maxBeingSent increases it.
//   If not it can not send.
// * When a new packet is received an output port should be looked up (LFT)
// * If HOQ in the VLA is empty - send a push to the VLA.
// * When VLA completes sending the credit it provides back the "sent". Then
//   a new credit is moved to the HOQ in the VLA.
// * On the last credit of sent packet we decreas the numBusyPorts. Send "done"
//   to all the connected VLAs
//
//

#ifndef __IBUF_H
#define __IBUF_H

#include <omnetpp.h>
#include <map>
#include <vector>

#define MAX_LIDS 10

// Store packet specific information to store the packet state  
class PacketState {
  int outPort; // the out port

 public:
  PacketState(int pn) {
    outPort = pn;
  };
};

/**
 * Input Buffer for Receiving IB credits and VL credit updates
 */
class IBInBuf : public cSimpleModule
{
 private:
  enum credRetModes {
	 RET_CRED_SIMPLE,  
	 RET_CRED_HYSTERESIS,
	 RET_CRED_WRR
  };
    enum DRModes {
        DR_DISABLED, 
        DR_SADP, 
        DR_FF, 
        DR_GREEDY_RANDOM,
        DR_RANDOM,
        DR_ALLRANDOM
    };

  cMessage *p_popMsg;
  cMessage *p_minTimeMsg;
 
  cLongHistogram staticUsageHist[8];
  cOutVector usedStaticCredits;
  cOutVector wrrResult;
  cOutVector wrrArbVlsWithStaticUsed;
  cOutVector wrrArbUsedWeight;
  cOutVector wrrArbUsedPoorWeight;
  cOutVector CredChosenPort;
  cOutVector dsLidDR;
  cOutVector outPortDR;
  cOutVector pktidDR;

  // Routing
  std::vector<int> *FDB;        // out port by dlid

  // parameters:
  int ISWDelay ; // delay in ns brought by SW in IBUF
  std::vector<int> maxStatic; // max number of static credits allocated for each VL
  int maxBeingSent;     // The max number of packets that can be sent out at a given time
  int totalBufferSize;  // The total buffer size in credits
  int numPorts;        // the number of ports we drive
  int hcaIBuf;         // > 0 if an HCA port IBuf
  int maxVL;              // Maximum value of VL
  int recordVectors;   // Control recording of vectors
  int width;

  // data strcture
  int numBeingSent;   // Number of packets being currently sent
  cQueue **Q;       // Incoming packets Q per VL per out port
  int hoqOutPort[8];  // The output port the packet at the HOQ is targetted to
  std::vector<int> staticFree;  // number of free credits per VL
  std::vector<long> ABR;        // accumulative number of received credits per VL

  // there is only one packet stream allowed on the input so we track its
  // parameters simply by having the "current" values. We check for mix on the
  // push handler
  int curPacketId;   
  int curPacketCredits;
  int curPacketVL;
  short curPacketOutPort;

  // as we might have multiple sends we need to track the "active sends"
  // given the packet ID we track various state variables.
  std::map<int, PacketState, std::less<int> > activeSendPackets;

  cModule* Switch; 

  // methods
  long getDoneMsgId();
  void parseIntListParam(char *parName, int numEntries, std::vector<int> &out);
  void sendOutMessage(IBWireMsg *p_msg);
  void qMessage(IBWireMsg *p_msg);
  void handleSent(IBSentMsg *p_msg);
  void sendRxCred(int vl, double delay); // send a RxCred message to the OBUF
  void sendTxCred(int vl, long FCCS); // send a TxCred message to the VLA
  void updateVLAHoQ(short int portNum, short vl); // send the HoQ if you can
  void simpleCredFree(int vl); // perform a simple credit free flow

  // return 1 if the HoQ at the given port and VL is free
  int isHoqFree(int portNum, int vl);
  void handlePush(IBWireMsg *p_msg);

  virtual void initialize();
  virtual void handleMessage(cMessage *msg);
  virtual void finish();

 public:
  // return 1 if incremented the number of parallel sends
  int incrBusyUsedPorts();
  
};

#endif
