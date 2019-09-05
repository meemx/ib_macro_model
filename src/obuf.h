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
// IB Device Port Output Buffer
// ============================
// Ports
// * in
// * out 
// * rxCred - where updates from VLArb with incoming FlowControl come
//
// Parameters
// * CreditMinRate - the time between credit updates
// * OutRate - the rate by which credits leave the buffer
//
// Internal Events
// * MinTime - cause a new credit update packet to be inserted into
//   the Q if required 
// * Pop - packet with one credit is sent through OUT if we have what
//   to send
//
// External Events
// * Push - new credit is provided in
// * RxCred - information regarding the IBUF credits
//
// Function
// * FCTBS is a local parameter incremented on every data packet sent
// * On a push the "credit" provided is put into Q. If there was no 
//   pending Pop the data is send immediatly out.
// * When RxCred (carry ABR and Free credits per VL) is received it is
//   stored locally.
// * Each MinTime a "credit update" packet is placed in the Q if
//   needed by comparing to previous FCCL and FCTBS update
// * On "Pop" send a "Push" with one credit through OUT
// 

#ifndef __OBUF_H
#define __OBUF_H

#include <omnetpp.h>
#include <vector>

/**
 * Output Buffer for sending IB credits and VL credit updates
 */
class IBOutBuf : public cSimpleModule
{
 private:
  cMessage *p_popMsg;
  cMessage *p_minTimeMsg;
  
  cLongHistogram qDepthHist; 
  cDoubleHistogram packetStoreHist; 
  cDoubleHistogram flowControlDelay; // track the time between flow controls
  cOutVector qDepth; // track the Q usage over time
  simtime_t packetHeadTimeStamp; // track time stamp of the current pop packet
  double popDelayPerByte_us; // Rate of packets injection = 1000/width*speed

  // parameters:
  double credMinTime_us; // time between checing VL update and injecting an update
  int width;          // Interface width 1x 4x 8x 12x
  double speed;       // Interface speed 2.5Gbps, 5Gbps 10Gbps
  int qSize;          // Max number of credits the Q can handle
  int curFlowCtrVL;   // If < 8 will cause a the flow control packet of this VL
  int isMinTimeUpdate; // set on minTime event to flag the updates are cause by minTime
  int recordVectors;   // Control recording of vectors
  bool Enabled;        // Is this port enabled or is it part of a 8x/12x
  int maxVL;           // Maximum value of VL

  // data strcture
  cQueue queue;
  int  numDataCreditsQueued; // needed to make sure we do not overflow the qSize
  int  prevPopWasDataCredit; // track the last pop was a data credit to know when "free" msg 
  int  insidePacket;         // track the fact we are in the middle of sending a packet
  simtime_t prevFCTime;      // track the last time the VL0 flow control was possibly sent
  std::vector<long> prevSentFCCL;
  std::vector<long> prevSentFCTBS;
  std::vector<long> FCTBS; // number of data packet credits sent total in this VL
  std::vector<long> FCCL; // Pending value to be sent on next VL Credits update pkt

  // Methods
  void sendOutMessage(IBWireMsg *p_msg);
  void qMessage(IBDataMsg *p_msg);
  int  sendFlowControl();
  void handlePop();
  void handleMinTime();
  void handleRxCred(IBRxCredMsg *p_msg);

  virtual void initialize();
  virtual void handleMessage(cMessage *msg);
  virtual void finish();protected:

public:
   // used by the VLA to validate the last arbitration
   int  getNumFreeCredits() {
	  return(qSize - queue.length());
   };

   // used by VLA to kno whow many data packets were already sent
   int getFCTBS(int vl) {
	  return(FCTBS[vl]);
   };

};

#endif
