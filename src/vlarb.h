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
// * in[N]
// * out 
// * txCred - where flow control data is through
// * sent[N] - backward notification of sent data
//
// Parameters
// * VLHighLimit - IB style VLArb table limit of number of credits sent
//                 from high level before low level checked.
// * HighVLs[C], HighWeights[C], LowVLs[C], LowWeights[C] - the IB VLA config
// * popDelayPerByte_us - control the rate of Pop events
// 
// Internal Events
// * Pop - cause a credit to leave the 
// 
// External Events
// * Push - data is available on the INj (stored locally)
// * txCred - FCCL = credits availability on remote port
//
// Functionality
// Every time a packet can be sent out as dictated by the Pop rate performs
// IB style VLArb by inspecting available credits and packets.
// Use round robin to select input port if data is availble on several.
// Track sent packets on each VL (FCTBS). The txCred event updates the FCCL[vl]
// Free credits per VL are calculated as FCCL - FCTBS. 
// When a credit is sent out of the VLArb a "Sent" event is provided back to
// the IBUF (such that it can free buffers).
//
// Data Structure
// To avoid duplicating and multiply by the number of ports the IBUF only 
// keeps one credit message per input port, per VL. 
// The IBUF should push its available credit immediaty to the VLArb
// The VLArb notify the IBUF that packet has left and the buffer is
// not empty using the Sent message  
// 
//
#ifndef __VLARB_H
#define __VLARB_H

#include <omnetpp.h>
#include <vector>

class ArbTableEntry {
public:
  short int VL;
  short int weight; // max credits to send on this entry - rounded up for a packet
  int used; // used credits from the above
};

/**
 * Output Buffer for sending IB credits and VL credit updates
 */
class IBVLArb: public cSimpleModule
{
 private:
  cMessage *p_popMsg;
  
  // statistics
  cOutVector vl0Credits;  // the credits on VL0
  cOutVector vl1Credits;  // the credits on VL1
  cOutVector readyData;   // the VLs with ready data in binaru code 
  cOutVector arbDecision; // the resulting VL arbitrated -1 is invalid
  cLongHistogram portXmitWaitHist; 
  double popDelayPerByte_us; // Rate of packets injection = 1000/width*speed

  // parameters:
  int vlHighLimit;          // Max number of credits sent from High till Low
  ArbTableEntry HighTbl[8]; // The High Priority Arbitration Table
  ArbTableEntry LowTbl[8];  // The Low Priority Arbitration Table
  int           hcaArb;     // If 1 means the arbiter is an HCA arbiter
  int width;                // Interface width 1x 4x 8x 12x
  int recordVectors;        // Control recording of vectors
  int maxVL;              // Maximum value of VL
  int VSWDelay;        // Delay brought by VLArb in Switch   (ns)

  // data strcture:
  IBDataMsg ***inPktHoqPerVL;     // the head of the send Q on every VL
  short **hoqFreeProvided;        // set when a "free" HoQ provided and cleared on push
  int HighIndex, LowIndex;        // points to the index in the VLArb tables.
  std::vector<int> LastSentPort; // last port that have sent data on each VL
  int LastSentVL;          // the VL of the last sent packet
  int SentHighCounter;     // counts how many credits were sent from the high
  int LastSentWasHigh;     // 1 if we were previously sending from high
  int LastSentWasLast;     // 1 if the sent data was last in the packet
  unsigned int numInPorts; // The number of input ports
  int InsidePacket;        // if 1 we are sending a packet so we already arbitrated it


  // methods
  void setVLArbParams(const char *cfgStr, ArbTableEntry *tbl);
  void sendOutMessage(IBDataMsg *p_msg);
  void sendSentMessage(int portNum, int vl);
  int  isValidArbitration(int portNum, int vl, int isFirstPacket, int numPacketCredits);
  int  findNextSend( int &curIdx, ArbTableEntry *Tbl, int &curPortNum, int &curVl );
  int  findNextSendOnVL0( int &curPortNum );
  void displayState();
  void arbitrate();
  void handlePush(IBDataMsg *p_msg);
  void handlePop();
  void handleTxCred(IBTxCredMsg *p_msg);
  int  getOBufFCTBS(int vl);

 public:
	int isHoQFree(int pn, int vl); // return 1 if the HoQ for that port/VL is free
    std::vector<long> FCTBS; // number of data packet credits sent total in this VL
    std::vector<long> FCCL; // The last number of credits the receive port provided

 protected:
  virtual void initialize();
  virtual void handleMessage(cMessage *msg);
  virtual void finish();
};

#endif

