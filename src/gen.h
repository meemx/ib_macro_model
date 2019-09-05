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
// An IB Packet Generator - Injecting Credits into the "out" port
//

// Traffic Distribution Modes 
// TRF_FLOOD: In flood mode the generator keeps all VLs with at
//            least 2 outstanding packets. If zero (regular mode)
//            the Generator simulates real packet insertion flow.
// TRF_UNI: In Uniform mode, packets are send in burts with delay between packets and between bursts
//          We can chose  numbers of bursts, number of packets per burst, etc,,,  In this case the simulator 
//          simulates real packet insertion flow
// TRF_MIXLEN: Same as TRF_UNI but in this case the packet lenth in bytes is given by a distribution 
//             as follows: PktLenDist is a vector with the length of packets we are covering
//                         PktLenProb is the probability related to each of the Length in PktLenDist
// General Traffic characteristics
//   dstLid
//   mtu
//   srcLid
//   VL
//   packetlengthbytes 
// TRF_FLOOD
//   floodVLs
//   msgLength
// TRF_UNI
//   burstLength
//   burstNumber
//   IntraBurstDelay
//   InterBurstDelay
// TRF_MIXLEN
//   PktLenDist   (comes instead packetlengthbytes)
//   PktLenProp
// TRF_APP
//   dstSeqVecFile
//   sizeSeqVecFile
// If we work in mode GenModel = 0 = ib_credits:
// The parameter packetlengthbytes is ensured to be multiple of 64B.
// The parameter msglen is ensured to be multiple of MTU
// The packets length will always be according to packetlengthbytes
// The packet dstLid will be either dstLid in case we are in sequence mode dstLid
// or one of dstSeq. We advance in indexes of dstSeq if the packet is part of a new message
// If we work in GenModel  = 1 = ib_switch:
//            PacketLength is determined:
//              1. in MIXLEN according to PktlenDist and Prob
//              2. in UNIFORM according to packetlengthbytes
//              3. In flood mode according to msglength: MTu until last packet which can be < MTU
//            dstLid is determined:
//              1. in DSTLID: as the parameter in the file
//              2. in DSTSEQ chose new dst for new packet
//              3. In HOT SPOT as mix of the 2.



#ifndef __GEN_H
#define __GEN_H

#include <omnetpp.h>


/*
 * Generates IB Packet Credit (messages); see NED file for more info.
 */
class IBGenerator : public cSimpleModule
{
 private:
	// Sequencing modes:
	enum dstSeqModes { 
		DST_BY_PARAM,  // invoke the dstLid param every MTU
		DST_SEQ_ONCE,  // use the dstSeq vector od dstLids only once.
		DST_SEQ_LOOP,  // continously loop through the dstSeq vector od dstLids.
      DST_RANDOM,    // Destination is randomly set
      DST_HOT_SPOT,  // Work DST_SEQ* but on a % of traffic work BY_PARAM
      DST_BURST_HS   // Works HOTSPOT for given time intervals, and SEQ_LOOP in between
	};
	enum trafficDists {
      TRF_FLOOD, 
      TRF_UNI,  
      TRF_MIXLEN,
      TRF_APP
	};

	// Initialize a new set of parameters for a new packet
	void initIBPacketParams(int vl = -1);
	// get a globally uniq packet ID
	int getPacketId();
	// set the current credit params
	IBDataMsg *getNewDataMsg();
	void handlePush(cMessage *msg);
	void sendDataOut(IBDataMsg *p_msg); 
	void handleSent(IBSentMsg *p_sent);
	void finish();
	int  isRemoteHoQFree(int vl);
	int getPacketLen(); 
	// makes sure the VL Q has at least 10 credits.
	void floodVLQ(int vl, int duringInit = 0); 
	simtime_t FirstPktSendTime;
	bool firstPacketSent;  // 0 when First pcket has not yet been sent
	int AccBytesSent;

	// The following parameters control burstiness of the packet generator
	int burstLength;        // Number of packets in burst
	int burstNumber;        // Number of bursts
	int burstCounter;       // what packet we are in the burst

	// Dequence Mode controls
	dstSeqModes dstSeqMode;   // a flag to set destination sequence mode
	std::vector<int> *dstSeq; // a destination lid sequence
	std::vector<int> *sizeSeq; // a message size sequence  YS
	bool dstBHS;              // true if we are in bursty Hot Spot mode. In this case dstSeqMode
                             // will either be DST_HOTSPOT or DST_SEQ_LOOP
	trafficDists trafficDist; // a flag to set traffic distribution mode
	int msgLenInMTUs;         // the number of MTUs each message has
	int mtuInMsgIdx ;         // Used only for genModel = ib_credits = 0
  
	// Global settings of this generator:
	int width;            // Interface width 1x 4x 8x 12x
	double speed;         // Interface speed 2.5Gbps, 5Gbps 10Gbps
	double genDelay_us;   // Rate of credits injection = 1000/width*speed
	int SL;               // Injects packets on this SL
	int creditSize;       // The size of injected credit in bytes
	int srcLid;           // The source LID
	int parDstLid;
	int PktLenTotProb;     // Sum of PktLenProb vector in TRF_MIXLEN mode
	bool  GenModel;       // 0: ib_credits ; 1: ib_switch
	std::vector<int> PktLenDist; 
	std::vector<int> PktLenProb; 
	double dstBurstHSDel;
	double dstBurstHSInterDel;
	double dstHotSpotPerc;// % of traffic that should go to hot spot
	simtime_t dstBHS_start;
	simtime_t TimeLastSent; // Time last credit was sent

	// Current packet parameters
	int packetId;         // Each IB Packet in the system has it own unique ID.
	int packetVL;         // The current sent packet VL
	int packetLength;     // Current packet length in credits
	int packetLengthBytes;// Current packet length in bytes
	int RemainMsgLength ; // Remaining message length in B
	int RemainPktLength ; // Remaining packet length in B
	int MsgLength ; 		// Total message length in B
	int CredLength ; 	// Length of current credit being sent in BB
	int creditCounter;    // Current credit counter (starts at zero
	int dstLid;           // The target LID of this packet
	int dstSeqIdx;        // When using a sequence of dLids the next index to use
	int dstSeqDone;       // When using a sequence once 1 if entire seq was gen
	int mtu ;		        // Number of credits in MTU
	bool firstPacket ; 	// Indicates dealing with first packet 

	// we need to hold some packets to facilitate VLA pushback...
	cQueue Q[8];

	// methods
 protected:
	void parseIntListParam(char *parName, std::vector<int> &out);
	virtual void initialize();
	virtual void handleMessage(cMessage *msg);
};

#endif
