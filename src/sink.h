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

#ifndef __SINK_H
#define __SINK_H

#include <omnetpp.h>

/**
 * Consumes IB Credits; see NED file for more info.
 */
class IBSink : public cSimpleModule
{
 private:
  cMessage *p_hiccupMsg;
  cMessage *p_drainMsg;

  int     duringHiccup;
  cStdDev waitStats;
  cStdDev hiccupStats;
  cDoubleHistogram PakcetFabricTime; 
  int pciExpWidth;
  int maxVL;              // Maximum value of VL
  double byteDrainRate_us; // the time it takes PCIex to drain a single byte
  int creditSize;
  cQueue queue;
  std::vector<int> VLPackets; // track number of data (credit size) per VL
  int recordVectors;   // Control recording of vectors
  double lastConsumedPakcet; // the last time a packet was consumed

  simtime_t FirstPktRcvTime;
  bool firstPacketRcv;  // 0 when First pcket has not yet been sentreceived
  int AccBytesRcv;

  // methods
  void newDrainMessage(double delay);  
  void consumeDataMsg(IBDataMsg *p_msg);
  void handlePop(cMessage *p_msg);
  void handleData(IBDataMsg *p_msg);
  void handleHiccup(cMessage *p_msg);

 protected:
  virtual void initialize();
  virtual void handleMessage(cMessage *msg);
  virtual void finish();
};

#endif













