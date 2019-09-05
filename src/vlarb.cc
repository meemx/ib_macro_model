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
// The IBVLArb implements an IB VL Arbiter
// See functional description in the header file.
//
#include "ib_m.h"
#include "vlarb.h"
#include "obuf.h"
#include "ibuf.h"
#include <iomanip>
using namespace std;

// TODO: Actually fillin the portXmitWaitHost

Define_Module( IBVLArb );

void IBVLArb::setVLArbParams(const char *cfgStr, ArbTableEntry *tbl)
{
   int idx = 0;
   char *buf, *p_vl, *p_weight;
   buf = new char[strlen(cfgStr)+1];
   strcpy(buf, cfgStr);
   p_vl = strtok(buf, ":");
   while (p_vl && (idx < 8))
   {
		int vl = atoi(p_vl);
		if (vl > 8)
		{
			ev << "-E- " << fullPath() << " VL:" << vl << " > 8 in VLA:" << cfgStr << endl;
			ev.flush();
			exit(1);
		}

		int weight;
		p_weight = strtok(NULL, ", ");
		if (! p_weight)
		{
			ev << "-E- " << fullPath() << " badly formatted VLA:" << cfgStr << endl;
			ev.flush();
			exit(1);
		}
		weight = atoi(p_weight);
		if (weight > 255)
		{
			ev << "-E- " << fullPath() << " weight:" << weight << " > 255 in VLA:" << cfgStr << endl;
			ev.flush();
			exit(1);
		}

		tbl[idx].VL = vl;
		tbl[idx].weight = weight;
		tbl[idx].used = 0;
		idx++;
		p_vl = strtok(NULL, ":");
	}
   
   // the rest are zeros
   for (;idx < 8; idx++ )
   {
		tbl[idx].VL = 0;
		tbl[idx].weight = 0;
		tbl[idx].used = 0;
   }
   delete [] buf;
}

void IBVLArb::initialize()
{
	double coreFreq_mh = par("coreFreq_MH");
	// read parameters
	hcaArb = par("isHcaArbiter");
	width = par("width");
	recordVectors = par("recordVectors");
	maxVL = par("maxVL");
    if (!hcaArb) {
		ev << "-I- " << fullPath() << " is Switch IBuf " << id() <<  endl;
		cModule*    sw = parentModule()->parentModule();
        VSWDelay = sw->par("VSWDelay");
    }
	setVLArbParams(par("highVLArbEntries"), HighTbl);
	setVLArbParams(par("lowVLArbEntries"), LowTbl);
	vlHighLimit = par("vlHighLimit");
	// for 4x we can send 8 bytes per cycle
	popDelayPerByte_us =  1 / (2.0 * width) / coreFreq_mh;
	WATCH(popDelayPerByte_us);

	// Initiazlize the statistical collection elements
	portXmitWaitHist.setName("Packet Waits for Credits");
	portXmitWaitHist.setRangeAutoUpper(0, 10, 1);
	vl0Credits.setName("free credits on VL0");
	vl1Credits.setName("free credits on VL1");
	readyData.setName("Binary coded VL's with data");
	arbDecision.setName("arbitrated VL");

	// Initialize the ready to be sent credit pointers
	numInPorts = gate("in")->size();
	int numSentPorts = gate("sent")->size();
	ASSERT(numInPorts == numSentPorts);
  
	// we need a two dimentional array of data packets
	inPktHoqPerVL = new IBDataMsg**[numInPorts];
	for (int pn = 0; pn < numInPorts; pn++)
		inPktHoqPerVL[pn] = new IBDataMsg*[maxVL+1];

	for ( int pn = 0; pn < numInPorts; pn++ )
		for ( int vl = 0; vl < maxVL+1; vl++ )
		{
			inPktHoqPerVL[pn][vl] = NULL;
			WATCH(inPktHoqPerVL[pn][vl]);
		}

	// we also need a two dim array for tracking our promise 
	// to bufs such to avoid a race betwen two requets
	hoqFreeProvided = new short*[numInPorts];
	for (int pn = 0; pn < numInPorts; pn++)
		hoqFreeProvided[pn] = new short[maxVL+1];
	for ( int pn = 0; pn < numInPorts; pn++ )
		for ( int vl = 0; vl < maxVL+1; vl++ )
			hoqFreeProvided[pn][vl] = 0;

	// Init FCCL and FCTBS and the last sent port...
	for ( int vl = 0; vl < maxVL+1; vl++ )
	{
		LastSentPort.push_back(0);
		FCTBS.push_back(0);
		FCCL.push_back(0);
	}

	WATCH_VECTOR(LastSentPort);
	WATCH_VECTOR(FCTBS);
	WATCH_VECTOR(FCCL);

	LastSentVL = 0;
	LastSentWasHigh = 0;
	InsidePacket = 0;
	LowIndex = 0;
	HighIndex = 0;
	SentHighCounter = vlHighLimit*4096/64;

	// The pop message is set every time we send a packet
	// when it is null we are ready for arbitration
	p_popMsg = NULL;
}

// return the FCTBS of the OBUF driven by the VLA
// The hardware does not use this model
int IBVLArb::getOBufFCTBS(int vl)
{
   cGate *p_gate = gate("out")->destinationGate();
   IBOutBuf *p_oBuf = dynamic_cast<IBOutBuf *>(p_gate->ownerModule());
   if ((p_oBuf == NULL) || strcmp(p_oBuf->name(), "obuf"))
   {
		ev << "-E- " << fullPath() << " fail to get OBUF from out port" << endl;
		ev.flush();
		exit(3);
   }

   return(p_oBuf->getFCTBS(vl));
}

// return 1 if the HoQ for that port/VL is free
int IBVLArb::isHoQFree(int pn, int vl)
{
	if ((pn < 0) || (pn >= numInPorts) )
	{
		ev << "-E- " << fullPath() << " got out of range port num:" << pn << endl;
		ev.flush();
		exit(1);
	}
	if ( (vl < 0) || (vl >= maxVL+1))
	{
		opp_error(" got out of range vl: %d" , vl);
	}

	// since there might be races between simultanous requests
	// and when they actually send we keep a parallel array 
	// for tracking "free" HoQ returned and clear them on
	// message push
	if ((inPktHoqPerVL[pn][vl] == NULL) && (hoqFreeProvided[pn][vl] == 0))
	{
		hoqFreeProvided[pn][vl] = 1;
		return(1);
	}
	return(0);
}

// Sending the message out:
// Update FCTBS
void IBVLArb::sendOutMessage(IBDataMsg *p_msg)
{
	double delay = p_msg->length() / 8.0 * popDelayPerByte_us;
	// we can only send if there is no such message as we use
	// it to flag the port is clear to send.
	if ( ! p_popMsg )
	{
		p_popMsg = new cMessage("pop", IB_POP_MSG);
		scheduleAt(simTime() + delay*1e-6, p_popMsg);
	}
	else
	{
		ev << "-E- " << fullPath() << " How can we have two messgaes leaving at the same time" << endl;
		return;
	}

	// remember if last send was last of packet:
	LastSentWasLast = (p_msg->getCreditSn() + 1 == p_msg->getPacketLength());

    if (!hcaArb) {
        sendDelayed(p_msg, VSWDelay*1e-9, "out");
    }else {
        send(p_msg, "out");
    }

	FCTBS[p_msg->getVL()]++;
}

// Notify the IBUF that the credit was sent out
void IBVLArb::sendSentMessage(int portNum, int vl)
{
	if (!ev.disabled()) 
		ev << "-I- " << fullPath() << " informing ibuf with 'sent' message through:" << portNum 
			<< " vl:" << vl << " last:" << LastSentWasLast << endl;
	IBSentMsg *p_sentMsg = new IBSentMsg("sent", IB_SENT_MSG);
	p_sentMsg->setVL(vl);
	p_sentMsg->setWasLast(LastSentWasLast);
	send(p_sentMsg, "sent", portNum );
}

// an arbitration is valid on two conditions:
// 1. The output port OBUF has free entries
// 2. The IBUF is not already to busy with other ports
int IBVLArb::isValidArbitration(int portNum, int vl, int isFirstPacket, int numPacketCredits)
{
	cGate *p_gate = gate("out")->destinationGate();
	IBOutBuf *p_oBuf = dynamic_cast<IBOutBuf *>(p_gate->ownerModule());
	if ((p_oBuf == NULL) || strcmp(p_oBuf->name(), "obuf"))
	{
		ev << "-E- " << fullPath() << " fail to get OBUF from out port" << endl;
		ev.flush();
		exit(3);
	}

	// check the entire packet an fit in
	int obufFree = p_oBuf->getNumFreeCredits();
	if (isFirstPacket && (obufFree < numPacketCredits))
	{
		if (!ev.disabled()) 
			ev << "-I- " << fullPath() 
				<< " not enough free OBUF credits:" << obufFree << " requierd:" 
				<< numPacketCredits <<" invalid arbitration." << endl;
		return 0;
	}

	// only for non HCA Arbiters and in case of new packet being sent
	if (!hcaArb && isFirstPacket)
	{
		cGate *p_remOutPort = gate("in", portNum)->sourceGate();
		IBInBuf *p_inBuf = dynamic_cast<IBInBuf *>(p_remOutPort->ownerModule());
		if ((p_inBuf == NULL) || strcmp(p_inBuf->name(), "ibuf") )
		{
			ev << "-E- " << fullPath() << " fail to get InBuf from in port:" << portNum << endl;
			ev.flush();
			exit(3);
		}

		if (!p_inBuf->incrBusyUsedPorts())
		{  
			if (!ev.disabled()) 
				ev << "-I- " << fullPath() 
					<< " no free ports on IBUF - invalid arbitration." << endl;
			return 0;
		}
	}
	return 1;
}

// NOTE: If vlHighLimit was reached a single packet
// of the lower table is transmitted.

// Find the port and VL to be sent next from a High or Low entries.
// Given:
// * current index in the table
// * The table of entries (VL,Weight) pairs
// Return:
// * 1 if found a port to send data from or 0 if nothing to send
// * Update the provided entry index
// * Update the port number
// * update the VL
int
IBVLArb::findNextSend( int &curIdx, ArbTableEntry *Tbl, int &curPortNum, int &curVl )
{
	int idx;
	short int vl;
	int found = 0;
	int numCredits = 0;
	int portNum;
	IBDataMsg *p_credit;

	// we need to scan through all entries starting with last one used
	for (int i = 0; i <= maxVL+1 ; i++)
	{
		idx = (curIdx + i) % (maxVL+1);
		// if we changed index we need to restat the weight counter
		if (i) Tbl[idx].used = 0;

		// we should skip the entry if it has zero credits (weights) not used
		if (!Tbl[idx].weight || (Tbl[idx].used > Tbl[idx].weight)) continue;

		vl = Tbl[idx].VL;

		// how many credits are available for this VL
		numCredits = FCCL[vl] - FCTBS[vl];

		// start with the next port to the last one we sent
		for (int pn = 1; pn <= numInPorts; pn++)
		{
			portNum = (curPortNum + pn) % numInPorts;
			p_credit = inPktHoqPerVL[portNum][vl];
			// do we have anything to send?
			if (p_credit == NULL)  continue;
        
			// just make sure it is a first credit

			// we can have another messages leaving at the same time so
			// ignore that port/vl if in the middle of another transfer
			if (p_credit->getCreditSn())
			{
				if (! ev.disabled())
					ev << "-I- " << fullPath() << " ignoring non first packet:"
						<< p_credit->name() << " on port:" << portNum
						<< " vl:" << vl << endl;
			}
			else
			{
				// so can we send it?
				if (p_credit->getPacketLength() <= numCredits)
				{
					found = 1;
					break;
				}
				else
				{
					if (! ev.disabled())
						ev << "-I- " << fullPath() << " not enough credits available:"
							<< numCredits << " < " << p_credit->getPacketLength() 
							<< " required for sending:"
							<< p_credit->name() << " on port:" << portNum
							<< " vl:" << vl << endl;
				}
			}
		}

		if (found)
		{
			curIdx = idx;
			curPortNum = portNum;
			curVl = vl;
			break;
		}
	}

	return(found);
}

// Find the port on VL0 to send from 
// Given:
// Return:
// * 1 if found a port to send data from or 0 if nothing to send
// * Update the port number
int
IBVLArb::findNextSendOnVL0( int &curPortNum )
{
	int found = 0;
	int numCredits = 0;
	int portNum;
	IBDataMsg *p_credit;
  
	// how many credits are available for this VL
	numCredits = FCCL[0] - FCTBS[0];
  
	// start with the next port to the last one we sent
	for (int pn = 1; pn <= numInPorts; pn++)
	{
		portNum = (curPortNum + pn) % numInPorts;
		p_credit = inPktHoqPerVL[portNum][0];
		// do we have anything to send?
		if (p_credit == NULL)  continue;
        
		// just make sure it is a first credit

		// we can have another messages leaving at the same time so
		// ignore that port/vl if in the middle of another transfer
		if (p_credit->getCreditSn())
		{
			if (! ev.disabled())
				ev << "-I- " << fullPath() << " ignoring non first packet:"
					<< p_credit->name() << " on port:" << portNum
					<< " vl:" << 0 << endl;
		}
		else
		{
			// so can we send it?
			if (p_credit->getPacketLength() <= numCredits)
			{
				found = 1;
				break;
			}
			else
			{
				if (! ev.disabled())
					ev << "-I- " << fullPath() << " not enough credits available:"
						<< numCredits << " < " << p_credit->getPacketLength() 
						<< " required for sending:"
						<< p_credit->name() << " on port:" << portNum
						<< " vl:" << 0 << endl;
			}
		}
	}
  
	if (found)
		curPortNum = portNum;
	return(found);
}

void IBVLArb::displayState()
{
	if (!ev.disabled()) {
		ev << "-I- " << fullPath() << " ARBITER STATE as VL/Used/Weight" 
			<< endl;
		ev << "-I- High:";
		for (int e = 0; e < maxVL+1; e++)
		{
			if (LastSentWasHigh && HighIndex == e)
				ev << "*" << HighTbl[e].VL << " " 
					<< setw(3) << HighTbl[e].used
					<< "/" << setw(3) << HighTbl[e].weight << "*";
			else
				ev << "|" << HighTbl[e].VL << " " 
					<< setw(3) << HighTbl[e].used
					<< "/" << setw(3) << HighTbl[e].weight << " ";
		}
		if (LastSentWasHigh)
			ev << "<----" << SentHighCounter << endl;
		else
			ev << endl;
		
		ev << "-I- Low: ";
		for (int e = 0; e < maxVL+1; e++)
		{
			if (!LastSentWasHigh && LowIndex == e)
				ev << "*" << LowTbl[e].VL << " " 
					<< setw(3) << LowTbl[e].used
					<< "/" << setw(3) << LowTbl[e].weight << "*";
			else
				ev << "|" << LowTbl[e].VL << " " 
					<< setw(3) << LowTbl[e].used
					<< "/" << setw(3) << LowTbl[e].weight << " ";
		}
		ev << endl;
	}

   int vlsWithData = 0;
   for (int vl = 0; vl < maxVL+1; vl++)
   {
		int fctbs = FCTBS[vl];
		int freeCredits = FCCL[vl] - fctbs;
		if (!ev.disabled()) 
			ev << "-I- " << fullPath() << " vl:" << vl
				<< " " << FCCL[vl] << "-" << fctbs << "=" 
				<< freeCredits << " Ports " ;
		int anyInput = 0;
		for (int pn = 0; pn < numInPorts ; pn++)
		{
			if (inPktHoqPerVL[pn][vl]) {
				anyInput = 1;
				if (!ev.disabled()) ev << pn << ":Y ";
			} else {
				if (!ev.disabled()) ev << pn << ":n ";
			}
		}
	  
		if (!ev.disabled()) ev << endl; 
		if (anyInput)
			vlsWithData |= 1<<vl;

		if (recordVectors && (vl == 0))
			vl0Credits.record(freeCredits);
		else if (recordVectors && (vl == 1))
			vl1Credits.record(freeCredits);
   }
	if ( recordVectors ) {
		readyData.record(10*vlsWithData);
	}
}

// Arbitration:
//
// Decides which data to send and provide back a "sent" notification.
//
// Data Structure:
// HighIndex, LowIndex - points to the index in the VLArb tables.
// LastSentPort[VL] - points to the last port that have sent data on a VL
// SentHighCounter - counts how many credits still needs to be sent from the high
// LastSentWasHigh - let us know if we were previously sending from
//    low or high table
// inPktHoqPerVL[pn][vl] - an array per port and VL pointing to first
//    credit of the packet
//
// NOTE: As we decide to arbitrate on complete packets only we might need to
//       increase the credit count if what is left is smaller then the number
//       of credits the packet carries.
//
// Algorithm:
// NOTE: as we only need to decide on a packet boundary we keep track of the
// last port and vl used and simply use all credits from it.
// * If the SentHighCounter is 0 then we use LowTable otherwise HighTable
// * Find the first port, after the last one that was sent, that has data
//   ready to send on the current index vl.
// * If not found incr index and repeat until all entries
//   searched (ignore 0 weight entries).
// * If found a new index use it and update the credits sent vs the weight.
// * If not found and we are HighTable - use LowTable and do as above.
// * Dec the SentHighCounter if we are HighTable. Load it to vlHighLimit*4k/64
//   otherwise.
//
void IBVLArb::arbitrate()
{
	int found = 0;
	IBDataMsg *nextSendHoq;
	int isFirstPacket = 0;
	int isLastCredit;
	int portNum;
	int vl;

	if (!ev.disabled()) 
		recordVectors = par("recordVectors");

	// can not arbitrate if we are in a middle of send
	if (p_popMsg)
	{
		if (!ev.disabled()) 
			ev << "-I- " << fullPath() 
				<< " can not arbitrate while packet is being sent" << endl;
		return;
	}

	// display arbiter state:
	if (!ev.disabled() || recordVectors)
		displayState();

	// if we did not reach the end of the current sent packet simply send this credit
	if (InsidePacket)
	{
		vl = LastSentVL;
		portNum = LastSentPort[vl];

		nextSendHoq = inPktHoqPerVL[portNum][vl];
		if (! nextSendHoq)
		{ 
			if (!ev.disabled()) 
				ev << "-I- " << fullPath() << " HoQ empty for port:"
					<< portNum << " VL:" << vl << endl;
			return;
		}

		isLastCredit = (nextSendHoq->getCreditSn() + 1 == nextSendHoq->getPacketLength());

		if (isLastCredit)
		{
			if (!ev.disabled())
				ev << "-I- " << fullPath() << " sending last credit packet:"
					<< nextSendHoq->name() << " from port:" << portNum
					<< " vl:" <<  vl << endl; 
			//	  LastSentWasLast = 1;
			InsidePacket = 0;
		}
		else
		{
			if (!ev.disabled())
				ev << "-I- " << fullPath() << " sending continuation credit packet:"
					<< nextSendHoq->name() << " from port:" << portNum
					<< " vl:" << vl << endl;
		}

		// need to decrement the SentHighCounter if we are sending high packets
		if (LastSentWasHigh)
		{
			SentHighCounter--;
		}
		else
		{
			// If we are sending the last credit of packet when the SentHighCounter
			// is zero we need to reload it as this was the last credit of forced low
			// packet
			if ( isLastCredit && (SentHighCounter == 0))
				SentHighCounter = vlHighLimit*4096/64;
		}
		found = 1;
	}
	else
	{
		isFirstPacket = 1;

		// IF WE ARE HERE WE NEED TO FIND FIRST PACKET TO SEND
		portNum = LastSentPort[LastSentVL];
		vl = LastSentVL;

		if (maxVL > 0) {
			// if we are in High Limit case try first from low
			if ( SentHighCounter <= 0 ) {
				found = 1;
				if (findNextSend(LowIndex, LowTbl, portNum, vl))
					LastSentWasHigh = 0;
				else if ( findNextSend(HighIndex, HighTbl, portNum, vl) )
					LastSentWasHigh = 1; 
				else
					found = 0;
			} else { 
				found = 1;
				if ( findNextSend(HighIndex, HighTbl, portNum, vl) )
					LastSentWasHigh = 1; 
				else if (findNextSend(LowIndex, LowTbl, portNum, vl))
					LastSentWasHigh = 0;
				else
					found = 0;
			}
		} else {
			vl = 0;
			found = findNextSendOnVL0(portNum);
		}

		if (found)
		{
			if (!ev.disabled())
				if (LastSentWasHigh)
					ev << "-I- " << fullPath() << " Result High idx:" << HighIndex << " vl:" << vl
						<< " port:" << portNum << " used:" << HighTbl[HighIndex].used
						<< " weight:" << HighTbl[HighIndex].weight
						<< " high count:" << SentHighCounter << endl;
				else
					ev << "-I- " << fullPath() << " Result Low idx:" << LowIndex << " vl:" << vl
						<< " port:" << portNum << " used:" << LowTbl[LowIndex].used
						<< " weight:" << LowTbl[LowIndex].weight << endl;

			nextSendHoq = inPktHoqPerVL[portNum][vl];
			isLastCredit = (nextSendHoq->getCreditSn() + 1 == nextSendHoq->getPacketLength());

			if (!isLastCredit) {
				InsidePacket = 1; 
				if (!ev.disabled())
					ev << "-I- " << fullPath() << " sending first credit packet:"
						<< nextSendHoq->name() << " from port:" << portNum
						<< " vl:" << vl << endl; 
			} 
			else 
			{ 
				InsidePacket = 0;
				if (!ev.disabled())
					ev << "-I- " << fullPath() << " sending single credit packet:"
						<< nextSendHoq->name() << " from port:" << portNum
						<< " vl:" << vl << endl; 
			}

		}
		else
		{
			if (!ev.disabled())
				ev << "-I- " << fullPath() << " nothing to send" <<endl;

			if ( recordVectors )
				arbDecision.record(-1);
			return;
		}
	} // first or not

	// we could arbitrate an invalid selection due to lack of output Q
	// or busy ports of the input port we want to arbitrate.
	if (isValidArbitration(portNum, vl, isFirstPacket, 
								  nextSendHoq->getPacketLength()))
	{
		// do the actual send and record our successful arbitration
		if (nextSendHoq->getCreditSn() == 0)
		{
			LastSentVL = vl;
			LastSentPort[vl] = portNum;
		}

		inPktHoqPerVL[LastSentPort[LastSentVL]][LastSentVL] = NULL;

		if (LastSentWasHigh)
			HighTbl[HighIndex].used ++;
		else
			LowTbl[LowIndex].used ++;

		if ( recordVectors )
			arbDecision.record(10*(vl+1));


		sendOutMessage(nextSendHoq);
		sendSentMessage(LastSentPort[LastSentVL],LastSentVL);
	}
	else
	{
		// if we are in the first data credit cleanup the InsidePacket flag
		if (nextSendHoq->getCreditSn() == 0)
			InsidePacket = 0;
		if ( recordVectors )
			arbDecision.record(-1);
	}
}

// Handle push message
// A new credit is being provided by some port
// We need to know which port we get the data on,
// which VL is it on
// if the HOQ is aleady taken assert
void IBVLArb::handlePush(IBDataMsg *p_msg)
{
	// what port did we get it from ?
	int pn = p_msg->arrivalGate()->index();
	short int vl = p_msg->getVL();
	if ((pn < 0) || (pn >= numInPorts) )
	{
		ev << "-E- " << fullPath() << " got out of range port num:" << pn << endl;
		ev.flush();
		exit(1);
	}

	if ( (vl < 0) || (vl >= maxVL+1))
	{
		opp_error("VLA got out of range vl: %d by %s, arrived at port %d", vl, p_msg->name(), pn); 
	}


	if (inPktHoqPerVL[pn][vl] != NULL)
	{
		ev << "-E- " << fullPath() << " Overwriting HoQ port:" << pn << " VL:" << vl
			<< " by msg:" << p_msg->name() << " arrived at port:" << pn
			<< endl;
		ev.flush();
		exit(2);
	}

	if (hoqFreeProvided[pn][vl] == 0)
	{
		ev << "-E- " << fullPath() << " No previous HoQ free port:" << pn << " VL:" << vl
			<< " by msg:" << p_msg->name() << " arrived at port:" << pn
			<< endl;
		ev.flush();
		exit(2);
	}

	if (!ev.disabled())
		ev << "-I- " << fullPath() << " filled HoQ for port:" 
			<< pn << " vl:" << vl << " with:" << p_msg->name() <<  endl;

	inPktHoqPerVL[pn][vl] = p_msg;
	hoqFreeProvided[pn][vl] = 0;
	arbitrate();
}

// Handle Pop Message
// clear the pop message and send the "sent message" then try to arbitrate
//
// NOTE: Only now when the packet was sent we can tell the IBUF
// to free its credits and busy ports.
void IBVLArb::handlePop()
{
	cancelAndDelete(p_popMsg);
	p_popMsg = NULL;
	arbitrate();
}

// Handle TxCred
void IBVLArb::handleTxCred(IBTxCredMsg *p_msg)
{
	int vl = p_msg->getVL();
	// update FCCL...
	FCCL[vl] = p_msg->getFCCL();

	if (!ev.disabled())
		ev << "-I- " << fullPath() << " updated vl:" << vl
			<< " fccl:" << p_msg->getFCCL()
			<< " can send :" << FCCL[vl] - FCTBS[vl] << endl;

	delete p_msg;
	arbitrate();
}

void IBVLArb::handleMessage(cMessage *p_msg)
{
	int msgType = p_msg->kind();
	if ( msgType == IB_POP_MSG )
	{
		handlePop();
	}
	else if ( msgType == IB_DATA_MSG )
	{
		handlePush((IBDataMsg*)p_msg);
	}
	else if ( msgType == IB_TXCRED_MSG )
	{
		handleTxCred((IBTxCredMsg*)p_msg);
	}
	else if ( (msgType == IB_DONE_MSG) || (msgType == IB_FREE_MSG) )
	{
		delete p_msg;
		arbitrate();
	}
	else
	{
		ev << "-E- " << fullPath() << " does not know how to handle message:" << msgType << endl;
		delete p_msg;
	}
}

void IBVLArb::finish()
{
	/*ev << "STAT: " << fullPath() << " Wait for credits num/avg/max/std "
     << portXmitWaitHist.samples() 
     << " / " << portXmitWaitHist.mean()
     << " / " << portXmitWaitHist.max() 
     << " / " << portXmitWaitHist.stddev() << endl;
	*/
}
