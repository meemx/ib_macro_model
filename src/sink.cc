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
// The IBSink implements an IB endport FIFO that is filled by the push 
// message and drained by the pop self-message.
// To simulate hiccups in PCI Exp bus, we schedule self-messages of type hiccup
// hiccup message alternate between an ON and OFF state. During ON state any
// drain message is ignored. On transition to OFF a new drain message can
// be generated
// 
#include "ib_m.h"
#include "sink.h"

Define_Module( IBSink );

void IBSink::initialize()
{
	waitStats.setName("Waiting time statistics");
	hiccupStats.setName("Hiccup Statistics");
	maxVL = par("maxVL");

	PakcetFabricTime.setName("Packet Fabric Time");
	PakcetFabricTime.setRangeAutoUpper(0, 10, 1.5);

	// calculate the drain rate
	creditSize = par("creditSize");
	pciExpWidth = par("pciExpWidth"); // 1, 4, 8
	// drain rate is the num usec to drain one bit
	// =10 bits/byte * usec/sec / PCIExp Freq / NumLanes * 105% [usec/bit]
	byteDrainRate_us = 10 *1.0e6 / 2.5e9 / pciExpWidth * 1.05; 
	WATCH(byteDrainRate_us);

	// we will allocate a drain messgae only on the first credit getting in
	// which consumed immediatly...
	p_drainMsg = NULL;
	FirstPktRcvTime = 0;
	firstPacketRcv = 1;
	AccBytesRcv = 0;



	duringHiccup = 0;
	WATCH(duringHiccup);

	p_hiccupMsg = new cMessage("pop");
	p_hiccupMsg->setKind(IB_HICCUP_MSG);
	scheduleAt(simTime()+1e-9, p_hiccupMsg);

   // we track number of packets per VL:
   for (int vl = 0; vl < maxVL+1; vl++) 
		VLPackets.push_back(0);

	WATCH_VECTOR(VLPackets);
}

// Init a new drain message and schedule it after delay
void IBSink::newDrainMessage(double delay_us) {
	p_drainMsg = new cMessage("pop", IB_POP_MSG);

	// we track the start time so we can hiccup left over...
	p_drainMsg->setTimestamp(simTime()); 
	scheduleAt(simTime()+delay_us*1e-6, p_drainMsg);
}

// track consumed messages and send "sent" event to the IBUF
void IBSink::consumeDataMsg(IBDataMsg *p_msg)
{

	if (! ev.disabled()) 
		ev << "-I- " << fullPath() << " consumed data:" << p_msg->name() << endl;

	// track the absolute time this packet was consumed
	lastConsumedPakcet = simTime();

	// track the time this credit waited in the HCA
   double d = lastConsumedPakcet - p_msg->timestamp();
   //waitStats.collect( d );

   // track the time this credit spent on the wire...
   if (p_msg->getCreditSn() == (p_msg->getPacketLength() -1)) {
		d = simTime() - p_msg->timestamp();
		PakcetFabricTime.collect( d );
   }

	int vl = p_msg->getVL();
	VLPackets[vl]++;

	if (0) {
		if (p_msg->getPacketLength() == p_msg->getCreditSn() + 1)
		{
			for (int sn = p_msg->getPacketLength(); sn != 0; sn--)
			{
				// send the IBUF a "sent" messgae
				IBSentMsg *p_sentMsg = new IBSentMsg("hca_sent", IB_SENT_MSG);
				p_sentMsg->setVL(vl);
				p_sentMsg->setWasLast(sn == 1);
				send(p_sentMsg, "sent");
			}
		}
	} else {
		IBSentMsg *p_sentMsg = new IBSentMsg("hca_sent", IB_SENT_MSG);
		p_sentMsg->setVL(vl);
		p_sentMsg->setWasLast(p_msg->getPacketLength() == p_msg->getCreditSn() + 1);
		send(p_sentMsg, "sent");
	}

	delete p_msg;
}

void IBSink::handleData(IBDataMsg *p_msg)
{
	double delay;

	// for iBW calculations
	if (firstPacketRcv) {
		firstPacketRcv = 0;
		FirstPktRcvTime = simTime();
	}
	AccBytesRcv += p_msg->length()/8;

   // we might be arriving on empty buffer:
   if ( ! p_drainMsg )
   {
		if (! ev.disabled()) 
			ev << "-I- " << fullPath() << " data:" << p_msg->name() 
				<< " arrived on empty FIFO" << endl;
		// this credit should take this time consume:
		delay = p_msg->length() / 8.0 * byteDrainRate_us;
		newDrainMessage(delay);
   }

	if (! ev.disabled()) 
		ev << "-I- " << fullPath() << " queued data:" << p_msg->name() << endl;
	queue.insert(p_msg);
}

// simply consume one message from the Q or stop the drain if Q is empty
// also under hiccup do nothing
void IBSink::handlePop(cMessage *p_msg)
{
	// if we are under hiccup - do nothing or
   // got to pop from the queue if anything there
	if ( !queue.empty() && ! duringHiccup )
	{
		IBDataMsg *p_dataMsg = (IBDataMsg *)queue.pop();
		if (! ev.disabled()) 
			ev << "-I- " << fullPath() << " De-queued data:" << p_dataMsg->name() << endl;

		// when is our next pop event?
		double delay_us = p_dataMsg->length() / 8.0 * byteDrainRate_us;
		/*  	if (! ev.disabled()) 
			 ev << "-I- " << fullPath() << " next drain in: " << delay_us << " usec" << endl; */

		// consume actually discards the message !!!
		consumeDataMsg(p_dataMsg);

		scheduleAt(simTime()+delay_us*1e-6, p_drainMsg);
   }
   else
   {
		// The queue is empty. Next message needs to immediatly pop
		// so we clean the drain event
		ev << "-I- " << fullPath() << " Nothing to POP" << endl;
		cancelAndDelete(p_drainMsg);
		p_drainMsg = 0;
   }
}

// hickup really means we  drain and set another one.
void IBSink::handleHiccup(cMessage *p_msg)
{
	double delay;

	if ( duringHiccup )
	{
		// we are inside a hiccup - turn it off and schedule next ON
		duringHiccup = 0;
		delay = par("hiccupDelay_us");
		if (!ev.disabled())
			ev << "-I- " << fullPath() << " Hiccup OFF for:" << delay << "usec" << endl;

		// as we are out of hiccup make sure we have at least one outstanding drain
		if (! p_drainMsg)
			newDrainMessage(1e-3); // 1ns
	}
	else
	{
		// we need to start a new hiccup
		duringHiccup = 1;
		delay = par("hiccupDuration_us");
		
		if (!ev.disabled())
			ev << "-I- " << fullPath() << " Hiccup ON for:" << delay << "usec" << endl ;
	}

	hiccupStats.collect( simTime() );
	scheduleAt(simTime()+delay*1e-6, p_hiccupMsg);
}

void IBSink::handleMessage(cMessage *p_msg)
{
	double delay;
	int kind = p_msg->kind();

	if ( kind == IB_DATA_MSG )
	{
		handleData((IBDataMsg *)p_msg);
	}
	else if ( kind == IB_POP_MSG )
	{
		handlePop(p_msg);
	}
	else if ( kind == IB_HICCUP_MSG )
	{
		handleHiccup(p_msg);
	}
	else if ( kind == IB_FLOWCTRL_MSG )
	{
		if (!ev.disabled())
			ev << "-I- " << fullPath() << " Dropping flow control message";
		delete p_msg;
	}
	else if ( kind == IB_DONE_MSG )
	{
		delete p_msg;
	}
	else
	{
		ev << "-E- " << fullPath() << " does not know what to with msg:" 
		   <<  p_msg->kind() << "is local:" << p_msg->isSelfMessage() 
         << " senderModule:" << p_msg->senderModule()
         << endl;
		delete p_msg;
	}
}

void IBSink::finish()
{
	char buf[128];
	recordScalar("Time last packet consumed:", lastConsumedPakcet);
	ev << "STAT: " << fullPath()
		<< " Data Packets Wait Time: num/avg/max/std: " 
		<< waitStats.samples() << " / " << waitStats.mean() << " / "
		<< waitStats.max() << " / "  << waitStats.stddev() << endl; 

	ev << "STAT: " << fullPath() << " Packet Lifetime: num/avg/max/std:" 
		<< PakcetFabricTime.samples() << " / " << PakcetFabricTime.mean() << " / "
		<< PakcetFabricTime.max() << " / "  << PakcetFabricTime.stddev() << endl;


	double iBW = AccBytesRcv / (simTime() - FirstPktRcvTime);
	ev << "STAT: " << fullPath() << " Sink input BW (B/s):" << iBW  << endl;

   ev << "STAT: " << fullPath() << " Packet per VL:";
	for (int vl = 0; vl < maxVL+1; vl++) {
	  ev << VLPackets[vl] << " "; 
	  sprintf(buf, "%s VL:%d total packets", fullPath().c_str() , vl);
	  recordScalar(buf, VLPackets[vl]);
	}
	ev << endl;
	waitStats.recordScalar();
	PakcetFabricTime.recordScalar();
	
}



