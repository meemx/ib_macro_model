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
// The IBOutBuf implements an IB port FIFO
// See functional description in the header file.
//
#include "ib_m.h"
#include "obuf.h"

Define_Module( IBOutBuf );

void IBOutBuf::initialize()
{
	// read parameters
	speed = par("speed");
	width = par("width");
	qSize = par("size");
	maxVL = par("maxVL");
	recordVectors = par("recordVectors");

	Enabled = false;
	if (width==4 || width==8 || width==12) {
      Enabled = true;
	}
	// MinTime is a function of width
	credMinTime_us = par("credMinTime");
	credMinTime_us = credMinTime_us*4/width;

	// Initiazlize the statistical collection elements
	qDepthHist.setName("Queue Usage");
	qDepthHist.setRangeAutoUpper(0, 10, 1);
	packetStoreHist.setName("Packet Storage Time");
	packetStoreHist.setRangeAutoUpper(0, 10, 1.5);
	flowControlDelay.setName("Time between VL0 FC");
	flowControlDelay.setRangeAutoUpper(0,10,1.2);
	qDepth.setName("Queue Depth");
	// 4x 2.5Gbps = 1Byte/nsec ; but we need the 10/2.5/4.0 ...
	popDelayPerByte_us = 1.0e-3 * 10 / width / speed;
	WATCH(popDelayPerByte_us);
	WATCH(credMinTime_us);

	// we will allocate a pop messgae only when the first credit is pushed in
	p_popMsg = NULL;

	// queue is empty
	prevPopWasDataCredit = 0;
	insidePacket = 0;
	prevFCTime = 0;
	isMinTimeUpdate = 0;

	for ( int i = 0; i < maxVL+1; i++ )
	{
		prevSentFCTBS.push_back(-9999);
		prevSentFCCL.push_back(-9999);
		FCCL.push_back(0);
		FCTBS.push_back(0);
	}

	WATCH_VECTOR(prevSentFCTBS);
	WATCH_VECTOR(prevSentFCCL);
	WATCH_VECTOR(FCTBS);
	WATCH_VECTOR(FCCL);

	if (Enabled) {
      // but we do want to have a continous flow of MinTime
      p_minTimeMsg = new cMessage("minTime", IB_MINTIME_MSG);
      // Send the first mintime immediately so that all is initialised when we get first packets
      //scheduleAt(simTime() + credMinTime_us*1e-6, p_minTimeMsg);
      scheduleAt(simTime() , p_minTimeMsg);
	}

	ev << "-I- " << fullPath() << " init completed" << endl;
}

// send the message out
// Init a new pop message and schedule it after delay
// Note that at this stage the Q might be empty but a
// data packet will be streamed out
void IBOutBuf::sendOutMessage(IBWireMsg *p_msg) {
	double delay = p_msg->length() / 8.0 * popDelayPerByte_us;
	if ( ! p_popMsg )
	{
		p_popMsg = new cMessage("pop", IB_POP_MSG);
		scheduleAt(simTime() + delay*1e-6, p_popMsg);
	}
	if (!ev.disabled())
	   ev << "-I- " << fullPath() << " sending msg:" << p_msg->name() << " at time " << simTime() <<endl;
	// track out going packets
	if ( p_msg->kind() == IB_DATA_MSG )
	{
		IBDataMsg *p_dataMsg = (IBDataMsg *)p_msg;

		// track if we are in the middle of packet
		if (!p_dataMsg->getCreditSn() && (p_dataMsg->getPacketLength() > 1))
			insidePacket = 1;
		else if (p_dataMsg->getCreditSn() + 1 == p_dataMsg->getPacketLength())
			insidePacket = 0;

		FCTBS[p_msg->getVL()]++;
	}
	send(p_msg, "out");
}

// Q a message to be sent out.
// If there is no pop message pending can directly send...
void
IBOutBuf::qMessage(IBDataMsg *p_msg) {
	// we stamp it to know how much time it stayed with us
	//p_msg->setTimestamp(simTime());

	if ( p_popMsg )
	{
		if ( qSize <= queue.length() )
		{
			ev << "-E- " << fullPath() << " need to insert into a full Q" << endl;
			ev.flush();
			exit(1);
		}

		if (!ev.disabled()) {
			ev << "-I- " << fullPath() << " queued data msg:" << p_msg->name() << " Qdepth " << queue.length() << endl;
			recordVectors = par("recordVectors");
		}

		queue.insert(p_msg);
		if ( recordVectors )
			qDepth.record(queue.length());
	}
	else
	{
		// track the time this PACKET (all credits) spent in the Q
		// the last credit of a packet always
		if ( p_msg->getCreditSn() + 1 == p_msg->getPacketLength() )
		{
			//packetStoreHist.collect( simTime() - packetHeadTimeStamp );
		}
		else if ( p_msg->getCreditSn() == 0 )
		{
			packetHeadTimeStamp = p_msg->timestamp();
		}
		sendOutMessage(p_msg);
	}
}

// check if need to send a flow control and send it if required.
// return 1 if sent or 0 if not
// this function should be called by the pop event to check if flow control
// is required to be sent
// The minTime event only zeros out the curFlowCtrVL such that the operation 
// restarts.
// New Hermon mode provides extra cases where a flow control might be sent:
// 1. If there are no other messages in the Q 
//
// Also if there are messages in the Q we might not send FC unless the difference
// is large enough
int IBOutBuf::sendFlowControl()
{
   static long flowCtrlId = 0;
   int sentUpdate = 0;

   // we should not continue if the Q is not empty if we aren't in mintime mode
   if (! isMinTimeUpdate && ! queue.empty())
		return(0);

   if (curFlowCtrVL >= maxVL+1) 
   {
	  return(0);
   }

   for (; (sentUpdate == 0) && (curFlowCtrVL < maxVL+1); curFlowCtrVL++ )
   {
		int i = curFlowCtrVL;

		if (i == 0)
		{
			// avoid the first send...
			if (prevFCTime)
				// flowControlDelay.collect(simTime() - prevFCTime);
				prevFCTime = simTime();
		}

		// ignore prevSentFCTBS[i] == FCTBS[i] since the other side tracks ABR...
		if ( (prevSentFCTBS[i] != FCTBS[i]))
		{
			// create a new message and place in the Q
			char name[128];
			sprintf(name, "fc-%d-%d", i, flowCtrlId++);
			IBFlowControl *p_msg = new IBFlowControl(name, IB_FLOWCTRL_MSG);
		 
			p_msg->setLength(8*8);
			p_msg->setVL(i);
			p_msg->setFCCL(FCCL[i]);
			p_msg->setFCTBS(FCTBS[i]);
			prevSentFCCL[i] = FCCL[i];
			prevSentFCTBS[i] = FCTBS[i];
			if (!ev.disabled())
				ev << "-I- " << fullPath() << " generated:" << p_msg->name() 
					<< " vl:" << p_msg->getVL() << " FCTBS:" 
					<< p_msg->getFCTBS() << " FCCL:" << p_msg->getFCCL() << endl;

			// we do not need to Q as we are only called in pop
			sendOutMessage(p_msg);

			sentUpdate = 1;
		}
		// last VL zeros the min time update flag
		if (curFlowCtrVL == 7)
			isMinTimeUpdate = 0;
   }

   return(sentUpdate);
}

// Handle Pop Message
// Should not be called if the Q is empty.
// Simply pop and schedule next pop.
// Also schedule a "free" message to be sent out later
void IBOutBuf::handlePop()
{
	double delay;
  
	cancelAndDelete( p_popMsg );
	p_popMsg = NULL;

	// if we got a pop - it means the previous message just left the
	// OBUF. In that case if it was a data credit packet we have now a
	// new space for it. tell the VLA.
	if (prevPopWasDataCredit)
	{
		cMessage *p_msg = new cMessage("free", IB_FREE_MSG);
		if (!ev.disabled())
			ev << "-I- " << fullPath() << " sending 'free' to VLA as last packet just completed." << endl;
		send(p_msg, "free");
	}

	// try sending a flow control if required:
	if (!insidePacket && sendFlowControl()) 
	{
		prevPopWasDataCredit = 0;
		return;
	}

	// got to pop from the queue if anything there
	if ( !queue.empty() )
	{
		IBWireMsg *p_msg = (IBWireMsg *)queue.pop();
		if ( p_msg->kind() == IB_DATA_MSG )
		{
			IBDataMsg *p_cred = (IBDataMsg *)p_msg;
			if (!ev.disabled())
				ev << "-I- " << fullPath() << " popped data message:" << p_cred->name() << endl;
			sendOutMessage(p_msg);

			// track the time this PACKET (all credits) spent in the Q
			// the last credit of a packet always
			if ( p_cred->getCreditSn() + 1 == p_cred->getPacketLength() )
			{
				//packetStoreHist.collect( simTime() - packetHeadTimeStamp );
			}
			else if ( p_cred->getCreditSn() == 0 )
			{
				packetHeadTimeStamp = p_msg->timestamp();
			}

			// we just popped a real credit 
			prevPopWasDataCredit = 1;
		}
		else
		{
			ev << "-E- " << fullPath() << " unknown message type to pop:"
				<< p_msg->kind() << endl;
		}
	}
	else
	{
		// The queue is empty. Next message needs to immediatly pop
		// so we clean this event
		if (!ev.disabled())
			ev << "-I- " << fullPath() << " nothing to POP" << endl;
		prevPopWasDataCredit = 0;
	}

	if (!ev.disabled())
		recordVectors = par("recordVectors");

	if ( recordVectors )
		qDepth.record(queue.length());
}

// Handle MinTime:
// If the prev sent VL Credits are no longer valid send push an update
void IBOutBuf::handleMinTime()
{
   if (!ev.disabled())
		ev << "-I- " << fullPath() << " handling MinTime event" << endl;
   
	curFlowCtrVL = 0;
	isMinTimeUpdate = 1;
	// if we do not have any pop message - we need to create one immediatly
	if (! p_popMsg )
	{
		p_popMsg = new cMessage("pop", IB_POP_MSG);
		scheduleAt(simTime() + 1e-9, p_popMsg);
	}

	// we use the min time to collect Queue depth stats:
	//qDepthHist.collect( queue.length() );

	scheduleAt(simTime() + credMinTime_us*1e-6, p_minTimeMsg);
}
  
// Handle rxCred
void IBOutBuf::handleRxCred(IBRxCredMsg *p_msg)
{
	// update FCCL...
	FCCL[p_msg->getVL()] = p_msg->getFCCL();
	delete p_msg;
}

void IBOutBuf::handleMessage(cMessage *p_msg)
{
	int msgType = p_msg->kind();
	if ( msgType == IB_POP_MSG )
	{
		handlePop();
	}
	else if ( msgType == IB_MINTIME_MSG )
	{
		handleMinTime();
	}
	else if ( msgType == IB_DATA_MSG )
	{
		qMessage((IBDataMsg*)p_msg);
	}
	else if ( msgType == IB_RXCRED_MSG )
	{
		handleRxCred((IBRxCredMsg*)p_msg);
	}
	else
	{
		ev << "-E- " << fullPath() << " do no know how to handle message:" << msgType << endl;
		delete p_msg;
	}
}

void IBOutBuf::finish()
{
	/* ev << "STAT: " << fullPath() << " Data Packet Q time num/avg/max/std:"
		<< packetStoreHist.samples() << " / "
		<< packetStoreHist.mean() << " / "
		<< packetStoreHist.max() << " / "
		<< packetStoreHist.stddev() << endl; 
		ev << "STAT: " << fullPath() << " Q depth num/avg/max/std:"        
		<< qDepthHist.samples() << " / "
		<< qDepthHist.mean() << " / "
		<< qDepthHist.max() << " / "
		<< qDepthHist.stddev() << endl;
	 ev << "STAT: " << fullPath() << " FlowControl Delay num/avg/max/std:"
		<< flowControlDelay.samples() << " / "
		<< flowControlDelay.mean() << " / "
		<< flowControlDelay.max() << " / "
		<< flowControlDelay.stddev() << endl;
	*/
}
