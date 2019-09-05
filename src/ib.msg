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
// Base class for all type of IB wire messages
//

enum IB_MSGS {

    IB_DATA_MSG       	= 1;  	// Credit message
    IB_FLOWCTRL_MSG   	= 2;  	// Flow control message
    IB_SENT_MSG     	= 3;  	// IBRX internal send message; IBA internal sent message  
    IB_DESC_MSG       	= 4;  	// IB descriptor message
    IB_TXCRED_MSG     	= 5;  	// Txcred message
    IB_RXCRED_MSG     	= 6;  	// RxCred message
    CLK_MSG       	= 7;  	// Clock message
    IB_FIFOSTAT_MSG   	= 8;  	// FIFO full / non full message
    FDB_XDONE_MSG     	= 9;  	// Translation complete
    IB_POPDATA_MSG    	= 10; 	// Pop Data from IBA
    IB_ENDPKT_MSG	= 11;	// End packet information from IBA to DP / IBTX
    IB_MINTIME_MSG	= 12;	// Should send a flow control credit update
    IB_POP_MSG		= 13;	// Pop message for IBTX
    IB_HICCUP_MSG   	= 14;
    IB_FREE_MSG     	= 15;
    IB_DONE_MSG     	= 16;
    IB_FTDPPUSH_MSG    	= 17;	// Push FTDP FIFOs 
    IB_TQVLPUSH_MSG     = 18;	// Message to push TQxVL
    IB_VLARB_MSG    	= 19;	// Result of VL arbitration 
    IB_RQARB_MSG    	= 20;	// Result of RQ arbitration
    IB_FTDPPOP_MSG      = 21;   // Message to pop FTDP
    IB_TQPOPOP_MSG      = 22;   // Message to pop FTDP
    IB_VLARBINV_MSG     = 23;   // Indicates we should invalidate all ports related to a given TDM_index
    IB_DPRDY_MSG        = 24;   // DP tells IBA it can bring more data
    IB_TQVLPOP_MSG      = 25;	// Message to pop TQxVL
    IBAFDB_MSG          = 26;       // Interface IBA-FDB
    };

message IBAFDBMsg { 
    fields:
       int PacketID;
       long AccLen; // Indicates length to accumulate in bytes
};

message IBWireMsg
{       
    fields:
	short VL; // the VL for the packet
};

//
// NOTE the generic message already have length in bits !!!
// 

message IBDataMsg extends IBWireMsg
{
    fields:
	int packetId;       	// globally unique packet ID 
	int creditSn;       	// serial number with the packet: 0..N 
	int PacketLength;       // Total number of credits in this packet. 
				// Note that we also have length = number of bits in  current credit (=64*8)
	int PacketLengthBytes;  // Total packet Length in Bytes
	int dstLid;         	// target LID 
	int srcLid;         	// source LID 
	int SL;             	// the SL 
	int useStatic;       	// Used in ib_credits
	simtime_t  SwTimeStamp;	// Time stamp set when credit gets in IBRX     
};

message IBFlowControl extends IBWireMsg
{
    fields:
	long FCTBS;
	long FCCL;
};

message IBTQxVLPushPopMsg { 
    fields:
        int port;
};

message IBArbInvMsg { 
    fields:
        int idx; // 0,1,2 
};

message IBDescMsg extends IBWireMsg
{
    fields:
	int packetId;       	// globally unique packet ID 
	int packetLength;   	// packetLength in credits 
	int packetLengthBytes;  // packetLength in Bytes 
	int dstLid;         	// target LID 
	int srcLid;         	// source LID 
	int SL;             	// the SL 
	int sPortID ;           // Source PortID
	int dPortID;        	// Destination PortID
	short oVL;          	// Output VL
	simtime_t  SwTimeStamp;	// Time stamp set when descriptor gets in FDB     
        bool stfwd;             // True if packet is ready for transmission from FDB to DP 
        double ratio;              // ratio between Input port and output port speed
};

// This message carry the "just" received flow control packet FCCL&FCTBS
message IBTxCredMsg
{
    fields:
	int VL;
	long FCCL;
	long FCTBS;
};
    
// The following messages are internal to the device (do not go on the wire)
// This message carry the FCCL from the the IBUF to the OBUF
message IBRxCredMsg
{
    fields:
        int VL;
        long FCCL;
};

// Message FIFO Status
message IBFifoStatus
{
    fields:
	bool full;
	int FIFOindex; 
};

// Message translation done
message FDBXDone
{
    fields:
	int FDBindex;  
	int FTnumber; 
};


// Message used to POP IBA array , for self SEND message
message IBAPacketID
{
    fields:
	int PacketID;
};


// Message that passes RQxTQ*VL arbitrated     
message RQxTQxVLMsg  	
{
    fields:
	int RQ;
	int VL;
	int TQ;
        int PacketID;
};

// FOR IB_CREDITS ONLY

// When a credit is sent by the arbiter it needs to notify the IBUF
// which packet was sent
message IBSentMsg
{
    fields:
        short VL;
        short usedStatic;
        short wasLast;
};

// OBUF Informs the driver VLA about the number of credits it can hold
// this in turn can trigger a new arbitration.
// NOTE: to avoid races the arbiter only uses this info as a trigger for
// arbitration, it actually access the Q when it needs to know the Q free
// credits
message IBFreeMsg
{
    fields:
        short VL;
};
                                                                                                                             
// IBUF informs the VLAs it is driving about its ability to take a new arbitration
// NOTE: to avoid races the arbiter only uses this info as a trigger for
// arbitration, it actually access the IBUF BusyPorts structure to know if the
// arbitration succeeded message
message IBDoneMsg
{
    fields:
        short VL;
};


message IBRxNumMsg
{
    fields:
        int  RXPortId;
};


message FifoFree
{
    fields:
        int  NumFreeCredits;
};
