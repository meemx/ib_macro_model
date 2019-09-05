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
simple IBGenerator
    parameters:   
         floodVLs : string,         // list fo VLs to flood       
         width : numeric,           // the output link width ,4,8,12
         speed : numeric,           // the output speed 2.5, 5.0, 10.0
         intraBurstDelay : numeric, // sleep time between bursts in nsec
         interBurstDelay : numeric, // between packets of same burst in nsec
         burstLength : numeric,     // the length of the burst in packets  
			burstNumber : numeric,     // Number of burst overall
         msgLength : numeric,       // Total message length in bytes
         msgLengthInMTUs : numeric, // Total message length in credits
         mtu : numeric,		    		// Number of credits in MTU
         srcLid : numeric,          // the source LID of the packets
         dstLid : numeric,          // the maximal LID in the system
         dstSeqMode: string,        // "dstLid", "dstSeqOnce", "dstSeqLoop"
         dstSeqVecFile: string,     // vec file holding the destination sequnce
         sizeSeqVecFile: string,    // vec file holding the message size
         dstSeqIndex: numeric,      // index of this generator in sequence file
         dstHotSpotPerc : numeric,  // % of the traffic that go to hot spot
         dstBurstHSDel: numeric,    // [msec] HotSpot traffic in BURST_HOTSPOT
         dstBurstHSInterDel: numeric,// [msec] Regular Traffic in BURST_HOTSPOT
         trafficDist:string,        // Traffic distribution
         VL : numeric,              // VL of packets being generated flood mode
         SL : numeric,              // the SL the packets will be injected on
	 		creditSize : numeric,
	 		packetlengthbytes:numeric, // Length of packet in bytes
			packetlength:numeric,      // Length of packet in credit
         PktLenDist : string,       // Vector of packet lengths to inject in B
         PktLenProb : string;       // Probability (in %) of corresp. PktLen defined in PktLenDist 
   gates:
         in: sent;
         out: out;
endsimple
