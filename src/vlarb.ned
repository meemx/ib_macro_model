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
// VLArb: Packet Arbiter
//
simple IBVLArb
    parameters:	
		recordVectors: numeric, 	// control recording of vectors
      width : numeric,           // Interface width 1x 4x 8x 12x
		isHcaArbiter : numeric,    // if not zero emans the VLA is in HCA
		numPorts : numeric,	   	// number of input ports 
      coreFreq_MH : numeric,     // Core frequency in MHz
	   maxVL:numeric,             // Maximum VL we send on
		vlHighLimit : numeric,     // IB Vl Arb High Limit
      highVLArbEntries : string, // "vl:weight vl:weight ..."
      lowVLArbEntries : string;  // "vl:weight vl:weight ..."
    gates:
        in: in[], txCred, free;
        out: out, sent[];
endsimple

