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
// IBUF: Input Buffer
//
simple IBInBuf
    parameters:
			// Struactural parameters
         width :       			numeric, // input line width 4x,8x,12x
         isHcaIBuf : 			numeric, // if 1 means the input buffer is part
	                                    // of HCA and no routing done.
			numPorts : 				numeric, // number of output ports to drive to
			maxBeingSent : 		numeric, // max number of parallel 
	                                    // sends = switch "speed-up"
			totalBufferSize :    numeric, // the total buffer size in credits
			maxVL : 					numeric, // max VL num. If only VL0 value is 0

			// Each VL is assigned a max number of static credits 
			maxStatic0 : 			numeric, 
			maxStatic1 : 			numeric,
			maxStatic2 : 			numeric,
			maxStatic3 : 			numeric, 
			maxStatic4 : 			numeric,
			maxStatic5 : 			numeric,
			maxStatic6 : 			numeric, 
			maxStatic7 : 			numeric,
			recordVectors: 		numeric; // control recording of vectors
    gates:
        in: in, sent[];
        out: out[], txCred, rxCred;
endsimple


