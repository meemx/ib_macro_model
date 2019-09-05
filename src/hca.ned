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
// This NED provides an HCA model

import 
	"gen.ned", 
	"sink.ned", 
	"obuf.ned", 
	"vlarb.ned", 
	"ibuf.ned";

module HCA
    gates:
        out: out;
        in: in;

    submodules:
        gen: IBGenerator;
            display: "i=block/source;p=246,170";
        sink: IBSink;
            display: "i=block/sink;p=248,74";
        obuf: IBOutBuf;
            display: "i=block/queue;p=69,169";
        ibuf: IBInBuf;
            parameters:
                isHcaIBuf = 1,
                numPorts = 1;
            gatesizes:
                out[1],
                sent[1];
            display: "i=block/subqueue;p=72,73";
        vlarb: IBVLArb;
            parameters:
                isHcaArbiter = 1,
                numPorts = 1;
            gatesizes:
                in[1],
                sent[1];

            display: "i=block/dispatch;p=158,170";
    connections:
        gen.out --> vlarb.in[0];
        vlarb.sent[0] --> gen.sent display "m=m,68,20,68,20";
        vlarb.out --> obuf.in display "m=m,16,56,24,60";
        obuf.free --> vlarb.free display "m=m,20,28,4,24";

        ibuf.out[0] --> sink.in;
        sink.sent --> ibuf.sent[0];



        ibuf.rxCred --> obuf.rxCred display "m=m,28,12,36,12";
        ibuf.txCred --> vlarb.txCred;

        in --> ibuf.in;
        obuf.out --> out;
    display: "b=281,225";
endmodule
