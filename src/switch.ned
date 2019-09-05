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

import 
	"obuf.ned", 
	"vlarb.ned", 
	"ibuf.ned";

module SwitchPort
    parameters:
        numSwitchPorts: numeric;

    gates:
        in:  in;
        in:  sw_in[];
        in:  sw_in_sent[];
        out: out;
        out: sw_out[];
        out: sw_out_sent[];

    submodules:
        obuf: IBOutBuf;
            display: "i=block/boundedqueue;p=76,80";
        ibuf: IBInBuf;
            parameters:
	             isHcaIBuf = 0,
                numPorts  = numSwitchPorts;
            gatesizes:
                out[numSwitchPorts],
                sent[numSwitchPorts];
            display: "i=block/subqueue;p=191,80";
        vlarb: IBVLArb;
            parameters:
                isHcaArbiter = 0,
                numPorts = numSwitchPorts;
            gatesizes:
                in[numSwitchPorts],
                sent[numSwitchPorts];

            display: "p=76,156;i=block/dispatch";
    connections:
        obuf.out --> out;
        in --> ibuf.in;
        for i=0..numSwitchPorts - 1 do
            vlarb.sent[i] --> sw_out_sent[i];
            sw_in_sent[i] --> ibuf.sent[i];
            ibuf.out[i] --> sw_out[i];
            sw_in[i] --> vlarb.in[i];
        endfor;

        ibuf.rxCred --> obuf.rxCred;
        ibuf.txCred --> vlarb.txCred;
        vlarb.out --> obuf.in;
        obuf.free --> vlarb.free;

    display: "b=249,215";
endmodule

module Switch
    parameters:
			numSwitchPorts:	numeric, // Total num of switch ports
			fdbsVecFile:		string,  // FDBs vectors file
			fdbIndex:			numeric, // index of the FDB in the vec files

         ISWDelay : numeric,        // Delay in ns brought by IBUF in SWitch
         VSWDelay : numeric;        // Delay in ns brought by IBUF in SWitch

    gates:
        in: in[];
        out: out[];

    submodules:
        port: SwitchPort[numSwitchPorts];
            parameters:
                numSwitchPorts = numSwitchPorts;
            gatesizes:
                sw_out[numSwitchPorts],
                sw_in[numSwitchPorts],
                sw_out_sent[numSwitchPorts],
                sw_in_sent[numSwitchPorts];

            display: "p=90,44;b=48,32";
    connections:
        for p=0..numSwitchPorts - 1 do
            in[p] --> port[p].in;
            port[p].out --> out[p];
        endfor;
        for p=0..numSwitchPorts - 1, i=0..numSwitchPorts - 1 do
            port[p].sw_out[i] --> port[i].sw_in[p];
            port[p].sw_out_sent[i] --> port[i].sw_in_sent[p];
        endfor;
endmodule
