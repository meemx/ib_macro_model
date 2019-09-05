import "../../src/hca.ned", "../../src/switch.ned";

module h2_s1
	submodules:
		H_1:HCA; // LID 1
		H_2:HCA; // LID 2
		SW: Switch;
		parameters:
			numSwitchPorts = 2;
			gatesizes: out[2], in[2];
	connections:
		H_1.out --> delay 5ns --> SW.in[0];
		H_2.out --> delay 5ns --> SW.in[1];
		SW.out[0] --> delay 5ns --> H_1.in;
		SW.out[1] --> delay 5ns --> H_2.in;
endmodule

network FABRIC : h2_s1
endnetwork
