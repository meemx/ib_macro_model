import "../../src/hca.ned", "../../src/switch.ned";

module is1_3l_32n
	submodules:
		H_1_U1:HCA; // LID 1
		H_10_U1:HCA; // LID 10
		H_11_U1:HCA; // LID 11
		H_12_U1:HCA; // LID 12
		H_13_U1:HCA; // LID 13
		H_14_U1:HCA; // LID 14
		H_15_U1:HCA; // LID 15
		H_16_U1:HCA; // LID 16
		H_17_U1:HCA; // LID 17
		H_18_U1:HCA; // LID 18
		H_19_U1:HCA; // LID 19
		H_2_U1:HCA; // LID 2
		H_20_U1:HCA; // LID 20
		H_21_U1:HCA; // LID 21
		H_22_U1:HCA; // LID 22
		H_23_U1:HCA; // LID 23
		H_24_U1:HCA; // LID 24
		H_25_U1:HCA; // LID 25
		H_26_U1:HCA; // LID 26
		H_27_U1:HCA; // LID 27
		H_28_U1:HCA; // LID 28
		H_29_U1:HCA; // LID 29
		H_3_U1:HCA; // LID 3
		H_30_U1:HCA; // LID 30
		H_31_U1:HCA; // LID 31
		H_32_U1:HCA; // LID 32
		H_4_U1:HCA; // LID 4
		H_5_U1:HCA; // LID 5
		H_6_U1:HCA; // LID 6
		H_7_U1:HCA; // LID 7
		H_8_U1:HCA; // LID 8
		H_9_U1:HCA; // LID 9
		SW_L0_0_0_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L0_0_1_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L0_1_0_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L0_1_1_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L1_0_0_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L1_0_1_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L1_1_0_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L1_1_1_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L1_2_0_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L1_2_1_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L1_3_0_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L1_3_1_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L2_0_0_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L2_0_1_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L2_1_0_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L2_1_1_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L2_2_0_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L2_2_1_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L2_3_0_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
		SW_L2_3_1_U1: Switch;
		parameters:
			numSwitchPorts = 8;
			gatesizes: out[8], in[8];
	connections:
		H_1_U1.out --> delay 5ns --> SW_L2_0_0_U1.in[0];
		H_10_U1.out --> delay 5ns --> SW_L2_1_0_U1.in[1];
		H_11_U1.out --> delay 5ns --> SW_L2_1_0_U1.in[2];
		H_12_U1.out --> delay 5ns --> SW_L2_1_0_U1.in[3];
		H_13_U1.out --> delay 5ns --> SW_L2_1_1_U1.in[0];
		H_14_U1.out --> delay 5ns --> SW_L2_1_1_U1.in[1];
		H_15_U1.out --> delay 5ns --> SW_L2_1_1_U1.in[2];
		H_16_U1.out --> delay 5ns --> SW_L2_1_1_U1.in[3];
		H_17_U1.out --> delay 5ns --> SW_L2_2_0_U1.in[0];
		H_18_U1.out --> delay 5ns --> SW_L2_2_0_U1.in[1];
		H_19_U1.out --> delay 5ns --> SW_L2_2_0_U1.in[2];
		H_2_U1.out --> delay 5ns --> SW_L2_0_0_U1.in[1];
		H_20_U1.out --> delay 5ns --> SW_L2_2_0_U1.in[3];
		H_21_U1.out --> delay 5ns --> SW_L2_2_1_U1.in[0];
		H_22_U1.out --> delay 5ns --> SW_L2_2_1_U1.in[1];
		H_23_U1.out --> delay 5ns --> SW_L2_2_1_U1.in[2];
		H_24_U1.out --> delay 5ns --> SW_L2_2_1_U1.in[3];
		H_25_U1.out --> delay 5ns --> SW_L2_3_0_U1.in[0];
		H_26_U1.out --> delay 5ns --> SW_L2_3_0_U1.in[1];
		H_27_U1.out --> delay 5ns --> SW_L2_3_0_U1.in[2];
		H_28_U1.out --> delay 5ns --> SW_L2_3_0_U1.in[3];
		H_29_U1.out --> delay 5ns --> SW_L2_3_1_U1.in[0];
		H_3_U1.out --> delay 5ns --> SW_L2_0_0_U1.in[2];
		H_30_U1.out --> delay 5ns --> SW_L2_3_1_U1.in[1];
		H_31_U1.out --> delay 5ns --> SW_L2_3_1_U1.in[2];
		H_32_U1.out --> delay 5ns --> SW_L2_3_1_U1.in[3];
		H_4_U1.out --> delay 5ns --> SW_L2_0_0_U1.in[3];
		H_5_U1.out --> delay 5ns --> SW_L2_0_1_U1.in[0];
		H_6_U1.out --> delay 5ns --> SW_L2_0_1_U1.in[1];
		H_7_U1.out --> delay 5ns --> SW_L2_0_1_U1.in[2];
		H_8_U1.out --> delay 5ns --> SW_L2_0_1_U1.in[3];
		H_9_U1.out --> delay 5ns --> SW_L2_1_0_U1.in[0];
		SW_L0_0_0_U1.out[0] --> delay 5ns --> SW_L1_0_0_U1.in[4];
		SW_L0_0_0_U1.out[1] --> delay 5ns --> SW_L1_0_0_U1.in[5];
		SW_L0_0_0_U1.out[2] --> delay 5ns --> SW_L1_1_0_U1.in[4];
		SW_L0_0_0_U1.out[3] --> delay 5ns --> SW_L1_1_0_U1.in[5];
		SW_L0_0_0_U1.out[4] --> delay 5ns --> SW_L1_2_0_U1.in[4];
		SW_L0_0_0_U1.out[5] --> delay 5ns --> SW_L1_2_0_U1.in[5];
		SW_L0_0_0_U1.out[6] --> delay 5ns --> SW_L1_3_0_U1.in[4];
		SW_L0_0_0_U1.out[7] --> delay 5ns --> SW_L1_3_0_U1.in[5];
		SW_L0_0_1_U1.out[0] --> delay 5ns --> SW_L1_0_1_U1.in[4];
		SW_L0_0_1_U1.out[1] --> delay 5ns --> SW_L1_0_1_U1.in[5];
		SW_L0_0_1_U1.out[2] --> delay 5ns --> SW_L1_1_1_U1.in[4];
		SW_L0_0_1_U1.out[3] --> delay 5ns --> SW_L1_1_1_U1.in[5];
		SW_L0_0_1_U1.out[4] --> delay 5ns --> SW_L1_2_1_U1.in[4];
		SW_L0_0_1_U1.out[5] --> delay 5ns --> SW_L1_2_1_U1.in[5];
		SW_L0_0_1_U1.out[6] --> delay 5ns --> SW_L1_3_1_U1.in[4];
		SW_L0_0_1_U1.out[7] --> delay 5ns --> SW_L1_3_1_U1.in[5];
		SW_L0_1_0_U1.out[0] --> delay 5ns --> SW_L1_0_0_U1.in[6];
		SW_L0_1_0_U1.out[1] --> delay 5ns --> SW_L1_0_0_U1.in[7];
		SW_L0_1_0_U1.out[2] --> delay 5ns --> SW_L1_1_0_U1.in[6];
		SW_L0_1_0_U1.out[3] --> delay 5ns --> SW_L1_1_0_U1.in[7];
		SW_L0_1_0_U1.out[4] --> delay 5ns --> SW_L1_2_0_U1.in[6];
		SW_L0_1_0_U1.out[5] --> delay 5ns --> SW_L1_2_0_U1.in[7];
		SW_L0_1_0_U1.out[6] --> delay 5ns --> SW_L1_3_0_U1.in[6];
		SW_L0_1_0_U1.out[7] --> delay 5ns --> SW_L1_3_0_U1.in[7];
		SW_L0_1_1_U1.out[0] --> delay 5ns --> SW_L1_0_1_U1.in[6];
		SW_L0_1_1_U1.out[1] --> delay 5ns --> SW_L1_0_1_U1.in[7];
		SW_L0_1_1_U1.out[2] --> delay 5ns --> SW_L1_1_1_U1.in[6];
		SW_L0_1_1_U1.out[3] --> delay 5ns --> SW_L1_1_1_U1.in[7];
		SW_L0_1_1_U1.out[4] --> delay 5ns --> SW_L1_2_1_U1.in[6];
		SW_L0_1_1_U1.out[5] --> delay 5ns --> SW_L1_2_1_U1.in[7];
		SW_L0_1_1_U1.out[6] --> delay 5ns --> SW_L1_3_1_U1.in[6];
		SW_L0_1_1_U1.out[7] --> delay 5ns --> SW_L1_3_1_U1.in[7];
		SW_L1_0_0_U1.out[0] --> delay 5ns --> SW_L2_0_0_U1.in[4];
		SW_L1_0_0_U1.out[1] --> delay 5ns --> SW_L2_0_0_U1.in[5];
		SW_L1_0_0_U1.out[2] --> delay 5ns --> SW_L2_0_1_U1.in[4];
		SW_L1_0_0_U1.out[3] --> delay 5ns --> SW_L2_0_1_U1.in[5];
		SW_L1_0_0_U1.out[4] --> delay 5ns --> SW_L0_0_0_U1.in[0];
		SW_L1_0_0_U1.out[5] --> delay 5ns --> SW_L0_0_0_U1.in[1];
		SW_L1_0_0_U1.out[6] --> delay 5ns --> SW_L0_1_0_U1.in[0];
		SW_L1_0_0_U1.out[7] --> delay 5ns --> SW_L0_1_0_U1.in[1];
		SW_L1_0_1_U1.out[0] --> delay 5ns --> SW_L2_0_0_U1.in[6];
		SW_L1_0_1_U1.out[1] --> delay 5ns --> SW_L2_0_0_U1.in[7];
		SW_L1_0_1_U1.out[2] --> delay 5ns --> SW_L2_0_1_U1.in[6];
		SW_L1_0_1_U1.out[3] --> delay 5ns --> SW_L2_0_1_U1.in[7];
		SW_L1_0_1_U1.out[4] --> delay 5ns --> SW_L0_0_1_U1.in[0];
		SW_L1_0_1_U1.out[5] --> delay 5ns --> SW_L0_0_1_U1.in[1];
		SW_L1_0_1_U1.out[6] --> delay 5ns --> SW_L0_1_1_U1.in[0];
		SW_L1_0_1_U1.out[7] --> delay 5ns --> SW_L0_1_1_U1.in[1];
		SW_L1_1_0_U1.out[0] --> delay 5ns --> SW_L2_1_0_U1.in[4];
		SW_L1_1_0_U1.out[1] --> delay 5ns --> SW_L2_1_0_U1.in[5];
		SW_L1_1_0_U1.out[2] --> delay 5ns --> SW_L2_1_1_U1.in[4];
		SW_L1_1_0_U1.out[3] --> delay 5ns --> SW_L2_1_1_U1.in[5];
		SW_L1_1_0_U1.out[4] --> delay 5ns --> SW_L0_0_0_U1.in[2];
		SW_L1_1_0_U1.out[5] --> delay 5ns --> SW_L0_0_0_U1.in[3];
		SW_L1_1_0_U1.out[6] --> delay 5ns --> SW_L0_1_0_U1.in[2];
		SW_L1_1_0_U1.out[7] --> delay 5ns --> SW_L0_1_0_U1.in[3];
		SW_L1_1_1_U1.out[0] --> delay 5ns --> SW_L2_1_0_U1.in[6];
		SW_L1_1_1_U1.out[1] --> delay 5ns --> SW_L2_1_0_U1.in[7];
		SW_L1_1_1_U1.out[2] --> delay 5ns --> SW_L2_1_1_U1.in[6];
		SW_L1_1_1_U1.out[3] --> delay 5ns --> SW_L2_1_1_U1.in[7];
		SW_L1_1_1_U1.out[4] --> delay 5ns --> SW_L0_0_1_U1.in[2];
		SW_L1_1_1_U1.out[5] --> delay 5ns --> SW_L0_0_1_U1.in[3];
		SW_L1_1_1_U1.out[6] --> delay 5ns --> SW_L0_1_1_U1.in[2];
		SW_L1_1_1_U1.out[7] --> delay 5ns --> SW_L0_1_1_U1.in[3];
		SW_L1_2_0_U1.out[0] --> delay 5ns --> SW_L2_2_0_U1.in[4];
		SW_L1_2_0_U1.out[1] --> delay 5ns --> SW_L2_2_0_U1.in[5];
		SW_L1_2_0_U1.out[2] --> delay 5ns --> SW_L2_2_1_U1.in[4];
		SW_L1_2_0_U1.out[3] --> delay 5ns --> SW_L2_2_1_U1.in[5];
		SW_L1_2_0_U1.out[4] --> delay 5ns --> SW_L0_0_0_U1.in[4];
		SW_L1_2_0_U1.out[5] --> delay 5ns --> SW_L0_0_0_U1.in[5];
		SW_L1_2_0_U1.out[6] --> delay 5ns --> SW_L0_1_0_U1.in[4];
		SW_L1_2_0_U1.out[7] --> delay 5ns --> SW_L0_1_0_U1.in[5];
		SW_L1_2_1_U1.out[0] --> delay 5ns --> SW_L2_2_0_U1.in[6];
		SW_L1_2_1_U1.out[1] --> delay 5ns --> SW_L2_2_0_U1.in[7];
		SW_L1_2_1_U1.out[2] --> delay 5ns --> SW_L2_2_1_U1.in[6];
		SW_L1_2_1_U1.out[3] --> delay 5ns --> SW_L2_2_1_U1.in[7];
		SW_L1_2_1_U1.out[4] --> delay 5ns --> SW_L0_0_1_U1.in[4];
		SW_L1_2_1_U1.out[5] --> delay 5ns --> SW_L0_0_1_U1.in[5];
		SW_L1_2_1_U1.out[6] --> delay 5ns --> SW_L0_1_1_U1.in[4];
		SW_L1_2_1_U1.out[7] --> delay 5ns --> SW_L0_1_1_U1.in[5];
		SW_L1_3_0_U1.out[0] --> delay 5ns --> SW_L2_3_0_U1.in[4];
		SW_L1_3_0_U1.out[1] --> delay 5ns --> SW_L2_3_0_U1.in[5];
		SW_L1_3_0_U1.out[2] --> delay 5ns --> SW_L2_3_1_U1.in[4];
		SW_L1_3_0_U1.out[3] --> delay 5ns --> SW_L2_3_1_U1.in[5];
		SW_L1_3_0_U1.out[4] --> delay 5ns --> SW_L0_0_0_U1.in[6];
		SW_L1_3_0_U1.out[5] --> delay 5ns --> SW_L0_0_0_U1.in[7];
		SW_L1_3_0_U1.out[6] --> delay 5ns --> SW_L0_1_0_U1.in[6];
		SW_L1_3_0_U1.out[7] --> delay 5ns --> SW_L0_1_0_U1.in[7];
		SW_L1_3_1_U1.out[0] --> delay 5ns --> SW_L2_3_0_U1.in[6];
		SW_L1_3_1_U1.out[1] --> delay 5ns --> SW_L2_3_0_U1.in[7];
		SW_L1_3_1_U1.out[2] --> delay 5ns --> SW_L2_3_1_U1.in[6];
		SW_L1_3_1_U1.out[3] --> delay 5ns --> SW_L2_3_1_U1.in[7];
		SW_L1_3_1_U1.out[4] --> delay 5ns --> SW_L0_0_1_U1.in[6];
		SW_L1_3_1_U1.out[5] --> delay 5ns --> SW_L0_0_1_U1.in[7];
		SW_L1_3_1_U1.out[6] --> delay 5ns --> SW_L0_1_1_U1.in[6];
		SW_L1_3_1_U1.out[7] --> delay 5ns --> SW_L0_1_1_U1.in[7];
		SW_L2_0_0_U1.out[0] --> delay 5ns --> H_1_U1.in;
		SW_L2_0_0_U1.out[1] --> delay 5ns --> H_2_U1.in;
		SW_L2_0_0_U1.out[2] --> delay 5ns --> H_3_U1.in;
		SW_L2_0_0_U1.out[3] --> delay 5ns --> H_4_U1.in;
		SW_L2_0_0_U1.out[4] --> delay 5ns --> SW_L1_0_0_U1.in[0];
		SW_L2_0_0_U1.out[5] --> delay 5ns --> SW_L1_0_0_U1.in[1];
		SW_L2_0_0_U1.out[6] --> delay 5ns --> SW_L1_0_1_U1.in[0];
		SW_L2_0_0_U1.out[7] --> delay 5ns --> SW_L1_0_1_U1.in[1];
		SW_L2_0_1_U1.out[0] --> delay 5ns --> H_5_U1.in;
		SW_L2_0_1_U1.out[1] --> delay 5ns --> H_6_U1.in;
		SW_L2_0_1_U1.out[2] --> delay 5ns --> H_7_U1.in;
		SW_L2_0_1_U1.out[3] --> delay 5ns --> H_8_U1.in;
		SW_L2_0_1_U1.out[4] --> delay 5ns --> SW_L1_0_0_U1.in[2];
		SW_L2_0_1_U1.out[5] --> delay 5ns --> SW_L1_0_0_U1.in[3];
		SW_L2_0_1_U1.out[6] --> delay 5ns --> SW_L1_0_1_U1.in[2];
		SW_L2_0_1_U1.out[7] --> delay 5ns --> SW_L1_0_1_U1.in[3];
		SW_L2_1_0_U1.out[0] --> delay 5ns --> H_9_U1.in;
		SW_L2_1_0_U1.out[1] --> delay 5ns --> H_10_U1.in;
		SW_L2_1_0_U1.out[2] --> delay 5ns --> H_11_U1.in;
		SW_L2_1_0_U1.out[3] --> delay 5ns --> H_12_U1.in;
		SW_L2_1_0_U1.out[4] --> delay 5ns --> SW_L1_1_0_U1.in[0];
		SW_L2_1_0_U1.out[5] --> delay 5ns --> SW_L1_1_0_U1.in[1];
		SW_L2_1_0_U1.out[6] --> delay 5ns --> SW_L1_1_1_U1.in[0];
		SW_L2_1_0_U1.out[7] --> delay 5ns --> SW_L1_1_1_U1.in[1];
		SW_L2_1_1_U1.out[0] --> delay 5ns --> H_13_U1.in;
		SW_L2_1_1_U1.out[1] --> delay 5ns --> H_14_U1.in;
		SW_L2_1_1_U1.out[2] --> delay 5ns --> H_15_U1.in;
		SW_L2_1_1_U1.out[3] --> delay 5ns --> H_16_U1.in;
		SW_L2_1_1_U1.out[4] --> delay 5ns --> SW_L1_1_0_U1.in[2];
		SW_L2_1_1_U1.out[5] --> delay 5ns --> SW_L1_1_0_U1.in[3];
		SW_L2_1_1_U1.out[6] --> delay 5ns --> SW_L1_1_1_U1.in[2];
		SW_L2_1_1_U1.out[7] --> delay 5ns --> SW_L1_1_1_U1.in[3];
		SW_L2_2_0_U1.out[0] --> delay 5ns --> H_17_U1.in;
		SW_L2_2_0_U1.out[1] --> delay 5ns --> H_18_U1.in;
		SW_L2_2_0_U1.out[2] --> delay 5ns --> H_19_U1.in;
		SW_L2_2_0_U1.out[3] --> delay 5ns --> H_20_U1.in;
		SW_L2_2_0_U1.out[4] --> delay 5ns --> SW_L1_2_0_U1.in[0];
		SW_L2_2_0_U1.out[5] --> delay 5ns --> SW_L1_2_0_U1.in[1];
		SW_L2_2_0_U1.out[6] --> delay 5ns --> SW_L1_2_1_U1.in[0];
		SW_L2_2_0_U1.out[7] --> delay 5ns --> SW_L1_2_1_U1.in[1];
		SW_L2_2_1_U1.out[0] --> delay 5ns --> H_21_U1.in;
		SW_L2_2_1_U1.out[1] --> delay 5ns --> H_22_U1.in;
		SW_L2_2_1_U1.out[2] --> delay 5ns --> H_23_U1.in;
		SW_L2_2_1_U1.out[3] --> delay 5ns --> H_24_U1.in;
		SW_L2_2_1_U1.out[4] --> delay 5ns --> SW_L1_2_0_U1.in[2];
		SW_L2_2_1_U1.out[5] --> delay 5ns --> SW_L1_2_0_U1.in[3];
		SW_L2_2_1_U1.out[6] --> delay 5ns --> SW_L1_2_1_U1.in[2];
		SW_L2_2_1_U1.out[7] --> delay 5ns --> SW_L1_2_1_U1.in[3];
		SW_L2_3_0_U1.out[0] --> delay 5ns --> H_25_U1.in;
		SW_L2_3_0_U1.out[1] --> delay 5ns --> H_26_U1.in;
		SW_L2_3_0_U1.out[2] --> delay 5ns --> H_27_U1.in;
		SW_L2_3_0_U1.out[3] --> delay 5ns --> H_28_U1.in;
		SW_L2_3_0_U1.out[4] --> delay 5ns --> SW_L1_3_0_U1.in[0];
		SW_L2_3_0_U1.out[5] --> delay 5ns --> SW_L1_3_0_U1.in[1];
		SW_L2_3_0_U1.out[6] --> delay 5ns --> SW_L1_3_1_U1.in[0];
		SW_L2_3_0_U1.out[7] --> delay 5ns --> SW_L1_3_1_U1.in[1];
		SW_L2_3_1_U1.out[0] --> delay 5ns --> H_29_U1.in;
		SW_L2_3_1_U1.out[1] --> delay 5ns --> H_30_U1.in;
		SW_L2_3_1_U1.out[2] --> delay 5ns --> H_31_U1.in;
		SW_L2_3_1_U1.out[3] --> delay 5ns --> H_32_U1.in;
		SW_L2_3_1_U1.out[4] --> delay 5ns --> SW_L1_3_0_U1.in[2];
		SW_L2_3_1_U1.out[5] --> delay 5ns --> SW_L1_3_0_U1.in[3];
		SW_L2_3_1_U1.out[6] --> delay 5ns --> SW_L1_3_1_U1.in[2];
		SW_L2_3_1_U1.out[7] --> delay 5ns --> SW_L1_3_1_U1.in[3];
endmodule

network FABRIC : is1_3l_32n
endnetwork
