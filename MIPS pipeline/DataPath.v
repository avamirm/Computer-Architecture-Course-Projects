module Register # (parameter bit)(out, in, rst, load, clk);
    input [bit - 1:0] in;
    input rst, load, clk;
    output reg [bit - 1:0] out;
    always@(posedge clk, posedge rst) begin
        if(rst)
            out <= {bit{1'b0}};
        else if(load)
            out <= in; 
    end
endmodule

module PCRegister # (parameter bit)(out, in, rst, load, clk);
    input [bit - 1:0] in;
    input rst, load, clk;
    output reg [bit - 1:0] out;
    always@(posedge clk, posedge rst) begin
        if(rst)
            out <= 12'd220;
        else if(load)
            out <= in; 
    end
endmodule

module Mux2To1 # (parameter bit)(out, in1, in2, sel);
    input [bit - 1:0] in1, in2;
    input sel;
    output [bit - 1:0] out;
    assign out = (sel == 1'b0) ? in1 : in2;
endmodule

module Mux3To1 # (parameter bit)(out, in1, in2, in3, sel);
    input [bit - 1:0] in1, in2, in3;
    input [1:0] sel;
    output [bit - 1:0] out;
    assign out = (sel == 2'b00) ? in1:
                 (sel == 2'b01) ? in2:
                 (sel == 2'b10) ? in3:
                 in3 ;
endmodule 

module ALU(out, zero, in1, in2, sel);
    input [15:0] in1, in2;
    input [2:0] sel;
    output [15:0] out;
    output zero;
    assign out = (sel == 3'b000) ? (in1 & in2):
                 (sel == 3'b001) ? (in1 | in2):
                 (sel == 3'b010) ? (in1 + in2):
                 (sel == 3'b011) ? (in1 - in2):
                 (sel == 3'b110) ? (~in1):
                 (sel == 3'b101) ? (in1):
                 (sel == 3'b100) ? (in2): (16'd0);
    assign zero = (out == 16'd0) ? 1'b1 : 1'b0;
endmodule

module RegFile(readData1, readData2, writeData, readReg1, readReg2, writeReg, regWrite, rst, clk);
    input [15:0] writeData;
    input [2:0] readReg1, readReg2, writeReg;
    input clk, rst, regWrite;
    output [15:0] readData1, readData2;
    reg [15:0] registerFile [0:7];
    integer i;

    always@(posedge clk, posedge rst)begin
        if(rst)
            for(i = 0; i < 8; i = i + 1) 
                registerFile[i] = 16'd0;
        else if(regWrite)
            registerFile[writeReg] <= writeData;
    end

    assign readData1 = registerFile[readReg1];
    assign readData2 = registerFile[readReg2];
endmodule

module DataPath(memAddress, memWriteData, IROut, zero, memReadData, ALUOperation,
                PCSrc, ALUSrcB, PCLoad, IOrD, RegDst, MemToReg, IRWrite, ALUSrcA, RegWrite, clk, rst); 
    output [11:0] memAddress;
    output [15:0] memWriteData, IROut;
    output zero;
    input [15:0] memReadData;
    input [2:0] ALUOperation;
    input [1:0] PCSrc, ALUSrcB;
    input PCLoad, IOrD, RegDst, MemToReg, IRWrite, ALUSrcA, RegWrite, clk, rst;

    wire [11:0] pcOut;
    wire [11:0] muxPcSrcOut;
    PCRegister #(12) PC(pcOut, muxPcSrcOut, rst, PCLoad, clk); 

    wire [11:0] muxIOrDOut;
    Mux2To1 #(12) MuxIOrD(muxIOrDOut, pcOut, IROut[11:0], IOrD);

    Register #(16) IR(IROut, memReadData, rst, IRWrite, clk);

    wire [15:0] MDROut;
    Register #(16) MDR(MDROut, memReadData, rst, 1'b1, clk);

    wire [2:0] muxRegDstOut;
    Mux2To1 #(3) MuxRegDst(muxRegDstOut, 3'd0, IROut[11:9], RegDst);

    wire [15:0] ALURegOut;
    wire [15:0] muxMemToRegOut;
    Mux2To1 #(16) MuxMemToReg(muxMemToRegOut, ALURegOut, MDROut, MemToReg);
    
    wire [15:0] readData1, readData2;
    RegFile RegisterFile(readData1, readData2, muxMemToRegOut, 3'd0, IROut[11:9], muxRegDstOut, RegWrite, rst, clk);

    wire [15:0] signExtend;
    assign signExtend = {{4{IROut[11]}}, IROut[11:0]};

    wire [15:0] RegOutA, RegOutB;
    Register #(16) RegA(RegOutA, readData1, rst, 1'b1, clk);
    Register #(16) RegB(RegOutB, readData2, rst, 1'b1, clk);

    wire [15:0] AlUSrcOutA, AlUSrcOutB;
    wire [15:0] extendedPC;
    assign extendedPC = {4'd0, pcOut};
    Mux2To1 #(16) MuxALUSrcA(AlUSrcOutA, extendedPC, RegOutA, ALUSrcA);
    Mux3To1 #(16) MuxALUSrcB(AlUSrcOutB, RegOutB, 16'd1, signExtend, ALUSrcB);

    wire [15:0] ALUResult;
    ALU Alu(ALUResult, zero, AlUSrcOutA, AlUSrcOutB, ALUOperation);

    Register #(16) RegALU(ALURegOut, ALUResult, rst, 1'b1, clk);

    wire [11:0] extendedBranch;
    assign extendedBranch = {pcOut[11:9], IROut[8:0]};
    Mux3To1 #(12) MuxPCSrc(muxPcSrcOut, ALUResult[11:0], extendedBranch, IROut[11:0], PCSrc);

    assign memAddress= muxIOrDOut;
    assign memWriteData = RegOutA;

endmodule

