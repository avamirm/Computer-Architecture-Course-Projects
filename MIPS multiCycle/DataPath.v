module Register # (parameter bit)(out, in, rst, load, clk);
    input [31:0] in;
    input rst, load, clk;
    output reg [31:0] out;
    always@(posedge clk) begin
        if(rst)
            out <= {bit{1'b0}};
        else if(load)
            out <= in; 
    end
endmodule

module Adder(out, in1, in2);
    input [31:0] in1, in2;
    output [31:0] out;
    assign out = in1 + in2;
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

module Mux4To1 # (parameter bit)(out, in1, in2, in3, in4, sel);
    input [bit - 1:0] in1, in2, in3, in4;
    input [1:0] sel;
    output [bit - 1:0] out;
    assign out = (sel == 2'b00) ? in1:
                 (sel == 2'b01) ? in2:
                 (sel == 2'b10) ? in3:
                 (sel == 2'b11) ? in4:
                 in4 ;
endmodule 

module ALU(out, in1, in2, sel);
    input [31:0] in1, in2;
    input [2:0] sel;
    output [31:0] out;
    wire [31:0] subSLT;
    assign out = (sel == 3'b000) ? (in1 & in2):
                 (sel == 3'b001) ? (in1 | in2):
                 (sel == 3'b010) ? (in1 + in2):
                 (sel == 3'b011) ? (in1 - in2):
                 ((subSLT[31]) ? 32'd1 : 32'd0);
    assign subSLT = in2 - in1;
endmodule

module RegFile(readData1, readData2, writeData, readReg1, readReg2, writeReg, regWrite, rst, clk);
    input [31:0] writeData;
    input [4:0] readReg1, readReg2, writeReg;
    input clk, rst, regWrite;
    output [31:0] readData1, readData2;
    reg [31:0] registerFile [0:31];
    integer i;

    always@(posedge clk)begin
        if(rst)
            for(i = 0; i < 32; i = i + 1) 
                registerFile[i] <= 32'd0;
        else if(regWrite)
            if(writeReg != 5'd0)
                registerFile[writeReg] <= writeData;
    end

    assign readData1 = (readReg1 != 5'd0) ? registerFile[readReg1] : 32'd0; 
    assign readData2 = (readReg2 != 5'd0) ? registerFile[readReg2] : 32'd0; 
endmodule

module IFIDReg(instructionOut, out, instruction, in, load, Flush, clk, rst);
    output reg [31:0] instructionOut, out;
    input [31:0] instruction, in;
    input load, clk, rst, Flush;

    always@(posedge clk) begin
        if(Flush)
            instructionOut <= 32'd0;
        else if(rst) begin
            out <= 32'd0;
            instructionOut <= 32'd0;          
        end
        else if(load) begin
            out <= in;
            instructionOut <= instruction;
        end
    end
endmodule


module DataPath(instMemAddress, dataMemAddress, dataMemWriteData, IFIDInstructionOut, RtIDEX, RsIDEX, RdEXMEM, RdMEMWB,
                equal, MemReadIDEXOut, dataMemWrite, dataMemRead, RegWriteEXMEM, RegWriteMEMWB, instruction,
                dataMemReadData, AluOp, RegDst, PCSrc, MemToReg, forwardA, forwardB, HazardSel, ALUSrc,
                RegWrite, clk, rst, MemWrite, MemRead, PcLoad, IFIDLoad, Flush);

    output [31:0] instMemAddress, dataMemAddress, dataMemWriteData;
    output [31:0] IFIDInstructionOut;
    output [4:0] RtIDEX, RsIDEX;
    output [4:0] RdEXMEM;
    output [4:0] RdMEMWB;
    output equal;
    output MemReadIDEXOut;
    output dataMemWrite, dataMemRead;
    output RegWriteEXMEM, RegWriteMEMWB;
    input [31:0] instruction, dataMemReadData;
    input [2:0] AluOp;
    input [1:0] RegDst, PCSrc, MemToReg;
    input [1:0] forwardA, forwardB;
    input HazardSel, ALUSrc, RegWrite, clk, rst;
    input MemWrite, MemRead;
    input PcLoad, IFIDLoad;
    input Flush;

    wire [31:0] pcOut;
    wire [31:0] muxPcSrcOut;
    Register #(32) PcReg(pcOut, muxPcSrcOut, rst, PcLoad, clk);

    wire [31:0] adderPcOut;
    Adder AdderPc(adderPcOut, 32'd4, pcOut);

    wire [31:0] IFIDAdderOut;
    IFIDReg IFID(IFIDInstructionOut, IFIDAdderOut, instruction, adderPcOut, IFIDLoad, Flush, clk, rst);
    
    wire [27:0] shiftJOut;
    wire [31:0] jOut;
    assign shiftJOut = {IFIDInstructionOut [25:0] ,2'b00};
    assign jOut = {IFIDAdderOut [31:28], shiftJOut};

    wire [31:0] shift2Beq;
    wire [31:0] signExtended;
    assign signExtended = {{16{IFIDInstructionOut[15]}}, IFIDInstructionOut[15:0]};
    assign shift2Beq = signExtended << 2;
    wire [31:0] adderBeqOut;
    wire [31:0] readData1, readData2;
    
    Mux4To1 #(32) MuxPcSrc(muxPcSrcOut, readData1, jOut, adderBeqOut, adderPcOut, PCSrc);

    Adder BeqAdder(adderBeqOut, IFIDAdderOut, shift2Beq);

    wire [31:0] muxMemToRegOut;
    wire [4:0] writeRegMEMWB;

    RegFile RG(readData1, readData2, muxMemToRegOut, IFIDInstructionOut[25:21], IFIDInstructionOut[20:16], writeRegMEMWB, RegWriteMEMWB, rst, clk);
    
    assign equal = (readData1 == readData2);

    wire [31:0] readData1IDEX, readData2IDEX, signExtendIDEX;
    wire [31:0] adderPcOutIDEX;
    wire [4:0] RdIDEX;
    
    wire [2:0] AluOpIDEX;
    wire [1:0] RegDstIDEX, MemToRegIDEX;
    wire AluSrcIDEX, RegWriteIDEX, MemReadIDEX, MemWriteIDEX;
    
    wire [10:0] muxHazardOut;
    wire [10:0] hazard;
    assign hazard = {MemToReg/*10-9*/, MemWrite/*8*/, MemRead/*7*/, RegDst/*6-5*/, RegWrite/*4*/, ALUSrc/*3*/, AluOp/*2-0*/};
    Mux2To1 #(11) MuxHazard(muxHazardOut, 11'd0, hazard, HazardSel);
    IDEX IDEXreg(AluSrcIDEX, RegWriteIDEX, MemReadIDEX, MemWriteIDEX, RegDstIDEX, MemToRegIDEX, AluOpIDEX,
            RtIDEX, RsIDEX, RdIDEX, adderPcOutIDEX, readData1IDEX, readData2IDEX, signExtendIDEX, clk,
            rst, muxHazardOut[3], muxHazardOut[4], muxHazardOut[7], muxHazardOut[8], muxHazardOut[6:5],
            muxHazardOut[10:9], muxHazardOut[2:0], IFIDInstructionOut[20:16],
            IFIDInstructionOut[25:21], IFIDInstructionOut[15:11], adderPcOut, readData1, readData2, signExtended);

    assign MemReadIDEXOut = MemReadIDEX;

    wire [4:0] muxRegDstOut;
    Mux3To1 #(5) MuxRegDst(muxRegDstOut, RtIDEX, RdIDEX, 5'd31, RegDstIDEX);

    wire [31:0] muxBOut, muxAluSrcOut;
    Mux2To1 #(32) MuxAluSrc(muxAluSrcOut, muxBOut, signExtendIDEX, AluSrcIDEX);

    wire [31:0] muxAOut;
    wire [31:0] aluResultEXMEM;

    Mux3To1 #(32) MuxForwardA(muxAOut, readData1IDEX, muxMemToRegOut, aluResultEXMEM, forwardA);

    Mux3To1 #(32) MuxForwardB(muxBOut, readData2IDEX, muxMemToRegOut, aluResultEXMEM, forwardB);

    wire [31:0] aluResult;
    ALU Alu(aluResult, muxAluSrcOut, muxAOut, AluOpIDEX);

    wire [31:0] adderPcOutEXMEM;
    wire [31:0] muxBEXMEM;
    wire [4:0] RegDstEXMEM;
    wire [1:0] MemToRegEXMEM;
    wire MemReadEXMEM, MemWriteEXMEM;
    EXMEM EXMEMReg(RegWriteEXMEM, MemReadEXMEM, MemWriteEXMEM, MemToRegEXMEM, RegDstEXMEM, adderPcOutEXMEM,
             aluResultEXMEM, muxBEXMEM, clk, rst, RegWriteIDEX, MemReadIDEX, MemWriteIDEX, MemToRegIDEX, muxRegDstOut, adderPcOutIDEX,
             aluResult, muxBOut);

    assign dataMemRead = MemReadEXMEM;
    assign dataMemWrite = MemWriteEXMEM;
    assign RdEXMEM = RegDstEXMEM;

    wire [1:0] MemToRegMEMWB;

    wire [31:0] memoryDataMEMWB;
    wire [31:0] aluResultMEMWB;
    wire [31:0] adderPcOutMEMWB;

    MEMWB MEMWBReg(RegWriteMEMWB, MemToRegMEMWB, writeRegMEMWB, memoryDataMEMWB, aluResultMEMWB, adderPcOutMEMWB, clk, rst,
             RegWriteEXMEM, MemToRegEXMEM, RegDstEXMEM, dataMemReadData, aluResultEXMEM, adderPcOutEXMEM);

    assign RdMEMWB = writeRegMEMWB;

    Mux3To1 #(32) MuxMemToReg(muxMemToRegOut, aluResultMEMWB, memoryDataMEMWB, adderPcOutMEMWB, MemToRegMEMWB);

    assign instMemAddress = pcOut;
    assign dataMemAddress = aluResultEXMEM;
    assign dataMemWriteData = muxBEXMEM;

endmodule