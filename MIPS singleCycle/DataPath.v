module Reg32Bit(out, in, srst, load, clk);
    input [31:0] in;
    input srst, load, clk;
    output reg [31:0] out;
    always@(posedge clk) begin
        if(srst)
            out <= 32'd0;
        else if(load)
            out <= in; 
    end
endmodule

module Adder32Bit(out, in1, in2);
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

module ALU(out, zero, in1, in2, sel);
    input [31:0] in1, in2;
    input [2:0] sel;
    output zero;
    output [31:0] out;
    wire [31:0] subSLT;
    assign out = (sel == 3'b000) ? (in1 & in2):
                 (sel == 3'b001) ? (in1 | in2):
                 (sel == 3'b010) ? (in1 + in2):
                 (sel == 3'b011) ? (in1 - in2):
                 ((subSLT[31]) ? 32'd1 : 32'd0);
    assign subSLT = in1 - in2;
    assign zero = (out == 32'd0) ? 1'b1 : 1'b0;
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
                registerFile[i] = 32'd0;
        else if(regWrite)
            if(writeReg != 5'd0)
                registerFile[writeReg] <= writeData;
    end

    assign readData1 = (readReg1 != 5'd0) ? registerFile[readReg1] : 32'd0; 
    assign readData2 = (readReg2 != 5'd0) ? registerFile[readReg2] : 32'd0; 
endmodule

module DataPath(instMemAddress, dataMemAddress, dataMemWriteData, zero, instruction, dataMemReadData, ALUOperation,
                RegDst, DataToWrite, MemToReg, ALUSrc, PCSrc, RegWrite, Jr, J, clk, rst);
    output [31:0] instMemAddress, dataMemAddress, dataMemWriteData;
    output zero;
    input [31:0] instruction, dataMemReadData;
    input [2:0] ALUOperation;
    input [1:0] RegDst, DataToWrite;
    input MemToReg, ALUSrc, PCSrc, RegWrite, Jr, J, clk, rst;

    wire [31:0] pcOut;
    wire [31:0] muxJOut;
    Reg32Bit PC(pcOut, muxJOut, rst, 1'b1, clk);

    wire [31:0] adderPcOut;
    Adder32Bit AdderPc(adderPcOut, pcOut, 32'd4);

    wire [4:0] muxRegDstOut;
    Mux3To1 #(5) MuxRegDst(muxRegDstOut, instruction[20:16], instruction[15:11], 5'd31, RegDst);

    wire [31:0] muxDataToWriteOut;
    wire [31:0] ALUResult;
    wire [31:0] muxMemToRegOut;
    Mux3To1 #(32) MuxDataToWrite(muxDataToWriteOut, muxMemToRegOut, adderPcOut, ALUResult, DataToWrite);
    
    wire [31:0] readData1, readData2;
    RegFile RegisterFile(readData1, readData2, muxDataToWriteOut, instruction[25:21], instruction[20:16], muxRegDstOut, RegWrite, rst, clk);

    wire [31:0] signExtend;
    assign signExtend = {{16{instruction[15]}}, instruction[15:0]};

    wire [31:0] AlUSrcOut;
    Mux2To1 #(32) MuxALUSrc(AlUSrcOut, readData2, signExtend, ALUSrc);

    ALU Alu(ALUResult, zero, readData1, AlUSrcOut, ALUOperation);

    wire [31:0] shift2Beq;
    wire [31:0] adderBeqOut;
    assign shift2Beq = signExtend << 2;
    Adder32Bit AdderBeq(adderBeqOut, adderPcOut, shift2Beq);

    wire [31:0] muxPCSrcOut;
    Mux2To1 #(32) MuxPCSrc(muxPCSrcOut, adderPcOut, adderBeqOut, PCSrc);

    wire [27:0] shiftJOut;
    wire [31:0] jOut;
    assign shiftJOut = {instruction[25:0], 2'b00};
    assign jOut = {adderPcOut[31:28], shiftJOut};

    wire [31:0] muxJrOut;
    Mux2To1 #(32) MuxJr(muxJrOut, jOut, readData1, Jr);

    Mux2To1 #(32) MuxJ(muxJOut, muxPCSrcOut, muxJrOut, J);

    Mux2To1 #(32) MuxMemToReg(muxMemToRegOut, ALUResult, dataMemReadData, MemToReg);

    assign instMemAddress= pcOut;
    assign dataMemAddress = ALUResult;
    assign dataMemWriteData = readData2;

endmodule
