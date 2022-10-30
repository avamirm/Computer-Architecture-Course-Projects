module PipeLine(instMemAddress, dataMemAddress, dataMemWriteData, dataMemWrite, dataMemRead, instruction, dataMemReadData,
                clk, rst);

    output [31:0] instMemAddress, dataMemAddress, dataMemWriteData;
    output dataMemWrite, dataMemRead;
    input [31:0] instruction, dataMemReadData;
    input clk, rst;

    wire [31:0] IFIDInstructionOut;
    wire [4:0] RtIDEX, RsIDEX, RdEXMEM, RdMEMWB;
    wire [2:0] AluOp;
    wire [1:0] RegDst, PCSrc, MemToReg, forwardA, forwardB;
    wire MemReadIDEXOut, RegWriteEXMEM, RegWriteMEMWB;
    wire HazardSel, ALUSrc, RegWrite, PcLoad, IFIDLoad, Flush, MemWrite, MemRead;
    wire equal;

    DataPath Dp(instMemAddress, dataMemAddress, dataMemWriteData, IFIDInstructionOut, RtIDEX, RsIDEX, RdEXMEM, RdMEMWB,
                equal, MemReadIDEXOut, dataMemWrite, dataMemRead, RegWriteEXMEM, RegWriteMEMWB, instruction,
                dataMemReadData, AluOp, RegDst, PCSrc, MemToReg, forwardA, forwardB, HazardSel, ALUSrc,
                RegWrite, clk, rst, MemWrite, MemRead, PcLoad, IFIDLoad, Flush);
    
    Controller Cntrl(MemToReg, PCSrc, RegDst, AluOp, ALUSrc, RegWrite, MemWrite, MemRead, Flush, IFIDInstructionOut[31:26],
                     IFIDInstructionOut[5:0], equal);
    
     HazardDetectionUnit HDUnit(HazardSel, IFIDLoad, PcLoad, RtIDEX, IFIDInstructionOut[25:21], IFIDInstructionOut[20:16], MemReadIDEXOut);

     ForwardingUnit FDUnit(forwardA, forwardB, RsIDEX, RtIDEX, RdEXMEM, RdMEMWB, RegWriteEXMEM, RegWriteMEMWB);
endmodule