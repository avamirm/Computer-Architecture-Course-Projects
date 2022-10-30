module MultiCycle(memAddress, memWriteData,MemRead, MemWrite, memReadData, rst, clk);
    output [11:0] memAddress;
    output [15:0] memWriteData;
    output MemRead, MemWrite;
    input [15:0] memReadData;
    input rst, clk;

    wire [15:0] IROut;
    wire [2:0] ALUoperation;
    wire [1:0] PCSrc, ALUSrcB;
    wire PCLoad, IOrD, RegDst, MemToReg, IRWrite, ALUSrcA, RegWrite, zero;

    DataPath Dp(memAddress, memWriteData, IROut, zero, memReadData, ALUoperation,
                PCSrc, ALUSrcB, PCLoad, IOrD, RegDst, MemToReg, IRWrite, ALUSrcA, RegWrite, clk, rst);
    Controller Ctrl(ALUoperation, PCSrc, ALUSrcB, PCLoad, IOrD, RegDst, RegWrite, MemToReg, IRWrite, ALUSrcA,
                  MemRead, MemWrite, IROut[8:0], IROut[15:12], zero, rst, clk);
endmodule