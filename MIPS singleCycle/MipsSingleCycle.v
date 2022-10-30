module MipsSingleCycle(instMemAddress, dataMemAddress, dataMemWriteData, MemRead, MemWrite, 
                       instruction, dataMemReadData, rst, clk);
    output [31:0] instMemAddress, dataMemAddress, dataMemWriteData;
    output MemRead, MemWrite;
    input [31:0] instruction, dataMemReadData;
    input rst, clk;

    wire [2:0] ALUoperation;
    wire [1:0] RegDst, DataToWrite;
    wire MemToReg, ALUSrc, PCSrc, RegWrite, Jr, J, zero;
    
    DataPath Dp(instMemAddress, dataMemAddress, dataMemWriteData, zero, instruction, dataMemReadData, ALUoperation,
                RegDst, DataToWrite, MemToReg, ALUSrc, PCSrc, RegWrite, Jr, J, clk, rst);
    Controller Ctrl(RegDst, DataToWrite, ALUoperation, MemToReg, ALUSrc, RegWrite, Jr, J, MemWrite, MemRead,
        PCSrc, instruction[31:26], instruction[5:0], zero);
endmodule