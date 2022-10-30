`timescale 1ns/1ns
module TestBench();
    wire [31:0] instMemAddress, dataMemAddress, dataMemWriteData, instruction, dataMemReadData, max, maxIndex;
    wire MemRead,MemWrite;
    reg clk = 1'b0;
    reg rst = 1'b1;
    InstructionMem InstMem(instruction, instMemAddress);
    DataMem DataMemory(max, maxIndex, dataMemReadData, dataMemWriteData, dataMemAddress, MemWrite, MemRead, clk);
    MipsSingleCycle Mips(instMemAddress, dataMemAddress, dataMemWriteData, MemRead, MemWrite, 
                         instruction, dataMemReadData, rst, clk);
    always begin
        #10 clk = ~clk;
    end
    initial begin
        #50 rst = 1'b0;
        #4000 $stop;
    end
    initial begin
        #4000
        $display ("max number is %d", max);
        $display ("max number's index is %d", maxIndex);
    end
endmodule