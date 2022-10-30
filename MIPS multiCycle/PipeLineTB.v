`timescale 1ns/1ns
module PipeLineTB();
    wire [31:0] instMemAddress, instruction, dataMemAddress, dataMemWriteData, dataMemReadData;
    wire [31:0] max, maxIndex;
    wire dataMemRead, dataMemWrite;
    reg clk, rst;
    
    PipeLine PL(instMemAddress, dataMemAddress, dataMemWriteData, dataMemWrite, dataMemRead, instruction, dataMemReadData,
                clk, rst);

    DataMem DM(max, maxIndex, dataMemReadData, dataMemWriteData, dataMemAddress, dataMemWrite, dataMemRead, clk);
    
    InstructionMem IM(instruction, instMemAddress);
    
    initial
    begin
        rst     = 1'b1;
        clk     = 1'b0;
        #50 rst = 1'b0;
        #10000 $stop;
    end
    
    always
    begin
        #10 clk = ~clk;
    end

endmodule