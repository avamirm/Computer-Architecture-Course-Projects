module TestBench();
    wire [15:0] max, maxIndex, memReadData, memWriteData;
    wire [11:0] memAddress; 
    wire MemRead,MemWrite;
    reg clk = 1'b0;
    reg rst = 1'b0;

    DataMem Memory(max, maxIndex, memReadData, memWriteData, memAddress, MemWrite, MemRead, clk);
    MultiCycle Mc(memAddress, memWriteData, MemRead, MemWrite, memReadData, rst, clk);

    always begin
        #5 clk = ~clk;
    end
    initial begin
        #1 rst = 1'b1;
        #3 rst = 1'b0;
        #7000 $stop;
    end
    initial begin
        #7000
        $display ("max number is %d", max);
        $display ("max number's index is %d", maxIndex);
    end
endmodule