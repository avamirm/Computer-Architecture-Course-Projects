module DataMem(max, maxIndex, readData, writeData, address, MemWrite, MemRead, clk);
    output [31:0] max, maxIndex, readData;
    input [31:0] writeData, address;
    input MemWrite, MemRead, clk;

    reg [7:0] mem [0:65535];

    initial begin
        $readmemb("numbers.txt", mem);
    end

    assign max = {mem[2003], mem[2002], mem[2001], mem[2000]};
    assign maxIndex = {mem[2007], mem[2006], mem[2005], mem[2004]};
    assign readData = MemRead ? {mem[address + 3], mem[address + 2], mem[address + 1], mem[address]} : 32'd0;

    always@(posedge clk)begin
        if(MemWrite)
            {mem[address + 3], mem[address + 2], mem[address + 1], mem[address]} <= writeData;
    end

endmodule