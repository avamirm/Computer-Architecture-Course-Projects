module DataMem(max, maxIndex, readData, writeData, address, MemWrite, MemRead, clk);
    output [15:0] max, maxIndex;
    output reg [15:0] readData;
    input [15:0] writeData;
    input [11:0] address;
    input MemWrite, MemRead, clk;

    reg [15:0] mem [0:4095];

    initial begin
        $readmemb("data.txt", mem);
    end

    assign max = mem[200];
    assign maxIndex = mem[204];

    always @(posedge clk) begin
        if (MemWrite)
            mem[address] <= writeData;
    end

    always @(MemRead or address) begin
        if (MemRead)
            readData = mem[address];
    end
endmodule