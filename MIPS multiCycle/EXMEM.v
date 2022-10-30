module EXMEM(RegWriteEXMEM, MemReadEXMEM, MemWriteEXMEM, MemToRegEXMEM, muxRegDstEXMEM, adderPcOutEXMEM,
             aluResultEXMEM, muxBEXMEM, clk, rst, RegWrite, MemRead, MemWrite, MemToReg, muxRegDst, adderPcOut,
             aluResult, muxB);
    output reg RegWriteEXMEM;
    output reg MemReadEXMEM, MemWriteEXMEM;
    output reg [1:0] MemToRegEXMEM;
    output reg [4:0] muxRegDstEXMEM;
    output reg [31:0] adderPcOutEXMEM;
    output reg [31:0] aluResultEXMEM, muxBEXMEM;

    input clk, rst, RegWrite, MemRead, MemWrite;
    input [1:0] MemToReg;
    input [4:0] muxRegDst;
    input [31:0] adderPcOut;
    input [31:0] aluResult, muxB;

    always @(posedge clk) begin
        if(rst) begin
            {RegWriteEXMEM, MemToRegEXMEM, MemReadEXMEM, MemWriteEXMEM} <= 5'd0;
            {muxRegDstEXMEM, muxBEXMEM, aluResultEXMEM, adderPcOutEXMEM} <= 101'd0;
        end
        else begin
            {RegWriteEXMEM, MemToRegEXMEM, MemReadEXMEM, MemWriteEXMEM} <= {RegWrite, MemToReg, MemRead, MemWrite};
            {muxRegDstEXMEM, muxBEXMEM, aluResultEXMEM, adderPcOutEXMEM} <= {muxRegDst, muxB, aluResult, adderPcOut};
        end
    end
endmodule