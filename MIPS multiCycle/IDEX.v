module IDEX(AluSrcIDEX, RegWriteIDEX, MemReadIDEX, MemWriteIDEX, RegDstIDEX, MemToRegIDEX, AluOpIDEX,
            RtIDEX, RsIDEX, RdIDEX, adderPcOutIDEX, readData1IDEX, readData2IDEX, signExtendIDEX, clk,
            rst, AluSrc, RegWrite, MemRead, MemWrite, RegDst, MemToReg, AluOp, Rt, Rs, Rd, adderPcOut,
            readData1, readData2, signExtend);
    output reg AluSrcIDEX, RegWriteIDEX, MemReadIDEX, MemWriteIDEX;
    output reg [1:0] RegDstIDEX, MemToRegIDEX;
    output reg [2:0] AluOpIDEX;
    output reg [4:0] RtIDEX, RsIDEX, RdIDEX;
    output reg [31:0] adderPcOutIDEX, readData1IDEX, readData2IDEX, signExtendIDEX;

    input clk, rst, AluSrc, RegWrite, MemRead, MemWrite;
    input [1:0] RegDst, MemToReg;
    input [2:0] AluOp;
    input [4:0] Rt, Rs, Rd;
    input [31:0] adderPcOut, readData1, readData2, signExtend;

    always @(posedge clk) begin
        if(rst) begin
            {MemToRegIDEX, MemWriteIDEX, MemReadIDEX, RegDstIDEX, RegWriteIDEX, AluSrcIDEX, AluOpIDEX} <= 11'd0;
            {adderPcOutIDEX, signExtendIDEX, readData1IDEX, readData2IDEX, RtIDEX, RsIDEX, RdIDEX} <= 143'd0; 
        end     
        else begin
            {MemToRegIDEX, MemWriteIDEX, MemReadIDEX, RegDstIDEX, RegWriteIDEX, AluSrcIDEX, AluOpIDEX} <= {MemToReg, MemWrite, MemRead, RegDst, RegWrite, AluSrc, AluOp};
            {adderPcOutIDEX, signExtendIDEX, readData1IDEX, readData2IDEX, RtIDEX, RsIDEX, RdIDEX} <= {adderPcOut, signExtend, readData1, readData2, Rt, Rs, Rd};
        end
    end
endmodule