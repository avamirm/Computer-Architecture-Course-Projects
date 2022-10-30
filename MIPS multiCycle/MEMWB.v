module MEMWB(RegWriteMEMWB, MemToRegMEMWB, RegDstMEMWB, memoryDataMEMWB, aluResultMEMWB, adderPcOutMEMWB, clk, rst,
             RegWrite, MemToReg, RegDst, memoryData, aluResult, adderPcOut);
    output reg RegWriteMEMWB;
    output reg [1:0] MemToRegMEMWB;
    output reg [4:0] RegDstMEMWB;
    output reg [31:0] memoryDataMEMWB;
    output reg [31:0] aluResultMEMWB;
    output reg [31:0] adderPcOutMEMWB;

    input clk, rst;
    input RegWrite;
    input [1:0] MemToReg;
    input [4:0] RegDst;
    input [31:0] memoryData;
    input [31:0] aluResult;
    input [31:0] adderPcOut;

     always @(posedge clk) begin
        if (rst) begin
            {RegWriteMEMWB, MemToRegMEMWB} <= 3'd0;
            {adderPcOutMEMWB, RegDstMEMWB, aluResultMEMWB, memoryDataMEMWB} <= 101'd0;
        end
        else begin
            {RegWriteMEMWB, MemToRegMEMWB} <= {RegWrite, MemToReg};
            {adderPcOutMEMWB, RegDstMEMWB, aluResultMEMWB, memoryDataMEMWB} <= {adderPcOut, RegDst, aluResult, memoryData};
        end
     end
endmodule