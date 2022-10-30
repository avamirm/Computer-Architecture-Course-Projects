module ForwardingUnit(forwardA, forwardB, RsIDEX, RtIDEX, RdEXMEM, RdMEMWB, RegWriteEXMEM, RegWriteMEMWB);
    output reg [1:0] forwardA, forwardB;
    input [4:0] RsIDEX, RtIDEX, RdEXMEM, RdMEMWB;
    input RegWriteEXMEM, RegWriteMEMWB;

    always @(RsIDEX, RtIDEX, RegWriteEXMEM, RdEXMEM, RegWriteMEMWB, RdMEMWB) begin
        forwardA <= 2'b00;
        forwardB <= 2'b00;
        if ((RegWriteEXMEM == 1'b1) && (RdEXMEM != 5'b00000) && (RdEXMEM == RsIDEX))
            forwardA <= 2'b10;
        
        if ((RegWriteEXMEM == 1'b1) && (RdEXMEM != 5'b00000) && (RdEXMEM == RtIDEX))
            forwardB <= 2'b10;
        
        if ((RegWriteMEMWB == 1'b1) && (RdMEMWB != 5'b00000) && (!((RegWriteEXMEM == 1'b1) && (RdEXMEM != 5'b00000) && (RdEXMEM == RsIDEX))) 
            && (RdMEMWB == RsIDEX))
            forwardA <= 2'b01;
        
        if ((RegWriteMEMWB == 1'b1) && (RdMEMWB != 5'b00000) && !((RegWriteEXMEM == 1'b1) && (RdEXMEM != 5'b00000) && (RdEXMEM == RtIDEX)) 
            && (RdMEMWB == RtIDEX))
            forwardB <= 2'b01;
        
    end

endmodule