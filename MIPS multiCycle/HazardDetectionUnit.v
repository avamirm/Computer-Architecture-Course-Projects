module HazardDetectionUnit(HazardSel, IFIDLoad, PcLoad, RtIDEX, RsIFID, RtIFID, MemReadIDEX);
    output reg HazardSel, IFIDLoad, PcLoad;
    input [4:0] RtIDEX, RsIFID, RtIFID;
    input MemReadIDEX;

    always @(MemReadIDEX, RtIDEX, RsIFID, RtIFID) begin
        {HazardSel, IFIDLoad, PcLoad} <= 3'd7;

        if (MemReadIDEX && ((RtIDEX == RsIFID) || (RtIDEX == RtIFID))) begin
            {HazardSel, IFIDLoad, PcLoad} <= 3'd0;
        end
    end
endmodule