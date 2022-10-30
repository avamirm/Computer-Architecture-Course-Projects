module AluController(ALUoperation, ALUop, func);
    output reg [2:0] ALUoperation;
    input [1:0] ALUop;
    input [5:0] func;
    parameter[5:0] And = 6'b100100, Or = 6'b100101, Add =  6'b100000, Sub = 6'b100010,  Slt = 6'b101010;
    always@(ALUop, func) begin
        ALUoperation = 3'b010;
        if(ALUop == 2'd0) // lw or sw
            ALUoperation = 3'b010; 
        else if(ALUop == 2'd1) // beq
            ALUoperation = 3'b011;
        else if(ALUop == 2'd2) begin
            case(func)
                And: ALUoperation = 3'b000;
                Or:  ALUoperation = 3'b001;
                Add: ALUoperation = 3'b010;
                Sub: ALUoperation = 3'b011; 
                Slt: ALUoperation = 3'b111;  // slt & slti
                default:   ALUoperation = 3'b000;
            endcase
        end
        else
            ALUoperation = 3'b111;
    end
endmodule


module Controller(MemToReg, PCSrc, RegDst, ALUoperation, ALUSrc, RegWrite, MemWrite, MemRead, Flush,
                  opc, func, equal);
    output reg [1:0] MemToReg, PCSrc, RegDst;
    output [2:0] ALUoperation;
    output reg ALUSrc, RegWrite, MemWrite, MemRead, Flush;
    input [5:0] opc, func;
    input equal;
    reg [1:0] ALUop;
    reg [3:0] state;
    AluController ALUCtrl(ALUoperation, ALUop, func);

    parameter [5:0] RType = 6'b000000, Lw = 6'b100011, Sw = 6'b101011, Beq = 6'b000100, Addi = 6'b001000,
        Jump =6'b000010, Jal = 6'b000011, JumpR = 6'b000110, Slti = 6'b001010;

        always @(opc, equal)begin
        {Flush, ALUop, MemWrite, MemRead, ALUSrc, RegWrite, MemToReg, RegDst, PCSrc} = 13'd3;
        case (opc)
            RType : begin
                RegDst = 2'b01;
                RegWrite = 1'b1;
                ALUop = 2'b10;
                state = 4'd1;
            end

            Addi: begin
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
                state = 4'd5;
            end

            Sw : begin
                ALUSrc = 1'b1;
                MemWrite = 1'b1;
                state = 4'd3;
            end
            
            Lw: begin
                ALUSrc = 1'b1;
                MemToReg = 2'b01;
                RegWrite = 1'b1;
                MemRead = 1'b1;
                state = 4'd2;
            end
            
            Jump: begin
                PCSrc = 2'b01;
                Flush = 1'b1;
                state = 4'd6;
            end
            
            JumpR: begin
                PCSrc = 2'b00;
                state = 4'd8;
            end
            
            Jal: begin
                RegDst = 2'b10;
                MemToReg = 2'b10;
                PCSrc = 2'b01;
                state = 4'd7;
            end
            
            Beq : begin
		    PCSrc = equal ? 2'b10 : 2'b11;
		    Flush = equal;
                state = 4'd4;
            end
            
            Slti: begin
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
                RegDst = 2'b00;
                ALUop = 2'b11;
                MemToReg = 2'b00;
                state = 4'd9;
            end
        endcase
    end

endmodule