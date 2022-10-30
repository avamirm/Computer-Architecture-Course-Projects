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
                Or: ALUoperation = 3'b001;
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

module Controller(RegDst, DataToWrite, ALUoperation, MemToReg, ALUSrc, RegWrite, Jr, J, MemWrite, MemRead,
    PCSrc, opc, func, zero);
    output reg [1:0] RegDst, DataToWrite;
    output [2:0] ALUoperation;
    output reg MemToReg, ALUSrc, RegWrite, Jr, J, MemWrite, MemRead;
    output PCSrc;
    input [5:0] opc, func;
    input zero;
    reg [1:0] ALUop;
    reg branch;

    AluController ALUCtrl(ALUoperation, ALUop, func);

    parameter [5:0] RType = 6'b000000, Lw = 6'b100011, Sw = 6'b101011, Beq = 6'b000100, Addi = 6'b001000,
        Jump =6'b000010, Jal = 6'b000011, JumpR = 6'b000110, Slti = 6'b001010;

        always @(opc)begin
        {RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, branch, ALUop, J, Jr, DataToWrite} = 14'd0;
        case (opc)
            RType : begin
                RegDst = 2'b01;
                RegWrite = 1'b1;
                ALUop = 2'b10;
            end

            Addi: begin
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end

            Sw : begin
                ALUSrc = 1'b1;
                MemWrite = 1'b1;
            end
            
            Lw : begin
                ALUSrc = 1'b1;
                MemToReg = 1'b1;
                RegWrite = 1'b1;
                MemRead = 1'b1;
            end
            
            Jump: begin
                J = 1'b1;
            end
            
            JumpR: begin
                Jr = 1'b1;
                J = 1'b1;
            end
            
            Jal: begin
                RegDst = 2'b10;
                DataToWrite = 2'b01;
                J = 1'b1;
            end
            
            Beq : begin
                branch = 1'b1;
                ALUop = 2'b01;
            end
            
            Slti: begin
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
                ALUop = 2'b11;
                DataToWrite = 2'b10;
            end
        endcase
    end
     assign PCSrc = branch & zero;

endmodule