module ALUController(RegDst2, RegWrite2, ALUoperation, ALUop, func);
    output reg [2:0] ALUoperation;
    output reg RegDst2, RegWrite2;
    input [2:0] ALUop;
    input [8:0] func;
    always @(ALUop, func) begin
        ALUoperation = 3'b010;
        RegWrite2 = 1'b0;
        RegDst2 = 1'b0;
        if(ALUop == 3'b010) 
            ALUoperation = 3'b010;
        else if(ALUop == 3'b011)
            ALUoperation = 3'b011;
        else if(ALUop == 3'b000)
            ALUoperation = 3'b000;
        else if(ALUop == 3'b001)
            ALUoperation = 3'b001;
        else 
        begin
            case(func)
            9'b000000001: begin 
                ALUoperation = 3'b101;
                RegDst2 = 1'b1;
                RegWrite2 = 1'b1;
            end
            9'b000000010: begin ALUoperation = 3'b100; RegWrite2 = 1'b1; end
            9'b000000100: begin ALUoperation = 3'b010; RegWrite2 = 1'b1; end
            9'b000001000: begin ALUoperation = 3'b011; RegWrite2 = 1'b1; end
            9'b000010000: begin ALUoperation = 3'b000; RegWrite2 = 1'b1; end
            9'b000100000: begin ALUoperation = 3'b001; RegWrite2 = 1'b1; end
            9'b001000000: begin ALUoperation = 3'b110; RegWrite2 = 1'b1; end
            9'b010000000: begin
                ALUoperation = 3'b111;
                RegWrite2 = 1'b0;
            end 
            endcase
        end
    end
endmodule


module Controller(ALUoperation, PCSrc, ALUSrcB, PCLoad, IOrD, RegDst, RegWrite, MemToReg, IRWrite, ALUSrcA,
                  MemRead, MemWrite, func, opc, zero, rst, clk);
    output [2:0] ALUoperation;
    output reg [1:0] PCSrc, ALUSrcB;
    output reg IOrD, MemToReg, IRWrite, ALUSrcA;
    output PCLoad, RegDst, RegWrite;
    output reg MemRead, MemWrite;
    input [8:0] func;
    input [3:0] opc;
    input zero, rst, clk;
    reg [2:0] ALUop;
    reg PCWriteCond, PCWrite, RegWrite1;
    wire RegDst2, RegWrite2;


    wire and1;
    assign and1 = PCWriteCond & zero;
    wire oneBitZero;
    assign oneBitZero = 1'b0;
    assign PCLoad = PCWrite  | and1;
    assign RegDst = oneBitZero | RegDst2;
    assign RegWrite = RegWrite1 | RegWrite2;

    ALUController ALUCntrl(RegDst2, RegWrite2, ALUoperation, ALUop, func);
    parameter [3:0] IF = 0, ID = 1, Load1 = 2, Load2 = 3, Store = 4, Jump = 5, BranchZ = 6, TypeC1 = 7,
                    TypeC2 = 8, Addi = 9, IState = 10, Subi = 11, Andi = 12, Ori = 13;
        
    reg [3:0] ns, ps;
    parameter [3:0]  LOAD = 4'b0000, STORE = 4'b0001, JUMP = 4'b0010, BRANCHZ = 4'b0100, TYPEC = 4'b1000,
                     ADDI = 4'b1100, SUBI = 4'b1101, ANDI = 4'b1110, ORI = 4'b1111;
               
    always @(ps, opc) begin
        case(ps) 
            IF: ns = ID;
            ID: begin
                case(opc) 
                    LOAD:  ns = Load1;
                    STORE: ns = Store;
                    JUMP:  ns = Jump;
                    BRANCHZ: ns = BranchZ;
                    TYPEC: ns = TypeC1;
                    ADDI:  ns = Addi;
                    SUBI:  ns = Subi;
                    ANDI:  ns = Andi;            
                    ORI:  ns = Ori;                
                endcase
            end
            Load1: ns = Load2;
            Load2: ns = IF;
            Store: ns = IF;
            Jump: ns = IF;
            BranchZ: ns = IF;
            TypeC1: ns = TypeC2;
            TypeC2: ns = IF;
            Addi: ns = IState;
            IState: ns = IF;
            Subi: ns = IState;
            Andi: ns = IState;
            Ori: ns = IState;
        endcase
    end
    always @(ps) begin
        {PCWrite, PCWriteCond, IOrD, MemWrite, MemRead, IRWrite, MemToReg, RegWrite1, ALUSrcB, PCSrc, ALUop, ALUSrcA} = 16'd5;
        case(ps) 
            IF: begin
                IOrD = 1'b0;
                MemRead = 1'b1;
                IRWrite = 1'b1;
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b01;
                ALUop = 3'b010;
                PCSrc = 2'b00;
                PCWrite = 1'b1;
            end
            ID: begin
                {MemWrite, MemRead, IRWrite, RegWrite1, PCWrite, PCWriteCond} = 6'b000000;            
            end
            Load1: begin
                IOrD = 1'b1;
                MemRead = 1'b1;
            end
            Load2: begin
                RegWrite1 = 1'b1;
                MemToReg = 1'b1;
            end
            Store: begin
                IOrD = 1'b1;
                MemWrite = 1'b1;
            end
            Jump: begin
                PCSrc = 2'b10;
                PCWrite = 1'b1;
            end
            BranchZ: begin
                ALUop = 3'b011;
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b00;
                PCWriteCond = 1'b1;
                PCSrc = 2'b01;
            end
            TypeC1: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b00;
                ALUop = 3'b111;
            end
            TypeC2: begin
                ALUop = 3'b111;
                RegWrite1 = 1'b0;
                MemToReg = 1'b0;                
            end
            Addi: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b10;
                ALUop = 3'b010;
            end
            IState: begin
                RegWrite1 = 1'b1;
                MemToReg = 1'b0;
            end
            Subi: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b10;
                ALUop = 3'b011;
            end
            Andi: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b10;
                ALUop = 3'b000;
            end
            Ori: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b10;
                ALUop = 3'b001;
            end
        endcase
    end
    always @(posedge clk, posedge rst) begin
        if(rst)
            ps <= IF;
        else
            ps <= ns;
    end
endmodule