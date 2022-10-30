module InstructionMem(instruction, address);
    output [31:0] instruction;
    input [31:0] address;
    reg [7:0] mem [0:65535];

    initial begin
        $readmemb("instruction.txt", mem);
    end

    assign instruction = {mem[address + 3], mem[address + 2], mem[address + 1], mem[address]};
endmodule