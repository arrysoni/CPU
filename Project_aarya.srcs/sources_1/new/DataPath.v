`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Aarya Soni 
// 
// Create Date: 04/28/2025 04:32:28 PM
// Design Name: 
// Module Name: DataPath
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module DataPath(
    input clk,
    
    // IF/ID outputs
    output [31:0] pc,
    output [31:0] dinstOut,
    
    // ID/EXE outputs
    output ewreg,
    output em2reg, 
    output ewmem,
    output [3:0] ealuc,
    output ealuimm,
    output [4:0] edestReg,
    output [31:0] eqa,
    output [31:0] eqb,
    output [31:0] eimm32,
    
    // EXE/MEM outputs
    output mwreg,
    output mm2reg,
    output mwmem,
    output [4:0] mdestReg,
    output [31:0] mr,
    output [31:0] mqb,
    
    // MEM/WB outputs
    output wwreg,
    output wm2reg,
    output [4:0] wdestReg,
    output [31:0] wr,
    output [31:0] wdo,
    
    output [31:0] qa_regfile,
    output [31:0] qb_regfile
);

    // Declaring all the wires
    wire [31:0] nextPc;
    wire [31:0] instOut;
    
    wire [5:0] op    = dinstOut[31:26];
    wire [5:0] func  = dinstOut[5:0];
    wire [4:0] rt    = dinstOut[20:16];
    wire [4:0] rd    = dinstOut[15:11];
    wire [4:0] rs    = dinstOut[25:21];

    wire wreg, m2reg_sig, wmem, aluimm, regrt;
    wire [3:0] aluc;
    wire [31:0] qa, qb, imm32;
    wire [4:0] destReg;
    
    wire ewreg_wire, em2reg_wire, ewmem_wire;
    wire [3:0] ealuc_wire;
    wire ealuimm_wire;
    wire [4:0] edestReg_wire;
    wire [31:0] eqa_wire, eqb_wire, eimm32_wire;
    
    wire [31:0] alu_b;
    wire [31:0] alu_result;
    wire [31:0] mdo;
    
    wire [31:0] wbData;
    
    
    // Lab 5 part 
    // Wirs for the forwardig control signal
    wire [1:0] fwda;
    wire [1:0] fwdb;
    
    reg [31:0] forwardA;
    reg [31:0] forwardB;
    
    // MUX A
    always @(*) begin 
        case (fwda)
            2'b00: forwardA = qa;
            2'b01: forwardA = alu_result;
            2'b10: forwardA = mr;
            2'b11: forwardA = wbData;
            default: forwardA = qa;
         endcase
     end
     
     
     // MUX B
    always @(*) begin 
        case (fwdb)
            2'b00: forwardB = qb;
            2'b01: forwardB = alu_result;
            2'b10: forwardB = mr;
            2'b11: forwardB = wbData;
            default: forwardB = qb;
         endcase
     end
    
    
    // Instantiating all the modules
    
    // IF Stage
    ProgramCounter PC(
        .clk(clk),
        .nextPc(nextPc),
        .pc(pc)
    );
    
    InstructionMemory instr_mem(
        .pc(pc),
        .instOut(instOut)
    );
    
    PCAdder pc_adder(
        .pc(pc),
        .nextPc(nextPc)
    );
    
    IFID_PipelineRegister if_id_reg(
        .clk(clk),
        .instOut(instOut),
        .dinstOut(dinstOut)
    );
    
    // ID Stage
    Control_Unit control_unit (
        .op(op),
        .func(func),
        .wreg(wreg),
        .m2reg(m2reg_sig),
        .wmem(wmem),
        .aluimm(aluimm),
        .regrt(regrt),
        .aluc(aluc)
    );
    
    Regrt_MUX regrt_mux (
        .rt(rt),
        .rd(rd),
        .regrt(regrt),
        .destReg(destReg)
    );
    
    Register_File reg_file (
        .rs(rs),
        .rt(rt),
        .qa(qa),
        .qb(qb),
        .wdestReg(wdestReg),
        .wbData(wbData),
        .wwreg(wwreg),
        .clk(clk)
        
    );
    
    // Forward qa and qb to output ports
    assign qa_regfile = qa;
    assign qb_regfile = qb;
    
    Immediate_Extender imm_ext (
        .imm(dinstOut[15:0]),
        .imm32(imm32)
    );
    
    ID_EXE_Pipeline id_exe_pipeline (
        .wreg(wreg),
        .m2reg(m2reg_sig),
        .wmem(wmem),
        .aluc(aluc),
        .aluimm(aluimm),
        .destReg(destReg),
        .qa(forwardA),
        .qb(forwardB),
        .imm32(imm32),
        .clk(clk),
        .ewreg(ewreg_wire),
        .em2reg(em2reg_wire),
        .ewmem(ewmem_wire),
        .ealuc(ealuc_wire), 
        .ealuimm(ealuimm_wire),
        .edestReg(edestReg_wire),
        .eqa(eqa_wire),
        .eqb(eqb_wire),
        .eimm32(eimm32_wire)
    );
    
    assign ewreg    = ewreg_wire;
    assign em2reg   = em2reg_wire;
    assign ewmem    = ewmem_wire;
    assign ealuc    = ealuc_wire;
    assign ealuimm  = ealuimm_wire;
    assign edestReg = edestReg_wire;
    assign eqa      = eqa_wire;
    assign eqb      = eqb_wire;
    assign eimm32   = eimm32_wire;
    
    // EXE Stage
    ALU_Mux alu_mux(
        .ealuimm(ealuimm_wire),
        .eqb(eqb_wire),
        .eimm32(eimm32_wire),
        .b(alu_b)
    );
    
    ALU alu_inst(
        .eqa(eqa_wire),
        .b(alu_b),
        .ealuc(ealuc_wire),
        .r(alu_result)
    );
    
    // EXE/MEM Pipeline
    EXE_MEM_Pipeline exe_mem_pipeline (
        .clk(clk),
        .ewreg(ewreg_wire),
        .em2reg(em2reg_wire),
        .ewmem(ewmem_wire),
        .edestReg(edestReg_wire),
        .r(alu_result),
        .eqb(eqb_wire),
        .mwreg(mwreg),
        .mm2reg(mm2reg),
        .mwmem(mwmem),
        .mdestReg(mdestReg),
        .mr(mr),
        .mqb(mqb)
    );
    
    // Memory Stage
    DataMemory data_mem(
        .clk(clk),
        .mr(mr),
        .mqb(mqb),
        .mwmem(mwmem),
        .mdo(mdo)
    );
    
    // MEM/WB Pipeline
    MEM_WB_Pipeline mem_wb_pipeline (
        .clk(clk),
        .mwreg(mwreg),
        .mm2reg(mm2reg),
        .mdestReg(mdestReg),
        .mr(mr),
        .mdo(mdo),
        .wwreg(wwreg),
        .wm2reg(wm2reg),
        .wdestReg(wdestReg),
        .wr(wr),
        .wdo(wdo)
    );
    
    // WB MUX
    WB_Mux wb_mux(
        .wr(wr),
        .wdo(wdo),
        .wm2reg(wm2reg),
        .wbData(wbData)
    );
    
    // New addition as part of Project
    // Forwarding Unit
    ForwardingUnit forward_unit (
    .rs(rs),
    .rt(rt),
    .edestReg(edestReg_wire),
    .mdestReg(mdestReg),
    .wdestReg(wdestReg),
    .ewreg(ewreg_wire),
    .mwreg(mwreg),
    .wwreg(wwreg),
    .fwda(fwda),
    .fwdb(fwdb)
);
    
endmodule

// IF Stage Modules

// Program Counter
module ProgramCounter(
    input clk,
    input [31:0] nextPc,
    
    output reg [31:0] pc
);
    initial begin
        pc = 32'd100;  // Should be decimal value, as stated in the video
    end
    always @(posedge clk) begin
        pc <= nextPc;
    end
endmodule

// Instruction Memory
module InstructionMemory(
    input [31:0] pc,
    output reg [31:0] instOut
);
    // 32 x 64-word memory (each word is 32 bits): Given in the Lab file
    reg [31:0] memory [0:63];
    
    initial begin
    memory[25] = 32'h00221820;   // add $3, $1, $2
    memory[26] = 32'h01232022;   // sub $4, $9, $3
    memory[27] = 32'h00692825;   // or $5, $3, $9
    memory[28] = 32'h00693026;   // xor $6, $3, $9
    memory[29] = 32'h00693824;   // and $7, $3, $9
    end
    
    always @(*) begin
        instOut = memory[pc[7:2]];
    end
endmodule

// PC Adder
module PCAdder(
    input [31:0] pc,
    output reg [31:0] nextPc
);
    always @(*) begin
        nextPc = pc + 4;
    end
endmodule

// IF/ID Pipeline Register
module IFID_PipelineRegister(
    input clk,
    input [31:0] instOut,
    output reg [31:0] dinstOut
);
    always @(posedge clk) begin
        dinstOut <= instOut;
    end 
endmodule

// ID Stage Modules

// Control Unit
module Control_Unit(
    input [5:0] op,
    input [5:0] func,
    
    output reg wreg,
    output reg m2reg,
    output reg wmem,
    output reg aluimm,
    output reg regrt,
    output reg [3:0] aluc
);
    always @(*) begin
        case (op)
            6'b000000: begin // R-type
                case (func)
                    6'b100000: begin // add
                        wreg    = 1;
                        m2reg   = 0;
                        wmem    = 0;
                        aluc    = 4'b0010; // ADD
                        aluimm  = 0;
                        regrt   = 0;
                    end
                    6'b100010: begin // sub
                        wreg    = 1;
                        m2reg   = 0;
                        wmem    = 0;
                        aluc    = 4'b0110; // SUBTRACT
                        aluimm  = 0;
                        regrt   = 0;
                    end
                    6'b100101: begin // or
                        wreg    = 1;
                        m2reg   = 0;
                        wmem    = 0;
                        aluc    = 4'b0001; // OR
                        aluimm  = 0;
                        regrt   = 0;
                    end
                    6'b100110: begin // xor
                        wreg    = 1;
                        m2reg   = 0;
                        wmem    = 0;
                        aluc    = 4'b1100; // XOR
                        aluimm  = 0;
                        regrt   = 0;
                    end
                    6'b100100: begin // and
                        wreg    = 1;
                        m2reg   = 0;
                        wmem    = 0;
                        aluc    = 4'b0000; // AND
                        aluimm  = 0;
                        regrt   = 0;
                    end
                    default: begin
                        wreg = 0; m2reg = 0; wmem = 0; aluc = 4'b0000; aluimm = 0; regrt = 0;
                    end
                endcase
            end
            6'b100011: begin // Load Word (lw)
                wreg    = 1;
                m2reg   = 1;
                wmem    = 0;
                aluc    = 4'b0010; // ADD
                aluimm  = 1;
                regrt   = 1;
            end
            6'b101011: begin // Store Word (sw)
                wreg    = 0;
                m2reg   = 0;
                wmem    = 1;
                aluc    = 4'b0010; // ADD
                aluimm  = 1;
                regrt   = 1'bx;
            end
            default: begin
                wreg = 0; m2reg = 0; wmem = 0; aluc = 4'b0000; aluimm = 0; regrt = 0;
            end
        endcase
    end
endmodule
// Regrt MUX
module Regrt_MUX(
    input [4:0] rt,
    input [4:0] rd,
    input regrt,
    
    output reg [4:0] destReg
);
    always @(*) begin
        if (regrt)
            destReg = rt;  // For I-type instructions, destination is rt
        else
            destReg = rd;  // For R-type, destination is rd
    end
endmodule

// Register File
module Register_File(
    input [4:0] rs,
    input [4:0] rt,
    
    // Updated inputs for Lab 5
    input [4:0] wdestReg,
    input [31:0] wbData,
    input wwreg,
    input clk,
    
    output reg [31:0] qa,
    output reg [31:0] qb
);
    reg [31:0] registers [0:31];
    integer i;
    
    // Register File new words initialized for Lab 5
    initial begin
        registers[0] = 32'h00000000;
        registers[1] = 32'hA00000AA;
        registers[2] = 32'h10000011;
        registers[3] = 32'h20000022;
        registers[4] = 32'h30000033;
        registers[5] = 32'h40000044;
        registers[6] = 32'h50000055;
        registers[7] = 32'h60000066;
        registers[8] = 32'h70000077;
        registers[9] = 32'h80000088;
        registers[10] = 32'h90000099;
        
        for (i = 11; i < 32; i = i + 1)
            registers[i] = 32'h0;
end
    
    always @(*) begin
        qa = registers[rs];
        qb = registers[rt];
    end
    
    always @(negedge clk) begin
        if (wwreg && wdestReg != 0) begin      // If register write is enabled and the destination register is not $zero, then write wbData to that register

            registers[wdestReg] <= wbData;
         end
    end
endmodule

// Immediate Extender
module Immediate_Extender(
    input [15:0] imm,
    output reg [31:0] imm32
);
    always @(*) begin
        imm32 = {{16{imm[15]}}, imm[15:0]};
    end
endmodule

// Forwarding Unit 
 module ForwardingUnit(
    input [4:0] rs,       
    input [4:0] rt,       
    input [4:0] edestReg, 
    input [4:0] mdestReg, 
    input [4:0] wdestReg, 
    input ewreg,          
    input mwreg,          
    input wwreg,          

    output reg [1:0] fwda, 
    output reg [1:0] fwdb   
);

always @(*) begin

    fwda = 2'b00;
    fwdb = 2'b00;

    // Forward A
    if (ewreg && (edestReg != 0) && (edestReg == rs)) 
        fwda = 2'b01; // EXE stage
    else if (mwreg && (mdestReg != 0) && (mdestReg == rs)) 
        fwda = 2'b10; // MEM stage
    else if (wwreg && (wdestReg != 0) && (wdestReg == rs))
        fwda = 2'b11; // WB stage

    // Forward B
    if (ewreg && (edestReg != 0) && (edestReg == rt)) 
        fwdb = 2'b01; // EXE stage
    else if (mwreg && (mdestReg != 0) && (mdestReg == rt)) 
        fwdb = 2'b10; // MEM stage
    else if (wwreg && (wdestReg != 0) && (wdestReg == rt))
        fwdb = 2'b11; // WB stage
end

endmodule
     

// ID/EXE Pipeline Register
module ID_EXE_Pipeline(
    input wreg,
    input m2reg,
    input wmem,
    input [3:0] aluc,
    input aluimm,
    input [4:0] destReg,
    input [31:0] qa,
    input [31:0] qb,
    input [31:0] imm32,
    input clk,
    
    output reg ewreg,
    output reg em2reg,
    output reg ewmem,
    output reg [3:0] ealuc,
    output reg ealuimm,
    output reg [4:0] edestReg,
    output reg [31:0] eqa,
    output reg [31:0] eqb,
    output reg [31:0] eimm32
);
    always @(posedge clk) begin
        ewreg    <= wreg;
        em2reg   <= m2reg;
        ewmem    <= wmem;
        ealuc    <= aluc;
        ealuimm  <= aluimm;
        edestReg <= destReg;
        eqa      <= qa;
        eqb      <= qb;
        eimm32   <= imm32;
    end
endmodule

// EXE (Execute) Stage Modules

// ALU Multiplexer
module ALU_Mux(
    input ealuimm,
    input [31:0] eqb,
    input [31:0] eimm32,
    
    output reg [31:0] b
);
    always @(*) begin
        if (ealuimm)
            b = eimm32;
        else
            b = eqb;
    end
endmodule

// ALU
module ALU(
    input [31:0] eqa,
    input [31:0] b,
    input [3:0] ealuc,
    
    output reg [31:0] r
);
    always @(*) begin
        case(ealuc)                       // Adding additonal opeartions for future references
            4'b0000: r = eqa & b;         // AND
            4'b0001: r = eqa | b;         // OR
            4'b0010: r = eqa + b;         // ADD
            4'b0110: r = eqa - b;         // SUBTRACT
            4'b1100: r = eqa ^ b;         // XOR
            default: r = 0;
        endcase
    end
endmodule

// EXE/MEM Pipeline Register
module EXE_MEM_Pipeline(
    input clk,
    input ewreg,
    input em2reg,
    input ewmem,
    input [4:0] edestReg,
    input [31:0] r,
    input [31:0] eqb,
    
    output reg mwreg,
    output reg mm2reg,
    output reg mwmem,
    output reg [4:0] mdestReg,
    output reg [31:0] mr,
    output reg [31:0] mqb
);
    always @(posedge clk) begin
        mwreg   <= ewreg;
        mm2reg  <= em2reg;
        mwmem   <= ewmem;
        mdestReg<= edestReg;
        mr      <= r;
        mqb     <= eqb;
    end
endmodule

// MEM (Memory Access) Stage Modules

// DataMemory
module DataMemory(
    input clk,
    input [31:0] mr,
    input [31:0] mqb,
    input mwmem,
    
    output reg [31:0] mdo
);
    reg [31:0] dataMemory [0:63];
    integer i;
    
    initial begin
        // Part 6. of Lab 4
        dataMemory[0] = 32'hA00000AA;
        dataMemory[1] = 32'h10000011;
        dataMemory[2] = 32'h20000022;
        dataMemory[3] = 32'h30000033;
        dataMemory[4] = 32'h40000044;
        dataMemory[5] = 32'h50000055;
        dataMemory[6] = 32'h60000066;
        dataMemory[7] = 32'h70000077;
        dataMemory[8] = 32'h80000088;
        dataMemory[9] = 32'h90000099;
        
        for(i = 10; i < 64; i = i + 1) begin
            dataMemory[i] = 0;
        end
    end
    
    // Read operation
    always @(*) begin
        mdo = dataMemory[mr[7:2]];
    end
    
    // Write operation on negative edge
    always @(negedge clk) begin
        if(mwmem)
            dataMemory[mr[7:2]] <= mqb;
    end
endmodule

// WB (Write-Back) Stage Modules

// MEM/WB Pipeline Register
module MEM_WB_Pipeline(
    input clk,
    input mwreg,
    input mm2reg,
    input [4:0] mdestReg,
    input [31:0] mr,
    input [31:0] mdo,
    
    output reg wwreg,
    output reg wm2reg,
    output reg [4:0] wdestReg,
    output reg [31:0] wr,
    output reg [31:0] wdo
);
    always @(posedge clk) begin
        wwreg   <= mwreg;
        wm2reg  <= mm2reg;
        wdestReg<= mdestReg;
        wr      <= mr;
        wdo     <= mdo;
    end
endmodule
        
// Writeback Multiplexer
module WB_Mux(
    input [31:0] wr,
    input [31:0] wdo,
    input wm2reg,
    
    output reg [31:0] wbData
);
    always @(*) begin
        if (wm2reg == 1'b0)
            wbData = wr;
        else
            wbData = wdo;
    end
endmodule







