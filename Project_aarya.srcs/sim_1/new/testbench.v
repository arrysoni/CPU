`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Aarya Soni
// 
// Create Date: 04/28/2025 04:32:52 PM
// Design Name: 
// Module Name: testbench
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

module testbench();
    reg clk;
    
    // IF/ID outputs
    wire [31:0] pc;
    wire [31:0] dinstOut;
    
    // ID/EXE outputs
    wire ewreg;
    wire em2reg;
    wire ewmem;
    wire [3:0] ealuc;
    wire ealuimm;
    wire [4:0] edestReg;
    wire [31:0] eqa;
    wire [31:0] eqb;
    wire [31:0] eimm32;
    
    // EXE/MEM outputs
    wire mwreg;
    wire mm2reg;
    wire mwmem;
    wire [4:0] mdestReg;
    wire [31:0] mr;
    wire [31:0] mqb;
    
    // MEM/WB outputs
    wire wwreg;
    wire wm2reg;
    wire [4:0] wdestReg;
    wire [31:0] wr;
    wire [31:0] wdo;
    
    // Additions for Lab 5
    wire [31:0] qa_regfile;
    wire [31:0] qb_regfile;
    
    
    initial begin
        clk = 1'b0;
    end
    
    // Instantiating the DataPath module with extended outputs
    DataPath dp(
        .clk(clk),
        .pc(pc),
        .dinstOut(dinstOut),
        .ewreg(ewreg),
        .em2reg(em2reg),
        .ewmem(ewmem),
        .ealuc(ealuc),
        .ealuimm(ealuimm),
        .edestReg(edestReg),
        .eqa(eqa),
        .eqb(eqb),
        .eimm32(eimm32),
        .mwreg(mwreg),
        .mm2reg(mm2reg),
        .mwmem(mwmem),
        .mdestReg(mdestReg),
        .mr(mr),
        .mqb(mqb),
        .wwreg(wwreg),
        .wm2reg(wm2reg),
        .wdestReg(wdestReg),
        .wr(wr),
        .wdo(wdo),
        .qa_regfile(qa_regfile),
        .qb_regfile(qb_regfile)
    );
    
    // Clock generation
    always begin
        #1 clk = ~clk;
    end
endmodule