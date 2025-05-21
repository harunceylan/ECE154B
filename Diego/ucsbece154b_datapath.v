// ucsbece154b_datapath.v
// ECE 154B, RISC-V pipelined processor 
// All Rights Reserved
// Copyright (c) 2024 UCSB ECE
// Distribution Prohibited

// EDIT HERE 


module ucsbece154b_datapath (
    input                clk, reset, 
    input                PCSrcE_i, // Execute
    input                StallF_i, // Fetch
    output reg    [31:0] PCF_o, // Fetch
    input                StallD_i, // Decode
    input                FlushD_i,  // Decode
    input         [31:0] InstrF_i, // Fetch
    output wire    [6:0] op_o, // Decode
    output wire    [2:0] funct3_o,  // Decode
    output wire          funct7b5_o,    // Decode
    input                RegWriteW_i, // Writeback
    input          [2:0] ImmSrcD_i, // Decode
    output wire    [4:0] Rs1D_o,    // Decode
    output wire    [4:0] Rs2D_o,   // Decode
    input  wire          FlushE_i,  // Execute
    output reg     [4:0] Rs1E_o, // Execute
    output reg     [4:0] Rs2E_o,    // Execute
    output reg     [4:0] RdE_o,  // Execute
    input                ALUSrcE_i, // Execute
    input          [2:0] ALUControlE_i, // Execute
    input          [1:0] ForwardAE_i, // Execute
    input          [1:0] ForwardBE_i, // Execute
    output               ZeroE_o,   // Execute
    output reg     [4:0] RdM_o,  // Memory
    output reg    [31:0] ALUResultM_o, // Memory
    output reg    [31:0] WriteDataM_o, // Memory
    input         [31:0] ReadDataM_i, // Memory
    input          [1:0] ResultSrcW_i, // Writeback
    output reg     [4:0] RdW_o, // Writeback
    input          [1:0] ResultSrcM_i // Memory
);

`include "ucsbece154b_defines.vh"

// Pipeline registers
// Fetch -> Decode
wire [31:0] PCF, PCPlus4F;
reg [31:0] InstrD;

// Decode -> Execute
reg [31:0] PCPlus4D, PCD;
wire [31:0] ImmExtD;
wire [31:0] Rd1D, Rd2D;
wire [4:0] RdD;

// Execute -> Memory
reg [31:0] Rd1E, Rd2E, PCE, ImmExtE, PCPlus4E;
wire [31:0] ALUResultE, WriteDataE, WriteDateE, SrcAE, SrcBE, PCTargetE;

// Memory -> Writeback
reg [31:0] ImmExtM, PCPlus4M;
wire [31:0] FwdSrcM;  // Result for writeback stage

/*// Internal signals
wire [31:0] RD1, RD2;        // Register file outputs
wire [31:0] SrcAE, SrcBE;    // ALU inputs
reg  [31:0] ImmExt;          // Immediate value
wire [31:0] PCTargetE;
wire [31:0] PCNextF;
*/
// Writeback 
reg [31:0] ALUResultW, ReadDataW, ImmExtW, PCPlus4W;
wire [31:0] ResultW;


ucsbece154b_rf rf(
    .clk(!clk),
    .a1_i(Rs1D_o),
    .a2_i(Rs2D_o),
    .a3_i(RdW_o),
    .rd1_o(Rd1D),
    .rd2_o(Rd2D),
    .we3_i(RegWriteW_i),
    .wd3_i(ResultW)
);
// Fetch Stage
assign PCF = (PCSrcE_i == MuxPC_PCPlus4) ? PCPlus4F : PCTargetE;
assign PCPlus4F = PCF_o + 4;


// Decode Stage
assign funct7b5_o = InstrD[30];
assign Rs2D_o = InstrD[24:20];
assign Rs1D_o = InstrD[19:15];
assign funct3_o = InstrD[14:12];
assign RdD = InstrD[11:7];
assign op_o = InstrD[6:0];


// Immediate Generator
assign ImmExtD = 
    ImmSrcD_i === imm_Itype ? {{20{InstrD[31]}}, InstrD[31:20]} :
    ImmSrcD_i === imm_Stype ? {{20{InstrD[31]}}, InstrD[31:25], InstrD[11:7]} :
    ImmSrcD_i === imm_Btype ? {{20{InstrD[31]}}, InstrD[7], InstrD[30:25], InstrD[11:8], 1'b0} :
    ImmSrcD_i === imm_Jtype ? {{12{InstrD[31]}}, InstrD[19:12], InstrD[20], InstrD[30:21], 1'b0} :
    ImmSrcD_i === imm_Utype ? {InstrD[31:12], 12'b0} :
    32'bx
;

// Execute Stage

assign SrcAE = 
    ForwardAE_i === forward_ex  ? Rd1E :
    ForwardAE_i === forward_wb  ? ResultW :
    ForwardAE_i === forward_mem ? FwdSrcM :
    32'bx
;

assign SrcBE = (ALUSrcE_i === SrcB_reg) ? WriteDataE : ImmExtE;

assign WriteDataE = 
    ForwardBE_i === forward_ex  ? Rd2E :
    ForwardBE_i === forward_wb  ? ResultW :
    ForwardBE_i === forward_mem ? FwdSrcM :
    32'bx
;

assign PCTargetE = PCE + ImmExtE;

// Memory Stage

assign FwdSrcM = (ResultSrcM_i === MuxResult_imm) ? ImmExtM : ALUResultM_o;

// Writeback Stage
assign ResultW = 
    ResultSrcW_i === MuxResult_aluout  ? ALUResultW :
    ResultSrcW_i === MuxResult_mem     ? ReadDataW :
    ResultSrcW_i === MuxResult_PCPlus4 ? PCPlus4W:
    ResultSrcW_i === MuxResult_imm     ? ImmExtW:
    32'bx
;

// ALU
ucsbece154b_alu alu(
    .a_i(SrcAE),
    .b_i(SrcBE),
    .alucontrol_i(ALUControlE_i),
    .result_o(ALUResultE),
    .zero_o(ZeroE_o)
);

// Fetch Stage Pipeline Registers
always @(posedge clk or posedge reset) begin
    if (reset) begin
        PCF_o <= pc_start;
    end else if (!StallF_i) begin
        PCF_o <= PCF;
    end
end

// Decode Stage Pipeline Registers
always @(posedge clk or posedge reset) begin
    if (reset) begin
        InstrD <= 32'b0;
        PCD <= 32'b0;
        PCPlus4D <= 32'b0;
    end else begin
        if (FlushD_i) begin
            InstrD <= 32'b0;
            PCD <= 32'b0;
            PCPlus4D <= 32'b0;
        end else if (!StallD_i) begin
            InstrD <= InstrF_i;
            PCD <= PCF_o;
            PCPlus4D <= PCPlus4F;
        end
    end
end

// Execute Stage Pipeline Registers
always @(posedge clk or posedge reset) begin
    if (reset) begin
        Rd1E <= 32'b0;
        Rd2E <= 32'b0;
        Rs1E_o <= 5'b0;
        Rs2E_o <= 5'b0;
        RdE_o <= 5'b0;
        PCE <= 32'b0;
        PCPlus4E <= 32'b0;
        ImmExtE <= 32'b0;
    end else if (FlushE_i) begin
        Rd1E <= 32'b0;
        Rd2E <= 32'b0;
        Rs1E_o <= 5'b0;
        Rs2E_o <= 5'b0;
        RdE_o <= 5'b0;
        PCE <= 32'b0;
        PCPlus4E <= 32'b0;
        ImmExtE <= 32'b0;
    end else begin
        Rd1E <= Rd1D;
        Rd2E <= Rd2D;
        PCE <= PCD;
        Rs1E_o <= Rs1D_o;
        Rs2E_o <= Rs2D_o;
        RdE_o <= RdD;
        ImmExtE <= ImmExtD;
        PCPlus4E <= PCPlus4D;
    end
end

// Memory Stage Pipeline Registers
always @(posedge clk or posedge reset) begin
    if (reset) begin
        ALUResultM_o <= 32'b0;
        WriteDataM_o <= 32'b0;
        RdM_o <= 5'b0;
        PCPlus4M <= 32'b0;
        ImmExtM <= 32'b0;
    end else begin
        ALUResultM_o <= ALUResultE;
        WriteDataM_o <= WriteDataE;
        RdM_o <= RdE_o;
        PCPlus4M <= PCPlus4E;
        ImmExtM <= ImmExtE;
    end
end

// Writeback Stage Pipeline Registers
always @(posedge clk or posedge reset) begin
    if (reset) begin
        ReadDataW <= 32'b0;
        PCPlus4W <= 32'b0;
        ImmExtW <= 32'b0;
        RdW_o <= 5'b0;
        ALUResultW <= 32'b0;
    end else begin
        ReadDataW <= ReadDataM_i;
        PCPlus4W <= PCPlus4M;
        ImmExtW <= ImmExtM;
        RdW_o <= RdM_o;
        ALUResultW <= ALUResultM_o;
    end
end


endmodule

