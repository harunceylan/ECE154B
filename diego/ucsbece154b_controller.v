// ucsbece154b_controller.v
// ECE 154B, RISC-V pipelined processor 
// All Rights Reserved
// Copyright (c) 2024 UCSB ECE
// Distribution Prohibited

// EDIT HERE 


module ucsbece154b_controller (
    input                clk, reset,
    input         [6:0]  op_i, 
    input         [2:0]  funct3_i,
    input                funct7b5_i,
    input 	             ZeroE_i,
    input         [4:0]  Rs1D_i,
    input         [4:0]  Rs2D_i,
    input         [4:0]  Rs1E_i,
    input         [4:0]  Rs2E_i,
    input         [4:0]  RdE_i,
    input         [4:0]  RdM_i,
    input         [4:0]  RdW_i,
    output wire		     StallF_o,  
    output wire          StallD_o,
    output wire          FlushD_o,
    output wire    [2:0] ImmSrcD_o, // extended to support jal 
    output wire          PCSrcE_o,
    output reg     [2:0] ALUControlE_o,
    output reg           ALUSrcE_o,
    output wire          FlushE_o,
    output reg     [1:0] ForwardAE_o,
    output reg     [1:0] ForwardBE_o,
    output reg           MemWriteM_o,
    output reg          RegWriteW_o,
    output reg    [1:0] ResultSrcW_o, 
    output reg    [1:0] ResultSrcM_o
);

`include "ucsbece154b_defines.vh"


// Internal control signals
reg [11:0] controls;
wire [1:0] ALUOp;
wire lwStall;
wire jumpD;
wire branchD;
assign PCSrcE_o = (branchE & ZeroE_i) | jumpE;
reg [2:0] ImmSrcD_reg; // Internal register for ImmSrcD
reg  jumpE, branchE;

assign lwStall  = (ResultSrcE === MuxResult_mem) && ((Rs1D_i === RdE_i) || (Rs2D_i === RdE_i));
assign StallF_o = lwStall;
assign StallD_o = lwStall;
assign FlushE_o = lwStall || (PCSrcE_o === MuxPC_PCTarget);
assign FlushD_o = (PCSrcE_o === MuxPC_PCTarget);

wire RtypeSub;
assign RtypeSub = funct7b5_i & (op_i[5]);

wire RegWriteD;
wire MemWriteD;
reg MemWriteE;
reg RegWriteE, RegWriteM;
wire [1:0] ResultSrcD;
reg [1:0] ResultSrcE;
reg [2:0] ALUControlD;
wire ALUSrcD;


assign {RegWriteD, ImmSrcD_o, ALUSrcD, MemWriteD, ResultSrcD, branchD, ALUOp, jumpD} = controls;
// Main control decoder
always @* begin
    case (op_i)
        instr_lw_op:        controls = 12'b1_000_1_0_01_0_00_0;       
	    instr_sw_op:        controls = 12'b0_001_1_1_00_0_00_0; 
	    instr_Rtype_op:     controls = 12'b1_xxx_0_0_00_0_10_0;  
	    instr_beq_op:       controls = 12'b0_010_0_0_00_1_01_0;  
	    instr_ItypeALU_op:  controls = 12'b1_000_1_0_00_0_10_0; 
	    instr_jal_op:       controls = 12'b1_011_x_0_10_0_xx_1;  
        instr_lui_op:       controls = 12'b1_100_x_0_11_0_xx_0;   
	default: 	    controls = 12'b0; 
    endcase
end 
/*
    RegWriteW_o = controls[11];    // register write
    ImmSrcD_reg = controls[10:8];  // immediate source
    ALUSrcE_o = controls[7];       // ALU source
    MemWriteM_o = controls[6];     // memory write
    ResultSrcW_o = controls[5:4];  // result source
    ResultSrcM_o = controls[5:4];  // result source for memory stage
    branch = controls[3];          // branch
    ALUOp = controls[2:1];         // ALU operation
    jump = controls[0];            // jump
*/



// ALU control decoder
always @* begin
    case (ALUOp)
        ALUop_mem:   ALUControlD = ALUcontrol_add;  // lw/sw
        ALUop_beq:   ALUControlD = ALUcontrol_sub;  // beq
        ALUop_other: begin
            case (funct3_i)
                instr_addsub_funct3: ALUControlD = RtypeSub ? ALUcontrol_sub : ALUcontrol_add;
                instr_slt_funct3:    ALUControlD = ALUcontrol_slt;
                instr_or_funct3:     ALUControlD = ALUcontrol_or;
                instr_and_funct3:    ALUControlD = ALUcontrol_and;
                default:             ALUControlD = 3'bxxx;
            endcase
        end
    default: 
      `ifdef SIM
          $warning("Unsupported ALUop given: %h", ALUOp);
      `else
          ;
      `endif
    endcase
end




/*// Forwarding logic
always @* begin
    // Forward A
    if (Rs1E_i != 5'b0) begin
        if (RegWriteW_o && (RdW_i == Rs1E_i))
            ForwardAE_o = 2'b01;  // Forward from Writeback
        else if (RegWriteM_o && (RdM_i == Rs1E_i))
            ForwardAE_o = 2'b10;  // Forward from Memory
        else
            ForwardAE_o = 2'b00;  // No forwarding
    end else
        ForwardAE_o = 2'b00;  // x0 doesn't need forwarding

    // Forward B
    if (Rs2E_i != 5'b0) begin
        if (RegWriteW_o && (RdW_i == Rs2E_i))
            ForwardBE_o = 2'b01;  // Forward from Writeback
        else if (RegWriteM_o && (RdM_i == Rs2E_i))
            ForwardBE_o = 2'b10;  // Forward from Memory
        else
            ForwardBE_o = 2'b00;  // No forwarding
    end else
        ForwardBE_o = 2'b00;  // x0 doesn't need forwarding
end
*/


// D/E Stage Pipeline Control
always @(posedge clk or posedge reset) begin
    if (reset) begin
        RegWriteE       <= 1'b0;
        ResultSrcE     <= 2'b00;
        MemWriteE       <= 1'b0;
        jumpE           <= 1'b0;
        branchE         <= 1'b0;
        ALUControlE_o   <= 3'b0;
        ALUSrcE_o       <= 1'b0;
        ForwardAE_o     <= forward_ex;
        ForwardBE_o     <= forward_ex;
    end else if (FlushE_o) begin
        RegWriteE       <= 1'b0;
        ResultSrcE     <= 2'b00;
        MemWriteE       <= 1'b0;
        jumpE           <= 1'b0;
        branchE         <= 1'b0;
        ALUControlE_o   <= 3'b0;
        ALUSrcE_o       <= 1'b0;
        ForwardAE_o     <= forward_ex;
        ForwardBE_o     <= forward_ex;
    end else begin
        // Propagate control signals
        RegWriteE       <= RegWriteD;
        ResultSrcE     <= ResultSrcD;
        MemWriteE       <= MemWriteD;
        jumpE           <= jumpD;
        branchE         <= branchD;
        ALUControlE_o   <= ALUControlD;
        ALUSrcE_o       <= ALUSrcD;

        // Hazard detection
        ForwardAE_o     <= forward_ex;  
        ForwardBE_o     <= forward_ex;  
        

        if (Rs1D_i !== 0 && RegWriteE && (Rs1D_i === RdE_i))
            ForwardAE_o <= forward_mem;
        else if (Rs1D_i !== 0 && RegWriteM && (Rs1D_i === RdM_i))
            ForwardAE_o <= forward_wb;
            
        if (Rs2D_i !== 0 && RegWriteE && (Rs2D_i === RdE_i))
            ForwardBE_o <= forward_mem;
        else if (Rs2D_i !== 0 && RegWriteM && (Rs2D_i === RdM_i))
            ForwardBE_o <= forward_wb;
        
    end
end
// E/M Stage Pipeline
always @(posedge clk or posedge reset) begin
    if (reset) begin
        RegWriteM       <= 1'b0;
        ResultSrcM_o    <= 2'b00;
        MemWriteM_o     <= 1'b0;
    end else begin
        RegWriteM       <= RegWriteE;
        ResultSrcM_o    <= ResultSrcE;
        MemWriteM_o     <= MemWriteE;
    end
end

// M/W Stage Pipeline
always @(posedge clk or posedge reset) begin
    if (reset) begin
        RegWriteW_o     <= 1'b0;
        ResultSrcW_o    <= 2'b00;
    end else begin
        RegWriteW_o     <= RegWriteM;
        ResultSrcW_o    <= ResultSrcM_o;
    end
end



endmodule 

