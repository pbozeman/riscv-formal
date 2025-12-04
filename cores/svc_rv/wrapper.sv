//
// Configuration via defines (can be overridden via +define+ in .sby files)
//
`ifndef SVC_RV_PIPELINED
`define SVC_RV_PIPELINED 1
`endif

`ifndef SVC_RV_FWD_REGFILE
`define SVC_RV_FWD_REGFILE 1
`endif

`ifndef SVC_RV_FWD
`define SVC_RV_FWD 1
`endif

`ifndef SVC_RV_MEM_TYPE
`define SVC_RV_MEM_TYPE 0
`endif

`ifndef SVC_RV_BPRED
`define SVC_RV_BPRED 0
`endif

`ifndef SVC_RV_BTB_ENABLE
`define SVC_RV_BTB_ENABLE 0
`endif

`ifndef SVC_RV_RAS_ENABLE
`define SVC_RV_RAS_ENABLE 0
`endif

`ifndef SVC_RV_EXT_ZMMUL
`define SVC_RV_EXT_ZMMUL 0
`endif

`ifndef SVC_RV_EXT_M
`define SVC_RV_EXT_M 0
`endif

module rvfi_wrapper (
    input wire clock,
    input wire reset,
    `RVFI_OUTPUTS
    `RVFI_BUS_OUTPUTS
);

  //
  // Instruction memory configuration
  //
  localparam IMEM_WORDS = 32;
  localparam IMEM_AW = $clog2(IMEM_WORDS);

  //
  // Memory interface signals
  //
  // riscv-formal models architectural memory; the solver picks
  // instruction and data values that satisfy the formal properties.
  //
  // For instruction memory, we use immutable memory (constant array):
  // Each location gets a random value that stays constant throughout
  // the trace. This is required for BTB correctness, since BTB assumes
  // instruction memory doesn't change.
  //
  // Limited to 32 words to reduce solver complexity.
  //
  // Use generate block to create individual anyconst registers, then
  // wire them to an array for dynamic indexing (can't index generate
  // blocks directly with non-constant values).
  //
  wire [31:0] imem_array[IMEM_WORDS];

  genvar imem_i;
  generate
    for (imem_i = 0; imem_i < IMEM_WORDS; imem_i = imem_i + 1) begin : gen_imem
      `rvformal_rand_const_reg [31:0] data;
      assign imem_array[imem_i] = data;
    end
  endgenerate

  (* keep *)`rvformal_rand_reg [31:0] dmem_rdata_any;

  (* keep *)wire                      imem_arvalid;
  (* keep *)wire               [31:0] imem_araddr;
  (* keep *)wire               [31:0] imem_rdata;
  (* keep *)wire                      imem_rvalid;

  (* keep *)wire                      dmem_ren;
  (* keep *)wire               [31:0] dmem_raddr;
  (* keep *)wire               [31:0] dmem_rdata;

  (* keep *)wire                      dmem_we;
  (* keep *)wire               [31:0] dmem_waddr;
  (* keep *)wire               [31:0] dmem_wdata;
  (* keep *)wire               [ 3:0] dmem_wstrb;

  (* keep *)wire                      ebreak;
  (* keep *)wire                      trap;
  (* keep *)wire                      rvfi_mem_valid;
  (* keep *)wire                      rvfi_mem_instr;

  svc_rv #(
      .XLEN       (32),
      .IMEM_AW    (10),
      .DMEM_AW    (10),
      .PIPELINED  (`SVC_RV_PIPELINED),
      .FWD_REGFILE(`SVC_RV_FWD_REGFILE),
      .FWD        (`SVC_RV_FWD),
      .MEM_TYPE   (`SVC_RV_MEM_TYPE),
      .BPRED      (`SVC_RV_BPRED),
      .BTB_ENABLE (`SVC_RV_BTB_ENABLE),
      .RAS_ENABLE (`SVC_RV_RAS_ENABLE),
      .EXT_ZMMUL  (`SVC_RV_EXT_ZMMUL),
      .EXT_M      (`SVC_RV_EXT_M)
  ) dut (
      .clk  (clock),
      .rst_n(!reset),

      .imem_arvalid(imem_arvalid),
      .imem_araddr (imem_araddr),
      .imem_rdata  (imem_rdata),
      .imem_rvalid (imem_rvalid),

      .dmem_ren  (dmem_ren),
      .dmem_raddr(dmem_raddr),
      .dmem_rdata(dmem_rdata),

      .dmem_we   (dmem_we),
      .dmem_waddr(dmem_waddr),
      .dmem_wdata(dmem_wdata),
      .dmem_wstrb(dmem_wstrb),

      .ebreak(ebreak),
      .trap  (trap),

      .rvfi_valid    (rvfi_valid),
      .rvfi_order    (rvfi_order),
      .rvfi_insn     (rvfi_insn),
      .rvfi_trap     (rvfi_trap),
      .rvfi_halt     (rvfi_halt),
      .rvfi_intr     (rvfi_intr),
      .rvfi_mode     (rvfi_mode),
      .rvfi_ixl      (rvfi_ixl),
      .rvfi_rs1_addr (rvfi_rs1_addr),
      .rvfi_rs2_addr (rvfi_rs2_addr),
      .rvfi_rs1_rdata(rvfi_rs1_rdata),
      .rvfi_rs2_rdata(rvfi_rs2_rdata),
      .rvfi_rd_addr  (rvfi_rd_addr),
      .rvfi_rd_wdata (rvfi_rd_wdata),
      .rvfi_pc_rdata (rvfi_pc_rdata),
      .rvfi_pc_wdata (rvfi_pc_wdata),
      .rvfi_mem_addr (rvfi_mem_addr),
      .rvfi_mem_rmask(rvfi_mem_rmask),
      .rvfi_mem_wmask(rvfi_mem_wmask),
      .rvfi_mem_rdata(rvfi_mem_rdata),
      .rvfi_mem_wdata(rvfi_mem_wdata),
      .rvfi_mem_valid(rvfi_mem_valid),
      .rvfi_mem_instr(rvfi_mem_instr),
  );

  //
  // Instruction memory access
  //
  // Map reads to immutable memory. Addresses wrap within the configured
  // word count by masking to the address width.
  //
  // SRAM: Combinational read (0-cycle latency), imem_rvalid = imem_arvalid
  // BRAM: Registered read (1-cycle latency), imem_rvalid = registered arvalid
  //
  wire [IMEM_AW-1:0] imem_idx;
  assign imem_idx = imem_araddr[IMEM_AW+1:2] & ((1 << IMEM_AW) - 1);

  if (`SVC_RV_MEM_TYPE == 1) begin : g_bram_timing
    reg [31:0] imem_rdata_reg;
    reg        imem_rvalid_reg;

    always @(posedge clock) begin
      if (reset) begin
        // This is what the svc_rv_soc_bram does at startup
        imem_rdata_reg  <= 32'h00000013;
        imem_rvalid_reg <= 1'b0;
      end else begin
        imem_rvalid_reg <= imem_arvalid;
        if (imem_arvalid) begin
          imem_rdata_reg <= imem_array[imem_idx];
        end
      end
    end

    assign imem_rdata  = imem_rdata_reg;
    assign imem_rvalid = imem_rvalid_reg;
  end else begin : g_sram_timing
    assign imem_rdata  = imem_arvalid ? imem_array[imem_idx] : 32'hxxxxxxxx;
    assign imem_rvalid = imem_arvalid;
  end

  //
  // Data memory access timing
  //
  // For formal we don't model actual data contents here; the RVFI / RVFI_BUS
  // checkers track architectural memory. We only need to match the timing:
  //
  //   SRAM (MEM_TYPE=0): combinational read (0-cycle latency)
  //   BRAM (MEM_TYPE=1): registered read (1-cycle latency)
  //
  if (`SVC_RV_MEM_TYPE == 1) begin : g_dmem_bram_timing
    reg [31:0] dmem_rdata_reg;

    always @(posedge clock) begin
      if (reset) begin
        dmem_rdata_reg <= 32'hxxxxxxxx;
      end else if (dmem_ren) begin
        dmem_rdata_reg <= dmem_rdata_any;
      end
    end

    assign dmem_rdata = dmem_rdata_reg;
  end else begin : g_dmem_sram_timing
    assign dmem_rdata = dmem_ren ? dmem_rdata_any : 32'hxxxxxxxx;
  end

`ifdef RISCV_FORMAL_BUS
  //
  // Data memory bus interface - NOT IMPLEMENTED
  //
  // The RVFI_BUS dmem checker tracks writes via RVFI (rvfi_bus_dmem_check.sv:91)
  // but constrains reads from raw bus transactions (rvfi_bus_dmem_check.sv:65).
  // Our pipelined memory makes this timing mismatch - bus reads happen in MEM stage
  // but RVFI reports in WB stage, and with BRAM latency the data arrives one cycle
  // after the bus transaction. Would need additional work to align the timing.
  //

`endif

endmodule
