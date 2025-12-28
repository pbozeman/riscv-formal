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

`ifndef SVC_RV_STALL
`define SVC_RV_STALL 0
`endif

`ifndef SVC_RV_PC_REG
`define SVC_RV_PC_REG 0
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
  for (imem_i = 0; imem_i < IMEM_WORDS; imem_i = imem_i + 1) begin : gen_imem
    `rvformal_rand_const_reg [31:0] data;
    assign imem_array[imem_i] = data;
  end

  (* keep *)`rvformal_rand_reg [31:0] dmem_rdata_any;

  (* keep *)wire                      imem_ren;
  (* keep *)wire               [31:0] imem_raddr;
  (* keep *)wire               [31:0] imem_rdata;

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

  //
  // Stall modeling for formal verification (pipelined configs with STALL enabled)
  //
  // Solver picks stall values; assumes constrain to 0-2 consecutive cycles.
  // This models cache-like behavior where stalls are bounded.
  //
  if (`SVC_RV_PIPELINED == 1 && `SVC_RV_STALL == 1) begin : g_stall_model
    (* keep *)`rvformal_rand_reg       dmem_stall_in;
    (* keep *)`rvformal_rand_reg       imem_stall_in;

    // Bound stall duration (keeps BMC tractable)
    logic              [1:0] dmem_stall_count;
    logic              [1:0] imem_stall_count;
    logic              [1:0] any_stall_count;
    logic                    imem_pending;
    logic                    any_stall;

    assign any_stall = dmem_stall_in || imem_stall_in;

    always_ff @(posedge clock) begin
      if (reset) begin
        dmem_stall_count <= 0;
        imem_stall_count <= 0;
        any_stall_count  <= 0;
        imem_pending     <= 1'b0;
      end else begin
        if (dmem_stall_in) begin
          dmem_stall_count <= (dmem_stall_count == 2'd2) ? 2'd2 :
              dmem_stall_count + 1;
        end else begin
          dmem_stall_count <= 0;
        end

        if (imem_stall_in) begin
          imem_stall_count <= (imem_stall_count == 2'd2) ? 2'd2 :
              imem_stall_count + 1;
        end else begin
          imem_stall_count <= 0;
        end

        if (any_stall) begin
          any_stall_count <= (any_stall_count == 2'd2) ? 2'd2 :
              any_stall_count + 1;
        end else begin
          any_stall_count <= 0;
        end

        if (imem_ren) begin
          imem_pending <= 1'b1;
        end else if (!imem_stall_in) begin
          imem_pending <= 1'b0;
        end
      end
    end

    always_comb begin
      assume (dmem_stall_count < 2 || !dmem_stall_in);
      assume (imem_stall_count < 2 || !imem_stall_in);
      assume (any_stall_count < 2 || !any_stall);
      assume (!imem_stall_in || imem_pending);
    end

    // Hold dmem_rdata stable during stall
    logic [31:0] dmem_rdata_held;
    always_ff @(posedge clock) begin
      if (reset) begin
        dmem_rdata_held <= 32'h0;
      end else if (dmem_ren && !dmem_stall_in) begin
        dmem_rdata_held <= dmem_rdata_any;
      end
    end

    always_comb begin
      if (dmem_stall_in) begin
        assume (dmem_rdata_any == dmem_rdata_held);
      end
    end
  end else begin : g_stall_model
    // No stall injection - tie stall to 0
    wire dmem_stall_in = 1'b0;
    wire imem_stall_in = 1'b0;
  end

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
      .EXT_M      (`SVC_RV_EXT_M),
      .PC_REG     (`SVC_RV_PC_REG)
  ) dut (
      .clk  (clock),
      .rst_n(!reset),

      .imem_ren  (imem_ren),
      .imem_raddr(imem_raddr),
      .imem_rdata(imem_rdata),

      .dmem_ren  (dmem_ren),
      .dmem_raddr(dmem_raddr),
      .dmem_rdata(dmem_rdata),

      .dmem_we   (dmem_we),
      .dmem_waddr(dmem_waddr),
      .dmem_wdata(dmem_wdata),
      .dmem_wstrb(dmem_wstrb),

      .dmem_stall(g_stall_model.dmem_stall_in),
      .imem_stall(g_stall_model.imem_stall_in),

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
  // SRAM: Combinational read (0-cycle latency)
  // BRAM: Registered read (1-cycle latency)
  //
  wire [IMEM_AW-1:0] imem_idx;
  assign imem_idx = imem_raddr[IMEM_AW+1:2] & ((1 << IMEM_AW) - 1);

  if (`SVC_RV_MEM_TYPE == 1) begin : g_bram_timing
    reg  [31:0] imem_rdata_reg;

    // Get stall signal (0 if not pipelined)
    wire        stall = g_stall_model.imem_stall_in;

    always @(posedge clock) begin
      if (reset) begin
        // This is what the svc_rv_soc_bram does at startup
        imem_rdata_reg <= 32'h00000013;
      end else if (!stall) begin
        // Only advance when not stalled
        if (imem_ren) begin
          imem_rdata_reg <= imem_array[imem_idx];
        end
      end
      // When stalled, hold current rdata
    end

    assign imem_rdata = imem_rdata_reg;
  end else begin : g_sram_timing
    assign imem_rdata = imem_ren ? imem_array[imem_idx] : 32'hxxxxxxxx;
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
    reg  [31:0] dmem_rdata_reg;

    // Get stall signal (0 if not pipelined)
    wire        stall = g_stall_model.dmem_stall_in;

    always @(posedge clock) begin
      if (reset) begin
        dmem_rdata_reg <= 32'hxxxxxxxx;
      end else if (dmem_ren && !stall) begin
        // Data "arrives" when stall clears
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
