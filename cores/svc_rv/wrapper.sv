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
  // Memory interface signals
  //
  // riscv-formal models architectural memory; the solver picks
  // instruction and data values that satisfy the formal properties.
  //
  (* keep *)wire                      imem_ren;
  (* keep *)wire               [31:0] imem_raddr;
  (* keep *)`rvformal_rand_reg [31:0] imem_rdata;

  (* keep *)wire                      dmem_ren;
  (* keep *)wire               [31:0] dmem_raddr;
  (* keep *)`rvformal_rand_reg [31:0] dmem_rdata;

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

`ifdef RISCV_FORMAL_CSR_MCYCLE
      .rvfi_csr_mcycle_rmask(rvfi_csr_mcycle_rmask),
      .rvfi_csr_mcycle_wmask(rvfi_csr_mcycle_wmask),
      .rvfi_csr_mcycle_rdata(rvfi_csr_mcycle_rdata),
      .rvfi_csr_mcycle_wdata(rvfi_csr_mcycle_wdata),
`endif

`ifdef RISCV_FORMAL_CSR_MINSTRET
      .rvfi_csr_minstret_rmask(rvfi_csr_minstret_rmask),
      .rvfi_csr_minstret_wmask(rvfi_csr_minstret_wmask),
      .rvfi_csr_minstret_rdata(rvfi_csr_minstret_rdata),
      .rvfi_csr_minstret_wdata(rvfi_csr_minstret_wdata)
`endif
  );

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
