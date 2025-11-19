module rvfi_wrapper (
    input  wire         clock,
    input  wire         reset,

    output wire         rvfi_valid,
    output wire [63:0]  rvfi_order,
    output wire [31:0]  rvfi_insn,
    output wire [31:0]  rvfi_pc_rdata,
    output wire [31:0]  rvfi_pc_wdata,
    output wire [4:0]   rvfi_rs1_addr,
    output wire [4:0]   rvfi_rs2_addr,
    output wire [4:0]   rvfi_rd_addr,
    output wire [31:0]  rvfi_rs1_rdata,
    output wire [31:0]  rvfi_rs2_rdata,
    output wire [31:0]  rvfi_rd_wdata,
    output wire         rvfi_trap,
    output wire         rvfi_halt,
    output wire         rvfi_intr,
    output wire [1:0]   rvfi_mode,
    output wire [1:0]   rvfi_ixl,
    output wire         rvfi_mem_valid,
    output wire         rvfi_mem_instr,
    output wire [31:0]  rvfi_mem_addr,
    output wire [3:0]   rvfi_mem_rmask,
    output wire [3:0]   rvfi_mem_wmask,
    output wire [31:0]  rvfi_mem_rdata,
    output wire [31:0]  rvfi_mem_wdata
);

  //
  // Memory interface signals
  //
  // riscv-formal models architectural memory; the solver picks
  // instruction and data values that satisfy the formal properties.
  //
  (* keep *)      wire        imem_ren;
  (* keep *)      wire [31:0] imem_raddr;
  (* keep *) `rvformal_rand_reg [31:0] imem_rdata;

  (* keep *)      wire        dmem_ren;
  (* keep *)      wire [31:0] dmem_raddr;
  (* keep *) `rvformal_rand_reg [31:0] dmem_rdata;

  (* keep *)      wire        dmem_we;
  (* keep *)      wire [31:0] dmem_waddr;
  (* keep *)      wire [31:0] dmem_wdata;
  (* keep *)      wire [3:0]  dmem_wstrb;

  (* keep *)      wire        ebreak;

  svc_rv #(
    .XLEN        (32),
    .IMEM_AW     (10),
    .DMEM_AW     (10),
    .PIPELINED   (1),
    .FWD_REGFILE (1),
    .FWD         (1),
    .MEM_TYPE    (0),
    .BPRED       (0),
    .BTB_ENABLE  (0),
    .RAS_ENABLE  (0),
    .EXT_ZMMUL   (0),
    .EXT_M       (0)
  ) dut (
    .clk         (clock),
    .rst_n       (!reset),

    .imem_ren    (imem_ren),
    .imem_raddr  (imem_raddr),
    .imem_rdata  (imem_rdata),

    .dmem_ren    (dmem_ren),
    .dmem_raddr  (dmem_raddr),
    .dmem_rdata  (dmem_rdata),

    .dmem_we     (dmem_we),
    .dmem_waddr  (dmem_waddr),
    .dmem_wdata  (dmem_wdata),
    .dmem_wstrb  (dmem_wstrb),

    .rvfi_valid      (rvfi_valid),
    .rvfi_order      (rvfi_order),
    .rvfi_insn       (rvfi_insn),
    .rvfi_pc_rdata   (rvfi_pc_rdata),
    .rvfi_pc_wdata   (rvfi_pc_wdata),
    .rvfi_rs1_addr   (rvfi_rs1_addr),
    .rvfi_rs2_addr   (rvfi_rs2_addr),
    .rvfi_rd_addr    (rvfi_rd_addr),
    .rvfi_rs1_rdata  (rvfi_rs1_rdata),
    .rvfi_rs2_rdata  (rvfi_rs2_rdata),
    .rvfi_rd_wdata   (rvfi_rd_wdata),
    .rvfi_trap       (rvfi_trap),
    .rvfi_halt       (rvfi_halt),
    .rvfi_intr       (rvfi_intr),
    .rvfi_mode       (rvfi_mode),
    .rvfi_ixl        (rvfi_ixl),
    .rvfi_mem_valid  (rvfi_mem_valid),
    .rvfi_mem_instr  (rvfi_mem_instr),
    .rvfi_mem_addr   (rvfi_mem_addr),
    .rvfi_mem_rmask  (rvfi_mem_rmask),
    .rvfi_mem_wmask  (rvfi_mem_wmask),
    .rvfi_mem_rdata  (rvfi_mem_rdata),
    .rvfi_mem_wdata  (rvfi_mem_wdata),

    .ebreak      (ebreak)
  );

endmodule
