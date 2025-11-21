module rvfi_wrapper (
    input  wire         clock,
    input  wire         reset,
    `RVFI_OUTPUTS
    `RVFI_BUS_OUTPUTS
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
  (* keep *)      wire        trap;
  (* keep *)      wire        rvfi_mem_valid;
  (* keep *)      wire        rvfi_mem_instr;

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

    .ebreak      (ebreak),
    .trap        (trap),

    .rvfi_valid      (rvfi_valid),
    .rvfi_order      (rvfi_order),
    .rvfi_insn       (rvfi_insn),
    .rvfi_trap       (rvfi_trap),
    .rvfi_halt       (rvfi_halt),
    .rvfi_intr       (rvfi_intr),
    .rvfi_mode       (rvfi_mode),
    .rvfi_ixl        (rvfi_ixl),
    .rvfi_rs1_addr   (rvfi_rs1_addr),
    .rvfi_rs2_addr   (rvfi_rs2_addr),
    .rvfi_rs1_rdata  (rvfi_rs1_rdata),
    .rvfi_rs2_rdata  (rvfi_rs2_rdata),
    .rvfi_rd_addr    (rvfi_rd_addr),
    .rvfi_rd_wdata   (rvfi_rd_wdata),
    .rvfi_pc_rdata   (rvfi_pc_rdata),
    .rvfi_pc_wdata   (rvfi_pc_wdata),
    .rvfi_mem_addr   (rvfi_mem_addr),
    .rvfi_mem_rmask  (rvfi_mem_rmask),
    .rvfi_mem_wmask  (rvfi_mem_wmask),
    .rvfi_mem_rdata  (rvfi_mem_rdata),
    .rvfi_mem_wdata  (rvfi_mem_wdata),
    .rvfi_mem_valid  (rvfi_mem_valid),
    .rvfi_mem_instr  (rvfi_mem_instr),

`ifdef RISCV_FORMAL_CSR_MCYCLE
    .rvfi_csr_mcycle_rmask (rvfi_csr_mcycle_rmask),
    .rvfi_csr_mcycle_wmask (rvfi_csr_mcycle_wmask),
    .rvfi_csr_mcycle_rdata (rvfi_csr_mcycle_rdata),
    .rvfi_csr_mcycle_wdata (rvfi_csr_mcycle_wdata),
`endif

`ifdef RISCV_FORMAL_CSR_MINSTRET
    .rvfi_csr_minstret_rmask (rvfi_csr_minstret_rmask),
    .rvfi_csr_minstret_wmask (rvfi_csr_minstret_wmask),
    .rvfi_csr_minstret_rdata (rvfi_csr_minstret_rdata),
    .rvfi_csr_minstret_wdata (rvfi_csr_minstret_wdata)
`endif
  );

`ifdef RISCV_FORMAL_BUS

`define RISCV_FORMAL_CHANNEL_SIGNAL(channels, width, name) \
  (* keep *) reg [(width) - 1:0] imem_``name; assign rvfi_``name[0 * (width) +: (width)] = imem_``name;
`RVFI_BUS_SIGNALS
`undef RISCV_FORMAL_CHANNEL_SIGNAL

  //
  // Instruction memory bus interface
  //
  always_comb begin
    imem_bus_addr  = imem_raddr;
    imem_bus_insn  = 1'b1;
    imem_bus_data  = 1'b0;
    imem_bus_rmask = imem_ren ? 4'b1111 : 4'b0000;
    imem_bus_wmask = 4'b0000;
    imem_bus_rdata = imem_rdata;
    imem_bus_wdata = 32'h0;
    imem_bus_fault = 1'b0;
    imem_bus_valid = imem_ren;
  end

`endif

endmodule
