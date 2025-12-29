`include "svc.sv"
`include "svc_unused.sv"

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

`ifndef SVC_RV_PC_REG
`define SVC_RV_PC_REG 0
`endif

`ifndef SVC_RV_CACHE_ICACHE
`define SVC_RV_CACHE_ICACHE 1
`endif

`ifndef SVC_RV_CACHE_DCACHE
`define SVC_RV_CACHE_DCACHE 0
`endif

module rvfi_wrapper (
    input logic clock,
    input logic reset,
    `RVFI_OUTPUTS
    `RVFI_BUS_OUTPUTS
);

  //
  // Memory sizing
  //
  localparam int IMEM_DEPTH = 32;
  localparam int DMEM_DEPTH = 32;

  //
  // Cache sizing
  //
  localparam int CACHE_LINE_BYTES = 32;

  //
  // AXI parameters
  //
  localparam int AXI_ADDR_WIDTH = 12;
  localparam int AXI_DATA_WIDTH = 128;
  localparam int AXI_ID_WIDTH = 2;

`ifdef RISCV_FORMAL_FAIRNESS
  //
  // Locality windows (bytes)
  //
`ifndef SVC_RV_FORMAL_IMEM_LOCALITY_LINES
  `define SVC_RV_FORMAL_IMEM_LOCALITY_LINES 1
`endif

`ifndef SVC_RV_FORMAL_DMEM_LOCALITY_LINES
  `define SVC_RV_FORMAL_DMEM_LOCALITY_LINES 1
`endif

  localparam int IMEM_LOCALITY_LINES = `SVC_RV_FORMAL_IMEM_LOCALITY_LINES;
  localparam int DMEM_LOCALITY_LINES = `SVC_RV_FORMAL_DMEM_LOCALITY_LINES;

  localparam logic [AXI_ADDR_WIDTH-1:0] IMEM_LIMIT = IMEM_LOCALITY_LINES * CACHE_LINE_BYTES;
  localparam logic [AXI_ADDR_WIDTH-1:0] DMEM_BASE  = IMEM_LIMIT;
  localparam logic [AXI_ADDR_WIDTH-1:0] DMEM_LIMIT = IMEM_LIMIT + (DMEM_LOCALITY_LINES * CACHE_LINE_BYTES);
`endif

  //
  // MMIO interface
  //
  logic                                     io_ren;
  logic              [                31:0] io_raddr;
  logic              [                31:0] io_rdata;
  logic                                     io_wen;
  logic              [                31:0] io_waddr;
  logic              [                31:0] io_wdata;
  logic              [                 3:0] io_wstrb;

  //
  // AXI interface
  //
  logic                                     m_axi_arvalid;
  logic              [    AXI_ID_WIDTH-1:0] m_axi_arid;
  logic              [  AXI_ADDR_WIDTH-1:0] m_axi_araddr;
  logic              [                 7:0] m_axi_arlen;
  logic              [                 2:0] m_axi_arsize;
  logic              [                 1:0] m_axi_arburst;
  logic                                     m_axi_arready;

  logic                                     m_axi_rvalid;
  logic              [    AXI_ID_WIDTH-1:0] m_axi_rid;
  logic              [  AXI_DATA_WIDTH-1:0] m_axi_rdata;
  logic              [                 1:0] m_axi_rresp;
  logic                                     m_axi_rlast;
  logic                                     m_axi_rready;

  logic                                     m_axi_awvalid;
  logic              [    AXI_ID_WIDTH-1:0] m_axi_awid;
  logic              [  AXI_ADDR_WIDTH-1:0] m_axi_awaddr;
  logic              [                 7:0] m_axi_awlen;
  logic              [                 2:0] m_axi_awsize;
  logic              [                 1:0] m_axi_awburst;
  logic                                     m_axi_awready;

  logic                                     m_axi_wvalid;
  logic              [  AXI_DATA_WIDTH-1:0] m_axi_wdata;
  logic              [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb;
  logic                                     m_axi_wlast;
  logic                                     m_axi_wready;

  logic                                     m_axi_bvalid;
  logic              [    AXI_ID_WIDTH-1:0] m_axi_bid;
  logic              [                 1:0] m_axi_bresp;
  logic                                     m_axi_bready;

  (* keep *)`rvformal_rand_reg [                31:0] io_rdata_any;

  (* keep *)logic                                     ebreak;
  (* keep *)logic                                     trap;
  (* keep *)logic                                     rvfi_mem_valid;
  (* keep *)logic                                     rvfi_mem_instr;

  logic                                     formal_imem_ren;
  logic              [                31:0] formal_imem_raddr;
  logic                                     formal_dmem_ren;
  logic              [                31:0] formal_dmem_raddr;
  logic                                     formal_dmem_wen;
  logic              [                31:0] formal_dmem_waddr;

  assign io_rdata = io_rdata_any;

  //
  // RISC-V SoC with caching
  //
  svc_rv_soc_bram_cache #(
      .IMEM_DEPTH      (IMEM_DEPTH),
      .DMEM_DEPTH      (DMEM_DEPTH),
      .PIPELINED       (`SVC_RV_PIPELINED),
      .ICACHE_ENABLE   (`SVC_RV_CACHE_ICACHE),
      .DCACHE_ENABLE   (`SVC_RV_CACHE_DCACHE),
      .FWD_REGFILE     (`SVC_RV_FWD_REGFILE),
      .FWD             (`SVC_RV_FWD),
      .BPRED           (`SVC_RV_BPRED),
      .BTB_ENABLE      (`SVC_RV_BTB_ENABLE),
      .RAS_ENABLE      (`SVC_RV_RAS_ENABLE),
      .EXT_ZMMUL       (`SVC_RV_EXT_ZMMUL),
      .EXT_M           (`SVC_RV_EXT_M),
      .PC_REG          (`SVC_RV_PC_REG),
      .RESET_PC        (32'h0),
      .CACHE_LINE_BYTES(CACHE_LINE_BYTES),
      .AXI_ADDR_WIDTH  (AXI_ADDR_WIDTH),
      .AXI_DATA_WIDTH  (AXI_DATA_WIDTH),
      .AXI_ID_WIDTH    (AXI_ID_WIDTH)
  ) dut (
      .clk  (clock),
      .rst_n(!reset),

      .io_ren  (io_ren),
      .io_raddr(io_raddr),
      .io_rdata(io_rdata),

      .io_wen  (io_wen),
      .io_waddr(io_waddr),
      .io_wdata(io_wdata),
      .io_wstrb(io_wstrb),

      .m_axi_arvalid(m_axi_arvalid),
      .m_axi_arid   (m_axi_arid),
      .m_axi_araddr (m_axi_araddr),
      .m_axi_arlen  (m_axi_arlen),
      .m_axi_arsize (m_axi_arsize),
      .m_axi_arburst(m_axi_arburst),
      .m_axi_arready(m_axi_arready),

      .m_axi_rvalid(m_axi_rvalid),
      .m_axi_rid   (m_axi_rid),
      .m_axi_rdata (m_axi_rdata),
      .m_axi_rresp (m_axi_rresp),
      .m_axi_rlast (m_axi_rlast),
      .m_axi_rready(m_axi_rready),

      .m_axi_awvalid(m_axi_awvalid),
      .m_axi_awid   (m_axi_awid),
      .m_axi_awaddr (m_axi_awaddr),
      .m_axi_awlen  (m_axi_awlen),
      .m_axi_awsize (m_axi_awsize),
      .m_axi_awburst(m_axi_awburst),
      .m_axi_awready(m_axi_awready),

      .m_axi_wvalid(m_axi_wvalid),
      .m_axi_wdata (m_axi_wdata),
      .m_axi_wstrb (m_axi_wstrb),
      .m_axi_wlast (m_axi_wlast),
      .m_axi_wready(m_axi_wready),

      .m_axi_bvalid(m_axi_bvalid),
      .m_axi_bid   (m_axi_bid),
      .m_axi_bresp (m_axi_bresp),
      .m_axi_bready(m_axi_bready),

      .rvfi_valid    (rvfi_valid),
      .rvfi_order    (rvfi_order),
      .rvfi_insn     (rvfi_insn),
      .rvfi_pc_rdata (rvfi_pc_rdata),
      .rvfi_pc_wdata (rvfi_pc_wdata),
      .rvfi_rs1_addr (rvfi_rs1_addr),
      .rvfi_rs2_addr (rvfi_rs2_addr),
      .rvfi_rd_addr  (rvfi_rd_addr),
      .rvfi_rs1_rdata(rvfi_rs1_rdata),
      .rvfi_rs2_rdata(rvfi_rs2_rdata),
      .rvfi_rd_wdata (rvfi_rd_wdata),
      .rvfi_trap     (rvfi_trap),
      .rvfi_halt     (rvfi_halt),
      .rvfi_intr     (rvfi_intr),
      .rvfi_mode     (rvfi_mode),
      .rvfi_ixl      (rvfi_ixl),
      .rvfi_mem_valid(rvfi_mem_valid),
      .rvfi_mem_instr(rvfi_mem_instr),
      .rvfi_mem_addr (rvfi_mem_addr),
      .rvfi_mem_rmask(rvfi_mem_rmask),
      .rvfi_mem_wmask(rvfi_mem_wmask),
      .rvfi_mem_rdata(rvfi_mem_rdata),
      .rvfi_mem_wdata(rvfi_mem_wdata),

      .formal_imem_ren  (formal_imem_ren),
      .formal_imem_raddr(formal_imem_raddr),
      .formal_dmem_ren  (formal_dmem_ren),
      .formal_dmem_raddr(formal_dmem_raddr),
      .formal_dmem_wen  (formal_dmem_wen),
      .formal_dmem_waddr(formal_dmem_waddr),

      .ebreak(ebreak),
      .trap  (trap)
  );

  //
  // AXI backing memory
  //
  svc_axi_mem #(
      .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
      .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
      .AXI_ID_WIDTH  (AXI_ID_WIDTH)
  ) mem (
      .clk  (clock),
      .rst_n(!reset),

      .s_axi_awvalid(m_axi_awvalid),
      .s_axi_awid   (m_axi_awid),
      .s_axi_awaddr (m_axi_awaddr),
      .s_axi_awlen  (m_axi_awlen),
      .s_axi_awsize (m_axi_awsize),
      .s_axi_awburst(m_axi_awburst),
      .s_axi_awready(m_axi_awready),

      .s_axi_wvalid(m_axi_wvalid),
      .s_axi_wdata (m_axi_wdata),
      .s_axi_wstrb (m_axi_wstrb),
      .s_axi_wlast (m_axi_wlast),
      .s_axi_wready(m_axi_wready),

      .s_axi_bvalid(m_axi_bvalid),
      .s_axi_bid   (m_axi_bid),
      .s_axi_bresp (m_axi_bresp),
      .s_axi_bready(m_axi_bready),

      .s_axi_arvalid(m_axi_arvalid),
      .s_axi_arid   (m_axi_arid),
      .s_axi_araddr (m_axi_araddr),
      .s_axi_arlen  (m_axi_arlen),
      .s_axi_arsize (m_axi_arsize),
      .s_axi_arburst(m_axi_arburst),
      .s_axi_arready(m_axi_arready),

      .s_axi_rvalid(m_axi_rvalid),
      .s_axi_rid   (m_axi_rid),
      .s_axi_rdata (m_axi_rdata),
      .s_axi_rresp (m_axi_rresp),
      .s_axi_rlast (m_axi_rlast),
      .s_axi_rready(m_axi_rready)
  );

  //
  // Bound AXI response latency to keep liveness checks tractable
  //
  localparam int RD_WAIT_MAX = (AXI_DATA_WIDTH / 32);
  localparam int WR_WAIT_MAX = 2;

  logic                               rd_pending;
  logic                               wr_pending;
  logic [$clog2(RD_WAIT_MAX + 1)-1:0] rd_wait_count;
  logic [$clog2(WR_WAIT_MAX + 1)-1:0] wr_wait_count;

  always_ff @(posedge clock) begin
    if (reset) begin
      rd_pending    <= 1'b0;
      wr_pending    <= 1'b0;
      rd_wait_count <= '0;
      wr_wait_count <= '0;
    end else begin
      if (m_axi_arvalid && m_axi_arready) begin
        rd_pending <= 1'b1;
      end else if (m_axi_rvalid && m_axi_rready && m_axi_rlast) begin
        rd_pending <= 1'b0;
      end

      if (m_axi_awvalid && m_axi_awready) begin
        wr_pending <= 1'b1;
      end else if (m_axi_bvalid && m_axi_bready) begin
        wr_pending <= 1'b0;
      end

      if (rd_pending && !(m_axi_rvalid && m_axi_rready)) begin
        if (rd_wait_count != RD_WAIT_MAX[$bits(rd_wait_count)-1:0]) begin
          rd_wait_count <= rd_wait_count + 1'b1;
        end
      end else begin
        rd_wait_count <= '0;
      end

      if (wr_pending && !(m_axi_bvalid && m_axi_bready)) begin
        if (wr_wait_count != WR_WAIT_MAX[$bits(wr_wait_count)-1:0]) begin
          wr_wait_count <= wr_wait_count + 1'b1;
        end
      end else begin
        wr_wait_count <= '0;
      end
    end
  end

  always_comb begin
    if (!reset) begin
      if (rd_pending) begin
        assume (rd_wait_count < RD_WAIT_MAX);
      end
      if (wr_pending) begin
        assume (wr_wait_count < WR_WAIT_MAX);
      end
    end
  end

`ifdef RISCV_FORMAL_FAIRNESS
  //
  // Instruction consistency
  //
  // Disallow AXI writes into the instruction region for liveness checks.
  //
  always_comb begin
    if (!reset) begin
      if (m_axi_awvalid && m_axi_awready) begin
        assume (m_axi_awaddr >= IMEM_LIMIT);
      end
    end
  end

  //
  // No traps for liveness checks
  //
  always_comb begin
    if (!reset && rvfi_valid) begin
      assume (!rvfi_trap);
    end
  end

  //
  // Locality assumptions for tractable liveness checks
  //
  // Without these, the solver can (legally) construct address patterns that
  // intentionally thrash the caches and stretch liveness latencies.
  //
  always_comb begin
    if (!reset) begin
      if (formal_imem_ren) begin
        assume (formal_imem_raddr[31:AXI_ADDR_WIDTH] == '0);
        assume (formal_imem_raddr[AXI_ADDR_WIDTH-1:0] < IMEM_LIMIT);
      end

      if (formal_dmem_ren) begin
        assume (formal_dmem_raddr[31:AXI_ADDR_WIDTH] == '0);
        assume (formal_dmem_raddr[AXI_ADDR_WIDTH-1:0] >= DMEM_BASE);
        assume (formal_dmem_raddr[AXI_ADDR_WIDTH-1:0] < DMEM_LIMIT);
      end

      if (formal_dmem_wen) begin
        assume (formal_dmem_waddr[31:AXI_ADDR_WIDTH] == '0);
        assume (formal_dmem_waddr[AXI_ADDR_WIDTH-1:0] >= DMEM_BASE);
        assume (formal_dmem_waddr[AXI_ADDR_WIDTH-1:0] < DMEM_LIMIT);
      end
    end

    if (!reset && rvfi_valid) begin
      assume (rvfi_pc_rdata[31:AXI_ADDR_WIDTH] == '0);
      assume (rvfi_pc_wdata[31:AXI_ADDR_WIDTH] == '0);

      assume (rvfi_pc_rdata[AXI_ADDR_WIDTH-1:0] < IMEM_LIMIT);
      assume (rvfi_pc_wdata[AXI_ADDR_WIDTH-1:0] < IMEM_LIMIT);
    end
  end

  always_comb begin
    if (!reset && rvfi_valid) begin
      if ((rvfi_mem_rmask != '0) || (rvfi_mem_wmask != '0)) begin
        assume (rvfi_mem_addr[31:AXI_ADDR_WIDTH] == '0);

        assume (rvfi_mem_addr[AXI_ADDR_WIDTH-1:0] >= DMEM_BASE);
        assume (rvfi_mem_addr[AXI_ADDR_WIDTH-1:0] < DMEM_LIMIT);
      end
    end
  end
`endif

`ifdef RISCV_FORMAL_BUS
  assign rvfi_bus_valid = '0;
  assign rvfi_bus_insn  = '0;
  assign rvfi_bus_data  = '0;
  assign rvfi_bus_fault = '0;
  assign rvfi_bus_addr  = '0;
  assign rvfi_bus_rmask = '0;
  assign rvfi_bus_wmask = '0;
  assign rvfi_bus_rdata = '0;
  assign rvfi_bus_wdata = '0;
`endif

  `SVC_UNUSED({io_ren, io_raddr, io_wen, io_waddr, io_wdata, io_wstrb});
  `SVC_UNUSED({ebreak, trap, rvfi_mem_valid, rvfi_mem_instr});

endmodule
