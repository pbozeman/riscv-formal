`define RISCV_FORMAL
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_NRET 1

//
// ISA
//
// The ISA (rv32i, rv32im, etc.) is specified in the .cfg file [options] section.
// riscv-formal's genchecks.py automatically defines the appropriate ISA defines
// (RISCV_FORMAL_RV32I, RISCV_FORMAL_RV32M, etc.) based on that.
//

//
// Enable these later when RVFI wiring + checks are ready:
//
// `define RISCV_FORMAL_RV32C

//
// Only aligned memory accesses for now
//
`define RISCV_FORMAL_ALIGNED_MEM
