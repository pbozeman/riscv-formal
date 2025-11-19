`define RISCV_FORMAL
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_NRET 1

//
// ISA
//
`define RISCV_FORMAL_RV32I

//
// Enable these later when RVFI wiring + checks are ready:
//
// `define RISCV_FORMAL_RV32M
// `define RISCV_FORMAL_RV32C

//
// Only aligned memory accesses for now
//
`define RISCV_FORMAL_ALIGNED_MEM
