#!/usr/bin/env python3
#
#  Modified from disasm.py from the other cores, but decodes from imem instead
#  of the rvfi interface.
#
#  Copyright (C) 2020  Claire Xenia Wolf <claire@yosyshq.com>
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import struct
import subprocess
from sys import argv

from Verilog_VCD.Verilog_VCD import parse_vcd

imem_ren = None
imem_raddr = None
imem_rdata = None

for netinfo in parse_vcd(argv[1]).values():
    for net in netinfo["nets"]:
        # print(net["hier"], net["name"])
        if net["hier"] == "rvfi_testbench.wrapper" and net["name"] == "imem_ren":
            imem_ren = netinfo["tv"]
        if net["hier"] == "rvfi_testbench.wrapper" and net["name"] == "imem_raddr":
            imem_raddr = netinfo["tv"]
        if net["hier"] == "rvfi_testbench.wrapper" and net["name"] == "imem_rdata":
            imem_rdata = netinfo["tv"]

assert imem_ren is not None, "imem_ren not found in VCD"
assert imem_raddr is not None, "imem_raddr not found in VCD"
assert imem_rdata is not None, "imem_rdata not found in VCD"
assert len(imem_ren) == len(imem_raddr)
assert len(imem_ren) == len(imem_rdata)

#
# Collect unique (addr, insn) pairs from instruction fetches
#
prog = dict()

for tv_ren, tv_addr, tv_data in zip(imem_ren, imem_raddr, imem_rdata):
    if (
        tv_ren[1] == "1"
        and "x" not in tv_addr[1].lower()
        and "x" not in tv_data[1].lower()
    ):
        addr = int(tv_addr[1], 2)
        insn = int(tv_data[1], 2)
        prog[addr] = insn

#
# Disassemble each instruction individually
#
for addr, insn in sorted(prog.items()):
    with open("disasm.bin", "wb") as f:
        if insn & 3 != 3 and insn & 0xFFFF0000 == 0:
            f.write(struct.pack("<H", insn))
        else:
            f.write(struct.pack("<I", insn))
    result = subprocess.run(
        [
            "riscv64-none-elf-objdump",
            "-D",
            "-b",
            "binary",
            "-m",
            "riscv:rv32",
            "-M",
            "numeric,no-aliases",
            f"--adjust-vma=0x{addr:08x}",
            "disasm.bin",
        ],
        capture_output=True,
        text=True,
    )
    #
    # Extract the disassembly line (starts with whitespace, contains tab)
    #
    for line in result.stdout.splitlines():
        if line.startswith(" ") and "\t" in line:
            print(line.strip())
            break
