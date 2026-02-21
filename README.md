# Gatling-V

Gatling-V is an FPGA-optimized implementation of the RISC-V Vector (RVV) extension written in SystemVerilog.

Top-level module:
gatlingV.sv

## Overview

Gatling-V can concurrently execute up to four vector instructions using a multi-banked vector register file architecture optimized for FPGA BRAMs.

The key innovation is constructing a multi-ported vector register file from simple dual-port (1R1W) BRAMs instead of traditional FPGA multi-port techniques. This:

- Reduces LUT and BRAM usage
- Improves clock frequency
- Supports multiple overlapping in-flight instructions
- Enables vector chaining (convoying)

## Paper

This repository corresponds to:

Gatling-V: An FPGA-Optimized RISC-V Vector Processor  
https://doi.org/10.1145/3748173.3779184
