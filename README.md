# CMPEN 331 – Data Hazard Resolution with Forwarding in Pipelined MIPS CPU

## 🧠 Overview
This project extends the base 5-stage pipelined MIPS CPU to handle **data hazards** through **internal forwarding logic**. The focus is on verifying the correct **Write Back (WB)** stage behavior by ensuring the register file correctly receives and outputs updated data under hazard conditions.

## 🧰 Device Target
- **FPGA Board:** Zybo Board (XC7Z010-1CLG400C)

---

## 🧩 Project Objectives
1. Implement the full 5-stage MIPS pipeline including the **WB stage**.
2. Handle **data hazards** by:
   - Detecting dependency between instructions (e.g., `add`, `lw`)
   - Introducing **forwarding logic** to bypass intermediate registers
3. Modify the datapath to allow:
   - Writing back to register file in WB stage
   - Reading updated values correctly in ID stage

---

## 🛠️ Instruction Set Tested
The instruction sequence is used to validate hazard conditions and proper forwarding:
  lw $2, 00($1) # Load word x[0] into $2
  lw $3, 04($1) # Load word x[1] into $3
  lw $4, 08($1) # Load word x[2] into $4
  lw $5, 12($1) # Load word x[3] into $5
  add $6, $2, $10 # Add values (requires forwarding if $2 just loaded)


  
---

## 🧪 Testing & Initialization
### 🧾 Register File Initialization (Regfile)
First 11 registers initialized as:
$0: 00000000
$1: A00000AA
$2: 10000011
$3: 20000022
$4: 30000033
$5: 40000044
$6: 50000055
$7: 60000066
$8: 70000077
$9: 80000088
$10: 90000099

csharp
Copy
Edit



### 💾 Data Memory Initialization
Memory loaded with:
0xA00000AA
0x10000011
0x20000022
0x30000033
...


---

## 🧪 Simulation Goals
- Confirm correct values appear at `qa` and `qb` (outputs from Regfile).
- Display MEM/WB register values in waveforms.
- Show correct forwarding of values (e.g., from EXE/MEM to ID/EXE).

---

## 📊 Deliverables
- ✅ Verilog HDL code for pipelined CPU with forwarding
- ✅ Verilog testbench with `timescale 1ns/1ps`
- ✅ ModelSim/Vivado waveforms highlighting MEM/WB outputs and Regfile inputs
- ✅ I/O planning and floor planning snapshots
- ✅ Vivado synthesis schematics
- ✅ Functional bitstream
- ✅ Word document report (or LaTeX + PDF)

---

## 🧠 Key Concepts Implemented
- Internal **Forwarding Logic** (`fwda`, `fwdb`)
- **Pipeline Hazard Detection**
- **Write-Back Validation**
- Register File read-after-write correctness
- Full CPU simulation with test cases targeting hazard scenarios

---

