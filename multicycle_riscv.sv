//==============================================================
// ELE432 - Lab 3 : Multicycle RISC-V Processor
// Reference  : Harris & Harris, RISC-V Edition, Ch. 7
// Top module : top(clk, reset, WriteData, DataAdr, MemWrite)
//==============================================================

//---------------- TOP : RISC-V + unified memory ---------------
module top(input  logic        clk, reset,
           output logic [31:0] WriteData, DataAdr,
           output logic        MemWrite);

    logic [31:0] ReadData;

    // processor (controller + datapath)
    riscvmulti rvmulti(clk, reset, MemWrite, DataAdr, WriteData, ReadData);

    // unified instruction & data memory
    mem        memunit(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule


//---------------- Unified instruction / data memory ----------
module mem(input  logic        clk, we,
           input  logic [31:0] a,  wd,
           output logic [31:0] rd);

    logic [31:0] RAM[63:0];

    initial
        $readmemh("memfile.txt", RAM);

    assign rd = RAM[a[31:2]];                 // word aligned read

    always_ff @(posedge clk)
        if (we) RAM[a[31:2]] <= wd;
endmodule


//---------------- RISC-V multicycle processor ----------------
module riscvmulti(input  logic        clk, reset,
                  output logic        MemWrite,
                  output logic [31:0] Adr, WriteData,
                  input  logic [31:0] ReadData);

    logic [31:0] Instr;
    logic        Zero;
    logic        PCWrite, RegWrite, IRWrite, AdrSrc;
    logic [1:0]  ResultSrc, ALUSrcA, ALUSrcB, ImmSrc;
    logic [2:0]  ALUControl;

    controller c(clk, reset,
                 Instr[6:0], Instr[14:12], Instr[30], Zero,
                 ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc,
                 ALUControl, IRWrite, PCWrite, RegWrite, MemWrite);

    datapath   dp(clk, reset, ResultSrc, ALUSrcA, ALUSrcB, AdrSrc,
                  ImmSrc, ALUControl, IRWrite, PCWrite, RegWrite,
                  ReadData, Adr, WriteData, Instr, Zero);
endmodule


//---------------- Controller (FSM + ALU dec + Imm dec) -------
module controller(input  logic       clk, reset,
                  input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       Zero,
                  output logic [1:0] ImmSrc,
                  output logic [1:0] ALUSrcA, ALUSrcB,
                  output logic [1:0] ResultSrc,
                  output logic       AdrSrc,
                  output logic [2:0] ALUControl,
                  output logic       IRWrite, PCWrite,
                  output logic       RegWrite, MemWrite);

    logic [1:0] ALUOp;
    logic       Branch, PCUpdate;

    mainfsm fsm(clk, reset, op,
                ALUSrcA, ALUSrcB, ResultSrc, AdrSrc,
                IRWrite, PCUpdate, Branch,
                RegWrite, MemWrite, ALUOp);

    aludec  ad (op[5], funct3, funct7b5, ALUOp, ALUControl);

    instrdec id(op, ImmSrc);

    // PC is updated when an unconditional update occurs OR a taken branch
    assign PCWrite = (Branch & Zero) | PCUpdate;
endmodule


//---------------- Main FSM -----------------------------------
module mainfsm(input  logic        clk, reset,
               input  logic [6:0]  op,
               output logic [1:0]  ALUSrcA, ALUSrcB,
               output logic [1:0]  ResultSrc,
               output logic        AdrSrc,
               output logic        IRWrite, PCUpdate, Branch,
               output logic        RegWrite, MemWrite,
               output logic [1:0]  ALUOp);

    typedef enum logic [3:0] {
        S0_FETCH    = 4'd0,
        S1_DECODE   = 4'd1,
        S2_MEMADR   = 4'd2,
        S3_MEMREAD  = 4'd3,
        S4_MEMWB    = 4'd4,
        S5_MEMWRITE = 4'd5,
        S6_EXECUTER = 4'd6,
        S7_ALUWB    = 4'd7,
        S8_EXECUTEI = 4'd8,
        S9_JAL      = 4'd9,
        S10_BEQ     = 4'd10
    } statetype;

    statetype state, nextstate;

    // state register
    always_ff @(posedge clk, posedge reset)
        if (reset) state <= S0_FETCH;
        else       state <= nextstate;

    // next state logic
    always_comb
        case (state)
            S0_FETCH: nextstate = S1_DECODE;
            S1_DECODE:
                case (op)
                    7'b0000011: nextstate = S2_MEMADR;   // lw
                    7'b0100011: nextstate = S2_MEMADR;   // sw
                    7'b0110011: nextstate = S6_EXECUTER; // R-type
                    7'b0010011: nextstate = S8_EXECUTEI; // I-type ALU
                    7'b1101111: nextstate = S9_JAL;      // jal
                    7'b1100011: nextstate = S10_BEQ;     // beq
                    default:    nextstate = S0_FETCH;
                endcase
            S2_MEMADR:
                case (op)
                    7'b0000011: nextstate = S3_MEMREAD;  // lw
                    7'b0100011: nextstate = S5_MEMWRITE; // sw
                    default:    nextstate = S0_FETCH;
                endcase
            S3_MEMREAD : nextstate = S4_MEMWB;
            S4_MEMWB   : nextstate = S0_FETCH;
            S5_MEMWRITE: nextstate = S0_FETCH;
            S6_EXECUTER: nextstate = S7_ALUWB;
            S7_ALUWB   : nextstate = S0_FETCH;
            S8_EXECUTEI: nextstate = S7_ALUWB;
            S9_JAL     : nextstate = S7_ALUWB;
            S10_BEQ    : nextstate = S0_FETCH;
            default    : nextstate = S0_FETCH;
        endcase

    // output logic
    always_comb begin
        // safe defaults
        ALUSrcA   = 2'b00;  ALUSrcB   = 2'b00; ResultSrc = 2'b00;
        AdrSrc    = 1'b0;   IRWrite   = 1'b0;
        PCUpdate  = 1'b0;   Branch    = 1'b0;
        RegWrite  = 1'b0;   MemWrite  = 1'b0;
        ALUOp     = 2'b00;

        case (state)
            // IR <- Mem[PC];   OldPC <- PC;   PC <- PC + 4
            S0_FETCH: begin
                AdrSrc    = 1'b0;
                IRWrite   = 1'b1;
                ALUSrcA   = 2'b00;   // PC
                ALUSrcB   = 2'b10;   // 4
                ALUOp     = 2'b00;   // add
                ResultSrc = 2'b10;   // ALUResult
                PCUpdate  = 1'b1;
            end

            // ALUOut <- OldPC + ImmExt
            S1_DECODE: begin
                ALUSrcA = 2'b01;     // OldPC
                ALUSrcB = 2'b01;     // ImmExt
                ALUOp   = 2'b00;     // add
            end

            // ALUOut <- A + ImmExt   (memory address)
            S2_MEMADR: begin
                ALUSrcA = 2'b10;     // A
                ALUSrcB = 2'b01;     // ImmExt
                ALUOp   = 2'b00;     // add
            end

            // Read memory at ALUOut, latch into Data
            S3_MEMREAD: begin
                ResultSrc = 2'b00;   // ALUOut -> Result -> Adr
                AdrSrc    = 1'b1;
            end

            // rd <- Data   (load writeback)
            S4_MEMWB: begin
                ResultSrc = 2'b01;   // Data
                RegWrite  = 1'b1;
            end

            // Mem[ALUOut] <- WriteData (rs2)
            S5_MEMWRITE: begin
                ResultSrc = 2'b00;   // ALUOut -> Result -> Adr
                AdrSrc    = 1'b1;
                MemWrite  = 1'b1;
            end

            // ALUOut <- A op B
            S6_EXECUTER: begin
                ALUSrcA = 2'b10;     // A
                ALUSrcB = 2'b00;     // WriteData (B)
                ALUOp   = 2'b10;     // R-type/I-type funct
            end

            // rd <- ALUOut
            S7_ALUWB: begin
                ResultSrc = 2'b00;   // ALUOut
                RegWrite  = 1'b1;
            end

            // ALUOut <- A op ImmExt
            S8_EXECUTEI: begin
                ALUSrcA = 2'b10;     // A
                ALUSrcB = 2'b01;     // ImmExt
                ALUOp   = 2'b10;
            end

            // PC <- ALUOut(=target);  ALUOut <- OldPC + 4 (return addr)
            S9_JAL: begin
                ALUSrcA   = 2'b01;   // OldPC
                ALUSrcB   = 2'b10;   // 4
                ALUOp     = 2'b00;
                ResultSrc = 2'b00;   // ALUOut(prev) = target
                PCUpdate  = 1'b1;
            end

            // if (A == B) PC <- ALUOut(=target)
            S10_BEQ: begin
                ALUSrcA   = 2'b10;   // A
                ALUSrcB   = 2'b00;   // WriteData (B)
                ALUOp     = 2'b01;   // sub
                ResultSrc = 2'b00;   // ALUOut(prev) = target
                Branch    = 1'b1;
            end

            default: ;
        endcase
    end
endmodule


//---------------- ALU decoder --------------------------------
module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5,
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);

    logic RtypeSub;
    assign RtypeSub = funct7b5 & opb5; // TRUE for R-type sub

    always_comb
        case (ALUOp)
            2'b00: ALUControl = 3'b000;       // add  (lw, sw, addr calc)
            2'b01: ALUControl = 3'b001;       // sub  (beq)
            default: case (funct3)
                3'b000: ALUControl = RtypeSub ? 3'b001  // sub
                                              : 3'b000; // add / addi
                3'b010: ALUControl = 3'b101;             // slt / slti
                3'b110: ALUControl = 3'b011;             // or  / ori
                3'b111: ALUControl = 3'b010;             // and / andi
                default: ALUControl = 3'b000;
            endcase
        endcase
endmodule


//---------------- Instruction (Imm) decoder ------------------
module instrdec(input  logic [6:0] op,
                output logic [1:0] ImmSrc);
    always_comb
        case (op)
            7'b0110011: ImmSrc = 2'b00; // R-type (don't care)
            7'b0010011: ImmSrc = 2'b00; // I-type ALU
            7'b0000011: ImmSrc = 2'b00; // lw
            7'b0100011: ImmSrc = 2'b01; // sw  (S-type)
            7'b1100011: ImmSrc = 2'b10; // beq (B-type)
            7'b1101111: ImmSrc = 2'b11; // jal (J-type)
            default:    ImmSrc = 2'b00;
        endcase
endmodule


//---------------- Datapath -----------------------------------
module datapath(input  logic        clk, reset,
                input  logic [1:0]  ResultSrc,
                input  logic [1:0]  ALUSrcA, ALUSrcB,
                input  logic        AdrSrc,
                input  logic [1:0]  ImmSrc,
                input  logic [2:0]  ALUControl,
                input  logic        IRWrite, PCWrite,
                input  logic        RegWrite,
                input  logic [31:0] ReadData,
                output logic [31:0] Adr, WriteData,
                output logic [31:0] Instr,
                output logic        Zero);

    logic [31:0] PC, OldPC;
    logic [31:0] ImmExt;
    logic [31:0] RD1, RD2;
    logic [31:0] A;
    logic [31:0] SrcA, SrcB;
    logic [31:0] ALUResult, ALUOut;
    logic [31:0] Data;
    logic [31:0] Result;

    // PC
    flopenr #(32) pcreg   (clk, reset, PCWrite, Result, PC);

    // Adr mux: PC for fetch, Result for memory access
    mux2    #(32) adrmux  (PC, Result, AdrSrc, Adr);

    // Instruction Reg + OldPC (both enabled by IRWrite)
    flopenr #(32) irreg   (clk, reset, IRWrite, ReadData, Instr);
    flopenr #(32) oldpcreg(clk, reset, IRWrite, PC,       OldPC);

    // Data register
    flopr   #(32) datareg (clk, reset, ReadData, Data);

    // Register file
    regfile rf(clk, RegWrite,
               Instr[19:15], Instr[24:20], Instr[11:7],
               Result, RD1, RD2);

    // A and B (=WriteData) registers
    flopr   #(32) areg    (clk, reset, RD1, A);
    flopr   #(32) bregwd  (clk, reset, RD2, WriteData);

    // immediate extender
    extend  ext(Instr[31:7], ImmSrc, ImmExt);

    // ALU operand muxes
    mux3    #(32) srcamux (PC, OldPC, A,         ALUSrcA, SrcA);
    mux3    #(32) srcbmux (WriteData, ImmExt, 32'd4, ALUSrcB, SrcB);

    // ALU + ALUOut register
    alu     alunit(SrcA, SrcB, ALUControl, ALUResult, Zero);
    flopr   #(32) aluoutreg(clk, reset, ALUResult, ALUOut);

    // Result mux
    mux3    #(32) resmux  (ALUOut, Data, ALUResult, ResultSrc, Result);
endmodule


//---------------- Building blocks ----------------------------
module regfile(input  logic        clk,
               input  logic        we3,
               input  logic [4:0]  a1, a2, a3,
               input  logic [31:0] wd3,
               output logic [31:0] rd1, rd2);

    logic [31:0] rf[31:0];

    always_ff @(posedge clk)
        if (we3) rf[a3] <= wd3;

    assign rd1 = (a1 != 0) ? rf[a1] : 32'b0;
    assign rd2 = (a2 != 0) ? rf[a2] : 32'b0;
endmodule


module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
    always_comb
        case (immsrc)
            2'b00: immext = {{20{instr[31]}}, instr[31:20]};                                 // I
            2'b01: immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};                    // S
            2'b10: immext = {{20{instr[31]}}, instr[7],     instr[30:25], instr[11:8], 1'b0}; // B
            2'b11: immext = {{12{instr[31]}}, instr[19:12], instr[20],    instr[30:21], 1'b0}; // J
            default: immext = 32'bx;
        endcase
endmodule


module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

    logic [31:0] condinvb, sum;
    logic        v;
    logic        isAddSub;

    assign condinvb = alucontrol[0] ? ~b : b;
    assign sum      = a + condinvb + {31'b0, alucontrol[0]};
    assign isAddSub = (~alucontrol[2] & ~alucontrol[1]) |
                      (~alucontrol[1] &  alucontrol[0]);

    always_comb
        case (alucontrol)
            3'b000: result = sum;                       // add
            3'b001: result = sum;                       // sub
            3'b010: result = a & b;                     // and
            3'b011: result = a | b;                     // or
            3'b101: result = {31'b0, sum[31] ^ v};      // slt
            default: result = 32'bx;
        endcase

    assign zero = (result == 32'b0);
    assign v    = ~(alucontrol[0] ^ a[31] ^ b[31]) &
                   (a[31] ^ sum[31]) & isAddSub;
endmodule


module flopr  #(parameter WIDTH = 8)
              (input  logic              clk, reset,
               input  logic [WIDTH-1:0]  d,
               output logic [WIDTH-1:0]  q);
    always_ff @(posedge clk, posedge reset)
        if (reset) q <= '0;
        else       q <= d;
endmodule


module flopenr #(parameter WIDTH = 8)
               (input  logic              clk, reset, en,
                input  logic [WIDTH-1:0]  d,
                output logic [WIDTH-1:0]  q);
    always_ff @(posedge clk, posedge reset)
        if (reset)   q <= '0;
        else if (en) q <= d;
endmodule


module mux2  #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1,
              input  logic             s,
              output logic [WIDTH-1:0] y);
    assign y = s ? d1 : d0;
endmodule


module mux3  #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s,
              output logic [WIDTH-1:0] y);
    assign y = s[1] ? d2 : (s[0] ? d1 : d0);
endmodule
