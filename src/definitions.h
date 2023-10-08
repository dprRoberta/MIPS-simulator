
////////////////////////////////////////////////////////
/// Struct Definitions
////////////////////////////////////////////////////////
struct memory_stats_t{
    uint64_t lw_total;
    uint64_t sw_total;
};


struct instr_meta{
    uint32_t instr;
    int immediate; // sign extended
    int opcode;
    uint8_t reg_21_25;
    uint8_t reg_16_20;
    uint8_t reg_11_15;
    uint8_t  type; //
    int function;
    int jmp_offset;
};

struct ctrl_signals {
    // 1-bit signals
    int RegDst;
    int RegWrite;
    int ALUSrcA;

    int MemRead;
    int MemWrite;
    int MemtoReg;

    int IorD;
    int IRWrite;

    int PCWrite;
    int PCWriteCond;

    //2-bit signals
    int ALUOp;
    int ALUSrcB;
    int PCSource;

};

struct pipe_regs {
    int pc;
    int IR;
    int A;
    int B;
    int ALUOut;
    int MDR;
};


////////////////////////////////////////////////////////
/// Helper Defines & Functions
////////////////////////////////////////////////////////


//Offsets and sizes
#define OPCODE_OFFSET 26
#define OPCODE_SIZE 6 // 6 bits to encode an opcode (26-31)
#define REGISTER_ID_SIZE 5 // 5 bits to encode an opcode
#define IMMEDIATE_OFFSET 0


//Instruction types
#define MEM_TYPE 0
#define R_TYPE 1
#define BRANCH_TYPE 2
#define JUMP_TYPE 3
#define I_TYPE 4
#define EOP_TYPE 6
#define JR_TYPE 7

// OPCODES
#define R_INST 0 // 000000
#define ADD 32    // 100000
#define ADDI 8    // 001000
#define LW 35     // 100011
#define SW 43     // 101011
#define BEQ  4    // 000100
#define J 2       // 000010
#define SLT 42    // 101010
#define EOP 63    // 111111


// FSM STATES
#define INSTR_FETCH 0
#define DECODE 1
#define MEM_ADDR_CALC 2 
#define MEM_LD_READ 3 
#define WB_STEP 4
#define MEM_ST_WRITE 5 
#define EXEC 6
#define R_TYPE_COMPL 7
#define BRANCH_COMPL 8
#define JUMP_COMPL 9
#define EXIT_STATE 10
#define I_TYPE_EXEC 11
#define I_TYPE_COMPL 12



struct architectural_state {
    int state;
    uint64_t clock_cycle;
    struct ctrl_signals control;
    struct instr_meta IR_meta;
    struct pipe_regs curr_pipe_regs;
    struct pipe_regs next_pipe_regs;
    int bits_for_cache_tag;
    struct memory_stats_t mem_stats;
    int registers[REGISTER_NUM];
    uint32_t *memory;
};

