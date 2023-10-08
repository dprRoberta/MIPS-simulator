#pragma once
#include <math.h>
#include "parser.h"
#include <inttypes.h>

#include "definitions.h"


//Task type
#define INSTRUCTION_TYPE 1
#define FINITE_STATE_MACHINE_DEC 2
#define FINITE_STATE_MACHINE 3
#define FINITE_STATE_MACHINE_EXT 4
#define FULL 5


////////////////////////////////////////////////////////
/// Global Variables
////////////////////////////////////////////////////////
extern char mem_init_path[];
extern char reg_init_path[];
extern struct architectural_state arch_state;
extern uint32_t  task_number;
extern int BREAK_POINT;

////////////////////////////////////////////////////////
/// Simulation Functions
////////////////////////////////////////////////////////
uint8_t get_instruction_type(int opcode);
int FSM(int state, struct instr_meta *IR_meta);
void decode_and_read_RF();
void execute();
void memory_access();
void write_back();
void assign_pipeline_registers_for_the_next_cycle();


////////////////////////////////////////////////////////
/// Memory Functions
////////////////////////////////////////////////////////
void memory_state_init(struct architectural_state *);
int  memory_read(int address);
void memory_write (int address, int write_data);

////////////////////////////////////////////////////////////////
/// Marking Functions --> Do not (re)move those functions
////////////////////////////////////////////////////////////////
static inline void marking_after_clock_cycle() { }
static inline void marking_at_the_end(){ }


static inline void instruction_parser(uint32_t *memory,  char* instr_file_path,
                                      uint32_t *registers, char* reg_file_path)
{
    int instr_count = iterate_file(memory, instr_file_path, per_line_binary_parser, MEMORY_WORD_NUM);

    uint32_t *registers_but_zero = &registers[1]; // Init registers from $1 onwards
    int reg_count = iterate_file(registers_but_zero, reg_file_path, per_line_decimal_parser, REGISTER_NUM - 1);

    printf(" ~~~ Loaded Memory   :\n");
    print_uint32_bin_array(memory, instr_count);
    printf(" ~~~ Loaded Registers: (print starts from $1)\n");
    print_uint32_bin_array(registers_but_zero, reg_count);
}




static inline void check_is_valid_reg_id(int reg_id)
{
    assert(reg_id >= 0);
    assert(reg_id < REGISTER_NUM);
}

static inline void check_address_is_word_aligned(int address)
{
    assert(address >= 0);
    assert(address % WORD_SIZE == 0);
    assert(address <= MEMORY_WORD_NUM);
}



// Used to get a sign extended immediate (sign extension from 16 bit to 32)
static inline int get_sign_extended_imm_id(int instr, uint8_t offset)
{
    short int imm = (short int)(instr >> offset);
    return (int) imm;
}


static inline int get_piece_of_a_word(int word, uint8_t start, uint8_t size)
{
    int mask = 1 << size;
    mask--;
    return (word >> start) & mask;
}


static inline void parse_arguments(int argc, const char* argv[])
{
    assert(argc == 4 && "Three arguments are expected in the following order: " &&
           "1. <task_number> (1 -> instruction_type, 2 -> fsm_decode, 3 -> fsm, 4 -> fsm_ext, 5 -> full)" &&
           "2. <init_memory_file_path>" &&
           "3. <init_register_file_path>");
    sscanf(argv[1],"%d", &task_number); 
    sscanf(argv[2],"%s", mem_init_path);
    sscanf(argv[3],"%s", reg_init_path);
    printf("Task number: %d, Mem path: %s, Reg path: %s\n", task_number, mem_init_path, reg_init_path);

}

static inline void arch_state_init(struct architectural_state* arch_state_ptr)
{

    memset(arch_state_ptr, 0, sizeof(struct architectural_state));
    memory_state_init(arch_state_ptr);
    arch_state_ptr->state = INSTR_FETCH;
    // Loads the "binary" into the memory array, and init registers
    instruction_parser(arch_state_ptr->memory, mem_init_path,
                       (uint32_t *) arch_state_ptr->registers, reg_init_path);

}

static inline void memory_stats_init(struct architectural_state *state)
{
    state->mem_stats.sw_total= 0;
    state->mem_stats.lw_total= 0;
}

static inline const char* opcode_to_string(int opcode) {
	switch(opcode){
		case(R_INST): return "000000";
		case(ADD): return "100000";
		case(ADDI): return "001000";
		case(LW): return "100011";
		case(SW): return "101011";
		case(BEQ): return "000100";
		case(J): return "000010";
		case(SLT): return "101010";
		case(EOP): return "111111";
		}
	assert(false);
}

static inline void signal_to_string(struct ctrl_signals *signal, char* string) {
	sprintf(string, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
    signal->RegDst,
    signal->RegWrite,
    signal->ALUSrcA,
    signal->MemRead,
    signal->MemWrite,
    signal->MemtoReg,
    signal->IorD,
    signal->IRWrite,
    signal->PCWrite,
    signal->PCWriteCond,
    signal->ALUOp,
    signal->ALUSrcB,
    signal->PCSource
	);
}

static inline void signal_header_to_string(char* string) {
	sprintf(string, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
    "RegDst",
    "RegWrite",
    "ALUSrcA",
    "MemRead",
    "MemWrite",
    "MemtoReg",
    "IorD",
    "IRWrite",
    "PCWrite",
    "PCWriteCond",
    "ALUOp",
    "ALUSrcB",
    "PCSource"
	);
}


static inline void regs_header_to_string(char* string) {
	sprintf(string, "%s,%s,%s,%s,%s,%s",
    "pc",
    "IR",
    "A",
    "B",
    "ALUOut",
    "MDR"
	);
}

static inline void regs_to_string(struct pipe_regs *regs, char* string) {
	sprintf(string, "%u,%u,%u,%u,%u,%u",
    regs->pc,
    regs->IR,
    regs->A,
    regs->B,
    regs->ALUOut,
    regs->MDR
	);
}

static inline void regfile_entries_to_string(int registers[REGISTER_NUM], char* string) {
	sprintf(string, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
      registers[0],
      registers[1],
      registers[2],
      registers[3],
      registers[4],
      registers[5],
      registers[6],
      registers[7],
      registers[8],
      registers[9],
      registers[10],
      registers[11],
      registers[12],
      registers[13],
      registers[14],
      registers[15],
      registers[16],
      registers[17],
      registers[18],
      registers[19],
      registers[20],
      registers[21],
      registers[22],
      registers[23],
      registers[24],
      registers[25],
      registers[26],
      registers[27],
      registers[28],
      registers[29],
      registers[30],
      registers[31]
	);
}

static inline void task_1() {
    int IR;
    int opcode;
    int ins_type;
	FILE *output = fopen("task_1.out", "w");

    arch_state.control.MemRead = 1;
    fprintf(output, "opcode,type\n");
	fflush(output);
    while(true) {

        memory_access();
        arch_state.curr_pipe_regs.IR = arch_state.next_pipe_regs.IR;

        IR = arch_state.curr_pipe_regs.IR;

        opcode = get_piece_of_a_word(IR, OPCODE_OFFSET, OPCODE_SIZE);

        ins_type = get_instruction_type(opcode);


        fprintf(output, "%s,%d\n", opcode_to_string(opcode), ins_type);
		fflush(output);

        check_address_is_word_aligned(arch_state.curr_pipe_regs.pc);

        arch_state.curr_pipe_regs.pc += 4;

        arch_state.clock_cycle++;
        // Check exit statements
        if (ins_type == EOP_TYPE) { // I.E. EOP instruction!
            printf("Exiting because the exit state was reached \n");
            break;
        }
        if (arch_state.clock_cycle == BREAK_POINT) {
            printf("Exiting because the break point (%u) was reached \n", BREAK_POINT); break;
        }
    }
	fclose(output);
}

static inline void task_2() {
	FILE *output = fopen("task_2.out", "w");
	char buffer[2000];
	int state;

	int states[13] = {
		INSTR_FETCH,
		DECODE,
		MEM_ADDR_CALC,
		MEM_LD_READ,
		WB_STEP,
		MEM_ST_WRITE,
		EXEC,
		R_TYPE_COMPL,
		BRANCH_COMPL,
		JUMP_COMPL,
		EXIT_STATE,
		I_TYPE_EXEC,
		I_TYPE_COMPL};

	int opcodes[6] = {
		R_INST,
		LW,
		SW,
		BEQ,
		J,
		EOP
	};


	signal_header_to_string(buffer);
	fprintf(output, "%s,%s,%s,%s,%s\n", "state", "new_state",
				"opcode", "type", buffer);
	fflush(output);

    for (int state_idx = 0; state_idx < 2; state_idx ++) {
    	for (int opcode_idx = 0; opcode_idx < 6; opcode_idx ++) {
				arch_state.IR_meta.opcode = opcodes[opcode_idx];

				arch_state.IR_meta.type = get_instruction_type(arch_state.IR_meta.opcode);

				state = FSM(states[state_idx],&arch_state.IR_meta);

				signal_to_string(&(arch_state.control), buffer);

				fprintf(output, "%u,%u,%u,%u,%s\n", states[state_idx], state,
				opcodes[opcode_idx], arch_state.IR_meta.type, buffer);
				fflush(output);
		}
    }
	fclose(output);
}

static inline void task_3() {
	FILE *output = fopen("task_3.out", "w");
	char buffer[2000];
	int state;

	int states[13] = {
		INSTR_FETCH,
		DECODE,
		MEM_ADDR_CALC,
		MEM_LD_READ,
		WB_STEP,
		MEM_ST_WRITE,
		EXEC,
		R_TYPE_COMPL,
		BRANCH_COMPL,
		JUMP_COMPL,
		EXIT_STATE,
		I_TYPE_EXEC,
		I_TYPE_COMPL};

	int opcodes[7] = {
		R_INST,
		LW,
		SW,
		BEQ,
		J,
		EOP,
		ADDI
	};


	signal_header_to_string(buffer);
	fprintf(output, "%s,%s,%s,%s,%s\n", "state", "new_state",
				"opcode", "type", buffer);
	fflush(output);

    for (int state_idx = 0; state_idx < 10; state_idx ++) {
    	for (int opcode_idx = 0; opcode_idx < 7; opcode_idx ++) {
				if(state_idx==2 && opcode_idx != 1 && opcode_idx != 2) continue;
				arch_state.IR_meta.opcode = opcodes[opcode_idx];

				arch_state.IR_meta.type = get_instruction_type(arch_state.IR_meta.opcode);

				state = FSM(states[state_idx],&arch_state.IR_meta);

				signal_to_string(&(arch_state.control), buffer);

				fprintf(output, "%u,%u,%u,%u,%s\n", states[state_idx], state,
				opcodes[opcode_idx], arch_state.IR_meta.type, buffer);
				fflush(output);
		}
    }
	fclose(output);
}


static inline void task_4() {
	FILE *output = fopen("task_4.out", "w");
	char buffer[2000];
	int state;

	int states[13] = {
		INSTR_FETCH,
		DECODE,
		MEM_ADDR_CALC,
		MEM_LD_READ,
		WB_STEP,
		MEM_ST_WRITE,
		EXEC,
		R_TYPE_COMPL,
		BRANCH_COMPL,
		JUMP_COMPL,
		EXIT_STATE,
		I_TYPE_EXEC,
		I_TYPE_COMPL};

	int opcodes[7] = {
		R_INST,
		LW,
		SW,
		BEQ,
		J,
		EOP,
		ADDI
	};


	signal_header_to_string(buffer);
	fprintf(output, "%s,%s,%s,%s,%s\n", "state", "new_state",
				"opcode", "type", buffer);
	fflush(output);

    for (int state_idx = 0; state_idx < 13; state_idx ++) {
    	for (int opcode_idx = 0; opcode_idx < 7; opcode_idx ++) {
				if(state_idx==2 && opcode_idx != 1 && opcode_idx != 2) continue;
				arch_state.IR_meta.opcode = opcodes[opcode_idx];

				arch_state.IR_meta.type = get_instruction_type(arch_state.IR_meta.opcode);

				state = FSM(states[state_idx],&arch_state.IR_meta);

				signal_to_string(&(arch_state.control), buffer);

				fprintf(output, "%u,%u,%u,%u,%s\n", states[state_idx], state,
				opcodes[opcode_idx], arch_state.IR_meta.type, buffer);
				fflush(output);
		}
    }
	fclose(output);
}

static inline void task_5() {
	FILE *output = fopen("task_5.out", "w");
	char buffer[2000];
	char buffer2[2000];

	signal_header_to_string(buffer);
	regs_header_to_string(buffer2);
	fprintf(output, "%s,%s,%s\n", "cycle", buffer, buffer2);
	fflush(output);

	FILE *reg_output = fopen("task_5_reg.out", "w");

	char reg_buffer[2000];
	//char reg_buffer2[2000];
    while (true) {

		arch_state.state = FSM(arch_state.state,&arch_state.IR_meta);

        memory_access();

        decode_and_read_RF();

        execute();

        write_back();

        assign_pipeline_registers_for_the_next_cycle();

		signal_to_string(&(arch_state.control), buffer);
        fprintf(output, "%" PRIu64 ",%s,", arch_state.clock_cycle, buffer);

		regs_to_string(&(arch_state.curr_pipe_regs), buffer2);
        fprintf(output, "%s\n", buffer2);
		fflush(output);

        regfile_entries_to_string(arch_state.registers, reg_buffer);
        fprintf(reg_output, "%s", reg_buffer);

        arch_state.clock_cycle++;
        if (arch_state.state == EXIT_STATE) {
            printf("Exiting because the exit state was reached \n");
            break;
        }
        if (arch_state.clock_cycle == BREAK_POINT) {
            printf("Exiting because the break point (%u) was reached \n", BREAK_POINT);
            break;
        }
    }
	fclose(output);
	fclose(reg_output);
}
