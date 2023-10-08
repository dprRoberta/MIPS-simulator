
#include "mipssim.h"

int BREAK_POINT = 200000; // exit after so many cycles -- useful for debugging

// Global variables
char mem_init_path[1000];
char reg_init_path[1000];

uint32_t task_number = 0;
struct architectural_state arch_state;

uint8_t get_instruction_type(int opcode)
{
    switch (opcode) {
        /// opcodes are defined in definitions.h
        case R_INST:
            return R_TYPE;
        case EOP:
            return EOP_TYPE;
        ///@students task 1: fill in the rest
        case ADDI: 
            return I_TYPE;
        case LW:
            return MEM_TYPE;
        case SW:
            return MEM_TYPE;
        case BEQ:
            return BRANCH_TYPE;
        case J:
            return JUMP_TYPE;
        default:
            assert(false && "missing opcode case");
    }
    assert(false);
}



int FSM(int state, struct instr_meta *IR_meta)
{
    //states defined in definitions.h
    struct ctrl_signals *control = &arch_state.control;
    //reset control signals
    memset(control, 0, (sizeof(struct ctrl_signals)));

    int opcode = IR_meta->opcode;
    int type = IR_meta->type;
    switch (state) {
        case INSTR_FETCH:
            control->MemRead = 1;
            control->ALUSrcA = 0;
            control->IorD = 0;
            control->IRWrite = 1;
            control->ALUSrcB = 1;
            control->ALUOp = 1;
            control->PCWrite = 1;
            control->PCSource = 0;
            state = DECODE;
            break;
        case DECODE:
            control->ALUSrcA = 0;
            control->ALUSrcB = 3;
            control->ALUOp = 1;
            if (type == R_TYPE) state = EXEC;
            else if (opcode == EOP) state = EXIT_STATE;
            ///@students task 2: fill in the rest
            else if (type == MEM_TYPE) state = MEM_ADDR_CALC;
            else if (type == BRANCH_TYPE) state = BRANCH_COMPL;
            else if (type == JUMP_TYPE) state = JUMP_COMPL;
            else if (type == I_TYPE) state = I_TYPE_EXEC;
            else assert(false && "decode not yet implemented");
            break;
        case EXEC:
            control->ALUSrcA = 1;
            control->ALUSrcB = 0;
            control->ALUOp = 0;
            state = R_TYPE_COMPL;
            break;
        case R_TYPE_COMPL:
            control->RegDst = 1;
            control->RegWrite = 1;
            control->MemtoReg = 0;
            control->ALUOp = 1;
            state = INSTR_FETCH;
            break;
        case MEM_ADDR_CALC:
            control->ALUSrcA = 1;
            control->ALUSrcB = 2;
            control->ALUOp = 1;
            if (opcode == LW) state = MEM_LD_READ;
            else if (opcode == SW) state = MEM_ST_WRITE;
            break;
        case MEM_LD_READ:
            control->MemRead = 1;
            control->IorD = 1;
            control->ALUOp = 1;
            state = WB_STEP;
            break;
        case MEM_ST_WRITE:
            control->MemWrite = 1;
            control->IorD = 1;
            control->ALUOp = 1;
            state = INSTR_FETCH;
            break;
        case WB_STEP:
            control->RegDst = 0;
            control->RegWrite = 1;
            control->MemtoReg = 1;
            control->ALUOp = 1;
            state = INSTR_FETCH;
            break;
        case BRANCH_COMPL:
            control->ALUSrcA = 1;
            control->ALUSrcB = 0;
            control->ALUOp = 2;
            control->PCWriteCond = 1;
            control->PCSource = 1;
            state = INSTR_FETCH;
            break;
        case JUMP_COMPL:
            control->PCWrite = 1;
            control->PCSource = 2;
            control->ALUOp = 1;
            state = INSTR_FETCH;
            break;
        case EXIT_STATE:
            state = EXIT_STATE;
            break;
        case I_TYPE_EXEC:
            control->ALUSrcA = 1;
            control->ALUSrcB = 2;
            control->ALUOp = 1;
            state = I_TYPE_COMPL;
            break;
        case I_TYPE_COMPL:
            control->RegDst = 0;
            control->RegWrite = 1;
            control->MemtoReg = 0;
            control->ALUOp = 1;
            state = INSTR_FETCH;
            break;
        default: assert(false && "FSM state not yet implemented");
    }

    return state;
}

void decode_and_read_RF()
{
    int read_register_1 = arch_state.IR_meta.reg_21_25;
    int read_register_2 = arch_state.IR_meta.reg_16_20;
    int immediate = arch_state.IR_meta.immediate;
    int shift_imm = (immediate) << 2;
    check_is_valid_reg_id(read_register_1);
    check_is_valid_reg_id(read_register_2);
    arch_state.next_pipe_regs.A = arch_state.registers[read_register_1];
    arch_state.next_pipe_regs.B = arch_state.registers[read_register_2];
    arch_state.next_pipe_regs.ALUOut = arch_state.curr_pipe_regs.pc + shift_imm;
}

void execute()
{
    struct ctrl_signals *control = &arch_state.control;
    struct instr_meta *IR_meta = &arch_state.IR_meta;
    struct pipe_regs *curr_pipe_regs = &arch_state.curr_pipe_regs;
    struct pipe_regs *next_pipe_regs = &arch_state.next_pipe_regs;

    int alu_opA = control->ALUSrcA == 1 ? curr_pipe_regs->A : curr_pipe_regs->pc;
    int alu_opB = 0;
    int immediate = IR_meta->immediate;
    int shifted_immediate = (immediate) << 2;
    switch (control->ALUSrcB) {
        case 0:
            alu_opB = curr_pipe_regs->B;
            break;
        case 1:
            alu_opB = WORD_SIZE;
            break;
        case 2:
            alu_opB = immediate;
            break;
        case 3: 
            alu_opB = shifted_immediate;
            break;
        default:
            assert(false && "missing ALUSrcB control");
    }


    switch (control->ALUOp) {
        case 0:
            if (IR_meta->type == R_TYPE && IR_meta->function == ADD)
                next_pipe_regs->ALUOut = alu_opA + alu_opB;
            else if (IR_meta->type == R_TYPE && IR_meta->function == SLT)
                if (alu_opA < alu_opB) {
                    next_pipe_regs->ALUOut = 1;
                }
                else {
                    next_pipe_regs->ALUOut = 0;
                }
            else
                assert(false && "unimplemented ALUOp");
            break;
        case 1:
            next_pipe_regs->ALUOut = alu_opA + alu_opB;
            break;
        case 2: 
            next_pipe_regs->ALUOut = alu_opA - alu_opB;
            break;
        default:
            assert(false);
    }

    // PC calculation
    switch (control->PCSource) {
        case 0:
            next_pipe_regs->pc = next_pipe_regs->ALUOut;
            break; 
        case 1: 
            next_pipe_regs->pc = curr_pipe_regs->ALUOut;
            break;
        case 2: 
            int pc_bits = get_piece_of_a_word(curr_pipe_regs->pc, 31, 4);
            next_pipe_regs->pc = (pc_bits | (IR_meta->jmp_offset << 2));
            break;
        default:
            assert(false);
    }

}


void memory_access()
{
    if (arch_state.control.MemRead && arch_state.control.IorD == 0) {
        int address = arch_state.curr_pipe_regs.pc;
        arch_state.next_pipe_regs.IR = memory_read(address);
    }
    if (arch_state.control.MemRead && arch_state.control.IorD == 1) {
        int address = arch_state.curr_pipe_regs.ALUOut;
        arch_state.next_pipe_regs.IR = memory_read(address);
    }
    if (arch_state.control.MemWrite) {
        int value = arch_state.curr_pipe_regs.B;
        int address = arch_state.curr_pipe_regs.ALUOut;
        memory_write(address, value);
    }
}

void write_back()
{

    if (arch_state.control.RegWrite) {
        if(arch_state.control.RegDst == 1) {
            int write_reg_id =  arch_state.IR_meta.reg_11_15;
            int write_data =  arch_state.curr_pipe_regs.ALUOut;
            check_is_valid_reg_id(write_reg_id);
            arch_state.registers[write_reg_id] = write_data;
        
        } 
        else if (arch_state.control.RegDst == 0 && arch_state.control.MemtoReg == 1) {
            int write_reg_id = arch_state.IR_meta.reg_16_20;
            int write_data =  arch_state.curr_pipe_regs.MDR;
            check_is_valid_reg_id(write_reg_id);
            arch_state.registers[write_reg_id] = write_data;
        }
        else if (arch_state.control.RegDst == 0){
            int write_reg_id = arch_state.IR_meta.reg_16_20;
            int write_data =  arch_state.curr_pipe_regs.ALUOut;
            check_is_valid_reg_id(write_reg_id);
            arch_state.registers[write_reg_id] = write_data;
        }
    }

}


void set_up_IR_meta(int IR, struct instr_meta *IR_meta)
{
    IR_meta->opcode = get_piece_of_a_word(IR, OPCODE_OFFSET, OPCODE_SIZE);
    IR_meta->immediate = get_sign_extended_imm_id(IR, IMMEDIATE_OFFSET);
    IR_meta->function = get_piece_of_a_word(IR, 0, 6);
    IR_meta->jmp_offset = get_piece_of_a_word(IR, 0, 26);
    IR_meta->reg_11_15 = (uint8_t) get_piece_of_a_word(IR, 11, REGISTER_ID_SIZE);
    IR_meta->reg_16_20 = (uint8_t) get_piece_of_a_word(IR, 16, REGISTER_ID_SIZE);
    IR_meta->reg_21_25 = (uint8_t) get_piece_of_a_word(IR, 21, REGISTER_ID_SIZE);
    IR_meta->type = get_instruction_type(IR_meta->opcode);

    switch (IR_meta->opcode) {
        case R_INST:
            if (IR_meta->function == ADD)
                printf("Executing ADD(%d), $%u = $%u + $%u (function: %u) \n",
                       IR_meta->opcode,  IR_meta->reg_11_15, IR_meta->reg_21_25,  IR_meta->reg_16_20, IR_meta->function);
	    else if (IR_meta->function == SLT)
                printf("Executing SLT(%d), $%u = $%u < $%u ? 1 : 0 (function: %u) \n",
                       IR_meta->opcode,  IR_meta->reg_11_15, IR_meta->reg_21_25,  IR_meta->reg_16_20, IR_meta->function);
            else assert(false && "unknown R_INST");
            break;
        case EOP:
            printf("Executing EOP(%d) \n", IR_meta->opcode);
            break;
        case ADDI:
          printf("ADDI");
          printf("(%d),  $%u = $%u + #%u; \n",
                         IR_meta->opcode,  IR_meta->reg_16_20, IR_meta->reg_21_25, IR_meta->immediate);
          break;
        case SW:
          printf("SW");
          printf("(%d), mem[$%u + (%d)] = $%u; \n",
                         IR_meta->opcode,  IR_meta->reg_21_25, IR_meta->immediate, IR_meta->reg_16_20);
          break;
        case LW:
          printf("LW");
          printf("(%d), $%u = mem[$%u + (%d)];\n",
                         IR_meta->opcode, IR_meta->reg_16_20, IR_meta->reg_21_25, IR_meta->immediate);
          break;
        case BEQ:
          printf("BEQ");
          printf("(%d), if $%u == $%u then pc = pc + ((%d) * 4); \n",
                         IR_meta->opcode,  IR_meta->reg_16_20, IR_meta->reg_21_25, IR_meta->immediate);
          break;
        case J:
          printf("JUMP");
          printf("(%d), jump  to instruction word jmp_offset %u \n",
                         IR_meta->opcode,   IR_meta->jmp_offset);
          break;
            default: assert(false && "unknown opcode");
    }
}

void assign_pipeline_registers_for_the_next_cycle()
{
    struct ctrl_signals *control = &arch_state.control;
    struct instr_meta *IR_meta = &arch_state.IR_meta;
    struct pipe_regs *curr_pipe_regs = &arch_state.curr_pipe_regs;
    struct pipe_regs *next_pipe_regs = &arch_state.next_pipe_regs;

    if (control->IRWrite) {
        curr_pipe_regs->IR = next_pipe_regs->IR;
        printf("PC %d: ", curr_pipe_regs->pc / 4);
        set_up_IR_meta(curr_pipe_regs->IR, IR_meta);
    }
    curr_pipe_regs->ALUOut = next_pipe_regs->ALUOut;
    curr_pipe_regs->A = next_pipe_regs->A;
    curr_pipe_regs->B = next_pipe_regs->B;
    if (control->PCWrite) {
        check_address_is_word_aligned(next_pipe_regs->pc);
        curr_pipe_regs->pc = next_pipe_regs->pc;
    }
    if (control->PCWriteCond && curr_pipe_regs->ALUOut==0) {
        check_address_is_word_aligned(next_pipe_regs->pc);
        curr_pipe_regs->pc = next_pipe_regs->pc;
    }
    if (control->IorD == 1 && control->MemRead) {
        curr_pipe_regs->MDR = next_pipe_regs->IR;
    }
}


int main(int argc, const char* argv[])
{
    /*--------------------------------------
    /------- Global Variable Init ----------
    /--------------------------------------*/
    parse_arguments(argc, argv);
    arch_state_init(&arch_state);

    switch(task_number) {
        case(INSTRUCTION_TYPE):
        task_1();
        break;
        case(FINITE_STATE_MACHINE_DEC):
        task_2();
        break;
        case(FINITE_STATE_MACHINE):
        task_3();
        break;
        case(FINITE_STATE_MACHINE_EXT):
        task_4();
        break;
        case(FULL):
        task_5();
        break;
        default:
        assert(false && "No task given");
    }
}

