// Code your design here
module SCMP(input clk,rst,			//clock and reset
            input [31:0] next_add);	//to give address to pc
  wire [31:0] current_add;
  
  wire [31:0] inst;
  
  wire [1:0] aluCont_inp;
  wire aluSrc, regWrite, memToReg, br, memRead, memWrite, regDst, jump;//control inputs
  
  wire [31:0] mem_to_reg;	//this is data line-do NOT confuse with memToReg control input
  wire [31:0] data_out_1,data_out_2;
  
  wire [31:0] imm_32bit;	//converting the immediate input to 32 bit
  
  wire [31:0] aluSrc_inp_2;
  
  wire [2:0] cont_sig;  //Control input to ALU
  
  wire [31:0] res; //output result of ALU
  
  wire [31:0] datamem_out;	//output from data memeory-for lw
  
  //Program counter
  pc pc1(next_add, clk, rst,current_add);
  
  //Instruction Memory
  IM im1(current_add,inst);
  
  //Control Unit
  control cont1(inst[28:26],aluSrc, regWrite, memToReg, br, memRead,memWrite, regDst, jump, aluCont_inp );  //op code is given as part of instruction
  
  //Register Memory
  register  Reg1 (regWrite, inst [20:16] ,inst [4:0],inst [25:21], mem_to_reg, data_out_1,data_out_2);	//rs,rt,rd are given as a part of instruction
  
  
  //Sign extender
  SE SE1(inst[15:0],imm_32bit);
  
  
  
  //MUX 1
  mux_2x1_32bit mux1( aluSrc_inp_2, data_out_2, imm_32bit, aluSrc);
  
  
  
  //ALU Control
  ALU_cont ALU_Cont1( cont_sig, inst [31:29], aluCont_inp);		//func code is diectly given as a part of instruction memory
  
  
  
  //ALU
  ALU ALU1( res, data_out_1, aluSrc_inp_2, cont_sig );
  
  
  
  //Data Memory
  data_mem data_mem1( memRead, memWrite, res, data_out_2, datamem_out);
  
  //MUX 2
  mux_2x1_32bit mux2( mem_to_reg, res, datamem_out, memToReg);
 
endmodule
//----------------------------------------------------------------

//Program Counter
module pc(input [31:0] next_add,
          input clk, reset,
          output reg [31:0] current_add );
  always@(posedge clk or posedge reset)  
    //To implement after every clock cycle-Single Cycle Processor.
    begin
      if(reset)
        current_add<= 32'b0;
      else
        current_add <= next_add;
    end
  
endmodule
//----------------------------------------------------------------

//Instruction Memory
module IM(input [31:0] current_add,		//address of instruction
          output [31:0] inst);			//32-bit instruction
  reg [31:0] instmem [32:0];			//Place where instructions are 
  										//stored
  //Here we are having 32 instructions each of 32-bit length
  
  always@(current_add)
    begin
      instmem[0]<=32'b000_000_00000_00000_00000000_00000000;
      instmem[1]<=32'b000_010_00110_00001_00000000_00000100;
      instmem[2]<=32'b000_000_00000_00000_00000000_00000000;
      instmem[3]<=32'b000_000_00000_00000_00000000_00000000;
      instmem[4]<=32'b000_000_00000_00000_00000000_00000000;      
    end
  
  assign inst = (current_add < 32) ? instmem[current_add] : 32'b0;

endmodule
//-------------------------------------------------------------------

//Control Unit
module control(input [2:0] opcode,
               //input clk,
              output reg aluSrc, regWrite, memToReg, br, memRead, memWrite, regDst, jump,
               output reg [1:0] aluCont_inp );//br- branch
  
  always@(opcode)  //I didn't use clk here because, anyways opcode  
    begin          //changes during posedge of clk-pure combo
      
      case (opcode)
        3'b001 : begin aluSrc=0; regWrite=1; memToReg=0;  //add 
           br=0; memRead=0 ;memWrite=0; regDst=1; jump=0;
           aluCont_inp=2'b00; end
        
        3'b010 : begin aluSrc=1; regWrite=1;memToReg=0;   //addi
           br=0; memRead=0 ;memWrite=0; regDst=0; jump=0; 
           aluCont_inp=2'b01; end
        
        
        
        3'b011 : begin aluSrc=1; regWrite=1; memToReg=1;  //lw
          br=0; memRead=1 ;memWrite=0; regDst=0; jump=0; end
        
        //6'h03 : begin aluSrc=1; regWrite=0; memToReg=1'bx;  //sw
        //  br=0; memRead=0 ;memWrite=1; regDst=1'bx; jump=0; end
        
        //6'h04 : begin aluSrc=0; regWrite=0; memToReg=1'bx;  //beq
        //  br=1; memRead=0 ;memWrite=0; regDst=1'bx; jump=0; end
        
        //6'h05 : begin aluSrc=1'bx; regWrite=0; memToReg=1'bx;//jump
        //  br=1'bx; memRead=0 ;memWrite=0; regDst=1'bx; jump=1; end
        
        default :begin aluSrc=0; regWrite=0; memToReg=0; 
           br=0; memRead=0 ;memWrite=0; regDst=0; jump=0; end
        //add as default
        
      endcase     
    end
          
endmodule
//---------------------------------------------------------------------

//Register Module
module register (input regWrite,				//control to write in reg
            input [4:0]rs,rt,rd,		//sources and destination
            input [31:0] mem_to_reg,	//data memory to register modules
            output reg [31:0] data_out_1,data_out_2);//data inputs to ALU
  
  reg [31:0] regmem [31:0];		//register memory
  
  always@(rs or rt or rd or regWrite or mem_to_reg)		//for storing data initially in register memory which generally is taken care by OS
    begin
      regmem[0] = 32'b00000000_00000000_00000000_00000000; 
      regmem[1] = 32'b00000000_00000000_00000000_00000100;
      regmem[2] = 32'b00000000_00000000_00000000_00000000;
      regmem[3] = 32'b00000000_00000000_00000000_00000000;
      regmem[4] = 32'b00000000_00000000_00000000_00000000;// till 31
    end
  
  always@(rs or rt or rd or regWrite or mem_to_reg) // all inputs
    begin
      
      data_out_1 = regmem[rs];
      data_out_2 = regmem[rt];
      if (regWrite)
        regmem[rd] = mem_to_reg;
      
    end
  
endmodule
//---------------------------------------------------------------------
  
module SE (input [15:0] data_in,
           output [31:0] data_out);
  assign data_out={15'b0,data_in};
  
endmodule
//---------------------------------------------------------------------

module mux_2x1_32bit (output reg [31:0]data_out,
                    input [31:0] data_in_1,data_in_2,
                    input sel);		//select line
  always@(*)		//all inputs
    begin
      case(sel)
        1'b1: data_out = data_in_2;
        1'b0: data_out = data_in_1;
      endcase
    end
  
endmodule
//---------------------------------------------------------------------

//ALU Control Unit
module ALU_cont(output reg [2:0] cont_sig,		// control signal
                input [2:0] func,			// func and op code from
                input [1:0] aluCont_inp);		 
  											// instruction
  always@(*)
    begin
      case(aluCont_inp)
        2'b00: begin
          case (func)
            3'b000: cont_sig = 3'b000;	// add
            3'b001: cont_sig = 3'b001;	// sub
            3'b010: cont_sig = 3'b010;	// and
            3'b011: cont_sig = 3'b011;	// or 
          endcase
        end
        
        2'b01: begin
          case (func)						// for addi, same cont sig
            3'b000: cont_sig = 3'b000;	// addi
            3'b001: cont_sig = 3'b001;	// subi
          endcase
        end
      
        default: cont_sig = 3'b000; // default is add
        
      endcase
    end
endmodule
//---------------------------------------------------------------------

//ALU
module ALU (output reg [31:0] res,		 //output result of ALU 
            input [31:0] src_1, src_2,   // Sources
            input [2:0] alu_cont ); 	 // ALU Control input
  always@(*)
    begin
      case(alu_cont)
      3'b000: res = src_1 + src_2;
      3'b001: res = src_1 - src_2;
      3'b010: res = src_1 & src_2;
      3'b011: res = src_1 | src_2;
      endcase
    end
        
endmodule
        /* Should we do gate level mod
        for optimized circuits, add & 
        sub can be teamed in 1 circuit */

//---------------------------------------------------------------------

//Data Memory
module data_mem(input memRead,memWrite,			//control inputs
                input [31:0] Address,
                input [31:0] data_in,	
                output reg [31:0] data_out);	//data outlet when required
  reg [31:0] mem [31:0];	//storage memory
  
  always@(memRead or memWrite)	// for storing data initially in data memory which generally is taken care by OS
   begin
      mem[0] = 32'b00000000_00000000_00000000_00000000;
      mem[1] = 32'b00000000_00000000_00000000_00000000;
      mem[2] = 32'b00000000_00000000_00000000_00000000;
      mem[3] = 32'b00000000_00000000_00000000_00000000;
      mem[4] = 32'b00000000_00000000_00000000_00000000; //till 31 but not required for us
    end
  
//Address actually is 32 bit data coming from ALU, that means memory depth is 2^32, but we do not deal with that much data for now. 
  
  always@(memRead or memWrite)
    begin 
      
      if(memWrite)
        mem[Address] = data_in;
      else if(memRead)
        data_out = mem[Address];
      else data_out = 32'bz;		//incase outlet not being used,
      								//for example during sw instruction
      
    end
  
endmodule