// Code your testbench here
// or browse Examples
module SCMP_t();
  reg clk,rst;
  reg [31:0] next_add;
  
  SCMP scmp1 (clk,rst,next_add);
  
  initial #500 $finish;
  
  always #5 clk=!clk;
  
  initial begin
    clk=0;
    #2 next_add=32'b1;
    #1 rst=0;
    #1 rst=1;		//reset ends at 8units 
    #5 rst=0;    
  end
  initial begin
    $monitor($time,"-%b",scmp1.Reg1.regmem[6]);
  end
  
endmodule