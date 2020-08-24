//------------------------------------------------------------------
//-- File generated by RobustVerilog parser
//-- RobustVerilog version 1.2g (limited free version)
//-- Invoked Tue Jul 21 10:05:11 2020
//-- Source file: axi2apb_ctrl.v
//-- Parent file: axi2apb.v
//-- Run directory: C:/RobustVerilog_free1.2_win/
//-- Target directory: out/
//-- Command flags: src/base/axi2apb.v -od out -I src/gen -list list.txt -listpath -header -gui 
//-- www.provartec.com/edatools ... info@provartec.com
//------------------------------------------------------------------





module  axi2apb_ctrl (clk,reset,finish_wr,finish_rd,cmd_empty,cmd_read,WVALID,psel,penable,pwrite,pready);


   input              clk;
   input              reset;

   input              finish_wr;
   input              finish_rd;
   
   input              cmd_empty;
   input              cmd_read;
   input              WVALID;

   output               psel;
   output               penable;
   output               pwrite;
   input               pready;
   
   
   wire                   wstart;
   wire                       rstart;
   
   reg                        busy;
   reg                        psel;
   reg                   penable;
   reg                   pwrite;
   wire                       pack;
   wire                       cmd_ready;
   

   assign                     cmd_ready = (~busy) & (~cmd_empty);
   assign                     wstart = cmd_ready & (~cmd_read) & (~psel) & WVALID;
   assign                     rstart = cmd_ready & cmd_read & (~psel);
   
   assign             pack = psel & penable & pready;
   
   always @(posedge clk or posedge reset)
     if (reset)
       busy <= #1 1'b0;
     else if (psel)
       busy <= #1 1'b1;
     else if (finish_rd | finish_wr)
       busy <= #1 1'b0;
   
   always @(posedge clk or posedge reset)
     if (reset)
       psel <= #1 1'b0;
     else if (pack)
       psel <= #1 1'b0;
     else if (wstart | rstart)
       psel <= #1 1'b1;
   
   always @(posedge clk or posedge reset)
     if (reset)
       penable <= #1 1'b0;
     else if (pack)
       penable <= #1 1'b0;
     else if (psel)
       penable <= #1 1'b1;

   always @(posedge clk or posedge reset)
     if (reset)
       pwrite  <= #1 1'b0;
     else if (pack)
       pwrite  <= #1 1'b0;
     else if (wstart)
       pwrite  <= #1 1'b1;
   

endmodule

   

