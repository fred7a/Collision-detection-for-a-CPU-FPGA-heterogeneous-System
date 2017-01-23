/*************************************************************************
 *                                                                       *
 * Copyright (C) 2016,2017 Alves, Fredy.       							 *
 * All rights reserved.  Email: fredyamalves1@gmail.com                  *
 *                                                                       *
 * This design is free software; you can redistribute it and/or          *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this design in the      *
 *       file LICENSE.                                                   *
 *                                                                       *
 * This design is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

//------------------------------------------------------------------------
// AFU user module 
//
// Copies content of src buffer onto destination buffer in memory
//------------------------------------------------------------------------


//`define DEBUG 2
//`define DEBUGOUT 3
//`define DEBUGIN 4
module inputs_mem #(ADDR_LMT = 20, MDATA = 14, CACHE_WIDTH = 512) 
(
   input                   clk,                
   input                   reset_n,                  
   
   // Read Request
   output [ADDR_LMT-1:0]   rd_req_addr,           
   output [MDATA-1:0] 	   rd_req_mdata,            
   output reg              rd_req_en,             
   input                   rd_req_almostfull,           
   
   // Read Response
   input                   rd_rsp_valid,       
   input [MDATA-1:0] 	   rd_rsp_mdata,            
   input [CACHE_WIDTH-1:0] rd_rsp_data,
   
   output [CACHE_WIDTH-1:0] mem_out,
   input [31:0] addr_mem_out,              

   // Done output signal 
   output reg               done
);

   reg execcounter_clr;
   reg data_buf_we; 
   reg [CACHE_WIDTH-1:0] floatsReg, floatsReg2, floatsReg3, floatsReg4, floatsReg5, floatsReg6;
   reg [2:0] counter_aux = 0;
 
   // --- Address counter 
   
   reg addr_cnt_inc;
   reg addr_cnt_clr;
   reg addr_cnt_start;
   reg [31:0] addr_cnt;
   always @ (posedge clk) begin
      if(!reset_n) 
	addr_cnt <= 0;
      else 
        if(addr_cnt_inc) 
          addr_cnt <= addr_cnt + 1;
	else if(addr_cnt_clr)
	  addr_cnt <= 'd0;
        else if (addr_cnt_start)
          addr_cnt <= 'd1;
   end
   
   // --- Address counter continuous
   
   reg addr_cnt_continuous_inc;
   reg addr_cnt_continuous_clr;
   reg [31:0] addr_cnt_continuous;
   always @ (posedge clk) begin
      if(!reset_n) 
	addr_cnt_continuous <= 0;
      else 
        if(addr_cnt_continuous_inc) 
          addr_cnt_continuous <= addr_cnt_continuous + 1;
	else if(addr_cnt_continuous_clr)
	  addr_cnt_continuous <= 'd0;
   end
   
   // --- Address counter continuous mem
   
   reg addr_cnt_continuous_mem_inc;
   reg addr_cnt_continuous_mem_clr;
   reg [31:0] addr_cnt_continuous_mem;
   always @ (posedge clk) begin
      if(!reset_n) 
	addr_cnt_continuous_mem <= 0;
      else 
        if(addr_cnt_continuous_mem_inc) 
          addr_cnt_continuous_mem <= addr_cnt_continuous_mem + 1;
	else if(addr_cnt_continuous_mem_clr)
	  addr_cnt_continuous_mem <= 'd0;
   end
   
   assign rd_req_mdata = addr_cnt;            
   assign wr_req_mdata = addr_cnt;       
   
   // --- Address counter out
   
   reg addr_cnt_out_inc;
   reg addr_cnt_out_sub;
   reg addr_cnt_out_clr;
   reg [31:0] addr_cnt_out;
   always @ (posedge clk) begin
      if(!reset_n) 
	addr_cnt_out <= 0;
      else 
        if(addr_cnt_out_inc) 
          addr_cnt_out <= addr_cnt_out + 1;
	else if(addr_cnt_out_clr)
	  addr_cnt_out <= 'd0;
        else if(addr_cnt_out_sub)
        addr_cnt_out <= addr_cnt_out - 1;
   end
   
   // --- Address counter out
   
   reg addr_cnt_out_ram_inc;
   reg addr_cnt_out_ram_sub;
   reg addr_cnt_out_ram_clr;
   reg [31:0] addr_cnt_out_ram;
   always @ (posedge clk) begin
      if(!reset_n) 
	addr_cnt_out_ram <= 0;
      else 
        if(addr_cnt_out_ram_inc) 
          addr_cnt_out_ram <= addr_cnt_out_ram + 1;
	else if(addr_cnt_out_ram_clr)
	  addr_cnt_out_ram <= 'd0;
        else if(addr_cnt_out_ram_sub)
        addr_cnt_out_ram <= addr_cnt_out_ram - 1;
   end   
   

   // --- Rd and Wr Addr
   assign rd_req_addr = addr_cnt_continuous;           
   assign wr_req_addr = addr_cnt_out;           

   // --- Data buffer (keep read data here)
   reg [CACHE_WIDTH-1:0] data_buf;
   always @ (posedge clk) begin
      if(!reset_n) data_buf <= 0;
      else 
         if(data_buf_we) 
            data_buf <= rd_rsp_data;           
   end


   // --- Num cache lines to copy (from AFU context)
   wire [31:0] num_clines;
   assign num_clines = 32'd2;
   
   // --- Num cache lines to copy (from AFU context)
   wire [31:0] num_outlines;
   assign num_outlines = 32'd1;    
   

   /* Sudoku stuff */
   localparam DIM_S = 3;
   localparam DIM = (DIM_S*DIM_S);
   localparam DIM_Q = (DIM*DIM);
   reg [(96*DIM)-1:0] r_game;
   wire [(16*DIM)-1:0] w_cacheline_cells;
   reg 	t_start;
   assign w_done = 1'b1;   
   
   reg fetch_line1, fetch_line2, fetch_line3, fetch_line4;
   reg set_fetch_line1, set_fetch_line2, set_fetch_line3, set_fetch_line4;
   reg clr_fetch_line1, clr_fetch_line2, clr_fetch_line3, clr_fetch_line4;
   
   always @ (posedge clk) begin
      if(!reset_n) 
	begin
            fetch_line1 <= 1'b0;
	end
      else 
        if(set_fetch_line1) 
          fetch_line1 <= 1'b1;
	else if(clr_fetch_line1)
	  fetch_line1 <= 1'b0;
   end
   always @ (posedge clk) begin
      if(!reset_n) 
	begin
            fetch_line2 <= 1'b0;
	end
      else 
        if(set_fetch_line2) 
          fetch_line2 <= 1'b1;
	else if(clr_fetch_line2)
	  fetch_line2 <= 1'b0;
   end
   always @ (posedge clk) begin
      if(!reset_n) 
	begin
            fetch_line3 <= 1'b0;
	end
      else 
        if(set_fetch_line3) 
          fetch_line3 <= 1'b1;
	else if(clr_fetch_line3)
	  fetch_line3 <= 1'b0;
   end
   always @ (posedge clk) begin
      if(!reset_n) 
	begin
            fetch_line4 <= 1'b0;
	end
      else 
        if(set_fetch_line4) 
          fetch_line4 <= 1'b1;
	else if(clr_fetch_line4)
	  fetch_line4 <= 1'b0;
   end
   
   
   always@(posedge clk)
     begin
	if(!reset_n) begin 
            floatsReg  <= 0;
            floatsReg2 <= 0;
            floatsReg3 <= 0;
            floatsReg4 <= 0;
            set_fetch_line1 <= 1'b0;
            set_fetch_line2 <= 1'b0;
            set_fetch_line3 <= 1'b0;  
            set_fetch_line4 <= 1'b0;
            
	end
	else 
	begin
	  if(rd_rsp_valid)
	  begin
	     case(rd_rsp_mdata)
                'd0:
		 begin
		    floatsReg <= rd_rsp_data;
		    set_fetch_line1 <= 1'b1;
		 end
                'd1:
		 begin
		    floatsReg2 <= rd_rsp_data;
		    set_fetch_line2 <= 1'b1;
		 end
                'd2:
		 begin
		    floatsReg3 <= rd_rsp_data;
		    set_fetch_line3 <= 1'b1;
		 end
                'd3:
		 begin
		    floatsReg4 <= rd_rsp_data;
		    set_fetch_line4 <= 1'b1;
		 end
	     endcase // case (addr_cnt)
	  end // if (rd_rsp_valid)
	  else 
	  begin
            set_fetch_line1 <= 1'b0;
            set_fetch_line2 <= 1'b0;
            set_fetch_line3 <= 1'b0;
            set_fetch_line4 <= 1'b0;
	  end
	end
     end // always@ (posedge clk)
     
     wire w_done_rd;
     assign w_done_rd = fetch_line1 && fetch_line2 && fetch_line3 && fetch_line4;
     
     wire[511:0] wIn_mem;
   
   assign wIn_mem = (addr_cnt==0)?floatsReg:(addr_cnt==1)?floatsReg2:(addr_cnt==2)?floatsReg3:(addr_cnt==3)?floatsReg4:512'b01000000010101111010111000010100010000000101011110101110000101000100000001010111101011100001010001000000010101111010111000010100010000000101011110101110000101000100000001010111101011100001010001000000010101111010111000010100010000000101011110101110000101000100000001010111101011100001010001000000010101111010111000010100010000000101011110101110000101000100000001010111101011100001010001000000010101111010111000010100010000000101011110101110000101000100000001010111101011100001010001000000010101111010111000010100;
     
    reg mem_we;
     
     wire [511:0] out_Mem1;   
     
        ram_2_ports_d2000_w512 rammem1 (

                .clock(clk),

                .data(wIn_mem),

                .rdaddress(addr_mem_out),

                .wraddress(addr_cnt_continuous_mem),

                .wren(mem_we),

                .q(mem_out));
                
assign w_outGrid = out_Mem1;            

    localparam [4:0]
        FSM_IDLE   = 4'd0,
        FSM_RD_REQ = 4'd1,
        FSM_RD_RSP = 4'd2,
        FSM_WR_REQ = 4'd3,
        FSM_RUN_COLLISIONS = 4'd4,
        FSM_WAIT_COLLISIONS = 4'd5,
        FSM_WR_RSP = 4'd6,
        FSM_RD_REQ2 = 4'd7,
        FSM_RD_RSP2 = 4'd8,
        ENDSTATE = 4'd9,
        FSM_WR_MEM = 4'd10,
        FSM_WR_MEM_EN = 4'd11,
        FSM_AUX1 = 4'd12,
        DONESTATE = 4'd13,
        FSM_RD_DONE = 4'd14;

    reg [4:0] fsm_cs, fsm_ns; 
    reg [31:0] r_cnt,n_cnt,l_cnt,l_cnt2;
   
   
    always @ (posedge clk) 
    begin
        if(!reset_n) fsm_cs <= FSM_IDLE;
        else         fsm_cs <= fsm_ns; 
    end
    
    always@(posedge clk) r_cnt <= (!reset_n) ? 'd0 : n_cnt;
    always @ * 
    begin
   
    fsm_ns = fsm_cs;
    addr_cnt_inc = 1'b0;
    addr_cnt_clr = 1'b0;
    addr_cnt_start = 1'b0;
    addr_cnt_continuous_inc = 1'b0;
    addr_cnt_continuous_clr = 1'b0;
    addr_cnt_continuous_mem_inc = 1'b0;
    addr_cnt_continuous_mem_clr = 1'b0;
    addr_cnt_out_inc = 1'b0;
    addr_cnt_out_sub = 1'b0;
    addr_cnt_out_clr = 1'b0;      
    data_buf_we  = 1'b0;
    rd_req_en = 1'b0;               
    done = 1'b0;          
    t_start = 1'b0;
    n_cnt = r_cnt;
    addr_cnt_out_ram_inc = 1'b0;
    addr_cnt_out_ram_sub = 1'b0;
    addr_cnt_out_ram_clr = 1'b0;
    mem_we = 1'b0;
    clr_fetch_line1  = 1'b0;
    clr_fetch_line2  = 1'b0;
    clr_fetch_line3  = 1'b0;
    clr_fetch_line4  = 1'b0;
    
    case(fsm_cs)
         
        FSM_IDLE: 
        begin
            fsm_ns = FSM_RD_REQ;
            n_cnt = 'd0;
            l_cnt = 'd0;
            l_cnt2 = 'd0;
        end
        
        FSM_RD_REQ: 
        begin
            // If there's no more data to copy
            if(floatsReg[7:0] == 8'b11111111)
            begin
                fsm_ns = FSM_RD_REQ2;
                addr_cnt_start = 1'b1;
                l_cnt <= floatsReg[159:128]; 
                $display("wwwb l_cnt_inmem: %d", l_cnt);
                addr_cnt_continuous_inc = 1'b1;
            end 
            else 
            begin
                if(addr_cnt >= 1'b1) 
                begin
                    addr_cnt_clr = 1'b1;
                end             
                else 
                begin               
                    if(!rd_req_almostfull) 
                    begin           
                        rd_req_en = 1'b1;             
                        fsm_ns = FSM_RD_RSP;
                    end
                end
            end
        end
        
        FSM_RD_RSP: 
        begin
            if(rd_rsp_valid) 
            begin
                addr_cnt_inc = 1'b1;
                fsm_ns = FSM_RD_REQ;
            end
        end
        
        FSM_RD_REQ2: 
        begin
            if(addr_cnt >= 5) 
            begin                
                fsm_ns = FSM_RD_DONE;
                addr_cnt_clr = 1'b1;
                //$display("add_cnt_continuous after reading spheres: %d", addr_cnt_continuous);
            end 
            else 
            begin
                if(!rd_req_almostfull) 
                begin           
                    rd_req_en = 1'b1;             
                    fsm_ns = FSM_RD_RSP2;
                end
            end
        end
        
        FSM_RD_RSP2: 
        begin
            addr_cnt_inc = 1'b1;
            addr_cnt_continuous_inc = 1'b1;
            fsm_ns = FSM_RD_REQ2;
        end
        
        FSM_RD_DONE:
        begin
            if(w_done_rd)
            begin
                fsm_ns = FSM_WR_MEM;
                clr_fetch_line1  = 1'b1;
                clr_fetch_line2  = 1'b1;
                clr_fetch_line3  = 1'b1;
                clr_fetch_line4  = 1'b1;	 
            end
        end
        
        FSM_WR_MEM:
        begin
            if(addr_cnt >= 4) 
            begin
                if(addr_cnt_continuous_mem >= l_cnt)
                begin
                    fsm_ns = ENDSTATE;
                    addr_cnt_clr = 1'b1;                    
                end
                else
                begin
                    fsm_ns = FSM_RD_REQ2;
                    addr_cnt_clr = 1'b1;
                end                
            end 
            else 
            begin
                /*if(!rd_req_almostfull) 
                begin           
                    rd_req_en = 1'b1;*/             
                    fsm_ns = FSM_WR_MEM_EN;
                //end
            end
        end
        
        FSM_WR_MEM_EN: 
        begin
            mem_we = 1'b1;
            addr_cnt_inc = 1'b1;
            addr_cnt_continuous_mem_inc = 1'b1;
            fsm_ns = FSM_WR_MEM;
        end    
          
        ENDSTATE:
        begin
            done   = 1'b1;
        end
      endcase
   end

endmodule