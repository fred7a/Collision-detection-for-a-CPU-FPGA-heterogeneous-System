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
//`define DEBUG 2
//`define DEBUGOUT 3
//`define DEBUGIN 4
module afu_user #(ADDR_LMT = 20, MDATA = 14, CACHE_WIDTH = 512) 
(
   input                   clk,                
   input                   reset_n,                  
   
   // Read Request
   output [ADDR_LMT-1:0]   rd_req_addr,           
   output [MDATA-1:0] 	   rd_req_mdata,            
   output              rd_req_en,             
   input                   rd_req_almostfull,           
   
   // Read Response
   input                   rd_rsp_valid,       
   input [MDATA-1:0] 	   rd_rsp_mdata,            
   input [CACHE_WIDTH-1:0] rd_rsp_data,

   // Write Request 
   output [ADDR_LMT-1:0]    wr_req_addr,           
   output [MDATA-1:0] 	    wr_req_mdata,            
   output [CACHE_WIDTH-1:0] wr_req_data,    
   output reg               wr_req_en,             
   input                    wr_req_almostfull,           
   
   // Write Response 
   input                    wr_rsp0_valid,       
   input [MDATA-1:0] 	    wr_rsp0_mdata,            
   input                    wr_rsp1_valid,       
   input [MDATA-1:0] 	    wr_rsp1_mdata,            
   
   // Start input signal
   input                    start,                

   // Done output signal 
   output reg               done,          

   // Control info from software
   input [511:0] 	    afu_context
);

    





   reg execcounter_clr;
   reg data_buf_we; 
   wire [CACHE_WIDTH-1:0] floatsReg;
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
   wire [(DIM*DIM_Q)-1:0]       w_outGrid, w_outGrid2, w_outGrid3,w_outGrid4,w_outGrid5,w_outGrid6,w_outGrid7,w_outGrid8 ;
   reg 	t_start;
   wire w_done_aux1, w_done_aux2, w_done_aux3, w_done_aux4, w_done_aux5, w_done_aux6, w_done_aux7, w_done_aux8, w_done_aux9, w_done_aux10, w_done_aux11, w_done_aux12, w_done_aux13, w_done_aux14, w_done_aux15, w_done_aux16;
   assign w_done = w_done_aux1 && w_done_aux2 && w_done_aux3 && w_done_aux4 && w_done_aux5 && w_done_aux6 && w_done_aux7 && w_done_aux8 && w_done_aux9 && w_done_aux10 && w_done_aux11 && w_done_aux12 && w_done_aux13 && w_done_aux14 && w_done_aux15 && w_done_aux16;
   
   
   assign wr_req_data = (addr_cnt==0)?w_outGrid:(addr_cnt==1)?w_outGrid2:(addr_cnt==2)?w_outGrid3:(addr_cnt==3)?w_outGrid4:(addr_cnt==4)?w_outGrid5:(addr_cnt==5)?w_outGrid6:(addr_cnt==6)?w_outGrid7:(addr_cnt==7)?w_outGrid8:512'b01000000010101111010111000010100010000000101011110101110000101000100000001010111101011100001010001000000010101111010111000010100010000000101011110101110000101000100000001010111101011100001010001000000010101111010111000010100010000000101011110101110000101000100000001010111101011100001010001000000010101111010111000010100010000000101011110101110000101000100000001010111101011100001010001000000010101111010111000010100010000000101011110101110000101000100000001010111101011100001010001000000010101111010111000010100;    
   
   reg mem_we;
     
     
    wire [511:0] wmem_out;
    wire start_fsm;

   inputs_mem2 mem_inputs (
        .clk(clk),                
        .reset_n(reset_n),                  
        
        
        .rd_req_addr(rd_req_addr),           
        .rd_req_mdata(rd_req_mdata),            
        .rd_req_en(rd_req_en),             
        .rd_req_almostfull(rd_req_almostfull),           
        
        
        .rd_rsp_valid(rd_rsp_valid),       
        .rd_rsp_mdata(rd_rsp_mdata),            
        .rd_rsp_data(rd_rsp_data),
        
        .mem_out(floatsReg),
        .addr_mem_out(addr_cnt_continuous),              

        .done(start_fsm)
   );
     
     wire [511:0] out_Mem1;
     wire [511:0] out_Mem2;
     wire [511:0] out_Mem3;
     wire [511:0] out_Mem4;
     wire [511:0] out_Mem5;
     wire [511:0] out_Mem6;
     wire [511:0] out_Mem7;
     wire [511:0] out_Mem8;
     wire [511:0] out_Mem9;
     wire [511:0] out_Mem10;
     wire [511:0] out_Mem11;
     wire [511:0] out_Mem12;
     wire [511:0] out_Mem13;
     wire [511:0] out_Mem14;
     wire [511:0] out_Mem15;
     wire [511:0] out_Mem16;
     wire [511:0] out_Mem17;
     wire [511:0] out_Mem18;
     wire [511:0] out_Mem19;
     wire [511:0] out_Mem20;
     wire [511:0] out_Mem21;
     wire [511:0] out_Mem22;
     wire [511:0] out_Mem23;
     wire [511:0] out_Mem24;
     wire [511:0] out_Mem25;
     wire [511:0] out_Mem26;
     wire [511:0] out_Mem27;
     wire [511:0] out_Mem28;
     wire [511:0] out_Mem29;
     wire [511:0] out_Mem30;
     wire [511:0] out_Mem31;
     wire [511:0] out_Mem32;     
     
        ram_rw_512x128_700 rammem1 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[15:0]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem1));

        ram_rw_512x128_700 rammem2 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[31:16]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem2));

        ram_rw_512x128_700 rammem3 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[47:32]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem3));

        ram_rw_512x128_700 rammem4 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[63:48]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem4));

        ram_rw_512x128_700 rammem5 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[79:64]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem5));

        ram_rw_512x128_700 rammem6 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[95:80]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem6));

        ram_rw_512x128_700 rammem7 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[111:96]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem7));

        ram_rw_512x128_700 rammem8 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[127:112]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem8));

        ram_rw_512x128_700 rammem9 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[143:128]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem9));

        ram_rw_512x128_700 rammem10 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[159:144]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem10));

        ram_rw_512x128_700 rammem11 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[175:160]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem11));

        ram_rw_512x128_700 rammem12 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[191:176]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem12));

        ram_rw_512x128_700 rammem13 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[207:192]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem13));

        ram_rw_512x128_700 rammem14 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[223:208]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem14));

        ram_rw_512x128_700 rammem15 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[239:224]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem15));

        ram_rw_512x128_700 rammem16 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[255:240]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem16));

        ram_rw_512x128_700 rammem17 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[271:256]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem17));

        ram_rw_512x128_700 rammem18 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[287:272]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem18));

        ram_rw_512x128_700 rammem19 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[303:288]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem19));

        ram_rw_512x128_700 rammem20 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[319:304]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem20));

        ram_rw_512x128_700 rammem21 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[335:320]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem21));

        ram_rw_512x128_700 rammem22 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[351:336]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem22));

        ram_rw_512x128_700 rammem23 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[367:352]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem23));

        ram_rw_512x128_700 rammem24 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[383:368]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem24));

        ram_rw_512x128_700 rammem25 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[399:384]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem25));

        ram_rw_512x128_700 rammem26 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[415:400]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem26));

        ram_rw_512x128_700 rammem27 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[431:416]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem27));

        ram_rw_512x128_700 rammem28 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[447:432]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem28));

        ram_rw_512x128_700 rammem29 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[463:448]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem29));

        ram_rw_512x128_700 rammem30 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[479:464]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem30));

        ram_rw_512x128_700 rammem31 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[495:480]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem31));

        ram_rw_512x128_700 rammem32 (

                .clock(clk),

                .data(floatsReg),

                .rdaddress(floatsReg[511:496]),

                .wraddress(addr_cnt_continuous),

                .wren(mem_we),

                .q(out_Mem32));
    
    wire [31:0] IDLE_AUX1, IDLE_AUX2, IDLE_AUX3, IDLE_AUX4, IDLE_AUX5, IDLE_AUX6, IDLE_AUX7, IDLE_AUX8, IDLE_AUX9, IDLE_AUX10, IDLE_AUX11, IDLE_AUX12,IDLE_AUX13,IDLE_AUX14,IDLE_AUX15,IDLE_AUX16;
	
	reg [31:0] r_cnt,n_cnt,l_cnt,l_cnt2,l_cnt3;
    
    assign w_outGrid[255:224] =  32'b00111111110110011001100110011010; //1.7
    assign w_outGrid[511:480] =  l_cnt;
    
    assign w_outGrid2[255:224] = 32'b01000000001011001100110011001101; //2.7
    assign w_outGrid2[511:480] = l_cnt2;
    
    assign w_outGrid3[255:224] = 32'b01000000011011001100110011001101; //3.7
    assign w_outGrid3[511:480] = l_cnt3;
    
    assign w_outGrid4[255:224] = 32'b01000000100101100110011001100110; //4.7
    assign w_outGrid4[511:480] = addr_cnt_continuous;
	
    assign w_outGrid5[255:224] = 32'b01000000101101100110011001100110; //5.7
    assign w_outGrid5[511:480] = floatsReg[495:480];
    
    assign w_outGrid6[255:224] = 32'b01000000110101100110011001100110; //6.7
    assign w_outGrid6[511:480] = out_Mem31[31:0];
                                
    assign w_outGrid7[255:224] = 32'b01000000111101100110011001100110; //7.7
    assign w_outGrid7[511:480] = floatsReg[511:496];
                                
    assign w_outGrid8[255:224] = 32'b01000001000010110011001100110011; //8.7
    assign w_outGrid8[511:480] = out_Mem32[31:0];
    
        dCollideSpheres collisionHandler1 (

                     .x1(out_Mem1[31:0]),

                     .y1(out_Mem1[63:32]),

                     .z1(out_Mem1[95:64]),

                     .r1(out_Mem1[127:96]),

                     .x2(out_Mem2[31:0]),

                     .y2(out_Mem2[63:32]),

                     .z2(out_Mem2[95:64]),

                     .r2(out_Mem2[127:96]),

                     .cx(w_outGrid[31:0]),

                     .cy(w_outGrid[63:32]),

                     .cz(w_outGrid[95:64]),

                .normalx(w_outGrid[127:96]),

                .normaly(w_outGrid[159:128]),

                .normalz(w_outGrid[191:160]),

                  .depth(w_outGrid[223:192]),

                    .ret(IDLE_AUX1),

                   .done(w_done_aux1),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );


        dCollideSpheres collisionHandler2 (

                     .x1(out_Mem3[31:0]),

                     .y1(out_Mem3[63:32]),

                     .z1(out_Mem3[95:64]),

                     .r1(out_Mem3[127:96]),

                     .x2(out_Mem4[31:0]),

                     .y2(out_Mem4[63:32]),

                     .z2(out_Mem4[95:64]),

                     .r2(out_Mem4[127:96]),

                     .cx(w_outGrid[287:256]),

                     .cy(w_outGrid[319:288]),

                     .cz(w_outGrid[351:320]),

                .normalx(w_outGrid[383:352]),

                .normaly(w_outGrid[415:384]),

                .normalz(w_outGrid[447:416]),

                  .depth(w_outGrid[479:448]),

                    .ret(IDLE_AUX2),

                   .done(w_done_aux2),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );

        dCollideSpheres collisionHandler3 (

                     .x1(out_Mem5[31:0]),

                     .y1(out_Mem5[63:32]),

                     .z1(out_Mem5[95:64]),

                     .r1(out_Mem5[127:96]),

                     .x2(out_Mem6[31:0]),

                     .y2(out_Mem6[63:32]),

                     .z2(out_Mem6[95:64]),

                     .r2(out_Mem6[127:96]),

                     .cx(w_outGrid2[31:0]),

                     .cy(w_outGrid2[63:32]),

                     .cz(w_outGrid2[95:64]),

                .normalx(w_outGrid2[127:96]),

                .normaly(w_outGrid2[159:128]),

                .normalz(w_outGrid2[191:160]),

                  .depth(w_outGrid2[223:192]),

                    .ret(IDLE_AUX3),

                   .done(w_done_aux3),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );


        dCollideSpheres collisionHandler4 (

                     .x1(out_Mem7[31:0]),

                     .y1(out_Mem7[63:32]),

                     .z1(out_Mem7[95:64]),

                     .r1(out_Mem7[127:96]),

                     .x2(out_Mem8[31:0]),

                     .y2(out_Mem8[63:32]),

                     .z2(out_Mem8[95:64]),

                     .r2(out_Mem8[127:96]),

                     .cx(w_outGrid2[287:256]),

                     .cy(w_outGrid2[319:288]),

                     .cz(w_outGrid2[351:320]),

                .normalx(w_outGrid2[383:352]),

                .normaly(w_outGrid2[415:384]),

                .normalz(w_outGrid2[447:416]),

                  .depth(w_outGrid2[479:448]),

                    .ret(IDLE_AUX4),

                   .done(w_done_aux4),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );

        dCollideSpheres collisionHandler5 (

                     .x1(out_Mem9[31:0]),

                     .y1(out_Mem9[63:32]),

                     .z1(out_Mem9[95:64]),

                     .r1(out_Mem9[127:96]),

                     .x2(out_Mem10[31:0]),

                     .y2(out_Mem10[63:32]),

                     .z2(out_Mem10[95:64]),

                     .r2(out_Mem10[127:96]),

                     .cx(w_outGrid3[31:0]),

                     .cy(w_outGrid3[63:32]),

                     .cz(w_outGrid3[95:64]),

                .normalx(w_outGrid3[127:96]),

                .normaly(w_outGrid3[159:128]),

                .normalz(w_outGrid3[191:160]),

                  .depth(w_outGrid3[223:192]),

                    .ret(IDLE_AUX5),

                   .done(w_done_aux5),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );


        dCollideSpheres collisionHandler6 (

                     .x1(out_Mem11[31:0]),

                     .y1(out_Mem11[63:32]),

                     .z1(out_Mem11[95:64]),

                     .r1(out_Mem11[127:96]),

                     .x2(out_Mem12[31:0]),

                     .y2(out_Mem12[63:32]),

                     .z2(out_Mem12[95:64]),

                     .r2(out_Mem12[127:96]),

                     .cx(w_outGrid3[287:256]),

                     .cy(w_outGrid3[319:288]),

                     .cz(w_outGrid3[351:320]),

                .normalx(w_outGrid3[383:352]),

                .normaly(w_outGrid3[415:384]),

                .normalz(w_outGrid3[447:416]),

                  .depth(w_outGrid3[479:448]),

                    .ret(IDLE_AUX6),

                   .done(w_done_aux6),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );

        dCollideSpheres collisionHandler7 (

                     .x1(out_Mem13[31:0]),

                     .y1(out_Mem13[63:32]),

                     .z1(out_Mem13[95:64]),

                     .r1(out_Mem13[127:96]),

                     .x2(out_Mem14[31:0]),

                     .y2(out_Mem14[63:32]),

                     .z2(out_Mem14[95:64]),

                     .r2(out_Mem14[127:96]),

                     .cx(w_outGrid4[31:0]),

                     .cy(w_outGrid4[63:32]),

                     .cz(w_outGrid4[95:64]),

                .normalx(w_outGrid4[127:96]),

                .normaly(w_outGrid4[159:128]),

                .normalz(w_outGrid4[191:160]),

                  .depth(w_outGrid4[223:192]),

                    .ret(IDLE_AUX7),

                   .done(w_done_aux7),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );


        dCollideSpheres collisionHandler8 (

                     .x1(out_Mem15[31:0]),

                     .y1(out_Mem15[63:32]),

                     .z1(out_Mem15[95:64]),

                     .r1(out_Mem15[127:96]),

                     .x2(out_Mem16[31:0]),

                     .y2(out_Mem16[63:32]),

                     .z2(out_Mem16[95:64]),

                     .r2(out_Mem16[127:96]),

                     .cx(w_outGrid4[287:256]),

                     .cy(w_outGrid4[319:288]),

                     .cz(w_outGrid4[351:320]),

                .normalx(w_outGrid4[383:352]),

                .normaly(w_outGrid4[415:384]),

                .normalz(w_outGrid4[447:416]),

                  .depth(w_outGrid4[479:448]),

                    .ret(IDLE_AUX8),

                   .done(w_done_aux8),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );

        dCollideSpheres collisionHandler9 (

                     .x1(out_Mem17[31:0]),

                     .y1(out_Mem17[63:32]),

                     .z1(out_Mem17[95:64]),

                     .r1(out_Mem17[127:96]),

                     .x2(out_Mem18[31:0]),

                     .y2(out_Mem18[63:32]),

                     .z2(out_Mem18[95:64]),

                     .r2(out_Mem18[127:96]),

                     .cx(w_outGrid5[31:0]),

                     .cy(w_outGrid5[63:32]),

                     .cz(w_outGrid5[95:64]),

                .normalx(w_outGrid5[127:96]),

                .normaly(w_outGrid5[159:128]),

                .normalz(w_outGrid5[191:160]),

                  .depth(w_outGrid5[223:192]),

                    .ret(IDLE_AUX9),

                   .done(w_done_aux9),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );


        dCollideSpheres collisionHandler10 (

                     .x1(out_Mem19[31:0]),

                     .y1(out_Mem19[63:32]),

                     .z1(out_Mem19[95:64]),

                     .r1(out_Mem19[127:96]),

                     .x2(out_Mem20[31:0]),

                     .y2(out_Mem20[63:32]),

                     .z2(out_Mem20[95:64]),

                     .r2(out_Mem20[127:96]),

                     .cx(w_outGrid5[287:256]),

                     .cy(w_outGrid5[319:288]),

                     .cz(w_outGrid5[351:320]),

                .normalx(w_outGrid5[383:352]),

                .normaly(w_outGrid5[415:384]),

                .normalz(w_outGrid5[447:416]),

                  .depth(w_outGrid5[479:448]),

                    .ret(IDLE_AUX10),

                   .done(w_done_aux10),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );

        dCollideSpheres collisionHandler11 (

                     .x1(out_Mem21[31:0]),

                     .y1(out_Mem21[63:32]),

                     .z1(out_Mem21[95:64]),

                     .r1(out_Mem21[127:96]),

                     .x2(out_Mem22[31:0]),

                     .y2(out_Mem22[63:32]),

                     .z2(out_Mem22[95:64]),

                     .r2(out_Mem22[127:96]),

                     .cx(w_outGrid6[31:0]),

                     .cy(w_outGrid6[63:32]),

                     .cz(w_outGrid6[95:64]),

                .normalx(w_outGrid6[127:96]),

                .normaly(w_outGrid6[159:128]),

                .normalz(w_outGrid6[191:160]),

                  .depth(w_outGrid6[223:192]),

                    .ret(IDLE_AUX11),

                   .done(w_done_aux11),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );


        dCollideSpheres collisionHandler12 (

                     .x1(out_Mem23[31:0]),

                     .y1(out_Mem23[63:32]),

                     .z1(out_Mem23[95:64]),

                     .r1(out_Mem23[127:96]),

                     .x2(out_Mem24[31:0]),

                     .y2(out_Mem24[63:32]),

                     .z2(out_Mem24[95:64]),

                     .r2(out_Mem24[127:96]),

                     .cx(w_outGrid6[287:256]),

                     .cy(w_outGrid6[319:288]),

                     .cz(w_outGrid6[351:320]),

                .normalx(w_outGrid6[383:352]),

                .normaly(w_outGrid6[415:384]),

                .normalz(w_outGrid6[447:416]),

                  .depth(w_outGrid6[479:448]),

                    .ret(IDLE_AUX12),

                   .done(w_done_aux12),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );

        dCollideSpheres collisionHandler13 (

                     .x1(out_Mem25[31:0]),

                     .y1(out_Mem25[63:32]),

                     .z1(out_Mem25[95:64]),

                     .r1(out_Mem25[127:96]),

                     .x2(out_Mem26[31:0]),

                     .y2(out_Mem26[63:32]),

                     .z2(out_Mem26[95:64]),

                     .r2(out_Mem26[127:96]),

                     .cx(w_outGrid7[31:0]),

                     .cy(w_outGrid7[63:32]),

                     .cz(w_outGrid7[95:64]),

                .normalx(w_outGrid7[127:96]),

                .normaly(w_outGrid7[159:128]),

                .normalz(w_outGrid7[191:160]),

                  .depth(w_outGrid7[223:192]),

                    .ret(IDLE_AUX13),

                   .done(w_done_aux13),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );


        dCollideSpheres collisionHandler14 (

                     .x1(out_Mem27[31:0]),

                     .y1(out_Mem27[63:32]),

                     .z1(out_Mem27[95:64]),

                     .r1(out_Mem27[127:96]),

                     .x2(out_Mem28[31:0]),

                     .y2(out_Mem28[63:32]),

                     .z2(out_Mem28[95:64]),

                     .r2(out_Mem28[127:96]),

                     .cx(w_outGrid7[287:256]),

                     .cy(w_outGrid7[319:288]),

                     .cz(w_outGrid7[351:320]),

                .normalx(w_outGrid7[383:352]),

                .normaly(w_outGrid7[415:384]),

                .normalz(w_outGrid7[447:416]),

                  .depth(w_outGrid7[479:448]),

                    .ret(IDLE_AUX14),

                   .done(w_done_aux14),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );

        dCollideSpheres collisionHandler15 (

                     .x1(out_Mem29[31:0]),

                     .y1(out_Mem29[63:32]),

                     .z1(out_Mem29[95:64]),

                     .r1(out_Mem29[127:96]),

                     .x2(out_Mem30[31:0]),

                     .y2(out_Mem30[63:32]),

                     .z2(out_Mem30[95:64]),

                     .r2(out_Mem30[127:96]),

                     .cx(w_outGrid8[31:0]),

                     .cy(w_outGrid8[63:32]),

                     .cz(w_outGrid8[95:64]),

                .normalx(w_outGrid8[127:96]),

                .normaly(w_outGrid8[159:128]),

                .normalz(w_outGrid8[191:160]),

                  .depth(w_outGrid8[223:192]),

                    .ret(IDLE_AUX15),

                   .done(w_done_aux15),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );


        dCollideSpheres collisionHandler16 (

                     .x1(out_Mem31[31:0]),

                     .y1(out_Mem31[63:32]),

                     .z1(out_Mem31[95:64]),

                     .r1(out_Mem31[127:96]),

                     .x2(out_Mem32[31:0]),

                     .y2(out_Mem32[63:32]),

                     .z2(out_Mem32[95:64]),

                     .r2(out_Mem32[127:96]),

                     .cx(w_outGrid8[287:256]),

                     .cy(w_outGrid8[319:288]),

                     .cz(w_outGrid8[351:320]),

                .normalx(w_outGrid8[383:352]),

                .normaly(w_outGrid8[415:384]),

                .normalz(w_outGrid8[447:416]),

                  .depth(w_outGrid8[479:448]),

                    .ret(IDLE_AUX16),

                   .done(w_done_aux16),

                     .g1(32'd1),

                     .g2(32'd1),

                    .clk(clk),

                    .rst(!t_start)

                );
                

                


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
        FSM_RD_REQ3 = 4'd10,
        FSM_RD_RSP3 = 4'd11,
        FSM_AUX1 = 4'd12,
        DONESTATE = 4'd13;

    reg [4:0] fsm_cs, fsm_ns; 
    
   
   
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
    addr_cnt_out_inc = 1'b0;
    addr_cnt_out_sub = 1'b0;
    addr_cnt_out_clr = 1'b0;      
    data_buf_we  = 1'b0;          
    wr_req_en = 1'b0;             
    done = 1'b0;          
    t_start = 1'b0;
    n_cnt = r_cnt;
    addr_cnt_out_ram_inc = 1'b0;
    addr_cnt_out_ram_sub = 1'b0;
    addr_cnt_out_ram_clr = 1'b0;
    mem_we = 1'b0;   
    
    case(fsm_cs)
         
        FSM_IDLE: 
        begin
            fsm_ns = FSM_RD_REQ;
            n_cnt = 'd0;
            l_cnt = 'd0;
            l_cnt2 = 'd0;
			l_cnt3 = 'd0;
        end
        
        FSM_RD_REQ: 
        begin
            // If there's no more data to copy
            if(start_fsm)
            begin
                fsm_ns = FSM_RD_REQ2;
                addr_cnt_start = 1'b1;
                l_cnt <= floatsReg[95:64]; 
                l_cnt2 <=  floatsReg[127:96]; 
				l_cnt3 <= floatsReg[159:128];  
                $display("wwwb l_cnt_afu: %d", l_cnt);
                $display("wwwb l_cnt2_afu: %d", l_cnt2);
                addr_cnt_continuous_inc = 1'b1;
            end 
        end
        
        FSM_RD_REQ2: 
        begin
            if(addr_cnt_continuous > l_cnt) 
            begin                
                fsm_ns = FSM_RD_REQ3;
                addr_cnt_clr = 1'b1;
            end 
            else 
            begin           
                fsm_ns = FSM_RD_RSP2;
            end
        end
        
        FSM_RD_RSP2: 
        begin
            mem_we = 1'b1;
            addr_cnt_inc = 1'b1;
            addr_cnt_continuous_inc = 1'b1;
            fsm_ns = FSM_RD_REQ2;
        end
        
        FSM_RD_REQ3:
        begin
            if(addr_cnt >= 1) 
	      begin
                  fsm_ns = FSM_RUN_COLLISIONS;
                  addr_cnt_clr = 1'b1;
	      end 
	      else 
	      begin            
                  fsm_ns = FSM_RD_RSP3;
              end
        end
        
        FSM_RD_RSP3:
        begin
            addr_cnt_inc = 1'b1;
            addr_cnt_continuous_inc = 1'b1;
            fsm_ns = FSM_RD_REQ3;
        end
        
	FSM_RUN_COLLISIONS:
	  begin
             $display("wwwa These are the mem addresses %b",floatsReg);
             $display("wwwa This is the address continuous %d",addr_cnt_continuous);
	     t_start = 1'b1;
	     fsm_ns = FSM_WAIT_COLLISIONS;
	     
	  end
	FSM_WAIT_COLLISIONS:
	  begin
	     if(w_done)
	       begin
		  fsm_ns = FSM_WR_REQ;
		  addr_cnt_clr = 1'b1; 
	       end
	  end    
        
        FSM_WR_REQ: 
	  begin
             if(addr_cnt >= 8)
             begin
                 fsm_ns = DONESTATE;
                 addr_cnt_clr = 1'b1; 
             end
             else if(!wr_req_almostfull) 
             begin
                 wr_req_en = 1'b1;    // issue wr_req 
                 fsm_ns = FSM_WR_RSP; 	
                 //$display("\n\n\n\n\noutinfo wout 1: %d", floatsReg[31:0]);
                 //$display("outinfo wout 2: %d", floatsReg[63:32]);
                 //$display("outinfo address out: %d", addr_cnt_out);
                 //$display("outData1: %b", w_outGrid);
                 //$display("outData2: %b", w_outGrid2);

             end  
          end
          
	FSM_WR_RSP:
	  begin
                  addr_cnt_out_inc = 1'b1;
		  fsm_ns = FSM_WR_REQ;
		  addr_cnt_inc = 1'b1;
	  end
	  
        DONESTATE:
        begin
           if(addr_cnt_continuous >= l_cnt2)
           begin
               fsm_ns = ENDSTATE;
               addr_cnt_clr = 1'b1;
               addr_cnt_continuous_clr = 1'b1;
               addr_cnt_out_ram_clr = 1'b1;
           end else
           begin
               
               addr_cnt_clr = 1'b1;
               fsm_ns = FSM_RD_REQ3;
           end
        end
          
        ENDSTATE:
        begin
            
            done   = 1'b1;
        end
      endcase
   end

endmodule