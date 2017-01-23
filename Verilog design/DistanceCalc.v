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

module dCalcPointsDistance3
(
	input CLK,
	input [31:0] a1,
	input [31:0] a2,
	input [31:0] a3,
	input [31:0] b1,
	input [31:0] b2,
	input [31:0] b3,
	input RST,
	output [31:0] res,
	output reg out_rdy
	
);

wire CLK2;

wire [31:0] b1_neg;
wire [31:0] b2_neg;
wire [31:0] b3_neg;

wire [31:0] out_add1;
wire output_z_ack_wire;
wire done_add1;
wire input_a_ack_wire;
wire input_b_ack_wire;

wire [31:0] out_add2;
wire output_z_ack_wire1;
wire done_add2;
wire input_a_ack_wire1;
wire input_b_ack_wire1;

wire [31:0] out_add3;
wire output_z_ack_wire2;
wire done_add3;
wire input_a_ack_wire2;
wire input_b_ack_wire2;

wire out_rdy_calclen;

reg resetlen = 1'b1;

assign b1_neg = {~b1[31],b1[30:0]};
assign b2_neg = {~b2[31],b2[30:0]};
assign b3_neg = {~b3[31],b3[30:0]};

assign CLK2 = (done_add1 & done_add2 & done_add3) ? 1:0;

//jtag_debug jtag8(.in_debug(debugtest1));
//
//jtag_debug debug6(.in_debug(out_add2));
//
//jtag_debug debug9(.in_debug(out_add3));

add_comb	add1 (
	.clk_en ( 1'b1 ),
	.clock ( CLK ),
	.dataa ( a1 ),
	.datab ( b1_neg ),
	.result ( out_add1 ),
	.reset(RST),
	.done(done_add1)
	);
	
add_comb	add2 (
	.clk_en ( 1'b1 ),
	.clock ( CLK ),
	.dataa ( a2 ),
	.datab ( b2_neg ),
	.result ( out_add2 ),
	.reset(RST),
	.done(done_add2)
	);
	
add_comb	add3 (
	.clk_en ( 1'b1 ),
	.clock ( CLK ),
	.dataa ( a3 ),
	.datab ( b3_neg ),
	.result ( out_add3 ),
	.reset(RST),
	.done(done_add3)
	);

dCalcVectorLength3 calcLen
(
	.CLK(CLK),
	.a1(out_add1),
	.a2(out_add2),
	.a3(out_add3),
	.b1(out_add1),
	.b2(out_add2),
	.b3(out_add3),
	.RST(resetlen),
	.res(res),
	.out_rdy(out_rdy_calclen)
);

reg [31:0] debugtest1;
reg debugtest2;

always @(posedge CLK2 or negedge RST or posedge out_rdy_calclen)
begin
	if(RST == 1'b0)
	begin
		resetlen <= 1'b0;
		out_rdy <= 1'b0;
	end
	else
	begin
		if(CLK2)
		begin
			resetlen <= 1'b1;	
		end
		
		if(out_rdy_calclen)
		begin
			out_rdy <= 1'b1;
			debugtest1 <= res;
		end
	end
end


endmodule

