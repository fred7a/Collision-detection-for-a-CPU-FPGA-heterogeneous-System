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

module dCalcVectorLength3
(
	input CLK,
	input [31:0] a1,
	input [31:0] a2,
	input [31:0] a3,
	input [31:0] b1,
	input [31:0] b2,
	input [31:0] b3,
	input RST,
	output reg [31:0] res,
	output reg out_rdy
);

wire CLK2;
wire [31:0] out_mult1;
wire output_z_ack_wire;
wire done_mult1;
wire input_a_ack_wire;
wire input_b_ack_wire;

wire [31:0] out_mult2;
wire output_z_ack_wire1;
wire done_mult2;
wire input_a_ack_wire1;
wire input_b_ack_wire1;

wire [31:0] out_mult3;
wire output_z_ack_wire2;
wire done_mult3;
wire input_a_ack_wire2;
wire input_b_ack_wire2;

wire [31:0] resultadd3;
reg resetadd3 = 1'b1;
wire out_rdy_adder;



reg resetsqrt = 1'b1;
wire iddle;
wire [31:0] rootcalc;
wire out_rdy_sqrt;

assign CLK2 = (done_mult1 & done_mult2 & done_mult3) ? 1:0;

mult_comb	mult1 (
	.clk_en ( 1'b1 ),
	.clock ( CLK ),
	.dataa (a1),
	.datab (b1),
	.result ( out_mult1 ),
	.reset(RST),
	.done(done_mult1)
	);	
	
mult_comb	mult2 (
	.clk_en ( 1'b1 ),
	.clock ( CLK ),
	.dataa (a2),
	.datab (b2),
	.result ( out_mult2 ),
	.reset(RST),
	.done(done_mult2)
	);	
	
mult_comb	mult3 (
	.clk_en ( 1'b1 ),
	.clock ( CLK ),
	.dataa (a3),
	.datab (b3),
	.result ( out_mult3 ),
	.reset(RST),
	.done(done_mult3)
	);	

add3 adder
(
	.CLK(CLK),
	.a1(out_mult1),
	.a2(out_mult2),
	.a3(out_mult3),
	.reset(resetadd3),
	.result(resultadd3),
	.out_rdy(out_rdy_adder)
);

approx_fp_sqrt sqrt(
	.in(resultadd3),
	.out(rootcalc)
);


always @(posedge CLK2 or posedge out_rdy_adder or posedge out_rdy_sqrt or negedge RST)
begin
	if(RST == 1'b0)
	begin
		resetadd3 <= 1'b0;
		resetsqrt <= 1'b0;
		out_rdy <= 1'b0;
	end
	else
	begin
		if(CLK2)
		begin
			resetadd3 <= 1'b1;
		end
		if(out_rdy_adder)
		begin
			resetsqrt <= 1'b1;
			res <= rootcalc;
			out_rdy <= 1'b1;
		end
	end
	
end


endmodule

