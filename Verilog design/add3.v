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

module add3
(
	input CLK,
	input [31:0] a1,
	input [31:0] a2,
	input [31:0] a3,
	input reset,
	output reg [31:0] result,
	output reg out_rdy
);

wire done_add1;
wire done_add2;

wire [31:0] out_add1;
wire [31:0] out_add2;

reg resetadd2 = 1'b0;

add_comb	add1 (
	.clk_en ( 1'b1 ),
	.clock ( CLK ),
	.dataa ( a1 ),
	.datab ( a2 ),
	.result ( out_add1 ),
	.reset(reset),
	.done(done_add1)
	);	
	
add_comb	add2 (
	.clk_en ( 1'b1 ),
	.clock ( CLK ),
	.dataa ( out_add1 ),
	.datab ( a3 ),
	.result ( out_add2 ),
	.reset(resetadd2),
	.done(done_add2)
	);

always @(posedge done_add1 or posedge done_add2 or negedge reset)
begin
	if(reset == 1'b0)
	begin
		resetadd2 <= 1'b0;
		out_rdy <= 1'b0;
	end
	else
	begin
		if(done_add1)
		begin
			resetadd2 <= 1'b1;
		end
		if(done_add2)
		begin
			result <= out_add2;
			out_rdy <= 1'b1;
		end
	end

end


endmodule

