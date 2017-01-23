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

module dCollideSpheres( x1, y1, z1, r1, x2, y2, z2, r2, cx, cy, cz, 
								normalx, normaly, normalz, depth, g1, g2, ret, clk, rst, done, test, test2, test3, test4);
	input[31:0] x1, y1, z1, r1, x2, y2, z2, r2;
	input clk, rst;
	input[31:0] g1, g2;
	output reg[31:0] cx, cy, cz, normalx, normaly, normalz, depth;
	output reg	ret,		//if ret = 0 then data given to dCollideSpheres do not collide
					done;		//done- done flag, ret- arbitrary return value used in C++ code
					
	output reg[31:0] test = 0;
	output reg[31:0] test2;
	output reg[31:0] test3;
	output reg[31:0] test4;	
	reg[31:0] reg_test;
	reg[31:0] reg_test2;
	reg[31:0] reg_test3;
	reg[31:0] REG_test4;
	
	
	
	reg flag = 1'b0;
	
	reg input_a_stb = 1'b0;
/**
	dVector3 p1:		x1, y1, z1
	dReal r1:			r1
	dVector3 p2:		x2, y2, z2
	dReal r2:			r2
	dContactGeom *c:	
		dVector3 pos		cx, cy, cz
		dVector3 normal	normalX, normalY, normalZ
		dReal depth			depth
		dGeomID g1, g2		g1, g2
*/	
	
	
								

	reg ret_val, done_flag;
	
	reg pt_distFlag = 1'b0;
	
	reg CLK, CLK0, CLK1, CLK2, CLK3, CLK4, idle_clk;
	
	reg clk0,clk1;
	
	wire CLK_FINAL;
	

// =====================================ON RST======================================================	
	
	wire[31:0] d;
	wire CLK_d;
	dCalcPointsDistance3 dCalcPointsDist0(
		.CLK(clk),
		.a1(x1),
		.a2(y1),
		.a3(z1),
		.b1(x2),
		.b2(y2),
		.b3(z2),
		.RST(rst),
		.res(d),
		.out_rdy(CLK_d)
	);
	
	//(r1 + r2)
	wire[31:0] radius_sum;
	wire output_z_ack_wire0;
	wire CLK_radius_sum;
	wire input_a_ack_wire0;
	wire input_b_ack_wire0;
	
	add_comb	add0 (
	.clk_en ( 1'b1 ),
	.clock ( clk ),
	.dataa ( r1 ),
	.datab ( r2 ),
	.result ( radius_sum ),
	.reset(rst),
	.done(CLK_radius_sum)
	);
	
	//x2_neg
	wire[31:0] x2_neg;
	assign x2_neg = {~x2[31], x2[30:0]};
	
	//y2_neg
	wire[31:0] y2_neg;
	assign y2_neg = {~y2[31], y2[30:0]};
	
	//z2_neg
	wire[31:0] z2_neg;
	assign z2_neg = {~z2[31], z2[30:0]};
	
	//r1_neg
	wire[31:0] r1_neg;
	assign r1_neg = {~r1[31], r1[30:0]};
	
	//d_neg
	wire[31:0] d_neg;
	assign d_neg = {~d[31], d[30:0]};	
	
	
		//x1 + x2_neg
		wire CLK_xsum;
		wire[31:0] xsum;
		wire xsum_ack;
		wire xsum_input_a_ack;
		wire xsum_input_b_ack;
		add_comb	add_x1_x2neg (
		.clk_en ( 1'b1 ),
		.clock ( clk ),
		.dataa ( x1 ),
		.datab ( x2_neg ),
		.result ( xsum ),
		.reset(rst),
		.done(CLK_xsum)
		);
		
		//y1 + y2_neg
		wire CLK_ysum;
		wire[31:0] ysum;
		wire ysum_ack;
		wire ysum_input_a_ack;
		wire ysum_input_b_ack;
		add_comb    add_y1_y2neg (
		.clk_en ( 1'b1 ),
		.clock ( clk ),
		.dataa ( y1 ),
		.datab ( y2_neg ),
		.result ( ysum ),
		.reset(rst),
		.done(CLK_ysum)
		);	
		
		//z1 + z2_neg
		wire CLK_zsum;
		wire[31:0] zsum;
		wire zsum_ack;
		wire zsum_input_a_ack;
		wire zsum_input_b_ack;
		add_comb    add_z1_z2neg (
		.clk_en ( 1'b1 ),
		.clock ( clk ),
		.dataa ( z1 ),
		.datab ( z2_neg ),
		.result ( zsum ),
		.reset(rst),
		.done(CLK_zsum)
		);
			//(r2 + r1_neg)
		wire CLK_rsum;
		wire[31:0] rsum;
		wire rsum_ack;
		wire rsum_input_a_ack;
		wire rsum_input_b_ack;
		add_comb    add_r2_r1neg (
		.clk_en ( 1'b1 ),
		.clock ( clk ),
		.dataa ( r2 ),
		.datab ( r1_neg ),
		.result ( rsum ),
		.reset(rst),
		.done(CLK_rsum)
		);
	
// =====================================ON RST======================================================	

// =====================================ON CLK1=====================================================	
	
	//d1 = dRecip(d)
	wire CLK_dRecip;
	wire[31:0] d_recip;
	wire[31:0] input_a_dRecip;
	assign input_a_dRecip = 32'b00111111100000000000000000000000;
	wire dRecip_ack;
	wire dRecip_input_a_ack;
	wire dRecip_input_b_ack;
	inv_test	divide_1_d (
	.clk_en ( 1'b1 ),
	.clock ( clk ),
	.data ( d ),
	.result ( d_recip ),
	.reset(CLK1),
	.done(CLK_dRecip)
	);
	
	//x2_neg
	wire[31:0] radius_sum_neg;
	assign radius_sum_neg = {~radius_sum[31], radius_sum[30:0]};	
	
	//check if d > radius_sum
	wire[31:0] diff_res;
	wire diff_z_ack_wire;
	wire CLK_diff_rdy;
	wire diff_a_ack_wire;
	wire diff_b_ack_wire;
	add_comb    addDiff (
	.clk_en ( 1'b1 ),
	.clock ( clk ),
	.dataa ( d ),
	.datab ( radius_sum_neg ),
	.result ( diff_res ),
	.reset(CLK1),
	.done(CLK_diff_rdy)
	);
	
	

		
		//radius_sum + d_neg
		wire CLK_dep;
		wire[31:0] dep;
		wire dep_ack;
		wire dep_input_a_ack;
		wire dep_input_b_ack;
		add_comb    add_radius_sum_dneg (
		.clk_en ( 1'b1 ),
		.clock ( clk ),
		.dataa ( radius_sum ),
		.datab ( d_neg ),
		.result ( dep ),
		.reset(CLK1),
		.done(CLK_dep)
		);
		
			//radius_sum + d_neg = (r2 - r1 - d)
			wire CLK_drsum;
			wire[31:0] drsum;
			wire drsum_ack;
			wire drsum_input_a_ack;
			wire drsum_input_b_ack;
			add_comb    add_rsum_dneg (
			.clk_en ( 1'b1 ),
			.clock ( clk ),
			.dataa ( rsum ),
			.datab ( d_neg ),
			.result ( drsum ),
			.reset(CLK1),
			.done(CLK_drsum)
			);		
		
// =====================================ON CLK1=====================================================	

// =====================================ON CLK2=====================================================	
		
			//xsum * d_recip
			wire CLK_nx;
			wire[31:0] nx;
			wire nx_ack;
			wire nx_input_a_ack;
			wire nx_input_b_ack;
			mult_comb	multiply_xsum_dRecip (
				.clk_en ( 1'b1 ),
				.clock ( clk ),
				.dataa ( xsum ),
				.datab ( d_recip ),
				.result ( nx ),
				.reset(CLK2),
				.done(CLK_nx)
				);
			
			//ysum * d_recip
			wire CLK_ny;
			wire[31:0] ny;
			wire ny_ack;
			wire ny_input_a_ack;
			wire ny_input_b_ack;
			mult_comb	multiply_ysum_dRecip (
				.clk_en ( 1'b1 ),
				.clock ( clk ),
				.dataa ( ysum ),
				.datab ( d_recip ),
				.result ( ny ),
				.reset(CLK2),
				.done(CLK_ny)
				);	
			
			//zsum * d_recip
			wire CLK_nz;
			wire[31:0] nz;
			wire nz_ack;
			wire nz_input_a_ack;
			wire nz_input_b_ack;
			mult_comb	multiply_zsum_dRecip (
				.clk_en ( 1'b1 ),
				.clock ( clk ),
				.dataa ( zsum ),
				.datab ( d_recip ),
				.result ( nz ),
				.reset(CLK2),
				.done(CLK_nz)
				);
			
			//(drsum) / 2
			wire CLK_k;
			wire[31:0] k;
			wire k_ack;
			wire k_input_a_ack;
			wire k_input_b_ack;
			mult_comb	divide_drsum_2 (
				.clk_en ( 1'b1 ),
				.clock ( clk ),
				.dataa ( drsum ),
				.datab ( 32'b00111111000000000000000000000000 ),
				.result ( k ),
				.reset(CLK2),
				.done(CLK_k)
				);		
			

// =====================================ON CLK2=====================================================						
	
// =====================================ON CLK3=====================================================
				
					//nx * k
					wire CLK_nxk;
					wire[31:0] nxk;
					wire nxk_ack;
					wire nxk_input_a_ack;
					wire nxk_input_b_ack;
					mult_comb	multiply_nx_k (
						.clk_en ( 1'b1 ),
						.clock ( clk ),
						.dataa ( nx ),
						.datab ( k ),
						.result ( nxk ),
						.reset(CLK3),
						.done(CLK_nxk)
						);	
					
					//ny * k
					wire CLK_nyk;
					wire[31:0] nyk;
					wire nyk_ack;
					wire nyk_input_a_ack;
					wire nyk_input_b_ack;
					mult_comb	multiply_ny_k (
						.clk_en ( 1'b1 ),
						.clock ( clk ),
						.dataa ( ny ),
						.datab ( k ),
						.result ( nyk ),
						.reset(CLK3),
						.done(CLK_nyk)
						);
					
					//nz * k
					wire CLK_nzk;
					wire[31:0] nzk;
					wire nzk_ack;
					wire nzk_input_a_ack;
					wire nzk_input_b_ack;
					mult_comb	multiply_nz_k (
						.clk_en ( 1'b1 ),
						.clock ( clk ),
						.dataa ( nz ),
						.datab ( k ),
						.result ( nzk ),
						.reset(CLK3),
						.done(CLK_nzk)
						);	
// =====================================ON CLK3=====================================================					

// =====================================ON CLK4=====================================================
			
						//x1 + nxk
						wire CLK_posx;
						wire[31:0] posx;
						wire posx_ack;
						wire posx_input_a_ack;
						wire posx_input_b_ack;
						add_comb    add_x1_nxk (
						.clk_en ( 1'b1 ),
						.clock ( clk ),
						.dataa ( x1 ),
						.datab ( nxk ),
						.result ( posx ),
						.reset(CLK4),
						.done(CLK_posx)
						);
						
						//y1 + nyk
						wire CLK_posy;
						wire[31:0] posy;
						wire posy_ack;
						wire posy_input_a_ack;
						wire posy_input_b_ack;
						add_comb    add_y1_nyk (
						.clk_en ( 1'b1 ),
						.clock ( clk ),
						.dataa ( y1 ),
						.datab ( nyk ),
						.result ( posy ),
						.reset(CLK4),
						.done(CLK_posy)
						);
						
						//z1 + nzk
						wire CLK_posz;
						wire[31:0] posz;
						wire posz_ack;
						wire posz_input_a_ack;
						wire posz_input_b_ack;
						add_comb    add_z1_nzk (
						.clk_en ( 1'b1 ),
						.clock ( clk ),
						.dataa ( z1 ),
						.datab ( nzk ),
						.result ( posz ),
						.reset(CLK4),
						.done(CLK_posz)
						);	
						
// =====================================ON CLK4=====================================================	
assign CLK_FINAL = ~clk0 && ~clk1 && CLK_posx && CLK_posy && CLK_posz;
					
	
	always @(posedge CLK_d or posedge CLK_diff_rdy or posedge CLK_k or posedge CLK_radius_sum or posedge CLK_dRecip or posedge CLK_xsum or posedge CLK_ysum or posedge CLK_zsum or posedge CLK_rsum or posedge CLK_dep or posedge CLK_nx or posedge CLK_ny or posedge CLK_nz or posedge CLK_drsum or posedge CLK_nxk or posedge CLK_nyk or posedge CLK_nzk or posedge CLK_posx or posedge CLK_posy or posedge CLK_posz or negedge rst)
	begin
		if(rst == 1'b0)
		begin
			input_a_stb <= 1'b0;
			CLK1 <= 1'b0;
			CLK2 <= 1'b0;
			CLK3 <= 1'b0;
			CLK4 <= 1'b0;
			clk0 <= 1'b0;
			clk1 <= 1'b0;
			
			//cx <= 32'd0;
			//		
			//cy <= 32'd0;
			//	
			//cz <= 32'd0;
			//		
			//normalx <= 32'd0;
			//normaly <= 32'd0;
			//normalz <= 32'd0;
			//depth <= 32'd0;
			ret <= 1'b0;
			done <= 1'b0;	
			
		end
		else
		begin
			if(CLK_d)
			begin
				if(CLK_rsum)
				begin
					if(CLK_xsum)
					begin
						if(CLK_ysum)
						begin
							if(CLK_zsum)
							begin
								if(CLK_radius_sum)
								begin
									test <= d;
									input_a_stb <= 1'b1;
									CLK1 <= 1'b1;
								end
							end
						end
					end
				end
			end
			
			if(CLK_diff_rdy)
			begin
				if(CLK_dRecip)
				begin
					if(CLK_dep)
					begin
						if(CLK_drsum)
						begin
							if(diff_res[31] == 1'b0)
							begin
								cx <= 32'd0;
								cy <= 32'd0;
								cz <= 32'd0;
								normalx <= 32'd0;
								normaly <= 32'd0;
								normalz <= 32'd0;
								depth <= 32'd0;
								ret <= 1'b0;
								done <= 1'b1;
								clk0 <= 1'b1;
							end
							else
							begin
								clk0 <= 1'b0;
								if(d == 32'b00000000000000000000000000000000 || d[31] == 1'b1)
								begin
									clk1 <= 1'b1;
									idle_clk <= 1'b1;
								end
								else
								begin
									clk1 <= 1'b0;
									CLK2 <= 1'b1;
								end
							end
						end
					end
				end
			end
			
			if(CLK_nx)
			begin
				if(CLK_ny)
				begin
					if(CLK_nz)
					begin
						if(CLK_k)
						begin
							CLK3 <= 1'b1;
						end
					end	
				end
			end
			
			if(CLK_nxk)
			begin
				if(CLK_nyk)
				begin
					if(CLK_nzk)
					begin
						CLK4 <= 1'b1;
					end	
				end
			end			
			
			if(CLK_posx)
			begin
				if(CLK_posy)
				begin
					if(CLK_posz)
					begin
						cx <= (clk0) ? 32'd0:
								(clk1) ? x1:posx;
						cy <= (clk0) ? 32'd0:
								(clk1) ? y1:posy;
						cz <= (clk0) ? 32'd0:
								(clk1) ? z1:posz;
						normalx <= (clk0) ? 32'd0:
								(clk1) ? 32'd1:nx;
						normaly <= (clk0) ? 32'd0:
								(clk1) ? 32'd0:ny;
						normalz <= (clk0) ? 32'd0:
								(clk1) ? 32'd0:nz;
						depth <= (clk0) ? 32'd0:
								(clk1) ? radius_sum:dep;
						ret <= (clk0) ? 1'b0:1'b1;
						done <= 1'b1;	
						test <= test + 1;
					end	
				end
			end	
		end		
	end

	

endmodule
