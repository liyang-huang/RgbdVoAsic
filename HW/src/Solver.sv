// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2023


module Solver
    import RgbdVoConfigPk::*;
#(
)(
    // input
     input                 i_clk
    ,input                 i_rst_n
    ,input                 i_start
    ,input [MATRIX_BW-1:0] i_Mat_00
    ,input [MATRIX_BW-1:0] i_Mat_10
    ,input [MATRIX_BW-1:0] i_Mat_20
    ,input [MATRIX_BW-1:0] i_Mat_30
    ,input [MATRIX_BW-1:0] i_Mat_40
    ,input [MATRIX_BW-1:0] i_Mat_50
    ,input [MATRIX_BW-1:0] i_Mat_11
    ,input [MATRIX_BW-1:0] i_Mat_21
    ,input [MATRIX_BW-1:0] i_Mat_31
    ,input [MATRIX_BW-1:0] i_Mat_41
    ,input [MATRIX_BW-1:0] i_Mat_51
    ,input [MATRIX_BW-1:0] i_Mat_22
    ,input [MATRIX_BW-1:0] i_Mat_32
    ,input [MATRIX_BW-1:0] i_Mat_42
    ,input [MATRIX_BW-1:0] i_Mat_52
    ,input [MATRIX_BW-1:0] i_Mat_33
    ,input [MATRIX_BW-1:0] i_Mat_43
    ,input [MATRIX_BW-1:0] i_Mat_53
    ,input [MATRIX_BW-1:0] i_Mat_44
    ,input [MATRIX_BW-1:0] i_Mat_54
    ,input [MATRIX_BW-1:0] i_Mat_55
    ,input [MATRIX_BW-1:0] i_Vec_0
    ,input [MATRIX_BW-1:0] i_Vec_1
    ,input [MATRIX_BW-1:0] i_Vec_2
    ,input [MATRIX_BW-1:0] i_Vec_3
    ,input [MATRIX_BW-1:0] i_Vec_4
    ,input [MATRIX_BW-1:0] i_Vec_5
    // Output
    ,output logic                 o_done
    ,output logic                 o_div_zero
    ,output logic [MATRIX_BW-1:0] o_X0
    ,output logic [MATRIX_BW-1:0] o_X1
    ,output logic [MATRIX_BW-1:0] o_X2
    ,output logic [MATRIX_BW-1:0] o_X3
    ,output logic [MATRIX_BW-1:0] o_X4
    ,output logic [MATRIX_BW-1:0] o_X5
);

    //=================================
    // Signal Declaration
    //=================================
    localparam IDLE = 1'b0 , BUSY = 1'b1;
    logic  state_r, state_w;
    logic signed [MATRIX_BW - 1 : 0] X_r [6];	
    logic signed [2 * MATRIX_BW - 1 : 0] X_w [6];
    logic signed [MATRIX_BW - 1 : 0] D_w [6];	
    logic signed [MATRIX_BW - 1 : 0] L_w [15];	
    logic signed [MATRIX_BW - 1 : 0] D_r [6];	
    logic signed [MATRIX_BW - 1 : 0] L_r [15];	
    logic [7:0] cnt_r, cnt_w;
    genvar i;
    integer j, m, n;
    logic signed [MATRIX_BW+MUL-1:0] a;
    logic signed [MATRIX_BW-1:0] b;
    logic signed [MATRIX_BW+MUL-1:0] quotient;
    logic signed [MATRIX_BW-1:0] c, d;
    logic signed [MATRIX_BW+MATRIX_BW-1:0] product;
    logic signed [MATRIX_BW+MATRIX_BW:0] product_add;
    logic signed [MATRIX_BW+MATRIX_BW-MUL:0] product_shift_r;
    logic  done_r;

    //=================================
    // Combinational Logic
    //=================================
    assign cnt_w = (state_r == IDLE)? 0: cnt_r + 1;
    assign o_done = done_r;
    assign o_X0 = X_r[0];
    assign o_X1 = X_r[1];
    assign o_X2 = X_r[2];
    assign o_X3 = X_r[3];
    assign o_X4 = X_r[4];
    assign o_X5 = X_r[5];
	

    assign product_add = product + $signed({1'b0,{MUL{1'b1}}});

    always_comb begin
    	case(state_r)
    		IDLE : begin if(i_start == 1) state_w = BUSY; else state_w = IDLE; end
    		BUSY : begin if(done_r == 1) state_w = IDLE; else state_w = BUSY; end
    		default : state_w = IDLE;
    	endcase
    end


    always_comb begin
        case(cnt_r)
            'd46: begin a = {X_r[5],{MUL{1'b0}}}; b = D_r[5]; end
            'd47: begin a = {X_r[5],{MUL{1'b0}}}; b = D_r[5]; end
            'd49: begin a = {X_r[4],{MUL{1'b0}}}; b = D_r[4]; end
            'd50: begin a = {X_r[4],{MUL{1'b0}}}; b = D_r[4]; end
            'd52: begin a = {X_r[3],{MUL{1'b0}}}; b = D_r[3]; end
            'd53: begin a = {X_r[3],{MUL{1'b0}}}; b = D_r[3]; end
            'd55: begin a = {X_r[2],{MUL{1'b0}}}; b = D_r[2]; end
            'd56: begin a = {X_r[2],{MUL{1'b0}}}; b = D_r[2]; end
            'd58: begin a = {X_r[1],{MUL{1'b0}}}; b = D_r[1]; end
            'd59: begin a = {X_r[1],{MUL{1'b0}}}; b = D_r[1]; end
            'd61: begin a = {X_r[0],{MUL{1'b0}}}; b = D_r[0]; end	
            'd62: begin a = {X_r[0],{MUL{1'b0}}}; b = D_r[0]; end	
            default: begin a = '0; b = '1; end
        endcase
    end
    
    DW_div_pipe #(
         .a_width(MATRIX_BW+MUL)
        ,.b_width(MATRIX_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(a)
        ,.b(b)
        ,.quotient(quotient)
        ,.remainder()
        ,.divide_by_0()
    );

    always_comb begin
        case(cnt_r)
            'd1 : begin c = L_r[0]; d = X_r[0]; end
            'd4 : begin c = L_r[1]; d = X_r[0]; end
            'd7 : begin c = L_r[2]; d = X_r[1]; end
            'd10: begin c = L_r[3]; d = X_r[0]; end
            'd13: begin c = L_r[4]; d = X_r[1]; end
            'd16: begin c = L_r[5]; d = X_r[2]; end
            'd19: begin c = L_r[6]; d = X_r[0]; end
            'd22: begin c = L_r[7]; d = X_r[1]; end
            'd25: begin c = L_r[8]; d = X_r[2]; end		
            'd28: begin c = L_r[9]; d = X_r[3]; end
            'd31: begin c = L_r[10]; d = X_r[0]; end	
            'd34: begin c = L_r[11]; d = X_r[1]; end		
            'd37: begin c = L_r[12]; d = X_r[2]; end		
            'd40: begin c = L_r[13]; d = X_r[3]; end	
            'd43: begin c = L_r[14]; d = X_r[4]; end	
            'd64: begin c = L_r[14]; d = X_r[5]; end		
            'd67: begin c = L_r[13]; d = X_r[5]; end		
            'd70: begin c = L_r[9]; d = X_r[4]; end
            'd73: begin c = L_r[12]; d = X_r[5]; end
            'd76: begin c = L_r[8]; d = X_r[4]; end	
            'd79: begin c = L_r[5]; d = X_r[3]; end		
            'd82: begin c = L_r[11]; d = X_r[5]; end	
            'd85: begin c = L_r[7]; d = X_r[4]; end	
            'd88: begin c = L_r[4]; d = X_r[3]; end
            'd91: begin c = L_r[2]; d = X_r[2]; end	
            'd94: begin c = L_r[10]; d = X_r[5]; end		
            'd97: begin c = L_r[6]; d = X_r[4]; end		
            'd100: begin c = L_r[3]; d = X_r[3]; end		
            'd103: begin c = L_r[1]; d = X_r[2]; end		
            'd106: begin c = L_r[0]; d = X_r[1]; end	
            default: begin c = '0; d = '0; end
        endcase
    end
    
    DW_mult_pipe #(
         .a_width(MATRIX_BW)
        ,.b_width(MATRIX_BW)
        ,.num_stages(2)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_mult (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.tc(1'b1)
        ,.a(c)
        ,.b(d)
        ,.product(product)
    );

    always_comb begin
        if(state_r == IDLE) begin
            X_w[0]  = i_Vec_0;
            X_w[1]  = i_Vec_1;
            X_w[2]  = i_Vec_2;
            X_w[3]  = i_Vec_3;
            X_w[4]  = i_Vec_4;
            X_w[5]  = i_Vec_5;
	end
        else begin
            for (j= 0; j < 6 ; j = j + 1) 
                X_w[j] = X_r[j];				
            case(cnt_r)
                'd3 : X_w[1] = X_r[1] - product_shift_r; 
                'd6 : X_w[2] = X_r[2] - product_shift_r; 
                'd9 : X_w[2] = X_r[2] - product_shift_r; 
                'd12: X_w[3] = X_r[3] - product_shift_r; 
                'd15: X_w[3] = X_r[3] - product_shift_r; 
                'd18: X_w[3] = X_r[3] - product_shift_r; 
                'd21: X_w[4] = X_r[4] - product_shift_r; 
                'd24: X_w[4] = X_r[4] - product_shift_r; 
                'd27: X_w[4] = X_r[4] - product_shift_r; 		
                'd30: X_w[4] = X_r[4] - product_shift_r; 
                'd33: X_w[5] = X_r[5] - product_shift_r; 	
                'd36: X_w[5] = X_r[5] - product_shift_r; 	
                'd39: X_w[5] = X_r[5] - product_shift_r; 		
                'd42: X_w[5] = X_r[5] - product_shift_r; 
                'd45: X_w[5] = X_r[5] - product_shift_r; 
                'd48: X_w[5] = quotient;
                'd51: X_w[4] = quotient; 
                'd54: X_w[3] = quotient; 
                'd57: X_w[2] = quotient; 
                'd60: X_w[1] = quotient; 
                'd63: X_w[0] = quotient; 
                'd66: X_w[4] = X_r[4] - product_shift_r; 
                'd69: X_w[3] = X_r[3] - product_shift_r; 
                'd72: X_w[3] = X_r[3] - product_shift_r; 
                'd75: X_w[2] = X_r[2] - product_shift_r; 
                'd78: X_w[2] = X_r[2] - product_shift_r; 
                'd81: X_w[2] = X_r[2] - product_shift_r; 
                'd84: X_w[1] = X_r[1] - product_shift_r; 
                'd87: X_w[1] = X_r[1] - product_shift_r; 
                'd90: X_w[1] = X_r[1] - product_shift_r; 
                'd93: X_w[1] = X_r[1] - product_shift_r; 
                'd96: X_w[0] = X_r[0] - product_shift_r; 
                'd99: X_w[0] = X_r[0] - product_shift_r; 
                'd102: X_w[0] = X_r[0] - product_shift_r; 
                'd105: X_w[0] = X_r[0] - product_shift_r; 
                'd108: X_w[0] = X_r[0] - product_shift_r; 
                default: begin for (j = 0; j < 6 ; j = j + 1) X_w[j] = X_r[j]; end
            endcase			
        end
    end
	
    always_comb begin
        if(i_start) begin
            D_w[0] = i_Mat_00;
            D_w[1] = i_Mat_11;
            D_w[2] = i_Mat_22;
            D_w[3] = i_Mat_33;
            D_w[4] = i_Mat_44;
            D_w[5] = i_Mat_55;
	end
        else begin
            for (m= 0; m < 6 ; m = m + 1) 
                D_w[m] = D_r[m];				
        end
    end
	
    always_comb begin
        if(i_start) begin
            L_w[0] =  i_Mat_10;
            L_w[1] =  i_Mat_20;
            L_w[2] =  i_Mat_21;
            L_w[3] =  i_Mat_30;
            L_w[4] =  i_Mat_31;
            L_w[5] =  i_Mat_32;
            L_w[6] =  i_Mat_40;
            L_w[7] =  i_Mat_41;
            L_w[8] =  i_Mat_42;
            L_w[9] =  i_Mat_43;
            L_w[10] = i_Mat_50;
            L_w[11] = i_Mat_51;
            L_w[12] = i_Mat_52;
            L_w[13] = i_Mat_53;
            L_w[14] = i_Mat_54;
	end
        else begin
            for (n= 0; n < 15 ; n = n + 1) 
                L_w[n] = L_r[n];				
        end
    end
	

    //===================
    //    Sequential
    //===================
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) state_r <= IDLE;
        else  state_r <= state_w;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) cnt_r <= '0;
        else  cnt_r <= cnt_w;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) done_r <= '0;
        else  done_r <= (cnt_r == 'd109);
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) product_shift_r <= '0;
        else  product_shift_r <= (product[MATRIX_BW+MATRIX_BW-1])? product_add[MATRIX_BW+MATRIX_BW:MUL] : product[MATRIX_BW+MATRIX_BW-1:MUL];
    end

    generate
    for (i = 0; i < 6 ; i = i + 1) begin
        always_ff @(posedge i_clk or negedge i_rst_n) begin
            if (!i_rst_n)      X_r[i] <= '0;
            else  X_r[i] <= X_w[i];
        end
        always_ff @(posedge i_clk or negedge i_rst_n) begin
            if (!i_rst_n)      D_r[i] <= '0;
            else  D_r[i] <= D_w[i];
        end
    end
    for (i = 0; i < 15 ; i = i + 1) begin
        always_ff @(posedge i_clk or negedge i_rst_n) begin
            if (!i_rst_n)      L_r[i] <= '0;
            else  L_r[i] <= L_w[i];
        end
    end
    endgenerate

endmodule

