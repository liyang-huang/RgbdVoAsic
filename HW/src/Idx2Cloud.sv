// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2022

`ifndef __IDX2CLOUD_SV__
`define __IDX2CLOUD_SV__

`include "common/RgbdVoConfigPk.sv"
//`include "./DW02_mult_2_stage.v"
//`include "./DW_div.v"
//`include "./DW_div_pipe.v"
//`include "./DataDelay.sv"

module Idx2Cloud
    import RgbdVoConfigPk::*;
#(
)(
    // input
     input                            i_clk
    ,input                            i_rst_n
    ,input                            i_valid
    ,input        [H_SIZE_BW-1:0]     i_idx_x
    ,input        [V_SIZE_BW-1:0]     i_idx_y
    ,input        [DATA_DEPTH_BW-1:0] i_depth
    // Register
    ,input        [FX_BW-1:0]         r_fx
    ,input        [FY_BW-1:0]         r_fy
    ,input        [CX_BW-1:0]         r_cx
    ,input        [CY_BW-1:0]         r_cy
    // Output
    ,output logic                     o_valid
    ,output logic [CLOUD_BW-1:0]      o_cloud_x
    ,output logic [CLOUD_BW-1:0]      o_cloud_y
    ,output logic [CLOUD_BW-1:0]      o_cloud_z
);

    //=================================
    // Signal Declaration
    //=================================

    logic [H_SIZE_BW+MUL:0]                 idx_x_sub_cx, idx_x_sub_cx_r; //signed
    logic [V_SIZE_BW+MUL:0]                 idx_y_sub_cy, idx_y_sub_cy_r; //signed
    logic [DATA_DEPTH_BW-1:0]               depth_d1_r;
    logic [DATA_DEPTH_BW+H_SIZE_BW+MUL:0]   px_mul;
    logic [DATA_DEPTH_BW+V_SIZE_BW+MUL:0]   py_mul;
    logic [DATA_DEPTH_BW+H_SIZE_BW+MUL:0]   px_mul_div;
    logic [DATA_DEPTH_BW+V_SIZE_BW+MUL:0]   py_mul_div;

    logic [DATA_DEPTH_BW+H_SIZE_BW+MUL+MUL:0]   px_final;
    logic [DATA_DEPTH_BW+V_SIZE_BW+MUL+MUL:0]   py_final;
    logic [DATA_DEPTH_BW-1+MUL+MUL:0]           pz_final;
    logic [DATA_DEPTH_BW-1:0]                   depth_d4_r;
    logic                                       valid_d4_r;

    //=================================
    // Combinational Logic
    //=================================
    assign idx_x_sub_cx = {i_idx_x,{MUL{1'b0}}} - r_cx;
    assign idx_y_sub_cy = {i_idx_y,{MUL{1'b0}}} - r_cy;

    assign px_final = {px_mul_div,{MUL{1'b0}}};
    assign py_final = {py_mul_div,{MUL{1'b0}}};
    assign pz_final = {{MUL{1'b0}},depth_d4_r,{MUL{1'b0}}};

    assign o_cloud_x = px_final[CLOUD_BW-1:0];
    assign o_cloud_y = py_final[CLOUD_BW-1:0];
    assign o_cloud_z = pz_final[CLOUD_BW-1:0];
    assign o_valid = valid_d4_r;

    DW02_mult_2_stage #(
         .A_width(H_SIZE_BW+MUL+1)
        ,.B_width(DATA_DEPTH_BW)
    ) u_px_mul (
         .A(idx_x_sub_cx_r)
        ,.B(depth_d1_r)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(px_mul)
    );

    DW02_mult_2_stage #(
         .A_width(V_SIZE_BW+MUL+1)
        ,.B_width(DATA_DEPTH_BW)
    ) u_py_mul (
         .A(idx_y_sub_cy_r)
        ,.B(depth_d1_r)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(py_mul)
    );

    DW_div_pipe #(
         .a_width(DATA_DEPTH_BW+H_SIZE_BW+MUL+1)
        ,.b_width(FX_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_px_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(px_mul)
        ,.b(r_fx)
        ,.quotient(px_mul_div)
        ,.remainder()
        ,.divide_by_0()
    );

    DW_div_pipe #(
         .a_width(DATA_DEPTH_BW+V_SIZE_BW+MUL+1)
        ,.b_width(FY_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_py_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(py_mul)
        ,.b(r_fy)
        ,.quotient(py_mul_div)
        ,.remainder()
        ,.divide_by_0()
    );

    DataDelay
    #(
        .DATA_BW(DATA_DEPTH_BW)
       ,.STAGE(1)
    ) u_depth_d1 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_depth)
        // Output
        ,.o_data(depth_d1_r)
    );

    DataDelay
    #(
        .DATA_BW(DATA_DEPTH_BW)
       ,.STAGE(3)
    ) u_depth_d4 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(depth_d1_r)
        // Output
        ,.o_data(depth_d4_r)
    );

    DataDelay
    #(
        .DATA_BW(1)
       ,.STAGE(4)
    ) u_valid_d4 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_valid)
        // Output
        ,.o_data(valid_d4_r)
    );

    //===================
    //    Sequential
    //===================
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)      idx_x_sub_cx_r <= '0;
        else if (i_valid)  idx_x_sub_cx_r <= idx_x_sub_cx;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)      idx_y_sub_cy_r <= '0;
        else if (i_valid)  idx_y_sub_cy_r <= idx_y_sub_cy;
    end

endmodule

`endif // __IDX2CLOUD_SV__
