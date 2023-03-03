// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2022


`include "common/RgbdVoConfigPk.sv"
//`include "./DW02_mult_2_stage.v"
//`include "./DW_div.v"
//`include "./DW_div_pipe.v"
//`include "./DataDelay.sv"

module Proj
    import RgbdVoConfigPk::*;
#(
)(
    // input
     input                            i_clk
    ,input                            i_rst_n
    ,input                            i_valid
    ,input        [CLOUD_BW-1:0]      i_cloud_x
    ,input        [CLOUD_BW-1:0]      i_cloud_y
    ,input        [CLOUD_BW-1:0]      i_cloud_z
    // Register
    ,input        [FX_BW-1:0]         r_fx
    ,input        [FY_BW-1:0]         r_fy
    ,input        [CX_BW-1:0]         r_cx
    ,input        [CY_BW-1:0]         r_cy
    // Output
    ,output logic                     o_valid
    ,output logic [H_SIZE_BW-1:0]     o_idx_x
    ,output logic [V_SIZE_BW-1:0]     o_idx_y
);

    //=================================
    // Signal Declaration
    //=================================

    logic [CLOUD_BW+FX_BW-1:0]    px_mul;
    logic [CLOUD_BW+FX_BW-1:0]    px_mul_div;
    logic [CLOUD_BW+FX_BW:0]      px_final_r;
    logic [CLOUD_BW+FX_BW-1:0]    py_mul;
    logic [CLOUD_BW+FX_BW-1:0]    py_mul_div;
    logic [CLOUD_BW+FX_BW:0]      py_final_r;
    logic [CLOUD_BW-1:0]          cloud_z_d1;

    //=================================
    // Combinational Logic
    //=================================
    assign o_idx_x = px_final_r[H_SIZE_BW-1+MUL:MUL];
    assign o_idx_y = py_final_r[V_SIZE_BW-1+MUL:MUL];

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(FX_BW)
    ) u_px_mul (
         .A(i_cloud_x)
        ,.B(r_fx)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(px_mul)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(FY_BW)
    ) u_py_mul (
         .A(i_cloud_y)
        ,.B(r_fy)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(py_mul)
    );

    DW_div_pipe #(
         .a_width(CLOUD_BW+FX_BW)
        ,.b_width(CLOUD_BW)
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
        ,.b(cloud_z_d1)
        ,.quotient(px_mul_div)
        ,.remainder()
        ,.divide_by_0()
    );

    DW_div_pipe #(
         .a_width(CLOUD_BW+FY_BW)
        ,.b_width(CLOUD_BW)
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
        ,.b(cloud_z_d1)
        ,.quotient(py_mul_div)
        ,.remainder()
        ,.divide_by_0()
    );


    DataDelay
    #(
        .DATA_BW(CLOUD_BW)
       ,.STAGE(1)
    ) u_cloud_z_d1 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_cloud_z)
        // Output
        ,.o_data(cloud_z_d1)
    );

    DataDelay
    #(
        .DATA_BW(1)
       ,.STAGE(4)
    ) u_valid (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_valid)
        // Output
        ,.o_data(o_valid)
    );


    //===================
    //    Sequential
    //===================
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) px_final_r <= '0;
        else px_final_r <= $signed(px_mul_div) + $signed(r_cx);
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) py_final_r <= '0;
        else py_final_r <= $signed(py_mul_div) + $signed(r_cy);
    end


endmodule

