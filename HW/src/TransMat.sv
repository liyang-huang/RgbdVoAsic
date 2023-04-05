// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2022


`include "common/RgbdVoConfigPk.sv"
//`include "./DW02_mult_2_stage.v"
//`include "./DataDelay.sv"

module TransMat
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
    //,input                            i_pose_valid
    ,input        [POSE_BW-1:0]       i_pose        [12]  //3x4 matrix
    // Register
    // Output
    ,output logic                     o_valid
    ,output logic [CLOUD_BW-1:0]      o_cloud_x
    ,output logic [CLOUD_BW-1:0]      o_cloud_y
    ,output logic [CLOUD_BW-1:0]      o_cloud_z
);

    //=================================
    // Signal Declaration
    //=================================

    logic [CLOUD_BW+POSE_BW-1:0]    tpx_tmp1;
    logic [CLOUD_BW+POSE_BW-1:0]    tpx_tmp2;
    logic [CLOUD_BW+POSE_BW-1:0]    tpx_tmp3;
    logic [CLOUD_BW+POSE_BW-MUL:0]  tpx_tmp4_r;
    logic [CLOUD_BW+POSE_BW-MUL:0]  tpx_tmp5_r;
    logic [CLOUD_BW+POSE_BW-MUL+1:0]tpx_tmp6_r;

    logic [CLOUD_BW+POSE_BW-1:0]    tpy_tmp1;
    logic [CLOUD_BW+POSE_BW-1:0]    tpy_tmp2;
    logic [CLOUD_BW+POSE_BW-1:0]    tpy_tmp3;
    logic [CLOUD_BW+POSE_BW-MUL:0]  tpy_tmp4_r;
    logic [CLOUD_BW+POSE_BW-MUL:0]  tpy_tmp5_r;
    logic [CLOUD_BW+POSE_BW-MUL+1:0]tpy_tmp6_r;

    logic [CLOUD_BW+POSE_BW-1:0]    tpz_tmp1;
    logic [CLOUD_BW+POSE_BW-1:0]    tpz_tmp2;
    logic [CLOUD_BW+POSE_BW-1:0]    tpz_tmp3;
    logic [CLOUD_BW+POSE_BW-MUL:0]  tpz_tmp4_r;
    logic [CLOUD_BW+POSE_BW-MUL:0]  tpz_tmp5_r;
    logic [CLOUD_BW+POSE_BW-MUL+1:0]tpz_tmp6_r;

    //=================================
    // Combinational Logic
    //=================================
    assign o_cloud_x = tpx_tmp6_r[CLOUD_BW-1:0];
    assign o_cloud_y = tpy_tmp6_r[CLOUD_BW-1:0];
    assign o_cloud_z = tpz_tmp6_r[CLOUD_BW-1:0];

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(POSE_BW)
    ) u_tpx_tmp1 (
         .A(i_cloud_x)
        ,.B(i_pose[0])
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(tpx_tmp1)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(POSE_BW)
    ) u_tpx_tmp2 (
         .A(i_cloud_y)
        ,.B(i_pose[1])
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(tpx_tmp2)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(POSE_BW)
    ) u_tpx_tmp3 (
         .A(i_cloud_z)
        ,.B(i_pose[2])
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(tpx_tmp3)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(POSE_BW)
    ) u_tpy_tmp1 (
         .A(i_cloud_x)
        ,.B(i_pose[4])
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(tpy_tmp1)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(POSE_BW)
    ) u_tpy_tmp2 (
         .A(i_cloud_y)
        ,.B(i_pose[5])
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(tpy_tmp2)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(POSE_BW)
    ) u_tpy_tmp3 (
         .A(i_cloud_z)
        ,.B(i_pose[6])
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(tpy_tmp3)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(POSE_BW)
    ) u_tpz_tmp1 (
         .A(i_cloud_x)
        ,.B(i_pose[8])
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(tpz_tmp1)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(POSE_BW)
    ) u_tpz_tmp2 (
         .A(i_cloud_y)
        ,.B(i_pose[9])
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(tpz_tmp2)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(POSE_BW)
    ) u_tpz_tmp3 (
         .A(i_cloud_z)
        ,.B(i_pose[10])
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(tpz_tmp3)
    );

    DataDelay
    #(
        .DATA_BW(1)
       ,.STAGE(3)
    ) u_valid_d4 (
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
        if (!i_rst_n) tpx_tmp4_r <= '0;
        else tpx_tmp4_r <= tpx_tmp1[CLOUD_BW+POSE_BW-1:MUL] + tpx_tmp2[CLOUD_BW+POSE_BW-1:MUL];
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) tpx_tmp5_r <= '0;
        else tpx_tmp5_r <= tpx_tmp3[CLOUD_BW+POSE_BW-1:MUL] + i_pose[3];
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) tpx_tmp6_r <= '0;
        else tpx_tmp6_r <= tpx_tmp4_r + tpx_tmp5_r;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) tpy_tmp4_r <= '0;
        else tpy_tmp4_r <= tpy_tmp1[CLOUD_BW+POSE_BW-1:MUL] + tpy_tmp2[CLOUD_BW+POSE_BW-1:MUL];
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) tpy_tmp5_r <= '0;
        else tpy_tmp5_r <= tpy_tmp3[CLOUD_BW+POSE_BW-1:MUL] + i_pose[7];
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) tpy_tmp6_r <= '0;
        else tpy_tmp6_r <= tpy_tmp4_r + tpy_tmp5_r;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) tpz_tmp4_r <= '0;
        else tpz_tmp4_r <= tpz_tmp1[CLOUD_BW+POSE_BW-1:MUL] + tpz_tmp2[CLOUD_BW+POSE_BW-1:MUL];
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) tpz_tmp5_r <= '0;
        else tpz_tmp5_r <= tpz_tmp3[CLOUD_BW+POSE_BW-1:MUL] + i_pose[11];
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) tpz_tmp6_r <= '0;
        else tpz_tmp6_r <= tpz_tmp4_r + tpz_tmp5_r;
    end

endmodule

