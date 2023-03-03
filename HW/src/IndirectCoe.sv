// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2023

`include "common/RgbdVoConfigPk.sv"

module IndirectCoe
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
    ,output logic [ID_COE_BW-1:0]     o_Ax_0
    ,output logic [ID_COE_BW-1:0]     o_Ax_1
    ,output logic [ID_COE_BW-1:0]     o_Ax_2
    ,output logic [ID_COE_BW-1:0]     o_Ax_3
    ,output logic [ID_COE_BW-1:0]     o_Ax_4
    ,output logic [ID_COE_BW-1:0]     o_Ax_5
    ,output logic [ID_COE_BW-1:0]     o_Ay_0
    ,output logic [ID_COE_BW-1:0]     o_Ay_1
    ,output logic [ID_COE_BW-1:0]     o_Ay_2
    ,output logic [ID_COE_BW-1:0]     o_Ay_3
    ,output logic [ID_COE_BW-1:0]     o_Ay_4
    ,output logic [ID_COE_BW-1:0]     o_Ay_5
);

    //=================================
    // Signal Declaration
    //=================================
    logic [CLOUD_BW+CLOUD_BW-1:0]            cloud_xy;
    logic [CLOUD_BW+CLOUD_BW-1:0]            cloud_xx;
    logic [CLOUD_BW+CLOUD_BW-1:0]            cloud_yy;
    logic [CLOUD_BW+CLOUD_BW-1:0]            cloud_zz;
    logic [CLOUD_BW+CLOUD_BW-1:0]            cloud_zz_d1;
    logic [CLOUD_BW+CLOUD_BW+FX_BW-1:0]      cloud_xy_fx;
    logic [CLOUD_BW+CLOUD_BW+FX_BW-1:0]      cloud_xx_fx;
    logic [CLOUD_BW+CLOUD_BW+FY_BW-1:0]      cloud_yy_fy;
    logic [CLOUD_BW+CLOUD_BW+FX_BW-1:0]      ax0_tmp;
    logic [CLOUD_BW+CLOUD_BW+FX_BW-1:0]      ax1_tmp;
    logic [CLOUD_BW+CLOUD_BW+FX_BW-1:0]      ax0_final_r;
    logic [CLOUD_BW+CLOUD_BW+FX_BW-1:0]      ax1_final_r;
    logic [CLOUD_BW+FX_BW-1:0]      cloud_y_fx;
    logic [CLOUD_BW-1:0]            cloud_z_d1;
    logic [CLOUD_BW+FX_BW-1:0]      ax2_tmp;
    logic [CLOUD_BW+FX_BW-1:0]      ax2_final_r;
    logic [FX_BW+MUL-1:0]           fx_mul;
    logic [FX_BW+MUL-1:0]           ax3_tmp;
    logic [FX_BW+MUL-1:0]           ax3_final;
    logic [CLOUD_BW+FX_BW-1:0]      cloud_x_fx;
    logic [CLOUD_BW+FX_BW+MUL-1:0]  cloud_x_fx_mul;
    logic [CLOUD_BW+FX_BW+MUL-1:0]  ax5_tmp;
    logic [CLOUD_BW+FX_BW+MUL-1:0]  ax5_final_r;
    logic [ID_COE_BW-1:0]           ax2_final_dly;
    logic [ID_COE_BW-1:0]           ax3_final_dly;
    logic [ID_COE_BW-1:0]           ax5_final_dly;
    logic [CLOUD_BW+CLOUD_BW+FY_BW-1:0]      ay0_tmp;
    logic [CLOUD_BW+CLOUD_BW+FY_BW-1:0]      ay0_final_r;
    logic [CLOUD_BW+CLOUD_BW+FY_BW-1:0]      cloud_xy_fy;
    logic [CLOUD_BW+CLOUD_BW+FY_BW-1:0]      ay1_tmp;
    logic [ID_COE_BW-1:0]                    ay1_final_dly;
    logic [CLOUD_BW+FY_BW-1:0]      cloud_x_fy;
    logic [CLOUD_BW+FY_BW-1:0]      ay2_tmp;
    logic [ID_COE_BW-1:0]           ay2_final_dly;
    logic [FY_BW+MUL-1:0]           fy_mul;
    logic [FY_BW+MUL-1:0]           ay4_tmp;
    logic [ID_COE_BW-1:0]           ay4_final_dly;
    logic [CLOUD_BW+FY_BW-1:0]      cloud_y_fy;
    logic [CLOUD_BW+FY_BW+MUL-1:0]  cloud_y_fy_mul;
    logic [CLOUD_BW+FY_BW+MUL-1:0]  ay5_tmp;
    logic [CLOUD_BW+FY_BW+MUL-1:0]  ay5_final_r;
    logic [ID_COE_BW-1:0]           ay5_final_dly;

    //=================================
    // Combinational Logic
    //=================================
    assign o_Ax_0 = ax0_final_r[ID_COE_BW-1:0];
    assign o_Ax_1 = ax1_final_r[ID_COE_BW-1:0];
    assign o_Ax_2 = ax2_final_dly;
    assign o_Ax_3 = ax3_final_dly;
    assign o_Ax_4 = '0;
    assign o_Ax_5 = ax5_final_dly;
    assign o_Ay_0 = ay0_final_r[ID_COE_BW-1:0];
    assign o_Ay_1 = ay1_final_dly;
    assign o_Ay_2 = ay2_final_dly;
    assign o_Ay_3 = '0;
    assign o_Ay_4 = ay4_final_dly;
    assign o_Ay_5 = ay5_final_dly;

    assign fx_mul = {r_fx,{MUL{1'b0}}};
    assign fy_mul = {r_fy,{MUL{1'b0}}};
    assign cloud_x_fx_mul = {cloud_x_fx,{MUL{1'b0}}};
    assign cloud_y_fy_mul = {cloud_y_fy,{MUL{1'b0}}};
 
    DataDelay
    #(
        .DATA_BW(1)
       ,.STAGE(5)
    ) u_valid_dly (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_valid)
        // Output
        ,.o_data(o_valid)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(CLOUD_BW)
    ) u_cloud_xy (
         .A(i_cloud_x)
        ,.B(i_cloud_y)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_xy)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(CLOUD_BW)
    ) u_cloud_xx (
         .A(i_cloud_x)
        ,.B(i_cloud_x)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_xx)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(CLOUD_BW)
    ) u_cloud_yy (
         .A(i_cloud_y)
        ,.B(i_cloud_y)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_yy)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(CLOUD_BW)
    ) u_cloud_zz (
         .A(i_cloud_z)
        ,.B(i_cloud_z)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_zz)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW+CLOUD_BW)
        ,.B_width(FX_BW)
    ) u_cloud_xy_fx (
         .A(cloud_xy)
        ,.B(r_fx)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_xy_fx)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW+CLOUD_BW)
        ,.B_width(FY_BW)
    ) u_cloud_xy_fy (
         .A(cloud_xy)
        ,.B(r_fy)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_xy_fy)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW+CLOUD_BW)
        ,.B_width(FX_BW)
    ) u_cloud_xx_fx (
         .A(cloud_xx)
        ,.B(r_fx)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_xx_fx)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW+CLOUD_BW)
        ,.B_width(FY_BW)
    ) u_cloud_yy_fy (
         .A(cloud_yy)
        ,.B(r_fy)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_yy_fy)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(FX_BW)
    ) u_cloud_y_fx (
         .A(i_cloud_y)
        ,.B(r_fx)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_y_fx)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(FY_BW)
    ) u_cloud_x_fy (
         .A(i_cloud_x)
        ,.B(r_fy)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_x_fy)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(FX_BW)
    ) u_cloud_x_fx (
         .A(i_cloud_x)
        ,.B(r_fx)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_x_fx)
    );

    DW02_mult_2_stage #(
         .A_width(CLOUD_BW)
        ,.B_width(FY_BW)
    ) u_cloud_y_fy (
         .A(i_cloud_y)
        ,.B(r_fy)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(cloud_y_fy)
    );

    DataDelay
    #(
        .DATA_BW(CLOUD_BW+CLOUD_BW)
       ,.STAGE(1)
    ) u_cloud_zz_d1 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(cloud_zz)
        // Output
        ,.o_data(cloud_zz_d1)
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

    DW_div_pipe #(
         .a_width(CLOUD_BW+CLOUD_BW+FX_BW)
        ,.b_width(CLOUD_BW+CLOUD_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_ax0_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(cloud_xy_fx)
        ,.b(cloud_zz_d1)
        ,.quotient(ax0_tmp)
        ,.remainder()
        ,.divide_by_0()
    );

    DW_div_pipe #(
         .a_width(CLOUD_BW+CLOUD_BW+FX_BW)
        ,.b_width(CLOUD_BW+CLOUD_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_ax1_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(cloud_xx_fx)
        ,.b(cloud_zz_d1)
        ,.quotient(ax1_tmp)
        ,.remainder()
        ,.divide_by_0()
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
    ) u_ax2_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(cloud_y_fx)
        ,.b(cloud_z_d1)
        ,.quotient(ax2_tmp)
        ,.remainder()
        ,.divide_by_0()
    );

    DW_div_pipe #(
         .a_width(FX_BW+MUL)
        ,.b_width(CLOUD_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_ax3_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(fx_mul)
        ,.b(cloud_z_d1)
        ,.quotient(ax3_tmp)
        ,.remainder()
        ,.divide_by_0()
    );

    DataDelay
    #(
        .DATA_BW(FX_BW+MUL)
       ,.STAGE(1)
    ) u_ax3 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(ax3_tmp)
        // Output
        ,.o_data(ax3_final)
    );

    DW_div_pipe #(
         .a_width(CLOUD_BW+FX_BW+MUL)
        ,.b_width(CLOUD_BW+CLOUD_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_ax5_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(cloud_x_fx_mul)
        ,.b(cloud_zz)
        ,.quotient(ax5_tmp)
        ,.remainder()
        ,.divide_by_0()
    );

    DataDelay
    #(
        .DATA_BW(ID_COE_BW)
       ,.STAGE(1)
    ) u_ax2_dly (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(ax2_final_r[ID_COE_BW-1:0])
        // Output
        ,.o_data(ax2_final_dly)
    );

    DataDelay
    #(
        .DATA_BW(ID_COE_BW)
       ,.STAGE(1)
    ) u_ax3_dly (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(ax3_final[ID_COE_BW-1:0])
        // Output
        ,.o_data(ax3_final_dly)
    );

    DataDelay
    #(
        .DATA_BW(ID_COE_BW)
       ,.STAGE(1)
    ) u_ax5_dly (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(ax5_final_r[ID_COE_BW-1:0])
        // Output
        ,.o_data(ax5_final_dly)
    );

    DW_div_pipe #(
         .a_width(CLOUD_BW+CLOUD_BW+FY_BW)
        ,.b_width(CLOUD_BW+CLOUD_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_ay0_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(cloud_yy_fy)
        ,.b(cloud_zz_d1)
        ,.quotient(ay0_tmp)
        ,.remainder()
        ,.divide_by_0()
    );

    DW_div_pipe #(
         .a_width(CLOUD_BW+CLOUD_BW+FY_BW)
        ,.b_width(CLOUD_BW+CLOUD_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_ay1_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(cloud_xy_fy)
        ,.b(cloud_zz_d1)
        ,.quotient(ay1_tmp)
        ,.remainder()
        ,.divide_by_0()
    );

    DataDelay
    #(
        .DATA_BW(ID_COE_BW)
       ,.STAGE(1)
    ) u_ay1_dly (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(ay1_tmp[ID_COE_BW-1:0])
        // Output
        ,.o_data(ay1_final_dly)
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
    ) u_ay2_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(cloud_x_fy)
        ,.b(cloud_z_d1)
        ,.quotient(ay2_tmp)
        ,.remainder()
        ,.divide_by_0()
    );

    DataDelay
    #(
        .DATA_BW(ID_COE_BW)
       ,.STAGE(2)
    ) u_ay2_dly (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(ay2_tmp[ID_COE_BW-1:0])
        // Output
        ,.o_data(ay2_final_dly)
    );

    DW_div_pipe #(
         .a_width(FY_BW+MUL)
        ,.b_width(CLOUD_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_ay4_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(fy_mul)
        ,.b(cloud_z_d1)
        ,.quotient(ay4_tmp)
        ,.remainder()
        ,.divide_by_0()
    );

    DataDelay
    #(
        .DATA_BW(ID_COE_BW)
       ,.STAGE(2)
    ) u_ay4_dly (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(ay4_tmp[ID_COE_BW-1:0])
        // Output
        ,.o_data(ay4_final_dly)
    );

    DW_div_pipe #(
         .a_width(CLOUD_BW+FY_BW+MUL)
        ,.b_width(CLOUD_BW+CLOUD_BW)
        ,.tc_mode(1)
        ,.rem_mode(1)
        ,.num_stages(3)
        ,.stall_mode(0)
        ,.rst_mode(1)
        ,.op_iso_mode(1)
    ) u_ay5_div (
         .clk(i_clk)
        ,.rst_n(i_rst_n)
        ,.en(1'b1)
        ,.a(cloud_y_fy_mul)
        ,.b(cloud_zz)
        ,.quotient(ay5_tmp)
        ,.remainder()
        ,.divide_by_0()
    );

    DataDelay
    #(
        .DATA_BW(ID_COE_BW)
       ,.STAGE(1)
    ) u_ay5_dly (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(ay5_final_r[ID_COE_BW-1:0])
        // Output
        ,.o_data(ay5_final_dly)
    );


    //===================
    //    Sequential
    //===================
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) ax0_final_r <= '0;
        else ax0_final_r <= ~ax0_tmp + 1;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) ax1_final_r <= '0;
        else ax1_final_r <= ax1_tmp + r_fx;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) ax2_final_r <= '0;
        else ax2_final_r <= ~ax2_tmp + 1;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) ax5_final_r <= '0;
        else ax5_final_r <= ~ax5_tmp + 1;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) ay0_final_r <= '0;
        else ay0_final_r <= ~r_fy + 1 - ay0_tmp;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) ay5_final_r <= '0;
        else ay5_final_r <= ~ay5_tmp + 1;
    end


endmodule

