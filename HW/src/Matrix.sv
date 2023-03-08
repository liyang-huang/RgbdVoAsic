// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2023


module Matrix
    import RgbdVoConfigPk::*;
#(
)(
    // input
     input                     i_clk
    ,input                     i_rst_n
    ,input                     i_frame_start
    ,input                     i_frame_end
    ,input                     i_valid
    ,input [ID_COE_BW-1:0]     i_Ax_0
    ,input [ID_COE_BW-1:0]     i_Ax_1
    ,input [ID_COE_BW-1:0]     i_Ax_2
    ,input [ID_COE_BW-1:0]     i_Ax_3
    ,input [ID_COE_BW-1:0]     i_Ax_4
    ,input [ID_COE_BW-1:0]     i_Ax_5
    ,input [ID_COE_BW-1:0]     i_Ay_0
    ,input [ID_COE_BW-1:0]     i_Ay_1
    ,input [ID_COE_BW-1:0]     i_Ay_2
    ,input [ID_COE_BW-1:0]     i_Ay_3
    ,input [ID_COE_BW-1:0]     i_Ay_4
    ,input [ID_COE_BW-1:0]     i_Ay_5
    ,input [H_SIZE_BW:0]       i_diffs_x
    ,input [V_SIZE_BW:0]       i_diffs_y
    // Output
    ,output logic                 o_frame_end
    ,output logic [MATRIX_BW-1:0] o_Mat_00
    ,output logic [MATRIX_BW-1:0] o_Mat_10
    ,output logic [MATRIX_BW-1:0] o_Mat_20
    ,output logic [MATRIX_BW-1:0] o_Mat_30
    ,output logic [MATRIX_BW-1:0] o_Mat_40
    ,output logic [MATRIX_BW-1:0] o_Mat_50
    ,output logic [MATRIX_BW-1:0] o_Mat_11
    ,output logic [MATRIX_BW-1:0] o_Mat_21
    ,output logic [MATRIX_BW-1:0] o_Mat_31
    ,output logic [MATRIX_BW-1:0] o_Mat_41
    ,output logic [MATRIX_BW-1:0] o_Mat_51
    ,output logic [MATRIX_BW-1:0] o_Mat_22
    ,output logic [MATRIX_BW-1:0] o_Mat_32
    ,output logic [MATRIX_BW-1:0] o_Mat_42
    ,output logic [MATRIX_BW-1:0] o_Mat_52
    ,output logic [MATRIX_BW-1:0] o_Mat_33
    ,output logic [MATRIX_BW-1:0] o_Mat_43
    ,output logic [MATRIX_BW-1:0] o_Mat_53
    ,output logic [MATRIX_BW-1:0] o_Mat_44
    ,output logic [MATRIX_BW-1:0] o_Mat_54
    ,output logic [MATRIX_BW-1:0] o_Mat_55
    ,output logic [MATRIX_BW-1:0] o_Vec_0
    ,output logic [MATRIX_BW-1:0] o_Vec_1
    ,output logic [MATRIX_BW-1:0] o_Vec_2
    ,output logic [MATRIX_BW-1:0] o_Vec_3
    ,output logic [MATRIX_BW-1:0] o_Vec_4
    ,output logic [MATRIX_BW-1:0] o_Vec_5
);

    //=================================
    // Signal Declaration
    //=================================


    //=================================
    // Combinational Logic
    //=================================

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat00 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_0 )
        ,.i_data_x1 ( i_Ax_0 )
        ,.i_data_y0 ( i_Ay_0 )
        ,.i_data_y1 ( i_Ay_0 )
        // Output
        ,.o_data    ( o_Mat_00 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat10 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_1 )
        ,.i_data_x1 ( i_Ax_0 )
        ,.i_data_y0 ( i_Ay_1 )
        ,.i_data_y1 ( i_Ay_0 )
        // Output
        ,.o_data    ( o_Mat_10 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat20 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_2 )
        ,.i_data_x1 ( i_Ax_0 )
        ,.i_data_y0 ( i_Ay_2 )
        ,.i_data_y1 ( i_Ay_0 )
        // Output
        ,.o_data    ( o_Mat_20 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat30 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_3 )
        ,.i_data_x1 ( i_Ax_0 )
        ,.i_data_y0 ( i_Ay_3 )
        ,.i_data_y1 ( i_Ay_0 )
        // Output
        ,.o_data    ( o_Mat_30 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat40 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_4 )
        ,.i_data_x1 ( i_Ax_0 )
        ,.i_data_y0 ( i_Ay_4 )
        ,.i_data_y1 ( i_Ay_0 )
        // Output
        ,.o_data    ( o_Mat_40 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat50 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_5 )
        ,.i_data_x1 ( i_Ax_0 )
        ,.i_data_y0 ( i_Ay_5 )
        ,.i_data_y1 ( i_Ay_0 )
        // Output
        ,.o_data    ( o_Mat_50 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat11 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_1 )
        ,.i_data_x1 ( i_Ax_1 )
        ,.i_data_y0 ( i_Ay_1 )
        ,.i_data_y1 ( i_Ay_1 )
        // Output
        ,.o_data    ( o_Mat_11 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat21 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_2 )
        ,.i_data_x1 ( i_Ax_1 )
        ,.i_data_y0 ( i_Ay_2 )
        ,.i_data_y1 ( i_Ay_1 )
        // Output
        ,.o_data    ( o_Mat_21 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat31 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_3 )
        ,.i_data_x1 ( i_Ax_1 )
        ,.i_data_y0 ( i_Ay_3 )
        ,.i_data_y1 ( i_Ay_1 )
        // Output
        ,.o_data    ( o_Mat_31 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat41 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_4 )
        ,.i_data_x1 ( i_Ax_1 )
        ,.i_data_y0 ( i_Ay_4 )
        ,.i_data_y1 ( i_Ay_1 )
        // Output
        ,.o_data    ( o_Mat_41 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat51 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_5 )
        ,.i_data_x1 ( i_Ax_1 )
        ,.i_data_y0 ( i_Ay_5 )
        ,.i_data_y1 ( i_Ay_1 )
        // Output
        ,.o_data    ( o_Mat_51 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat22 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_2 )
        ,.i_data_x1 ( i_Ax_2 )
        ,.i_data_y0 ( i_Ay_2 )
        ,.i_data_y1 ( i_Ay_2 )
        // Output
        ,.o_data    ( o_Mat_22 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat32 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_3 )
        ,.i_data_x1 ( i_Ax_2 )
        ,.i_data_y0 ( i_Ay_3 )
        ,.i_data_y1 ( i_Ay_2 )
        // Output
        ,.o_data    ( o_Mat_32 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat42 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_4 )
        ,.i_data_x1 ( i_Ax_2 )
        ,.i_data_y0 ( i_Ay_4 )
        ,.i_data_y1 ( i_Ay_2 )
        // Output
        ,.o_data    ( o_Mat_42 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat52 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_5 )
        ,.i_data_x1 ( i_Ax_2 )
        ,.i_data_y0 ( i_Ay_5 )
        ,.i_data_y1 ( i_Ay_2 )
        // Output
        ,.o_data    ( o_Mat_52 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat33 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_3 )
        ,.i_data_x1 ( i_Ax_3 )
        ,.i_data_y0 ( i_Ay_3 )
        ,.i_data_y1 ( i_Ay_3 )
        // Output
        ,.o_data    ( o_Mat_33 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat43 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_4 )
        ,.i_data_x1 ( i_Ax_3 )
        ,.i_data_y0 ( i_Ay_4 )
        ,.i_data_y1 ( i_Ay_3 )
        // Output
        ,.o_data    ( o_Mat_43 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat53 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_5 )
        ,.i_data_x1 ( i_Ax_3 )
        ,.i_data_y0 ( i_Ay_5 )
        ,.i_data_y1 ( i_Ay_3 )
        // Output
        ,.o_data    ( o_Mat_53 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat44 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_4 )
        ,.i_data_x1 ( i_Ax_4 )
        ,.i_data_y0 ( i_Ay_4 )
        ,.i_data_y1 ( i_Ay_4 )
        // Output
        ,.o_data    ( o_Mat_44 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat54 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_5 )
        ,.i_data_x1 ( i_Ax_4 )
        ,.i_data_y0 ( i_Ay_5 )
        ,.i_data_y1 ( i_Ay_4 )
        // Output
        ,.o_data    ( o_Mat_54 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_mat55 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( i_Ax_5 )
        ,.i_data_x1 ( i_Ax_5 )
        ,.i_data_y0 ( i_Ay_5 )
        ,.i_data_y1 ( i_Ay_5 )
        // Output
        ,.o_data    ( o_Mat_55 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_vec0 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( {{(ID_COE_BW-H_SIZE_BW-1-MUL){i_diffs_x[H_SIZE_BW-1]}},i_diffs_x,{MUL{1'b0}}} )
        ,.i_data_x1 ( i_Ax_0 )
        ,.i_data_y0 ( {{(ID_COE_BW-V_SIZE_BW-1-MUL){i_diffs_y[V_SIZE_BW-1]}},i_diffs_y,{MUL{1'b0}}} )
        ,.i_data_y1 ( i_Ay_0 )
        // Output
        ,.o_data    ( o_Vec_0 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_vec1 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( {{(ID_COE_BW-H_SIZE_BW-1-MUL){i_diffs_x[H_SIZE_BW-1]}},i_diffs_x,{MUL{1'b0}}} )
        ,.i_data_x1 ( i_Ax_1 )
        ,.i_data_y0 ( {{(ID_COE_BW-V_SIZE_BW-1-MUL){i_diffs_y[V_SIZE_BW-1]}},i_diffs_y,{MUL{1'b0}}} )
        ,.i_data_y1 ( i_Ay_1 )
        // Output
        ,.o_data    ( o_Vec_1 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_vec2 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( {{(ID_COE_BW-H_SIZE_BW-1-MUL){i_diffs_x[H_SIZE_BW-1]}},i_diffs_x,{MUL{1'b0}}} )
        ,.i_data_x1 ( i_Ax_2 )
        ,.i_data_y0 ( {{(ID_COE_BW-V_SIZE_BW-1-MUL){i_diffs_y[V_SIZE_BW-1]}},i_diffs_y,{MUL{1'b0}}} )
        ,.i_data_y1 ( i_Ay_2 )
        // Output
        ,.o_data    ( o_Vec_2 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_vec3 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( {{(ID_COE_BW-H_SIZE_BW-1-MUL){i_diffs_x[H_SIZE_BW-1]}},i_diffs_x,{MUL{1'b0}}} )
        ,.i_data_x1 ( i_Ax_3 )
        ,.i_data_y0 ( {{(ID_COE_BW-V_SIZE_BW-1-MUL){i_diffs_y[V_SIZE_BW-1]}},i_diffs_y,{MUL{1'b0}}} )
        ,.i_data_y1 ( i_Ay_3 )
        // Output
        ,.o_data    ( o_Vec_3 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_vec4 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( {{(ID_COE_BW-H_SIZE_BW-1-MUL){i_diffs_x[H_SIZE_BW-1]}},i_diffs_x,{MUL{1'b0}}} )
        ,.i_data_x1 ( i_Ax_4 )
        ,.i_data_y0 ( {{(ID_COE_BW-V_SIZE_BW-1-MUL){i_diffs_y[V_SIZE_BW-1]}},i_diffs_y,{MUL{1'b0}}} )
        ,.i_data_y1 ( i_Ay_4 )
        // Output
        ,.o_data    ( o_Vec_4 )
    );

    MulAcc
    #(
         .INPUT_DATA_BW(ID_COE_BW)
        ,.OUTPUT_DATA_BW(MATRIX_BW)
    ) u_vec5 (
        // input
         .i_clk    ( i_clk )
        ,.i_rst_n  ( i_rst_n )
        ,.i_start  ( i_frame_start )
        ,.i_valid  ( i_valid )
        ,.i_data_x0 ( {{(ID_COE_BW-H_SIZE_BW-1-MUL){i_diffs_x[H_SIZE_BW-1]}},i_diffs_x,{MUL{1'b0}}} )
        ,.i_data_x1 ( i_Ax_5 )
        ,.i_data_y0 ( {{(ID_COE_BW-V_SIZE_BW-1-MUL){i_diffs_y[V_SIZE_BW-1]}},i_diffs_y,{MUL{1'b0}}} )
        ,.i_data_y1 ( i_Ay_5 )
        // Output
        ,.o_data    ( o_Vec_5 )
    );

    DataDelay
    #(
        .DATA_BW(1)
       ,.STAGE(3)
    ) u_frame_end_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_frame_end)
        // Output
        ,.o_data(o_frame_end)
    );

    //===================
    //    Sequential
    //===================


endmodule

