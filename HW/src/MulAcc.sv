// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2023


module MulAcc
    import RgbdVoConfigPk::*;
#(
     parameter    INPUT_DATA_BW = 42
    ,parameter    OUTPUT_DATA_BW = 52
)(
    // input
     input                             i_clk
    ,input                             i_rst_n
    ,input                             i_start
    ,input                             i_valid
    ,input        [INPUT_DATA_BW-1:0]  i_data_x0
    ,input        [INPUT_DATA_BW-1:0]  i_data_x1
    ,input        [INPUT_DATA_BW-1:0]  i_data_y0
    ,input        [INPUT_DATA_BW-1:0]  i_data_y1
    // Output
    ,output logic [OUTPUT_DATA_BW-1:0] o_data
);

    //=================================
    // Signal Declaration
    //=================================

    logic [OUTPUT_DATA_BW-1:0]     data_acc_r;
    logic [INPUT_DATA_BW+INPUT_DATA_BW-1:0] data_x;
    logic [INPUT_DATA_BW+INPUT_DATA_BW-1:0] data_y;
    logic [INPUT_DATA_BW+INPUT_DATA_BW:0]   data_x_y_r;
    logic valid_dly;

    //=================================
    // Combinational Logic
    //=================================
    assign o_data = data_acc_r;

    DW02_mult_2_stage #(
         .A_width(INPUT_DATA_BW)
        ,.B_width(INPUT_DATA_BW)
    ) u_data_x (
         .A(i_data_x0)
        ,.B(i_data_x1)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(data_x)
    );

    DW02_mult_2_stage #(
         .A_width(INPUT_DATA_BW)
        ,.B_width(INPUT_DATA_BW)
    ) u_data_y (
         .A(i_data_y0)
        ,.B(i_data_y1)
        ,.TC(1'b1)
        ,.CLK(i_clk)
        ,.PRODUCT(data_y)
    );

    DataDelay
    #(
        .DATA_BW(1)
       ,.STAGE(2)
    ) u_valid_d4 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_valid)
        // Output
        ,.o_data(valid_dly)
    );


    //===================
    //    Sequential
    //===================
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)      data_x_y_r <= '0;
        else data_x_y_r <= $signed(data_x[INPUT_DATA_BW+INPUT_DATA_BW-1:MUL]) + $signed(data_y[INPUT_DATA_BW+INPUT_DATA_BW-1:MUL]);
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)      data_acc_r <= '0;
        else if (i_start)  data_acc_r <= '0;
        else if (valid_dly)  data_acc_r <= data_acc_r + $signed(data_x_y_r);
    end


endmodule

