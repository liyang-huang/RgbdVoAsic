// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2022

`ifndef __DATADELAY_SV__
`define __DATADELAY_SV__

module DataDelay
#(
     parameter    DATA_BW = 10
    ,parameter    STAGE = 2
)(
    // input
     input                            i_clk
    ,input                            i_rst_n
    ,input        [DATA_BW-1:0]       i_data
    // Output
    ,output logic [DATA_BW-1:0]       o_data
);

    //=================================
    // Signal Declaration
    //=================================

    logic [DATA_BW-1:0]    tmp_r    [STAGE+1];
    genvar i;

    //=================================
    // Combinational Logic
    //=================================

    assign tmp_r[0]   =  i_data;
    assign o_data     =  tmp_r[STAGE];

    //===================
    //    Sequential
    //===================
    generate
    for (i = 1; i < STAGE+1 ; i = i + 1) begin
        always_ff @(posedge i_clk or negedge i_rst_n) begin
            if (!i_rst_n)      tmp_r[i] <= '0;
            else  tmp_r[i] <= tmp_r[i-1];
        end
    end
    endgenerate

endmodule

`endif // __DATADELAY_SV__
