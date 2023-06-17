// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2023


module LineBufCtrl
    import RgbdVoConfigPk::*;
#(
)(
    // input
     input                            i_clk
    ,input                            i_rst_n
    ,input                            i_frame_start
    ,input                            i_frame_end
    ,input                            i_valid
    ,input        [DATA_RGB_BW-1:0]   i_data1
    ,input        [DATA_DEPTH_BW-1:0] i_depth1
    // Register
    ,input        [H_SIZE_BW-1:0]     r_hsize
    ,input        [V_SIZE_BW-1:0]     r_vsize
    // SRAM
    ,input [DATA_DEPTH_BW-1:0]        i_lb_sram_QA   [0:29]
    ,input [DATA_DEPTH_BW-1:0]        i_lb_sram_QB   [0:29]
    ,output logic                     o_lb_sram_WENA [0:29]
    ,output logic                     o_lb_sram_WENB [0:29]
    ,output logic [DATA_DEPTH_BW-1:0] o_lb_sram_DA   [0:29]
    ,output logic [DATA_DEPTH_BW-1:0] o_lb_sram_DB   [0:29]
    ,output logic [H_SIZE_BW-1:0]     o_lb_sram_AA   [0:29]
    ,output logic [H_SIZE_BW-1:0]     o_lb_sram_AB   [0:29]
    // Output
    ,output logic                     o_frame_start
    ,output logic                     o_frame_end
    ,output logic                     o_valid
    ,output logic [DATA_DEPTH_BW-1:0] o_depth0
);

    //=================================
    // Signal Declaration
    //=================================
    logic [H_SIZE_BW-1:0]     idx1_x_w;
    logic [H_SIZE_BW-1:0]     idx1_x_r;
    logic [V_SIZE_BW-1:0]     idx1_y_w;
    logic [V_SIZE_BW-1:0]     idx1_y_r;
    logic                     idx1_x_clr;
    logic                     idx1_y_clr;
    logic                     lb_sram_WENA_r [0:29];
    logic                     lb_sram_WENB_r [0:29];
    logic [DATA_DEPTH_BW-1:0] lb_sram_DA_r   [0:29]; 
    logic [H_SIZE_BW-1:0]     lb_sram_AA_r   [0:29];
    logic [H_SIZE_BW-1:0]     lb_sram_AB_r   [0:29];

    //=================================
    // Combinational Logic
    //=================================
    assign idx1_x_clr = (idx1_x_r==r_hsize-1);
    assign idx1_y_clr = (idx1_y_r==r_vsize-1);
    for(genvar i = 0; i < 30; i = i+1) begin
        assign o_lb_sram_WENA[i] = lb_sram_WENA_r[i];
        assign o_lb_sram_WENB[i] = lb_sram_WENB_r[i];
        assign o_lb_sram_DA[i]   = lb_sram_DA_r[i];
        assign o_lb_sram_DB[i]   = 0;
        assign o_lb_sram_AA[i]   = lb_sram_AA_r[i];
        assign o_lb_sram_AB[i]   = lb_sram_AB_r[i];
    end


    always_comb begin
        if(i_valid) begin
            if(idx1_x_clr)
                idx1_x_w = 0;
            else
                idx1_x_w = idx1_x_r + 1;
        end
        else begin
            idx1_x_w = idx1_x_r;
        end
    end

    always_comb begin
        if(i_valid && idx1_x_clr) begin
            if(idx1_y_clr)
                idx1_y_w = 0;
            else
                idx1_y_w = idx1_y_r + 1;
        end
        else begin
            idx1_y_w = idx1_y_r;
        end
    end

    //===================
    //    Sequential
    //===================
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) idx1_x_r <= '0;
        else idx1_x_r <= idx1_x_w;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) idx1_y_r <= '0;
        else idx1_y_r <= idx1_y_w;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            for(int i = 0; i < 31; i = i+1) begin
                lb_sram_DA_r[i] <= '0;
                lb_sram_AA_r[i] <= '0;
                lb_sram_AB_r[i] <= '0;
                lb_sram_WENA_r[i] <= 1;
                lb_sram_WENB_r[i] <= 1;
            end
        end
        else if(i_valid) begin
            for(int i = 0; i < 31; i = i+1) begin
                if(idx1_y_r[4:0] == i) begin
                    lb_sram_DA_r[i] <= i_depth1;
                end
            end
        end
    end

endmodule

