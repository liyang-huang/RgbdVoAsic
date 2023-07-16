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
    ,input                            i_valid0
    ,input        [DATA_RGB_BW-1:0]   i_data0
    ,input        [DATA_DEPTH_BW-1:0] i_depth0
    ,input        [CLOUD_BW-1:0]      i_trans_z1
    ,input        [H_SIZE_BW-1:0]     i_idx0_x
    ,input        [V_SIZE_BW-1:0]     i_idx0_y
    ,input        [H_SIZE_BW-1:0]     i_idx1_x
    ,input        [V_SIZE_BW-1:0]     i_idx1_y
    ,input                            i_valid1
    ,input        [DATA_RGB_BW-1:0]   i_data1
    ,input        [DATA_DEPTH_BW-1:0] i_depth1
    // Register
    ,input        [H_SIZE_BW-1:0]     r_hsize
    ,input        [V_SIZE_BW-1:0]     r_vsize
    // SRAM
    ,input [DATA_DEPTH_BW-1:0]        i_lb_sram_QA   [0:127]
    ,input [DATA_DEPTH_BW-1:0]        i_lb_sram_QB   [0:127]
    ,output logic                     o_lb_sram_WENA [0:127]
    ,output logic                     o_lb_sram_WENB [0:127]
    ,output logic [DATA_DEPTH_BW-1:0] o_lb_sram_DA   [0:127]
    ,output logic [DATA_DEPTH_BW-1:0] o_lb_sram_DB   [0:127]
    ,output logic [H_SIZE_BW-2:0]     o_lb_sram_AA   [0:127]
    ,output logic [H_SIZE_BW-2:0]     o_lb_sram_AB   [0:127]
    // Output
    ,output logic                     o_frame_start
    ,output logic                     o_frame_end
    ,output logic                     o_valid
    ,output logic [H_SIZE_BW-1:0]     o_idx0_x
    ,output logic [V_SIZE_BW-1:0]     o_idx0_y
    ,output logic [H_SIZE_BW-1:0]     o_idx1_x
    ,output logic [V_SIZE_BW-1:0]     o_idx1_y
    ,output logic [DATA_DEPTH_BW-1:0] o_depth0
    ,output logic [DATA_DEPTH_BW-1:0] o_depth1
    ,output logic [DATA_RGB_BW-1:0]   o_data0
    ,output logic [DATA_RGB_BW-1:0]   o_data1
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
    logic                     lb_sram_WENA_r [0:127];
    logic                     lb_sram_WENB_r [0:127];
    logic [DATA_DEPTH_BW-1:0] lb_sram_DA_r   [0:127]; 
    logic [H_SIZE_BW-2:0]     lb_sram_AA_r   [0:127];
    logic [H_SIZE_BW-2:0]     lb_sram_AB_r   [0:127];
    logic [5:0]               sram_idx_r;
    logic [V_SIZE_BW-1:0]     i_idx1_y_d1;
    logic [5:0]               sram_read_idx_r;
    logic                     valid0_d1_r;
    logic                     valid0_d2_r;
    logic [H_SIZE_BW-1:0]     i_idx1_x_d2;
    logic [DATA_DEPTH_BW-1:0] lb_sram_QB_r   [0:127]; 
    logic [DATA_DEPTH_BW-1:0] lb_sram_QB_mux_r; 
    logic [DATA_DEPTH_BW-1:0] lb_sram_QB_mux2_r; 
    logic                     valid0_d4;
    logic [5:0]               sram_read_idx_d3;
    logic                     valid0_d5_r;
    logic                     valid0_d6_r;
    logic                     valid0_d7_r;
    logic [CLOUD_BW-1:0]      trans_z1_d6;
    logic [CLOUD_BW-1:0]      depth1_mul;
    logic [CLOUD_BW-1:0]      diff_z;
    logic [H_SIZE_BW-1:0]     i_idx0_x_d7;
    logic [V_SIZE_BW-1:0]     i_idx0_y_d7;
    logic [H_SIZE_BW-1:0]     i_idx1_x_d5;
    logic [H_SIZE_BW-1:0]     i_idx1_x_d7;
    logic [V_SIZE_BW-1:0]     i_idx1_y_d7;
    logic [CLOUD_BW-1:0]      max_diff_depth_w;

    //=================================
    // Combinational Logic
    //=================================
    assign o_valid = valid0_d7_r;
    assign o_idx0_x = i_idx0_x_d7;
    assign o_idx0_y = i_idx0_y_d7;
    assign o_idx1_x = i_idx1_x_d7;
    assign o_idx1_y = i_idx1_y_d7;

    assign idx1_x_clr = (idx1_x_r==r_hsize-1);
    assign idx1_y_clr = (idx1_y_r==r_vsize-1);
    generate
        for(genvar i = 0; i < 128; i = i+1) begin
            assign o_lb_sram_WENA[i] = lb_sram_WENA_r[i];
            assign o_lb_sram_WENB[i] = lb_sram_WENB_r[i];
            assign o_lb_sram_DA[i]   = lb_sram_DA_r[i];
            assign o_lb_sram_DB[i]   = 0;
            assign o_lb_sram_AA[i]   = lb_sram_AA_r[i];
            assign o_lb_sram_AB[i]   = lb_sram_AB_r[i];
        end
    endgenerate

    assign depth1_mul = {lb_sram_QB_mux_r,{MUL{1'b0}}};
    assign diff_z = (trans_z1_d6 > depth1_mul) ? trans_z1_d6 - depth1_mul :
                                                 depth1_mul - trans_z1_d6;
    assign max_diff_depth_w = {MAX_DIFF_DEPTH,{MUL{1'b0}}};

    always_comb begin
        if(i_valid1) begin
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
        if(i_valid1 && idx1_x_clr) begin
            if(idx1_y_clr)
                idx1_y_w = 0;
            else
                idx1_y_w = idx1_y_r + 1;
        end
        else begin
            idx1_y_w = idx1_y_r;
        end
    end

    DataDelay
    #(
        .DATA_BW(V_SIZE_BW)
       ,.STAGE(7)
    ) u_idx1_y_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_idx1_y)
        // Output
        ,.o_data(i_idx1_y_d7)
    );

    DataDelay
    #(
        .DATA_BW(V_SIZE_BW)
       ,.STAGE(7)
    ) u_idx0_y_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_idx0_y)
        // Output
        ,.o_data(i_idx0_y_d7)
    );

    DataDelay
    #(
        .DATA_BW(H_SIZE_BW)
       ,.STAGE(7)
    ) u_idx0_x_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_idx0_x)
        // Output
        ,.o_data(i_idx0_x_d7)
    );

    DataDelay
    #(
        .DATA_BW(H_SIZE_BW)
       ,.STAGE(2)
    ) u_idx1_x_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_idx1_x)
        // Output
        ,.o_data(i_idx1_x_d2)
    );

    DataDelay
    #(
        .DATA_BW(H_SIZE_BW)
       ,.STAGE(3)
    ) u_idx1_x_delay2 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_idx1_x_d2)
        // Output
        ,.o_data(i_idx1_x_d5)
    );

    DataDelay
    #(
        .DATA_BW(H_SIZE_BW)
       ,.STAGE(2)
    ) u_idx1_x_delay3 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_idx1_x_d5)
        // Output
        ,.o_data(i_idx1_x_d7)
    );

    DataDelay
    #(
        .DATA_BW(1)
       ,.STAGE(2)
    ) u_valid0_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(valid0_d2_r)
        // Output
        ,.o_data(valid0_d4)
    );

    DataDelay
    #(
        .DATA_BW(6)
       ,.STAGE(3)
    ) u_sram_read_idx_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(sram_read_idx_r)
        // Output
        ,.o_data(sram_read_idx_d3)
    );

    DataDelay
    #(
        .DATA_BW(CLOUD_BW)
       ,.STAGE(6)
    ) u_trans_z_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_trans_z1)
        // Output
        ,.o_data(trans_z1_d6)
    );

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
        if (!i_rst_n) sram_idx_r <= '0;
        else begin
            if(i_valid1 && idx1_x_clr) begin
                sram_idx_r <= sram_idx_r + 1; //auto overflow
            end
        end
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) i_idx1_y_d1 <= '0;
        else  i_idx1_y_d1 <= i_idx1_y;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) valid0_d1_r <= '0;
        else valid0_d1_r <= i_valid0;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) valid0_d2_r <= '0;
        else valid0_d2_r <= valid0_d1_r;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) sram_read_idx_r <= '0;
        else begin
            if(valid0_d1_r) begin
                sram_read_idx_r <= i_idx1_y_d1[5:0];
            end
        end
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            for(int i = 0; i < 128; i = i+1) begin
                lb_sram_DA_r[i] <= '0;
                lb_sram_AA_r[i] <= '0;
                lb_sram_WENA_r[i] <= 1;
            end
        end
        else if(i_valid1) begin
            for(int i = 0; i < 64; i = i+1) begin
                if(sram_idx_r == i) begin
                    if(!idx1_x_r[0])begin
                        lb_sram_DA_r[i] <= i_depth1;
                        lb_sram_AA_r[i] <= idx1_x_r[H_SIZE_BW-1:1];
                        lb_sram_WENA_r[i] <= !i_valid1;
                        lb_sram_DA_r[i+64] <= '0;
                        lb_sram_AA_r[i+64] <= 511;
                        lb_sram_WENA_r[i+64] <= 1;
                    end
                    else begin
                        lb_sram_DA_r[i] <= '0;
                        lb_sram_AA_r[i] <= 511;
                        lb_sram_WENA_r[i] <= 1;
                        lb_sram_DA_r[i+64] <= i_depth1;
                        lb_sram_AA_r[i+64] <= idx1_x_r[H_SIZE_BW-1:1];
                        lb_sram_WENA_r[i+64] <= !i_valid1;
                    end
                end
                else begin
                    lb_sram_DA_r[i] <= '0;
                    lb_sram_AA_r[i] <= 511;
                    lb_sram_WENA_r[i] <= 1;
                    lb_sram_DA_r[i+64] <= '0;
                    lb_sram_AA_r[i+64] <= 511;
                    lb_sram_WENA_r[i+64] <= 1;
                end
                //if(sram_idx_r-1 == i) begin
                //    lb_sram_AB_r[i] <= idx1_x_r;
                //end
                //else begin
                //    lb_sram_AB_r[i] <= 1023;
                //end
            end
        end
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            for(int i = 0; i < 128; i = i+1) begin
                lb_sram_AB_r[i] <= 511;
                lb_sram_WENB_r[i] <= 1;
            end
        end
        else if(valid0_d2_r) begin
            for(int i = 0; i < 64; i = i+1) begin
                if(sram_read_idx_r == i) begin
                    lb_sram_AB_r[i] <= i_idx1_x_d2[H_SIZE_BW-1:1];
                    lb_sram_AB_r[i+64] <= i_idx1_x_d2[H_SIZE_BW-1:1];
                end
                else begin
                    lb_sram_AB_r[i] <= 511;
                    lb_sram_AB_r[i+64] <= 511;
                end
            end
        end
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            for(int i = 0; i < 128; i = i+1) begin
                lb_sram_QB_r[i] <= '0;
            end
        end
        else begin
            for(int i = 0; i < 128; i = i+1) begin
                lb_sram_QB_r[i] <= i_lb_sram_QB[i];
            end
        end
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            lb_sram_QB_mux_r <= '0;
        end
        else if(valid0_d5_r) begin
            for(int i = 0; i < 64; i = i+1) begin
                if(sram_read_idx_d3 == i && !i_idx1_x_d5[0])
                    lb_sram_QB_mux_r <= lb_sram_QB_r[i];
                else if(sram_read_idx_d3 == i && i_idx1_x_d5[0])
                    lb_sram_QB_mux_r <= lb_sram_QB_r[i+64];
            end
        end
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            lb_sram_QB_mux2_r <= '0;
        end
        else if(valid0_d5_r) begin
            for(int i = 0; i < 64; i = i+1) begin
                if(sram_read_idx_d3 == i && i_idx1_x_d5[0])
                    lb_sram_QB_mux2_r <= i_lb_sram_QB[i];
                else if(sram_read_idx_d3 == i && !i_idx1_x_d5[0])
                    lb_sram_QB_mux2_r <= i_lb_sram_QB[i+64];
            end
        end
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) valid0_d5_r <= '0;
        else valid0_d5_r <= valid0_d4;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) valid0_d6_r <= '0;
        else valid0_d6_r <= valid0_d5_r;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            valid0_d7_r <= 0;
        end
        else if(valid0_d6_r) begin
            if(lb_sram_QB_mux_r>MIN_DEPTH && lb_sram_QB_mux_r<MAX_DEPTH && diff_z < max_diff_depth_w)
                valid0_d7_r <= 1;
            else
                valid0_d7_r <= 0;
        end
        else begin
            valid0_d7_r <= 0;
        end
    end

endmodule

