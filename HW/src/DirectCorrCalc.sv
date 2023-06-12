// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2023


module DirectCorrCalc
    import RgbdVoConfigPk::*;
#(
)(
    // input
     input                            i_clk
    ,input                            i_rst_n
    ,input                            i_frame_start
    ,input                            i_frame_end
    ,input                            i_valid
    ,input        [DATA_RGB_BW-1:0]   i_data0
    ,input        [DATA_DEPTH_BW-1:0] i_depth0
    ,input        [POSE_BW-1:0]       i_pose [12]
    // Register
    ,input        [FX_BW-1:0]         r_fx
    ,input        [FY_BW-1:0]         r_fy
    ,input        [CX_BW-1:0]         r_cx
    ,input        [CY_BW-1:0]         r_cy
    ,input        [H_SIZE_BW-1:0]     r_hsize
    ,input        [V_SIZE_BW-1:0]     r_vsize
    // Output
    ,output logic                     o_frame_start
    ,output logic                     o_frame_end
    ,output logic                     o_valid
    ,output logic [DATA_DEPTH_BW-1:0] o_depth0
    ,output logic [H_SIZE_BW-1:0]     o_idx0_x
    ,output logic [V_SIZE_BW-1:0]     o_idx0_y
    ,output logic [H_SIZE_BW-1:0]     o_idx1_x
    ,output logic [V_SIZE_BW-1:0]     o_idx1_y
);

    //=================================
    // Signal Declaration
    //=================================
    logic                     cloud_valid;
    logic [CLOUD_BW-1:0]      cloud_x;
    logic [CLOUD_BW-1:0]      cloud_y;
    logic [CLOUD_BW-1:0]      cloud_z;
    logic                     trans_valid;
    logic [CLOUD_BW-1:0]      trans_x;
    logic [CLOUD_BW-1:0]      trans_y;
    logic [CLOUD_BW-1:0]      trans_z;
    logic                     proj_valid;
    logic [H_SIZE_BW-1:0]     proj_x;
    logic [V_SIZE_BW-1:0]     proj_y;
    logic [H_SIZE_BW-1:0]     idx0_x_w;
    logic [H_SIZE_BW-1:0]     idx0_x_r;
    logic [V_SIZE_BW-1:0]     idx0_y_w;
    logic [V_SIZE_BW-1:0]     idx0_y_r;
    logic                     idx0_x_clr;
    logic                     idx0_y_clr;
    logic                     valid_depth0_r;
    logic [DATA_RGB_BW-1:0]   data0_d1;
    logic [DATA_DEPTH_BW-1:0] depth0_d1;
    logic [H_SIZE_BW-1:0]     idx0_x_d1;
    logic [V_SIZE_BW-1:0]     idx0_y_d1;
    logic [DATA_DEPTH_BW-1:0] depth0_d12;
    logic [H_SIZE_BW-1:0]     idx0_x_d12;
    logic [V_SIZE_BW-1:0]     idx0_y_d12;

    //=================================
    // Combinational Logic
    //=================================
    assign o_idx0_x = idx0_x_d12;
    assign o_idx0_y = idx0_y_d12;
    assign o_idx1_x = proj_x;
    assign o_idx1_y = proj_y;
    assign o_valid = proj_valid;
    assign idx0_x_clr = (idx0_x_r==r_hsize-1);
    assign idx0_y_clr = (idx0_y_r==r_vsize-1);
    
    //4T
    Idx2Cloud u_idx2cloud (
        // input
         .i_clk   ( i_clk )
        ,.i_rst_n ( i_rst_n )
        ,.i_valid ( valid_depth0_r )
        ,.i_idx_x ( idx0_x_d1 )
        ,.i_idx_y ( idx0_y_d1 )
        ,.i_depth ( depth0_d1 )
        // Register
        ,.r_fx     ( r_fx )
        ,.r_fy     ( r_fy )
        ,.r_cx     ( r_cx )
        ,.r_cy     ( r_cy )
        // Output
        ,.o_valid   (cloud_valid)
        ,.o_cloud_x (cloud_x)
        ,.o_cloud_y (cloud_y)
        ,.o_cloud_z (cloud_z)
    );

    //3T
    TransMat u_transmat(
        // input
         .i_clk      ( i_clk )
        ,.i_rst_n    ( i_rst_n)
        ,.i_valid    ( cloud_valid )
        ,.i_cloud_x  ( cloud_x )
        ,.i_cloud_y  ( cloud_y )
        ,.i_cloud_z  ( cloud_z )
        ,.i_pose     ( i_pose  )
        // Output
        ,.o_valid    ( trans_valid )
        ,.o_cloud_x  ( trans_x )
        ,.o_cloud_y  ( trans_y )
        ,.o_cloud_z  ( trans_z )
    );

    //4T
    Proj u_proj (
        // input
         .i_clk      ( i_clk )
        ,.i_rst_n    ( i_rst_n )
        ,.i_valid   ( trans_valid )
        ,.i_cloud_x ( trans_x )
        ,.i_cloud_y ( trans_y )
        ,.i_cloud_z ( trans_z )
        // Register
        ,.r_fx      ( r_fx )
        ,.r_fy      ( r_fy )
        ,.r_cx      ( r_cx )
        ,.r_cy      ( r_cy )
        // Output
        ,.o_valid   ( proj_valid ) 
        ,.o_idx_x   ( proj_x )
        ,.o_idx_y   ( proj_y )
    );

    DataDelay
    #(
        .DATA_BW(DATA_RGB_BW)
       ,.STAGE(1)
    ) u_data0_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_data0)
        // Output
        ,.o_data(data0_d1)
    );

    DataDelay
    #(
        .DATA_BW(DATA_DEPTH_BW)
       ,.STAGE(1)
    ) u_depth0_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(i_depth0)
        // Output
        ,.o_data(depth0_d1)
    );

    DataDelay
    #(
        .DATA_BW(H_SIZE_BW)
       ,.STAGE(1)
    ) u_idx0_x_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(idx0_x_r)
        // Output
        ,.o_data(idx0_x_d1)
    );

    DataDelay
    #(
        .DATA_BW(V_SIZE_BW)
       ,.STAGE(1)
    ) u_idx0_y_delay (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(idx0_y_r)
        // Output
        ,.o_data(idx0_y_d1)
    );

    DataDelay
    #(
        .DATA_BW(DATA_DEPTH_BW)
       ,.STAGE(11)
    ) u_depth0_delay_2 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(depth0_d1)
        // Output
        ,.o_data(depth0_d12)
    );

    DataDelay
    #(
        .DATA_BW(H_SIZE_BW)
       ,.STAGE(11)
    ) u_idx0_x_delay_2 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(idx0_x_d1)
        // Output
        ,.o_data(idx0_x_d12)
    );

    DataDelay
    #(
        .DATA_BW(V_SIZE_BW)
       ,.STAGE(11)
    ) u_idx0_y_delay_2 (
        // input
         .i_clk(i_clk)
        ,.i_rst_n(i_rst_n)
        ,.i_data(idx0_y_d1)
        // Output
        ,.o_data(idx0_y_d12)
    );

    always_comb begin
        if(i_valid) begin
            if(idx0_x_clr)
                idx0_x_w = 0;
            else
                idx0_x_w = idx0_x_r + 1;
        end
        else begin
            idx0_x_w = idx0_x_r;
        end
    end

    always_comb begin
        if(i_valid && idx0_x_clr) begin
            if(idx0_y_clr)
                idx0_y_w = 0;
            else
                idx0_y_w = idx0_y_r + 1;
        end
        else begin
            idx0_y_w = idx0_y_r;
        end
    end

    //===================
    //    Sequential
    //===================
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) idx0_x_r <= '0;
        else idx0_x_r <= idx0_x_w;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) idx0_y_r <= '0;
        else idx0_y_r <= idx0_y_w;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) valid_depth0_r <= 0;
        else if(i_valid && i_depth0>0 && i_depth0<'d20000) valid_depth0_r <= 1;
        else valid_depth0_r <= 0;
    end


endmodule

