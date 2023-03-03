`timescale 1ns/10ps
`define CYCLE    10           	         // Modify your clock period here
`define TIME_OUT 640*1500*10       
// `define TIME_OUT 640*100*10     


// `ifdef SYN
//     `include "FFT_syn.v"
//     // `include "tsmc13.v"
//     `define SDF
//     `define SDFFILE "FFT_syn.sdf"
// `endif

// simulation
// RTL: ncverilog CHIP_alltb.sv +incdir+/opt/CAD/synopsys/synthesis/2019.12/dw/sim_ver/ -y /opt/CAD/synopsys/synthesis/2019.12/dw/sim_ver +libext+.v+notimingchecks +define+RTL +access+r 

`include "common/RgbdVoConfigPk.sv"
`include "./DW02_mult_2_stage.v"
`include "./DW_div.v"
`include "./DW_div_pipe.v"
`include "./Idx2Cloud.sv"
`include "./TransMat.sv"
`include "./Proj.sv"
`include "./DataDelay.sv"
`include "./IndirectCoe.sv"
`include "./IndirectCalc.sv"

module CHIP_tb;
    import RgbdVoConfigPk::*;
    
    integer i, j, f1, f2, err, index;
    // genvar s;
    logic clk, rst_n;
    logic clk_ref;

    logic [9:0] u0_in [0:280];
    logic [9:0] v0_in [0:280];
    logic [9:0] u1_in [0:280];
    logic [9:0] v1_in [0:280];
    logic [15:0] z0_in [0:280];
    logic [9:0] u0_i;
    logic [9:0] v0_i;
    logic [9:0] u1_i;
    logic [9:0] v1_i;
    logic [15:0] z0_i;

    logic start;
    logic tx_en;
    logic valid;
    
    // `ifdef SDF
    //     initial $sdf_annotate(`SDFFILE, chip0);
    // `endif
    
    initial	begin
        f1 = $fopen("./result.txt","w");
        $readmemh ("./u0.txt", u0_in);
        $readmemh ("./v0.txt", v0_in);
        $readmemh ("./u1.txt", u1_in);
        $readmemh ("./v1.txt", v1_in);
        $readmemh ("./z0.txt", z0_in);
    end


    initial begin
        // f = $fopen("fft_o.txt","w");
        clk_ref         = 1'b1;
        rst_n       = 1'b1;  
        i           = 0;
        j           = 0;
        index       = 0;
        err         = 0;
        valid     = 0;
        start     = 0;
        #5 rst_n=1'b0;         
        #5 rst_n=1'b1;
        #5 start=1;
        #(`CYCLE*2) start=0;
    end

    always begin #(`CYCLE/2) clk_ref = ~clk_ref; end

    initial begin
        $fsdbDumpfile("CHIP.fsdb");
        $fsdbDumpvars(0, CHIP_tb, "+mda");
    end

    initial #(`TIME_OUT) begin
        $display("Time_out! AAAAAA");
        $display("⠄⠄⠄⠄⠄⠄⠄⠈⠉⠁⠈⠉⠉⠙⠿⣿⣿⣿⣿⣿");
        $display("⠄⠄⠄⠄⠄⠄⠄⠄⣀⣀⣀⠄⠄⠄⠄⠄⠹⣿⣿⣿");
        $display("⠄⠄⠄⠄⠄⢐⣲⣿⣿⣯⠭⠉⠙⠲⣄⡀⠄⠈⢿⣿");
        $display("⠐⠄⠄⠰⠒⠚⢩⣉⠼⡟⠙⠛⠿⡟⣤⡳⡀⠄⠄⢻");
        $display("⠄⠄⢀⣀⣀⣢⣶⣿⣦⣭⣤⣭⣵⣶⣿⣿⣏⠄⠄⣿");
        $display("⠄⣼⣿⣿⣿⡉⣿⣀⣽⣸⣿⣿⣿⣿⣿⣿⣿⡆⣀⣿");
        $display("⢠⣿⣿⣿⠿⠟⠛⠻⢿⠿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣼");
        $display("⠄⣿⣿⣿⡆⠄⠄⠄⠄⠳⡈⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⠄⢹⣿⣿⡇⠄⠄⠄⠄⢀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⠄⠄⢿⣿⣷⣨⣽⣭⢁⣡⣿⣿⠟⣩⣿⣿⣿⠿⠿⠟");
        $display("⠄⠄⠈⡍⠻⣿⣿⣿⣿⠟⠋⢁⣼⠿⠋⠉⠄⠄⠄⠄");
        $display("⠄⠄⠄⠈⠴⢬⣙⣛⡥⠴⠂⠄⠄⠄⠄⠄⠄⠄⠄.");
        $finish;
    end

    always @(posedge clk_ref or negedge rst_n)begin
        if(!rst_n) clk <= 0;
        else clk <= !clk; 
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) tx_en <= 0;
        else if(start) tx_en <= 1; 
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) begin
           u0_i <= 0;
           v0_i <= 0;
           u1_i <= 0;
           v1_i <= 0;
           z0_i <= 0;

           valid <= 0;
           i <= 0; 
        end
        if(tx_en) begin
            if(i < 281) begin
                u0_i <= u0_in[i];
                v0_i <= v0_in[i];
                u1_i <= u1_in[i];
                v1_i <= v1_in[i];
                z0_i <= z0_in[i];

                valid <= 1;
                i <= i+1; 
            end
            else
                valid <= 0;
        end
    end

    logic [34:0] r_fx;
    logic [34:0] r_fy;
    logic [34:0] r_cx;
    logic [34:0] r_cy;

    assign r_fx = 35'd8678853836;
    assign r_fy = 35'd8665432064;
    assign r_cx = 35'd5345221017;
    assign r_cy = 35'd4283223244;

    logic                     id_valid;
    logic [ID_COE_BW-1:0]     Ax_0;
    logic [ID_COE_BW-1:0]     Ax_1;
    logic [ID_COE_BW-1:0]     Ax_2;
    logic [ID_COE_BW-1:0]     Ax_3;
    logic [ID_COE_BW-1:0]     Ax_4;
    logic [ID_COE_BW-1:0]     Ax_5;
    logic [ID_COE_BW-1:0]     Ay_0;
    logic [ID_COE_BW-1:0]     Ay_1;
    logic [ID_COE_BW-1:0]     Ay_2;
    logic [ID_COE_BW-1:0]     Ay_3;
    logic [ID_COE_BW-1:0]     Ay_4;
    logic [ID_COE_BW-1:0]     Ay_5;
    logic [H_SIZE_BW:0]       diffs_x;
    logic [V_SIZE_BW:0]       diffs_y;

    IndirectCalc u_indirect_calc(
        // input
         .i_clk     ( clk )
        ,.i_rst_n   ( rst_n)
        ,.i_valid   ( valid )
        ,.i_idx0_x  ( u0_i )
        ,.i_idx0_y  ( v0_i )
        ,.i_depth0  ( z0_i )
        ,.i_idx1_x  ( u1_i )
        ,.i_idx1_y  ( v1_i )
        ,.i_pose_0  ( 42'd16777216 )
        ,.i_pose_1  ( 42'd0 )
        ,.i_pose_2  ( 42'd0 )
        ,.i_pose_3  ( 42'd0 )
        ,.i_pose_4  ( 42'd0 )
        ,.i_pose_5  ( 42'd16777216 )
        ,.i_pose_6  ( 42'd0 )
        ,.i_pose_7  ( 42'd0 )
        ,.i_pose_8  ( 42'd0 )
        ,.i_pose_9  ( 42'd0 )
        ,.i_pose_10 ( 42'd16777216 )
        ,.i_pose_11 ( 42'd0 )
        // Register
        ,.r_fx     ( r_fx )
        ,.r_fy     ( r_fy )
        ,.r_cx     ( r_cx )
        ,.r_cy     ( r_cy )
        // Output
        ,.o_valid  ( id_valid )
        ,.o_Ax_0  ( Ax_0 )
        ,.o_Ax_1  ( Ax_1 )
        ,.o_Ax_2  ( Ax_2 )
        ,.o_Ax_3  ( Ax_3 )
        ,.o_Ax_4  ( Ax_4 )
        ,.o_Ax_5  ( Ax_5 )
        ,.o_Ay_0  ( Ay_0 )
        ,.o_Ay_1  ( Ay_1 )
        ,.o_Ay_2  ( Ay_2 )
        ,.o_Ay_3  ( Ay_3 )
        ,.o_Ay_4  ( Ay_4 )
        ,.o_Ay_5  ( Ay_5 )
        ,.o_diffs_x  ( diffs_x )
        ,.o_diffs_y  ( diffs_y )
    );


    always @(posedge clk)begin
        //if(cloud_valid) begin
        //  $fwrite(f1, "%d\n", $signed(cloud_x));
        //  $fwrite(f1, "%d\n", $signed(cloud_y));
        //  $fwrite(f1, "%d\n", $signed(cloud_z));
        //if(trans_valid) begin
        //    $fwrite(f1, "%d\n", $signed(trans_x));
        //    $fwrite(f1, "%d\n", $signed(trans_y));
        //    $fwrite(f1, "%d\n", $signed(trans_z));
        //if(proj_valid) begin
        //    $fwrite(f1, "%d\n", proj_x);
        //    $fwrite(f1, "%d\n", proj_y);
        if(id_valid) begin
            $fwrite(f1, "%d\n", $signed(Ax_0));
            $fwrite(f1, "%d\n", $signed(Ax_1));
            $fwrite(f1, "%d\n", $signed(Ax_2));
            $fwrite(f1, "%d\n", $signed(Ax_3));
            $fwrite(f1, "%d\n", $signed(Ax_4));
            $fwrite(f1, "%d\n", $signed(Ax_5));
            $fwrite(f1, "%d\n", $signed(Ay_0));
            $fwrite(f1, "%d\n", $signed(Ay_1));
            $fwrite(f1, "%d\n", $signed(Ay_2));
            $fwrite(f1, "%d\n", $signed(Ay_3));
            $fwrite(f1, "%d\n", $signed(Ay_4));
            $fwrite(f1, "%d\n", $signed(Ay_5));
        end
    end


endmodule
