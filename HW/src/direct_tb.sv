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
`include "./DW02_mult.v"
`include "./DW_mult_pipe.v"
`include "./Idx2Cloud.sv"
`include "./TransMat.sv"
`include "./Proj.sv"
`include "./DataDelay.sv"
`include "./IndirectCoe.sv"
`include "./IndirectCalc.sv"
`include "./MulAcc.sv"
`include "./Matrix.sv"
`include "./LDLT.sv"
`include "./Solver.sv"
`include "./DW_sqrt.v"
`include "./DW_sqrt_pipe.v"
`include "./DW_sincos.v"
`include "./Rodrigues.sv"
`include "./UpdatePose.sv"

module direct_tb;
    import RgbdVoConfigPk::*;
    
    integer i, j, f1, f2, err, index;
    // genvar s;
    logic clk, rst_n;
    logic clk_ref;

    logic [7:0] pixel_in [0:307199];
    logic [7:0] pixel_in2 [0:307199];
    logic [15:0] depth_in [0:307199];
    logic [15:0] depth_in2 [0:307199];
    logic [7:0] pixel0_i;
    logic [7:0] pixel1_i;
    logic [15:0] depth0_i;
    logic [15:0] depth1_i;

    logic frame_start;
    logic tx_en;
    logic valid;
    
    // `ifdef SDF
    //     initial $sdf_annotate(`SDFFILE, chip0);
    // `endif
    
    initial	begin
        f1 = $fopen("./result.txt","w");
        $readmemh ("../sim_data/pixel_in.txt", pixel_in);
        $readmemh ("../sim_data/pixel_in2.txt", pixel_in2);
        $readmemh ("../sim_data/depth_in1.txt", depth_in);
        $readmemh ("../sim_data/depth_in2.txt", depth_in2);
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
        frame_start     = 0;
        #5 rst_n=1'b0;         
        #5 rst_n=1'b1;
        #5 frame_start=1;
        #(`CYCLE*2) frame_start=0;
    end

    always begin #(`CYCLE/2) clk_ref = ~clk_ref; end

    initial begin
        $fsdbDumpfile("direct.fsdb");
        $fsdbDumpvars(0, direct_tb, "+mda");
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

    logic valid_d1, frame_end;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) valid_d1 <= 0;
        else valid_d1 <= valid; 
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) frame_end <= 0;
        else if(!valid && valid_d1) frame_end <= 1; 
        else frame_end <= 0; 
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) tx_en <= 0;
        else if(frame_start) tx_en <= 1; 
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) begin
           pixel0_i <= 0;
           pixel1_i <= 0;
           depth0_i <= 0;
           depth1_i <= 0;

           valid <= 0;
           i <= 0; 
        end
        if(tx_en) begin
            if(i < 307200) begin
                pixel0_i <= pixel_in[i];
                pixel1_i <= pixel_in2[i];
                depth0_i <= depth_in[i];
                depth1_i <= depth_in2[i];

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
    logic [41:0] initial_pose [12];

    //assign initial_pose[0]  = 42'd16777216;
    //assign initial_pose[1]  = 42'd0;
    //assign initial_pose[2]  = 42'd0;
    //assign initial_pose[3]  = 42'd0;
    //assign initial_pose[4]  = 42'd0;
    //assign initial_pose[5]  = 42'd16777216;
    //assign initial_pose[6]  = 42'd0;
    //assign initial_pose[7]  = 42'd0;
    //assign initial_pose[8]  = 42'd0;
    //assign initial_pose[9]  = 42'd0;
    //assign initial_pose[10] = 42'd16777216;
    //assign initial_pose[11] = 42'd0;
    assign initial_pose[0] = 42'sd16777001;
    assign initial_pose[1] = -42'sd69812;
    assign initial_pose[2] = 42'sd48217;
    assign initial_pose[3] = 42'sd318084634;
    assign initial_pose[4] = 42'sd68808;
    assign initial_pose[5] = 42'sd16773537;
    assign initial_pose[6] = 42'sd344465;
    assign initial_pose[7] = 42'sd628608084;
    assign initial_pose[8] = -42'sd49639;
    assign initial_pose[9] = -42'sd344263;
    assign initial_pose[10] = 42'sd16773609;
    assign initial_pose[11] = -42'sd381085875;

    assign r_fx = 35'd8678853836;
    assign r_fy = 35'd8665432064;
    assign r_cx = 35'd5345221017;
    assign r_cy = 35'd4283223244;



                           
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
        //if(id_valid) begin
        //    $fwrite(f1, "%d\n", $signed(Ax_0));
        //    $fwrite(f1, "%d\n", $signed(Ax_1));
        //    $fwrite(f1, "%d\n", $signed(Ax_2));
        //    $fwrite(f1, "%d\n", $signed(Ax_3));
        //    $fwrite(f1, "%d\n", $signed(Ax_4));
        //    $fwrite(f1, "%d\n", $signed(Ax_5));
        //    $fwrite(f1, "%d\n", $signed(Ay_0));
        //    $fwrite(f1, "%d\n", $signed(Ay_1));
        //    $fwrite(f1, "%d\n", $signed(Ay_2));
        //    $fwrite(f1, "%d\n", $signed(Ay_3));
        //    $fwrite(f1, "%d\n", $signed(Ay_4));
        //    $fwrite(f1, "%d\n", $signed(Ay_5));
        //if(test_valid) begin
        //    $fwrite(f1, "%d\n", $signed(Mat_10));
        //end
    end


endmodule