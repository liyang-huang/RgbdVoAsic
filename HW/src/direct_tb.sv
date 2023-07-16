`timescale 1ns/10ps
`define CYCLE    10           	         // Modify your clock period here
`define H_ACT 640
`define H_BLANK 64
`define H_TOTAL (`H_ACT+`H_BLANK)
`define V_ACT 480
`define V_TOTAL `V_ACT
`define TIME_OUT `H_TOTAL*(`V_TOTAL+31+10)*2*`CYCLE       
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
//`include "./IndirectCoe.sv"
//`include "./IndirectCalc.sv"
//`include "./MulAcc.sv"
//`include "./Matrix.sv"
//`include "./LDLT.sv"
//`include "./Solver.sv"
//`include "./DW_sqrt.v"
//`include "./DW_sqrt_pipe.v"
//`include "./DW_sincos.v"
//`include "./Rodrigues.sv"
//`include "./UpdatePose.sv"
`include "./DirectCorrCalc.sv"
`include "./LineBufCtrl.sv"
//`include "sram_v3/sram_dp_depth.v"
//`include "sram_v3/sram_lb_FAST.v"
//`include "sram_v3/sram_lb_16b.v"
`include "sram_v3/sram_half_lb_16b.v"


module direct_tb;
    import RgbdVoConfigPk::*;
    
    integer ix, iy, jx, jy, f1, f2, err, index;
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
    logic valid0;
    logic valid1;
    
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
        index       = 0;
        err         = 0;
        valid0     = 0;
        valid1     = 0;
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
        else valid_d1 <= valid0; 
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) frame_end <= 0;
        else if(!valid0 && valid_d1) frame_end <= 1; 
        else frame_end <= 0; 
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) tx_en <= 0;
        //else if(frame_start) tx_en <= 1; 
        else if(jy==31) tx_en <= 1; 
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)
           ix <= 0; 
        else if(tx_en) begin
            if(ix == `H_TOTAL-1 && iy == `V_TOTAL-1) 
               ix <= ix; 
            else if(ix == `H_TOTAL-1) 
               ix <= 0; 
            else
                ix <= ix+1; 
        end
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) 
           iy <= 0; 
        else if(tx_en) begin
            if(ix == `H_TOTAL-1 && iy == `V_TOTAL-1) 
               iy <= iy; 
            else if(ix == `H_TOTAL-1) 
                iy <= iy+1; 
        end
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) begin
           pixel0_i <= 0;
           depth0_i <= 0;
           valid0 <= 0;
        end
        if(tx_en) begin
            if(tx_en && ix<`H_ACT) begin
                pixel0_i <= pixel_in[iy*`H_ACT+ix];
                depth0_i <= depth_in[iy*`H_ACT+ix];
                valid0 <= 1;
            end
            else
                valid0 <= 0;
        end
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)
           jx <= 0; 
        else if(jx == `H_TOTAL-1 && jy == `V_TOTAL-1) 
           jx <= jx; 
        else if(jx == `H_TOTAL-1) 
           jx <= 0; 
        else
            jx <= jx+1; 
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) 
           jy <= 0; 
        else if(jx == `H_TOTAL-1 && jy == `V_TOTAL-1) 
           jy <= jy; 
        else if(jx == `H_TOTAL-1) 
            jy <= jy+1; 
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n) begin
           pixel1_i <= 0;
           depth1_i <= 0;
           valid1 <= 0;
        end
        else if(jx < `H_ACT) begin
            pixel1_i <= pixel_in2[jy*`H_ACT+jx];
            depth1_i <= depth_in2[jy*`H_ACT+jx];
            valid1 <= 1;
        end
        else
            valid1 <= 0;
    end

    logic [34:0] r_fx;
    logic [34:0] r_fy;
    logic [34:0] r_cx;
    logic [34:0] r_cy;
    logic [41:0] initial_pose [12];
    logic [H_SIZE_BW-1:0] r_hsize;
    logic [V_SIZE_BW-1:0] r_vsize;

    logic                     corr_frame_start;
    logic                     corr_frame_end;
    logic                     corr_valid;
    logic [DATA_DEPTH_BW-1:0] corr_depth0;
    logic [CLOUD_BW-1:0]      corr_trans_z1;
    logic [H_SIZE_BW-1:0]     corr_idx0_x;
    logic [V_SIZE_BW-1:0]     corr_idx0_y;
    logic [H_SIZE_BW-1:0]     corr_idx1_x;
    logic [V_SIZE_BW-1:0]     corr_idx1_y;

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
    //assign initial_pose[0] = 42'sd16777001;
    //assign initial_pose[1] = -42'sd69812;
    //assign initial_pose[2] = 42'sd48217;
    //assign initial_pose[3] = 42'sd318084634;
    //assign initial_pose[4] = 42'sd68808;
    //assign initial_pose[5] = 42'sd16773537;
    //assign initial_pose[6] = 42'sd344465;
    //assign initial_pose[7] = 42'sd628608084;
    //assign initial_pose[8] = -42'sd49639;
    //assign initial_pose[9] = -42'sd344263;
    //assign initial_pose[10] = 42'sd16773609;
    //assign initial_pose[11] = -42'sd381085875;
    assign initial_pose[0] = 42'sd16776931;
    assign initial_pose[1] = -42'sd84702;
    assign initial_pose[2] = 42'sd47951;
    assign initial_pose[3] = 42'sd318592230;
    assign initial_pose[4] = 42'sd83604;
    assign initial_pose[5] = 42'sd16772789;
    assign initial_pose[6] = 42'sd376116;
    assign initial_pose[7] = 42'sd502279940;
    assign initial_pose[8] = -42'sd49839;
    assign initial_pose[9] = -42'sd375874;
    assign initial_pose[10] = 42'sd16772927;
    assign initial_pose[11] = -42'sd141558549;

    assign r_fx = 35'd8678853836;
    assign r_fy = 35'd8665432064;
    assign r_cx = 35'd5345221017;
    assign r_cy = 35'd4283223244;
    assign r_hsize = 'd640;
    assign r_vsize = 'd480;
    
    DirectCorrCalc u_directcorrcalc(
        // input
         .i_clk         ( clk )
        ,.i_rst_n       ( rst_n)
        ,.i_frame_start ( frame_start )
        ,.i_frame_end   ( frame_end )
        ,.i_valid       ( valid0 )
        ,.i_data0       ( pixel0_i ) 
        ,.i_depth0      ( depth0_i )
        ,.i_pose        ( initial_pose )
        // Register
        ,.r_fx           ( r_fx )
        ,.r_fy           ( r_fy )
        ,.r_cx           ( r_cx )
        ,.r_cy           ( r_cy )
        ,.r_hsize        ( r_hsize )
        ,.r_vsize        ( r_vsize )
        // Output
        ,.o_frame_start  ( corr_frame_start )
        ,.o_frame_end    ( corr_frame_end   )
        ,.o_valid        ( corr_valid       )
        ,.o_depth0       ( corr_depth0      )
        ,.o_trans_z1     ( corr_trans_z1    )
        ,.o_idx0_x       ( corr_idx0_x      )
        ,.o_idx0_y       ( corr_idx0_y      )
        ,.o_idx1_x       ( corr_idx1_x      )
        ,.o_idx1_y       ( corr_idx1_y      )
    );
    
    logic [15:0]     bus1_sram_QA [0:127];
    logic [15:0]     bus1_sram_QB [0:127];
    logic          bus1_sram_WENA [0:127];
    logic          bus1_sram_WENB [0:127];
    logic [15:0]    bus1_sram_DA  [0:127]; // pixel + depth
    logic [15:0]    bus1_sram_DB  [0:127]; // pixel + depth
    logic [8:0]    bus1_sram_AA   [0:127];
    logic [8:0]    bus1_sram_AB   [0:127];

    logic                     buf_frame_start;
    logic                     buf_frame_end;
    logic                     buf_valid;
    logic [H_SIZE_BW-1:0]     buf_idx0_x;
    logic [V_SIZE_BW-1:0]     buf_idx0_y;
    logic [H_SIZE_BW-1:0]     buf_idx1_x;
    logic [V_SIZE_BW-1:0]     buf_idx1_y;
    logic [DATA_DEPTH_BW-1:0] buf_depth0;
    logic [DATA_DEPTH_BW-1:0] buf_depth1;
    logic [DATA_RGB_BW-1:0]   buf_data0;
    logic [DATA_RGB_BW-1:0]   buf_data1;

    LineBufCtrl u_line_buf_ctrl(
        // input
         .i_clk         ( clk )
        ,.i_rst_n       ( rst_n)
        ,.i_frame_start ( corr_frame_start )
        ,.i_frame_end   ( corr_frame_end   )
        ,.i_valid0       ( corr_valid       )
        ,.i_depth0      ( corr_depth0      )
        ,.i_trans_z1    ( corr_trans_z1    )
        ,.i_idx0_x      ( corr_idx0_x      )
        ,.i_idx0_y      ( corr_idx0_y      )
        ,.i_idx1_x      ( corr_idx1_x      )
        ,.i_idx1_y      ( corr_idx1_y      )
        ,.i_valid1      ( valid1 )
        ,.i_data1       ( pixel1_i ) 
        ,.i_depth1      ( depth1_i )
        // Register
        ,.r_hsize        ( r_hsize )
        ,.r_vsize        ( r_vsize )
        // SRAM
        ,.i_lb_sram_QA   ( bus1_sram_QA )
        ,.i_lb_sram_QB   ( bus1_sram_QB )
        ,.o_lb_sram_WENA ( bus1_sram_WENA )
        ,.o_lb_sram_WENB ( bus1_sram_WENB )
        ,.o_lb_sram_DA   ( bus1_sram_DA )
        ,.o_lb_sram_DB   ( bus1_sram_DB )
        ,.o_lb_sram_AA   ( bus1_sram_AA )
        ,.o_lb_sram_AB   ( bus1_sram_AB )
        // Output
        ,.o_frame_start  ( buf_frame_start )
        ,.o_frame_end    ( buf_frame_end   )
        ,.o_valid        ( buf_valid       )
        ,.o_idx0_x      ( buf_idx0_x )
        ,.o_idx0_y      ( buf_idx0_y )
        ,.o_idx1_x      ( buf_idx1_x )
        ,.o_idx1_y      ( buf_idx1_y )
        ,.o_depth0      ( buf_depth0 )
        ,.o_depth1      ( buf_depth1 )
        ,.o_data0       ( buf_pixel0 ) 
        ,.o_data1       ( buf_pixel1 ) 
    );

    generate
        for(genvar s = 0; s < 128; s = s+1) begin
            //sram_dp_depth uut1 (
            sram_half_lb_16b uut1 (
                // clock signal
                .CLKA(clk),
                .CLKB(clk),

                // sync clock (active high)
                .STOVA(1'b1),
                .STOVB(1'b1),

                // setting
                // In the event of a write/read collision, if COLLDISN is disabled, then the write is guaranteed and
                // the read data is undefined. However, if COLLDISN is enabled, then the write is not guaranteed
                // if the read row address and write row address match.
                .COLLDISN(1'b0),

                // address
                .AA(bus1_sram_AA[s]),
                .AB(bus1_sram_AB[s]),
                // data 
                .DA(bus1_sram_DA[s]),
                .DB(bus1_sram_DB[s]),

                // chip enable (active low, 0 for ON and 1 for OFF)
                // .CENA(1'b1),
                // .CENB(1'b1),
                .CENA(1'b0),
                .CENB(1'b0),

                // write enable (active low, 0 for WRITE and 1 for READ)
                .WENA(bus1_sram_WENA[s]),
                .WENB(bus1_sram_WENB[s]),

                // data output bus
                .QA(bus1_sram_QA[s]),
                .QB(bus1_sram_QB[s]),

                // test mode (active low, 1 for regular operation)
                .TENA(1'b1),
                .TENB(1'b1),

                // bypass
                .BENA(1'b1),
                .BENB(1'b1),

                // useless
                .EMAA(3'd0),
                .EMAB(3'd0),
                .EMAWA(2'd0),
                .EMAWB(2'd0),
                .EMASA(1'b0),
                .EMASB(1'b0),
                .TCENA(1'b1),
                .TWENA(1'b1),
                .TAA(9'd0),
                .TDA(16'd0),
                .TQA(16'd0),
                .TCENB(1'b1),
                .TWENB(1'b1),
                .TAB(9'd0),
                .TDB(16'd0),
                .TQB(16'd0),
                .RET1N(1'b1)
            );
        end
    endgenerate

                           
    always @(posedge clk)begin
        if(buf_valid) begin
          $fwrite(f1, "%h\n", buf_idx0_x);
          $fwrite(f1, "%h\n", buf_idx0_y);
          $fwrite(f1, "%h\n", buf_idx1_x);
          $fwrite(f1, "%h\n", buf_idx1_y);
        end
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
