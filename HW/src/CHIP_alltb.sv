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

    logic frame_start;
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
        frame_start     = 0;
        #5 rst_n=1'b0;         
        #5 rst_n=1'b1;
        #5 frame_start=1;
        #(`CYCLE*2) frame_start=0;
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

    logic                     id_frame_start;
    logic                     id_frame_end;
    logic                     id_frame_valid;
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

    logic                 mat_frame_end;
    logic [MATRIX_BW-1:0] Mat_00;
    logic [MATRIX_BW-1:0] Mat_10;
    logic [MATRIX_BW-1:0] Mat_20;
    logic [MATRIX_BW-1:0] Mat_30;
    logic [MATRIX_BW-1:0] Mat_40;
    logic [MATRIX_BW-1:0] Mat_50;
    logic [MATRIX_BW-1:0] Mat_11;
    logic [MATRIX_BW-1:0] Mat_21;
    logic [MATRIX_BW-1:0] Mat_31;
    logic [MATRIX_BW-1:0] Mat_41;
    logic [MATRIX_BW-1:0] Mat_51;
    logic [MATRIX_BW-1:0] Mat_22;
    logic [MATRIX_BW-1:0] Mat_32;
    logic [MATRIX_BW-1:0] Mat_42;
    logic [MATRIX_BW-1:0] Mat_52;
    logic [MATRIX_BW-1:0] Mat_33;
    logic [MATRIX_BW-1:0] Mat_43;
    logic [MATRIX_BW-1:0] Mat_53;
    logic [MATRIX_BW-1:0] Mat_44;
    logic [MATRIX_BW-1:0] Mat_54;
    logic [MATRIX_BW-1:0] Mat_55;
    logic [MATRIX_BW-1:0] Vec_0;
    logic [MATRIX_BW-1:0] Vec_1;
    logic [MATRIX_BW-1:0] Vec_2;
    logic [MATRIX_BW-1:0] Vec_3;
    logic [MATRIX_BW-1:0] Vec_4;
    logic [MATRIX_BW-1:0] Vec_5;
                                
    logic                 ldlt_done;
    logic [MATRIX_BW-1:0] LDLT_Mat_00;
    logic [MATRIX_BW-1:0] LDLT_Mat_10;
    logic [MATRIX_BW-1:0] LDLT_Mat_20;
    logic [MATRIX_BW-1:0] LDLT_Mat_30;
    logic [MATRIX_BW-1:0] LDLT_Mat_40;
    logic [MATRIX_BW-1:0] LDLT_Mat_50;
    logic [MATRIX_BW-1:0] LDLT_Mat_11;
    logic [MATRIX_BW-1:0] LDLT_Mat_21;
    logic [MATRIX_BW-1:0] LDLT_Mat_31;
    logic [MATRIX_BW-1:0] LDLT_Mat_41;
    logic [MATRIX_BW-1:0] LDLT_Mat_51;
    logic [MATRIX_BW-1:0] LDLT_Mat_22;
    logic [MATRIX_BW-1:0] LDLT_Mat_32;
    logic [MATRIX_BW-1:0] LDLT_Mat_42;
    logic [MATRIX_BW-1:0] LDLT_Mat_52;
    logic [MATRIX_BW-1:0] LDLT_Mat_33;
    logic [MATRIX_BW-1:0] LDLT_Mat_43;
    logic [MATRIX_BW-1:0] LDLT_Mat_53;
    logic [MATRIX_BW-1:0] LDLT_Mat_44;
    logic [MATRIX_BW-1:0] LDLT_Mat_54;
    logic [MATRIX_BW-1:0] LDLT_Mat_55;
                                
    logic                 solver_done;
    logic [MATRIX_BW-1:0] X0;
    logic [MATRIX_BW-1:0] X1;
    logic [MATRIX_BW-1:0] X2;
    logic [MATRIX_BW-1:0] X3;
    logic [MATRIX_BW-1:0] X4;
    logic [MATRIX_BW-1:0] X5;
                                
    IndirectCalc u_indirect_calc(
        // input                
         .i_clk         ( clk )
        ,.i_rst_n       ( rst_n)
        ,.i_frame_start ( frame_start )
        ,.i_frame_end   ( frame_end )
        ,.i_valid       ( valid )
        ,.i_idx0_x      ( u0_i )
        ,.i_idx0_y      ( v0_i )
        ,.i_depth0      ( z0_i )
        ,.i_idx1_x      ( u1_i )
        ,.i_idx1_y      ( v1_i )
        ,.i_pose_0      ( 42'd16777216 )
        ,.i_pose_1      ( 42'd0 )
        ,.i_pose_2      ( 42'd0 )
        ,.i_pose_3      ( 42'd0 )
        ,.i_pose_4      ( 42'd0 )
        ,.i_pose_5      ( 42'd16777216 )
        ,.i_pose_6      ( 42'd0 )
        ,.i_pose_7      ( 42'd0 )
        ,.i_pose_8      ( 42'd0 )
        ,.i_pose_9      ( 42'd0 )
        ,.i_pose_10     ( 42'd16777216 )
        ,.i_pose_11     ( 42'd0 )
        // Register
        ,.r_fx           ( r_fx )
        ,.r_fy           ( r_fy )
        ,.r_cx           ( r_cx )
        ,.r_cy           ( r_cy )
        // Output
        ,.o_frame_start  ( id_frame_start )
        ,.o_frame_end    ( id_frame_end )
        ,.o_valid        ( id_valid )
        ,.o_Ax_0         ( Ax_0 )
        ,.o_Ax_1         ( Ax_1 )
        ,.o_Ax_2         ( Ax_2 )
        ,.o_Ax_3         ( Ax_3 )
        ,.o_Ax_4         ( Ax_4 )
        ,.o_Ax_5         ( Ax_5 )
        ,.o_Ay_0         ( Ay_0 )
        ,.o_Ay_1         ( Ay_1 )
        ,.o_Ay_2         ( Ay_2 )
        ,.o_Ay_3         ( Ay_3 )
        ,.o_Ay_4         ( Ay_4 )
        ,.o_Ay_5         ( Ay_5 )
        ,.o_diffs_x      ( diffs_x )
        ,.o_diffs_y      ( diffs_y )
    );

    Matrix u_matrix(
        // input
         .i_clk         ( clk )
        ,.i_rst_n       ( rst_n)
        ,.i_frame_start ( id_frame_start )
        ,.i_frame_end   ( id_frame_end )
        ,.i_valid       ( id_valid )
        ,.i_Ax_0        ( Ax_0 )
        ,.i_Ax_1        ( Ax_1 )
        ,.i_Ax_2        ( Ax_2 )
        ,.i_Ax_3        ( Ax_3 )
        ,.i_Ax_4        ( Ax_4 )
        ,.i_Ax_5        ( Ax_5 )
        ,.i_Ay_0        ( Ay_0 )
        ,.i_Ay_1        ( Ay_1 )
        ,.i_Ay_2        ( Ay_2 )
        ,.i_Ay_3        ( Ay_3 )
        ,.i_Ay_4        ( Ay_4 )
        ,.i_Ay_5        ( Ay_5 )
        ,.i_diffs_x     ( diffs_x )
        ,.i_diffs_y     ( diffs_y )
        // Output
        ,.o_frame_end   ( mat_frame_end )
        ,.o_Mat_00      ( Mat_00 ) 
        ,.o_Mat_10      ( Mat_10 ) 
        ,.o_Mat_20      ( Mat_20 ) 
        ,.o_Mat_30      ( Mat_30 ) 
        ,.o_Mat_40      ( Mat_40 ) 
        ,.o_Mat_50      ( Mat_50 ) 
        ,.o_Mat_11      ( Mat_11 ) 
        ,.o_Mat_21      ( Mat_21 ) 
        ,.o_Mat_31      ( Mat_31 ) 
        ,.o_Mat_41      ( Mat_41 ) 
        ,.o_Mat_51      ( Mat_51 ) 
        ,.o_Mat_22      ( Mat_22 ) 
        ,.o_Mat_32      ( Mat_32 ) 
        ,.o_Mat_42      ( Mat_42 ) 
        ,.o_Mat_52      ( Mat_52 ) 
        ,.o_Mat_33      ( Mat_33 ) 
        ,.o_Mat_43      ( Mat_43 ) 
        ,.o_Mat_53      ( Mat_53 ) 
        ,.o_Mat_44      ( Mat_44 ) 
        ,.o_Mat_54      ( Mat_54 ) 
        ,.o_Mat_55      ( Mat_55 ) 
        ,.o_Vec_0       ( Vec_0 ) 
        ,.o_Vec_1       ( Vec_1 ) 
        ,.o_Vec_2       ( Vec_2 ) 
        ,.o_Vec_3       ( Vec_3 ) 
        ,.o_Vec_4       ( Vec_4 ) 
        ,.o_Vec_5       ( Vec_5 ) 
    );                     
     
    LDLT u_ldlt (
        // input
         .i_clk        ( clk )
        ,.i_rst_n      ( rst_n)
        ,.i_start       ( mat_frame_end )
        ,.i_Mat_00      ( Mat_00 ) 
        ,.i_Mat_10      ( Mat_10 ) 
        ,.i_Mat_20      ( Mat_20 ) 
        ,.i_Mat_30      ( Mat_30 ) 
        ,.i_Mat_40      ( Mat_40 ) 
        ,.i_Mat_50      ( Mat_50 ) 
        ,.i_Mat_11      ( Mat_11 ) 
        ,.i_Mat_21      ( Mat_21 ) 
        ,.i_Mat_31      ( Mat_31 ) 
        ,.i_Mat_41      ( Mat_41 ) 
        ,.i_Mat_51      ( Mat_51 ) 
        ,.i_Mat_22      ( Mat_22 ) 
        ,.i_Mat_32      ( Mat_32 ) 
        ,.i_Mat_42      ( Mat_42 ) 
        ,.i_Mat_52      ( Mat_52 ) 
        ,.i_Mat_33      ( Mat_33 ) 
        ,.i_Mat_43      ( Mat_43 ) 
        ,.i_Mat_53      ( Mat_53 ) 
        ,.i_Mat_44      ( Mat_44 ) 
        ,.i_Mat_54      ( Mat_54 ) 
        ,.i_Mat_55      ( Mat_55 ) 
        // Output 
        ,.o_done        ( ldlt_done )
        ,.o_Mat_00      ( LDLT_Mat_00 ) 
        ,.o_Mat_10      ( LDLT_Mat_10 ) 
        ,.o_Mat_20      ( LDLT_Mat_20 ) 
        ,.o_Mat_30      ( LDLT_Mat_30 ) 
        ,.o_Mat_40      ( LDLT_Mat_40 ) 
        ,.o_Mat_50      ( LDLT_Mat_50 ) 
        ,.o_Mat_11      ( LDLT_Mat_11 ) 
        ,.o_Mat_21      ( LDLT_Mat_21 ) 
        ,.o_Mat_31      ( LDLT_Mat_31 ) 
        ,.o_Mat_41      ( LDLT_Mat_41 ) 
        ,.o_Mat_51      ( LDLT_Mat_51 ) 
        ,.o_Mat_22      ( LDLT_Mat_22 ) 
        ,.o_Mat_32      ( LDLT_Mat_32 ) 
        ,.o_Mat_42      ( LDLT_Mat_42 ) 
        ,.o_Mat_52      ( LDLT_Mat_52 ) 
        ,.o_Mat_33      ( LDLT_Mat_33 ) 
        ,.o_Mat_43      ( LDLT_Mat_43 ) 
        ,.o_Mat_53      ( LDLT_Mat_53 ) 
        ,.o_Mat_44      ( LDLT_Mat_44 ) 
        ,.o_Mat_54      ( LDLT_Mat_54 ) 
        ,.o_Mat_55      ( LDLT_Mat_55 ) 
    );

    Solver u_solver(
        // input
         .i_clk        ( clk )
        ,.i_rst_n      ( rst_n)
        ,.i_start      ( ldlt_done )
        ,.i_Mat_00     ( LDLT_Mat_00 ) 
        ,.i_Mat_10     ( LDLT_Mat_10 ) 
        ,.i_Mat_20     ( LDLT_Mat_20 ) 
        ,.i_Mat_30     ( LDLT_Mat_30 ) 
        ,.i_Mat_40     ( LDLT_Mat_40 ) 
        ,.i_Mat_50     ( LDLT_Mat_50 ) 
        ,.i_Mat_11     ( LDLT_Mat_11 ) 
        ,.i_Mat_21     ( LDLT_Mat_21 ) 
        ,.i_Mat_31     ( LDLT_Mat_31 ) 
        ,.i_Mat_41     ( LDLT_Mat_41 ) 
        ,.i_Mat_51     ( LDLT_Mat_51 ) 
        ,.i_Mat_22     ( LDLT_Mat_22 ) 
        ,.i_Mat_32     ( LDLT_Mat_32 ) 
        ,.i_Mat_42     ( LDLT_Mat_42 ) 
        ,.i_Mat_52     ( LDLT_Mat_52 ) 
        ,.i_Mat_33     ( LDLT_Mat_33 ) 
        ,.i_Mat_43     ( LDLT_Mat_43 ) 
        ,.i_Mat_53     ( LDLT_Mat_53 ) 
        ,.i_Mat_44     ( LDLT_Mat_44 ) 
        ,.i_Mat_54     ( LDLT_Mat_54 ) 
        ,.i_Mat_55     ( LDLT_Mat_55 ) 
        ,.i_Vec_0      ( Vec_0 ) 
        ,.i_Vec_1      ( Vec_1 ) 
        ,.i_Vec_2      ( Vec_2 ) 
        ,.i_Vec_3      ( Vec_3 ) 
        ,.i_Vec_4      ( Vec_4 ) 
        ,.i_Vec_5      ( Vec_5 ) 
        // Output
        ,.o_done       ( solver_done)
        ,.o_div_zero   (  )
        ,.o_X0         ( X0 )
        ,.o_X1         ( X1 )
        ,.o_X2         ( X2 )
        ,.o_X3         ( X3 )
        ,.o_X4         ( X4 )
        ,.o_X5         ( X5 )
    );



    logic test_valid;                      
    DataDelay
    #(
        .DATA_BW(1)
       ,.STAGE(3)
    ) u_valid_d4 (
        // input
         .i_clk(clk)
        ,.i_rst_n(rst_n)
        ,.i_data(id_valid)
        // Output
        ,.o_data(test_valid)
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
        if(test_valid) begin
            $fwrite(f1, "%d\n", $signed(Mat_10));
        end
    end


endmodule
