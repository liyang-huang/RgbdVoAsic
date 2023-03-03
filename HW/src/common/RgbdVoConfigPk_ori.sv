// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
//
// Contributors
// ---------------------------
// Li-Yang Huang <lyhuang@media.ee.ntu.edu.tw>, 2022

`ifndef __RGBDVOCONFIGDEFINES_SV__
`define __RGBDVOCONFIGDEFINES_SV__

// Configuration ==============================================================

package RgbdVoConfigPk;

    parameter DATA_RGB_BW = 8;
    parameter DATA_DEPTH_BW = 16;
    parameter CLOUD_BW = 10+SHIFT_BIT_NUM+DATA_DEPTH_BW; //FX_BW+DATA_DEPTH_BW
    parameter POSE_BW = CLOUD_BW;

    parameter MAX_SRC_WID = 640;
    parameter MAX_SRC_HGT = 480;
    parameter SRC_WID_BW = $clog2(MAX_SRC_WID + 1);
    parameter SRC_HGT_BW = $clog2(MAX_SRC_HGT + 1);

    parameter SHIFT_BIT_NUM = 40;
    //=================================
    // Register Memory Map Define
    //=================================
    typedef enum logic [31:0] {
         DISABLE                  //R/W
        ,H_SIZE                   //R/W
        ,V_SIZE                   //R/W
        ,FX                       //R/W
        ,FY                       //R/W
        ,CX                       //R/W
        ,CY                       //R/W
        ,DEPTH_MAX                //R/W
        ,DEPTH_MIN                //R/W
        ,RESERVED                 //R/W 
    } RegAddr;

    //=================================
    // Register Bitwidth Define
    //=================================
    
    parameter H_SIZE_BW           = SRC_WID_BW;
    parameter V_SIZE_BW           = SRC_HGT_BW;
    parameter FX_BW               = 10+SHIFT_BIT_NUM;
    parameter FY_BW               = FX_BW;
    parameter CX_BW               = FX_BW;
    parameter CY_BW               = FX_BW;
    parameter DEPTH_MAX_BW        = DATA_DEPTH_BW;
    parameter DEPTH_MIN_BW        = DATA_DEPTH_BW;

   //=================================
   // Register Default Define
   //=================================

    parameter H_SIZE_DEFAULT       = 'd640;
    parameter V_SIZE_DEFAULT       = 'd640;
    parameter FX_DEFAULT           = ;
    parameter FY_DEFAULT           = ;
    parameter CX_DEFAULT           = ;
    parameter CY_DEFAULT           = ;
    parameter DEPTH_MAX_DEFAULT    = ;
    parameter DEPTH_MIN_DEFAULT    = ;

endpackage


`endif // __RGBDVOCONFIGDEFINES_SV__
