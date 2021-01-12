module top_nexys4ddr(
    input  wire        w_clk,
    input  wire        w_cpu_rst,

    // sd
    input  wire        sd_cd,
    output wire        sd_rst,
    output wire        sd_clk,
    inout  wire        sd_cmd,
    inout  wire [ 3:0] sd_dat,

    input  wire        w_rxd,
	output wire        w_txd,

    output wire [15:0] w_led,
    output reg  [ 6:0] r_sg,
    output reg  [ 7:0] r_an,
    input  wire [15:0] w_sw,
	input  wire        w_btnc,
    input  wire        w_btnd,
    input  wire        w_btnr,
    input  wire        w_btnl,
	input  wire        w_btnu);

    localparam  TEST_SDCCONT = 0;
    localparam  TEST_SDCRAM  = 1;
    localparam  WRITE_SDCARD = 2;

    localparam  MODULE_TYPE = WRITE_SDCARD;

    wire CLK;
    wire RST = ~w_cpu_rst;

    clk_wiz_0 cw0(.reset(), .clk_in1(w_clk), .clk_out1(CLK), .locked());

    generate
    if(MODULE_TYPE == TEST_SDCCONT) begin
        wire [31:0] test_blk_cnt;
        wire [63:0] test_rd_cnt;
        wire [63:0] test_wr_cnt;
        wire        test_success;
        wire [31:0] test_dout;
        wire        send_log_en;

        test_sdccont#(
            .TEST_BLK_NUM(2**4),
            .LOG_LENGTH(1024 * 8)
        )m_test_sdccont(
            .CLK(CLK),
            .RST(RST),

            .w_txd(w_txd),
            // sd
            .sd_cd(sd_cd),
            .sd_rst(sd_rst),
            .sd_clk(sd_clk),
            .sd_cmd(sd_cmd),
            .sd_dat(sd_dat),

            .test_blk_cnt(test_blk_cnt),
            .test_rd_cnt(test_rd_cnt),
            .test_wr_cnt(test_wr_cnt),
            .test_success(test_success),
            .test_dout(test_dout),

            .send_log_en(send_log_en)
        );

        wire [31:0] w_seg_data = (w_btnr) ? test_rd_cnt[31: 0] :
                                 (w_btnu) ? test_rd_cnt[63:32] :
                                 (w_btnl) ? test_wr_cnt[31: 0] :
                                 (w_btnd) ? test_wr_cnt[63:32] :
                                            test_dout;

        assign w_led = {16{test_success}};

        wire [ 6:0] w_sg;
        wire [ 7:0] w_an;

        m_7segcon segcon(CLK, w_seg_data, w_sg, w_an);

        always @(posedge CLK) begin
            r_sg <= w_sg;
            r_an <= w_an;
        end

        assign test_blk_cnt = {16'd0, w_sw};
        assign send_log_en  = w_btnc;

    end
    else if(MODULE_TYPE == TEST_SDCRAM) begin
        wire        test_ren;
        wire        test_wen;
        wire [31:0] test_dout;
        wire [63:0] test_ccnt;
        test_sdcram#(
            .TEST_SIZE(2**27)
        )m_test_sdcram(
            .CLK(CLK),
            .RST(RST),

            .sd_cd(sd_cd),
            .sd_rst(sd_rst),
            .sd_clk(sd_clk),
            .sd_cmd(sd_cmd),
            .sd_dat(sd_dat),

            .test_ren(test_ren),
            .test_wen(test_wen),
            .test_dout(test_dout),
            .test_ccnt(test_ccnt)
        );

        assign test_ren = w_btnr;
        assign test_wen = w_btnl;

        wire [31:0] w_seg_data = (w_btnu) ? test_ccnt[63:32] :
                                 (w_btnd) ? test_ccnt[31: 0] :
                                            test_dout;

        wire [ 6:0] w_sg;
        wire [ 7:0] w_an;

        m_7segcon segcon(CLK, w_seg_data, w_sg, w_an);

        always @(posedge CLK) begin
            r_sg <= w_sg;
            r_an <= w_an;
        end

    end
    else if(MODULE_TYPE == WRITE_SDCARD) begin
        wire [31:0] sdcard_addr;
        wire [31:0] sdcard_data;
        write_sdcard m_write_sdcard(
            .CLK(CLK),
            .RST(RST),
            .w_rxd(w_rxd),
            .w_txd(w_txd),
            .sd_cd(sd_cd),
            .sd_rst(sd_rst),
            .sd_clk(sd_clk),
            .sd_cmd(sd_cmd),
            .sd_dat(sd_dat),
            .sdcard_data(sdcard_data),
            .sdcard_addr(sdcard_addr)
       );

       wire [31:0] w_seg_data = (w_btnc) ? sdcard_addr : sdcard_data;
       wire [ 6:0] w_sg;
       wire [ 7:0] w_an;

       m_7segcon segcon(CLK, w_seg_data, w_sg, w_an);

       always @(posedge CLK) begin
           r_sg <= w_sg;
           r_an <= w_an;
       end
    end
    endgenerate


endmodule

/******************************************************************************/
module m_7segled (w_in, r_led);
  input  wire [3:0] w_in;
  output reg  [6:0] r_led;
  always @(*) begin
    case (w_in)
      4'h0  : r_led <= 7'b1111110;
      4'h1  : r_led <= 7'b0110000;
      4'h2  : r_led <= 7'b1101101;
      4'h3  : r_led <= 7'b1111001;
      4'h4  : r_led <= 7'b0110011;
      4'h5  : r_led <= 7'b1011011;
      4'h6  : r_led <= 7'b1011111;
      4'h7  : r_led <= 7'b1110000;
      4'h8  : r_led <= 7'b1111111;
      4'h9  : r_led <= 7'b1111011;
      4'ha  : r_led <= 7'b1110111;
      4'hb  : r_led <= 7'b0011111;
      4'hc  : r_led <= 7'b1001110;
      4'hd  : r_led <= 7'b0111101;
      4'he  : r_led <= 7'b1001111;
      4'hf  : r_led <= 7'b1000111;
      default:r_led <= 7'b0000000;
    endcase
  end
endmodule

`define DELAY7SEG  100000 // 200000 for 100MHz,100000 for 50MHz,50000 for 25MHz
/******************************************************************************/
module m_7segcon (w_clk, w_din, r_sg, r_an);
  input  wire w_clk;
  input  wire [31:0] w_din;
  output reg [6:0] r_sg;  // cathode segments
  output reg [7:0] r_an;  // common anode

  reg [31:0] r_val   = 0;
  reg [31:0] r_cnt   = 0;
  reg  [3:0] r_in    = 0;
  reg  [2:0] r_digit = 0;
  always@(posedge w_clk) r_val <= w_din;

  always@(posedge w_clk) begin
    r_cnt <= (r_cnt>=(`DELAY7SEG-1)) ? 0 : r_cnt + 1;
    if(r_cnt==0) begin
      r_digit <= r_digit+ 1;
      if      (r_digit==0) begin r_an <= 8'b11111110; r_in <= r_val[3:0];   end
      else if (r_digit==1) begin r_an <= 8'b11111101; r_in <= r_val[7:4];   end
      else if (r_digit==2) begin r_an <= 8'b11111011; r_in <= r_val[11:8];  end
      else if (r_digit==3) begin r_an <= 8'b11110111; r_in <= r_val[15:12]; end
      else if (r_digit==4) begin r_an <= 8'b11101111; r_in <= r_val[19:16]; end
      else if (r_digit==5) begin r_an <= 8'b11011111; r_in <= r_val[23:20]; end
      else if (r_digit==6) begin r_an <= 8'b10111111; r_in <= r_val[27:24]; end
      else                 begin r_an <= 8'b01111111; r_in <= r_val[31:28]; end
    end
  end
  wire [6:0] w_segments;
  m_7segled m_7segled (r_in, w_segments);
  always@(posedge w_clk) r_sg <= ~w_segments;
endmodule
/******************************************************************************/
