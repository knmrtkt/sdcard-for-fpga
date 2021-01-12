module test_sdccont#(
    parameter TEST_BLK_NUM = 2**18,
    parameter LOG_LENGTH = 1024 * 8
    )(
    input  wire        CLK,
    input  wire        RST,

    output wire        w_txd,
    // sd
    input  wire        sd_cd,
    output wire        sd_rst,
    output wire        sd_clk,
    inout  wire        sd_cmd,
    inout  wire [ 3:0] sd_dat,

    input  wire [31:0] test_blk_cnt,
    output wire [63:0] test_rd_cnt,
    output wire [63:0] test_wr_cnt,
    output wire        test_success,
    output wire [31:0] test_dout,

    input  wire        send_log_en
    );

////////////////////////////////////////////////////////////////////////////////
    wire        sdc_rd;
    wire [ 7:0] sdc_dout;
    wire        sdc_byte_available;
    wire        sdc_wr;
    wire [ 7:0] sdc_din;
    wire        sdc_ready_for_next_byte;
    wire        sdc_ready;
    wire [31:0] sdc_address;
    wire [ 4:0] sdc_state;
    wire [31:0] sdc_blk_cnt;

    sd_controller sdc(
        .i_clk(CLK),
        .i_rst(RST),
        .o_ready(sdc_ready),

        .i_ren(sdc_rd),
        .o_data(sdc_dout),
        .o_data_en(sdc_byte_available),

        .i_wen(sdc_wr),
        .i_data(sdc_din),
        .o_data_ready(sdc_ready_for_next_byte),

        .i_blk_num(sdc_blk_cnt),
        .i_adr(sdc_address),

        .o_state(sdc_state),

        .sd_cd(sd_cd),
        .sd_rst(sd_rst),
        .sd_clk(sd_clk),
        .sd_cmd(sd_cmd),
        .sd_dat(sd_dat)
    );
////////////////////////////////////////////////////////////////////////////////
    localparam  BLOCK_SIZE = 512;

    localparam  SDC_INIT = 0;
    localparam  SDC_IDLE = 1;
    localparam  SDC_WRITE= 2;
    localparam  SDC_READ = 3;
    localparam  SDC_FIN  = 4;

    reg  [ 2:0] r_state = SDC_INIT;
    reg         r_accept = 0;
    reg  [31:0] r_sdc_byte_count = 0;
    reg  [ 7:0] r_sdc_din = 0;
    reg         r_wr = 0;
    reg         r_rd = 0;
    reg  [31:0] r_sdc_dout;
    reg  [31:0] r_test_cnt = 0;
    reg  [31:0] r_sdc_blk_cnt = 0;
    reg  [63:0] r_rd_cnt = 0;
    reg  [63:0] r_wr_cnt = 0;
    reg  [ 4:0] r_sdc_state = 0;

    reg  [31:0] r_bram_adr = 0;
    reg  [23:0] r_bram_cnt = 0;
    wire [31:0] w_bram_adr = (RST) ? 0 : (r_sdc_state == sdc_state) ? r_bram_adr : r_bram_adr + 1;
    wire [31:0] w_bram_dout;
    wire [23:0] w_state_cnt = w_bram_dout[23:0] + 1;
    wire [23:0] w_bram_cnt = (RST) ? 1 : (r_sdc_state == sdc_state) ? r_bram_cnt + 1 : 1;
    wire [31:0] w_bram_din = {3'd0, sdc_state, w_bram_cnt};
    wire        w_bram_we  = (r_state == SDC_INIT) | (r_state == SDC_WRITE) | (r_state == SDC_READ);
    wire        w_fin = (r_test_cnt == TEST_BLK_NUM);

    assign sdc_wr = r_wr;//(r_state == SDC_WRITE);
    assign sdc_rd = r_rd;//(r_state == SDC_READ);
    assign sdc_address = r_test_cnt;
    assign sdc_din = r_sdc_din;
    assign sdc_blk_cnt = r_sdc_blk_cnt;

    assign test_rd_cnt = r_rd_cnt;
    assign test_wr_cnt = r_wr_cnt;
    assign test_success= r_accept & (r_state==SDC_FIN);
    assign test_dout   = r_sdc_dout;

    always @ (posedge CLK) begin
        r_bram_adr <= w_bram_adr;
        r_bram_cnt <= w_bram_cnt;
        r_sdc_state <= sdc_state;
        if(RST) begin
            r_state <= SDC_INIT;
            r_bram_adr <= 0;
        end
        else begin
        case(r_state)
        SDC_INIT: begin
            if(sdc_ready) begin
                r_state <= SDC_WRITE;
                r_wr <= 1;
            end
            else begin
                r_wr <= 0;
            end
            r_sdc_byte_count <= 0;
            r_sdc_din <= 0;
            r_accept  <= 1;
            r_rd <= 0;
            r_sdc_dout <= 0;
            r_test_cnt <= 0;
            r_sdc_blk_cnt <= test_blk_cnt;//`TEST_BLOCK;
            r_rd_cnt <= 0;
            r_wr_cnt <= 0;
        end
        SDC_WRITE: begin
            r_wr_cnt <= r_wr_cnt + 1;
            if(sdc_ready_for_next_byte) begin
                r_sdc_din <= r_sdc_din + 1;
                r_wr <= 0;
            end
            else if(sdc_ready & !r_wr) begin
                if(w_fin) begin
                    r_state <= SDC_READ;
                    r_rd <= 1;
                    r_test_cnt <= 0;
                end
                else begin
                    r_wr <= 1;
                    r_test_cnt <= r_test_cnt + ((r_sdc_blk_cnt==0) ? 1 : r_sdc_blk_cnt);
                end

            end
        end
        SDC_READ: begin
            r_rd_cnt <= r_rd_cnt + 1;
            if(sdc_byte_available) begin
                r_accept <= r_accept & (sdc_dout == r_sdc_byte_count[7:0]);
                r_sdc_byte_count <= r_sdc_byte_count + 1;
                r_sdc_dout <= {r_sdc_dout[23:0], sdc_dout};
                r_rd <= 0;
            end
            else if(sdc_ready & !r_rd) begin
                if(w_fin) begin
                    r_state <= SDC_FIN;
                end
                else begin
                    r_rd <= 1;
                    r_test_cnt <= r_test_cnt + ((r_sdc_blk_cnt==0) ? 1 : r_sdc_blk_cnt);
                end
            end
        end
        SDC_FIN: begin

        end
        endcase
        end
    end
////////////////////////////////////////////////////////////////////////////////
    localparam  UT_INIT = 0;
    localparam  UT_WAIT = 1;
    localparam  UT_SEND = 2;
    localparam  UT_FIN  = 3;

    wire [31:0] w_log_data;

    reg  [31:0] r_ut_addr = 0;
    reg  [ 1:0] r_ut_state = UT_INIT;
    reg         r_ut_we = 0;
    reg         r_ut_lf = 0;
    wire [31:0] w_ut_dat  = w_log_data << {r_ut_addr[1:0], 3'b000};
    wire [ 7:0] w_ut_data = w_ut_dat[31:24];//(r_ut_lf) ? 8'h0A : {4'b0011, w_log_data};
    wire [31:0] w_ut_addr = {2'b00, r_ut_addr[31:2]};
    wire        w_ut_we = w_ut_ready & (r_ut_state == UT_SEND);//r_ut_we;
    wire        w_ut_ready;
    wire        w_sdc_fin = (r_state == SDC_FIN);

    UartTx ut(CLK, RST, w_ut_data, w_ut_we, w_txd, w_ut_ready);

    always @ (posedge CLK) begin
        if(RST) begin
            r_ut_state <= UT_INIT;
        end
        else begin
        case(r_ut_state)
        UT_INIT: begin
            r_ut_addr <= 0;
            r_ut_we   <= 0;
            r_ut_state<= UT_WAIT;
            r_ut_lf   <= 0;
        end
        UT_WAIT: begin
            if(send_log_en) begin
                r_ut_state <= UT_SEND;
            end
        end
        UT_SEND: begin
            if(r_ut_addr == LOG_LENGTH*4) begin
                r_ut_state <= UT_FIN;
            end
            if(w_ut_ready) begin
                r_ut_addr <= r_ut_addr + 1;
            end
        end
        UT_FIN: begin

        end
        endcase
        end
    end
////////////////////////////////////////////////////////////////////////////////

    xilinx_true_dual_port_read_first_byte_write_2_clock_ram #(
      .NB_COL(1),                           // Specify number of columns (number of bytes)
      .COL_WIDTH(32),                        // Specify column width (byte width, typically 8 or 9)
      .RAM_DEPTH(LOG_LENGTH),                     // Specify RAM depth (number of entries)
      .RAM_PERFORMANCE("LOW_LATENCY"), // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
      .INIT_FILE("")                        // Specify name/location of RAM initialization file if using one (leave blank if not)
    ) states_bram (
      .addra(w_bram_adr),   // Port A address bus, width determined from RAM_DEPTH
      .addrb(w_ut_addr),   // Port B address bus, width determined from RAM_DEPTH
      .dina(w_bram_din),     // Port A RAM input data, width determined from NB_COL*COL_WIDTH
      .dinb(0),     // Port B RAM input data, width determined from NB_COL*COL_WIDTH
      .clka(CLK),     // Port A clock
      .clkb(CLK),     // Port B clock
      .wea(w_bram_we),       // Port A write enable, width determined from NB_COL
      .web(0),       // Port B write enable, width determined from NB_COL
      .ena(1),       // Port A RAM Enable, for additional power savings, disable port when not in use
      .enb(1),       // Port B RAM Enable, for additional power savings, disable port when not in use
      .rsta(RST),     // Port A output reset (does not affect memory contents)
      .rstb(RST),     // Port B output reset (does not affect memory contents)
      .regcea(0), // Port A output register enable
      .regceb(0), // Port B output register enable
      .douta(w_bram_dout),   // Port A RAM output data, width determined from NB_COL*COL_WIDTH
      .doutb(w_log_data)    // Port B RAM output data, width determined from NB_COL*COL_WIDTH
    );

////////////////////////////////////////////////////////////////////////////////

endmodule

////////////////////////////////////////////////////////////////////////////////
`define SERIAL_WCNT 100
/******************************************************************************/
module UartTx(CLK, RST, DATA, WE, TXD, READY);
    input  wire        CLK, RST, WE;
    input  wire [7:0]  DATA;
    output reg         TXD, READY;
    reg [8:0]   cmd;
    reg [31:0]  waitnum;
    reg [3:0]   cnt;

    always @(posedge CLK) begin
        if(RST) begin
            TXD       <= 1'b1;
            READY     <= 1'b1;
            cmd       <= 9'h1ff;
            waitnum   <= 0;
            cnt       <= 0;
        end else if( READY ) begin
            TXD       <= 1'b1;
            waitnum   <= 0;
            if( WE )begin
                READY <= 1'b0;
                cmd   <= {DATA, 1'b0};
                cnt   <= 10;
            end
        end else if( waitnum >= `SERIAL_WCNT ) begin
            TXD       <= cmd[0];
            READY     <= (cnt == 1);
            cmd       <= {1'b1, cmd[8:1]};
            waitnum   <= 1;
            cnt       <= cnt - 1;
        end else begin
            waitnum   <= waitnum + 1;
        end
    end
endmodule

//  Xilinx True Dual Port RAM Byte Write Read First Dual Clock RAM
//  This code implements a parameterizable true dual port memory (both ports can read and write).
//  The behavior of this RAM is when data is written, the prior memory contents at the write
//  address are presented on the output port.

module xilinx_true_dual_port_read_first_byte_write_2_clock_ram #(
  parameter NB_COL = 4,                           // Specify number of columns (number of bytes)
  parameter COL_WIDTH = 9,                        // Specify column width (byte width, typically 8 or 9)
  parameter RAM_DEPTH = 1024,                     // Specify RAM depth (number of entries)
  parameter RAM_PERFORMANCE = "HIGH_PERFORMANCE", // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
  parameter INIT_FILE = ""                        // Specify name/location of RAM initialization file if using one (leave blank if not)
) (
  input [clogb2(RAM_DEPTH-1)-1:0] addra,   // Port A address bus, width determined from RAM_DEPTH
  input [clogb2(RAM_DEPTH-1)-1:0] addrb,   // Port B address bus, width determined from RAM_DEPTH
  input [(NB_COL*COL_WIDTH)-1:0] dina,   // Port A RAM input data
  input [(NB_COL*COL_WIDTH)-1:0] dinb,   // Port B RAM input data
  input clka,                            // Port A clock
  input clkb,                            // Port B clock
  input [NB_COL-1:0] wea,                // Port A write enable
  input [NB_COL-1:0] web,                // Port B write enable
  input ena,                             // Port A RAM Enable, for additional power savings, disable port when not in use
  input enb,                             // Port B RAM Enable, for additional power savings, disable port when not in use
  input rsta,                            // Port A output reset (does not affect memory contents)
  input rstb,                            // Port B output reset (does not affect memory contents)
  input regcea,                          // Port A output register enable
  input regceb,                          // Port B output register enable
  output [(NB_COL*COL_WIDTH)-1:0] douta, // Port A RAM output data
  output [(NB_COL*COL_WIDTH)-1:0] doutb  // Port B RAM output data
);

  reg [(NB_COL*COL_WIDTH)-1:0] BRAM [RAM_DEPTH-1:0];
  reg [(NB_COL*COL_WIDTH)-1:0] ram_data_a = {(NB_COL*COL_WIDTH){1'b0}};
  reg [(NB_COL*COL_WIDTH)-1:0] ram_data_b = {(NB_COL*COL_WIDTH){1'b0}};

  // The following code either initializes the memory values to a specified file or to all zeros to match hardware
  generate
    if (INIT_FILE != "") begin: use_init_file
      initial
        $readmemh(INIT_FILE, BRAM, 0, RAM_DEPTH-1);
    end else begin: init_bram_to_zero
      integer ram_index;
      initial
        for (ram_index = 0; ram_index < RAM_DEPTH; ram_index = ram_index + 1)
          BRAM[ram_index] = {(NB_COL*COL_WIDTH){1'b0}};
    end
  endgenerate

  always @(posedge clka)
    if (ena) begin
      ram_data_a <= BRAM[addra];
    end

  always @(posedge clkb)
    if (enb) begin
      ram_data_b <= BRAM[addrb];
    end

  generate
  genvar i;
     for (i = 0; i < NB_COL; i = i+1) begin: byte_write
       always @(posedge clka)
         if (ena)
           if (wea[i])
             BRAM[addra][(i+1)*COL_WIDTH-1:i*COL_WIDTH] <= dina[(i+1)*COL_WIDTH-1:i*COL_WIDTH];
       always @(posedge clkb)
         if (enb)
           if (web[i])
             BRAM[addrb][(i+1)*COL_WIDTH-1:i*COL_WIDTH] <= dinb[(i+1)*COL_WIDTH-1:i*COL_WIDTH];
end
  endgenerate

  //  The following code generates HIGH_PERFORMANCE (use output register) or LOW_LATENCY (no output register)
  generate
    if (RAM_PERFORMANCE == "LOW_LATENCY") begin: no_output_register

      // The following is a 1 clock cycle read latency at the cost of a longer clock-to-out timing
       assign douta = ram_data_a;
       assign doutb = ram_data_b;

    end else begin: output_register

      // The following is a 2 clock cycle read latency with improve clock-to-out timing

      reg [(NB_COL*COL_WIDTH)-1:0] douta_reg = {(NB_COL*COL_WIDTH){1'b0}};
      reg [(NB_COL*COL_WIDTH)-1:0] doutb_reg = {(NB_COL*COL_WIDTH){1'b0}};

      always @(posedge clka)
        if (rsta)
          douta_reg <= {(NB_COL*COL_WIDTH){1'b0}};
        else if (regcea)
          douta_reg <= ram_data_a;

      always @(posedge clkb)
        if (rstb)
          doutb_reg <= {(NB_COL*COL_WIDTH){1'b0}};
        else if (regceb)
          doutb_reg <= ram_data_b;

      assign douta = douta_reg;
      assign doutb = doutb_reg;

    end
  endgenerate

  //  The following function calculates the address width based on specified RAM depth
  function integer clogb2;
    input integer depth;
      for (clogb2=0; depth>0; clogb2=clogb2+1)
        depth = depth >> 1;
  endfunction

endmodule

// The following is an instantiation template for xilinx_true_dual_port_read_first_byte_write_2_clock_ram
/*
  //  Xilinx True Dual Port RAM Byte Write Read First Dual Clock RAM
  xilinx_true_dual_port_read_first_byte_write_2_clock_ram #(
    .NB_COL(4),                           // Specify number of columns (number of bytes)
    .COL_WIDTH(9),                        // Specify column width (byte width, typically 8 or 9)
    .RAM_DEPTH(1024),                     // Specify RAM depth (number of entries)
    .RAM_PERFORMANCE("HIGH_PERFORMANCE"), // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
    .INIT_FILE("")                        // Specify name/location of RAM initialization file if using one (leave blank if not)
  ) your_instance_name (
    .addra(addra),   // Port A address bus, width determined from RAM_DEPTH
    .addrb(addrb),   // Port B address bus, width determined from RAM_DEPTH
    .dina(dina),     // Port A RAM input data, width determined from NB_COL*COL_WIDTH
    .dinb(dinb),     // Port B RAM input data, width determined from NB_COL*COL_WIDTH
    .clka(clka),     // Port A clock
    .clkb(clkb),     // Port B clock
    .wea(wea),       // Port A write enable, width determined from NB_COL
    .web(web),       // Port B write enable, width determined from NB_COL
    .ena(ena),       // Port A RAM Enable, for additional power savings, disable port when not in use
    .enb(enb),       // Port B RAM Enable, for additional power savings, disable port when not in use
    .rsta(rsta),     // Port A output reset (does not affect memory contents)
    .rstb(rstb),     // Port B output reset (does not affect memory contents)
    .regcea(regcea), // Port A output register enable
    .regceb(regceb), // Port B output register enable
    .douta(douta),   // Port A RAM output data, width determined from NB_COL*COL_WIDTH
    .doutb(doutb)    // Port B RAM output data, width determined from NB_COL*COL_WIDTH
  );
*/
