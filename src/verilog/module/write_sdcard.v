module write_sdcard(
    input  wire        CLK,
    input  wire        RST,

    input  wire        w_rxd,
    output wire        w_txd,

    input  wire        sd_cd,
    output wire        sd_rst,
    output wire        sd_clk,
    inout  wire        sd_cmd,
    inout  wire [ 3:0] sd_dat,

    output wire [31:0] sdcard_data,
    output wire [31:0] sdcard_addr
    );

////////////////////////////////////////////////////////////////////////////////
    wire clk_sd = CLK;
    wire [40:0] sdcram_addr;
    wire        sdcram_ren;
    wire [ 3:0] sdcram_wen;
    wire [31:0] sdcram_wdata;
    wire [31:0] sdcram_rdata;
    wire        sdcram_busy;
    wire [ 8:0] sdcram_state;
    wire [ 2:0] sdi_state;
    wire [ 4:0] sdc_state;

    sdcram#(
        .CACHE_DEPTH(2),
        .BLOCK_NUM(8),
        .POLLING_CYCLES(1024)
    )sdcram_0(
        .i_sys_clk(CLK),
        .i_sys_rst(RST),
        .i_sd_clk(clk_sd),
        .i_sd_rst(RST),

        // for user interface
        .i_sdcram_addr(sdcram_addr),
        .i_sdcram_ren(sdcram_ren),
        .i_sdcram_wen(sdcram_wen),
        .i_sdcram_wdata(sdcram_wdata),
        .o_sdcram_rdata(sdcram_rdata),
        .o_sdcram_busy(sdcram_busy),

        // for debug
        .sdcram_state(sdcram_state),
        .sdi_state(sdi_state),
        .sdc_state(sdc_state),

        // for sd
        .sd_cd(sd_cd),
        .sd_rst(sd_rst),
        .sd_clk(sd_clk),
        .sd_cmd(sd_cmd),
        .sd_dat(sd_dat)
    );
////////////////////////////////////////////////////////////////////////////////
    wire [ 7:0] w_fifo_in_data;
    wire        w_fifo_in_valid;
    wire [ 7:0] w_fifo_out_data;
    wire        w_fifo_out_valid;
    wire        w_fifo_out_ready;
    sync_fifo#(
        .DATA_WIDTH(8),
        .FIFO_DEPTH(64 * 1024)
    )m_sync_fifo(
        .in_data(w_fifo_in_data),
        .in_valid(w_fifo_in_valid),
        .in_ready(),

        .out_data(w_fifo_out_data),
        .out_valid(w_fifo_out_valid),
        .out_ready(w_fifo_out_ready),

        .count(),
        .clear(),

        .clk(CLK),
        .rstn(!RST)
    );

////////////////////////////////////////////////////////////////////////////////

    wire [ 7:0] w_ur_data;
    wire        w_ur_en;
    UartRx ur(CLK, !RST, w_rxd, w_ur_data, w_ur_en);

////////////////////////////////////////////////////////////////////////////////
    localparam  INIT = 0;
    localparam  IDLE = 1;
    localparam  SEND = 2;
    localparam  WAIT = 3;
    reg [1:0] r_state = INIT;

    wire        w_send_en = !sdcram_busy & w_fifo_out_valid & (r_state==IDLE);

    reg  [40:0] r_sdcard_addr;
    reg  [ 3:0] r_sdcard_wen;

    assign w_fifo_in_data  = w_ur_data;
    assign w_fifo_in_valid = w_ur_en;

    assign w_fifo_out_ready = (r_state == IDLE) & !sdcram_busy;

    assign sdcram_addr = r_sdcard_addr;
    assign sdcram_wen  = w_send_en ? r_sdcard_wen : 0;
    assign sdcram_wdata= w_fifo_out_data << {r_sdcard_addr[1:0], 3'd0};

    assign sdcard_data = sdcram_wdata;
    assign sdcard_addr = r_sdcard_addr[31:0];



    always @ (posedge CLK) begin
        if(RST) begin
            r_state <= INIT;
        end
        else begin
        case(r_state)
        INIT: begin
            r_state  <= IDLE;
            r_sdcard_wen <= 4'b0001;
            r_sdcard_addr<= 0;
        end
        IDLE: begin
            if(w_send_en) begin
                r_state      <= SEND;
                r_sdcard_wen <= {r_sdcard_wen[2:0], r_sdcard_wen[3]};
            end
        end
        SEND: begin
            r_state <= WAIT;
        end
        WAIT: begin
            if(!sdcram_busy) begin
                r_state       <= IDLE;
                r_sdcard_addr <= r_sdcard_addr + 1;
            end
        end
        endcase
        end
    end
endmodule

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

/**************************************************************************************************/
/* RS232C serial controller (deserializer):                                                       */
/**************************************************************************************************/
`define SS_SER_WAIT  'd0         // RS232C deserializer, State WAIT
`define SS_SER_RCV0  'd1         // RS232C deserializer, State Receive 0th bit
                                 // States Receive 1st bit to 7th bit are not used
`define SS_SER_DONE  'd9         // RS232C deserializer, State DONE
/**************************************************************************************************/
module UartRx(CLK, RST_X, RXD, DATA, EN);
    input  wire   CLK, RST_X, RXD; // clock, reset, RS232C input
    output [7:0]  DATA;            // 8bit output data
    output reg    EN;              // 8bit output data enable

    reg    [7:0]   DATA;
    reg    [3:0]   stage;
    reg    [12:0]  cnt;             // counter to latch D0, D1, ..., D7
    reg    [11:0]  cnt_start;       // counter to detect the Start Bit
    wire   [12:0]  waitcnt;

    assign waitcnt = `SERIAL_WCNT;

    always @(posedge CLK) begin
        if (!RST_X) cnt_start <= 0;
        else        cnt_start <= (RXD) ? 0 : cnt_start + 1;
    end

    always @(posedge CLK) begin
        if(!RST_X) begin
            EN     <= 0;
            stage  <= `SS_SER_WAIT;
            cnt    <= 1;
            DATA   <= 0;
        end else if (stage == `SS_SER_WAIT) begin // detect the Start Bit
            EN <= 0;
            stage <= (cnt_start == (waitcnt >> 1)) ? `SS_SER_RCV0 : stage;
        end else begin
            if (cnt != waitcnt) begin
                cnt <= cnt + 1;
                EN <= 0;
            end else begin               // receive 1bit data
                stage  <= (stage == `SS_SER_DONE) ? `SS_SER_WAIT : stage + 1;
                EN     <= (stage == 8)  ? 1 : 0;
                DATA   <= {RXD, DATA[7:1]};
                cnt <= 1;
            end
        end
    end
endmodule
