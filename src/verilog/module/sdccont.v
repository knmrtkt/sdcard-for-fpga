/*
MIT License

Copyright (c) 2021 takuto kanamori

Permission is hereby granted, free of charge, to any person obtaining a copy of 
this software and associated documentation files (the "Software"), to deal 
in the Software without restriction, including without limitation the rights 
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

///////////////////////////////////////////////////////////////////////////////
// 41bit address space
// adjust sd card ver 2.00 or later
///////////////////////////////////////////////////////////////////////////////

module sd_controller(
    input  wire        i_clk,
    input  wire        i_rst,

    output wire        o_ready,

    input  wire        i_ren,
    output wire [ 7:0] o_data,
    output wire        o_data_en,

    input  wire        i_wen,
    input  wire [ 7:0] i_data,
    output wire        o_data_ready,

    input  wire [31:0] i_blk_num,
    input  wire [31:0] i_adr,

    // for debug
    output wire [ 4:0] o_state,

    input  wire        sd_cd,
    output wire        sd_rst,
    output reg         sd_clk,
    inout  wire        sd_cmd,
    inout  wire [ 3:0] sd_dat
);

///////////////////////////////////////////////////////////////////////////////
    wire        dat_trans_o_ready;
    wire [ 1:0] dat_trans_i_funct;
    wire [31:0] dat_trans_i_blk_num;
    wire [ 3:0] dat_trans_o_state;
    sd_dat_transceiver sd_dat_trans(
        .i_clk(i_clk),
        .i_rst(i_rst),

        .o_ready(dat_trans_o_ready),
        // 00 - idle, 01 - read, 10 - write, 11 - busy wait
        .i_funct(dat_trans_i_funct), 
        .i_blk_num(dat_trans_i_blk_num), // >= 1

        .o_data(o_data),
        .o_data_en(o_data_en),

        .i_data(i_data),
        .o_data_ready(o_data_ready),

        .sd_clk(sd_clk),
        .sd_dat(sd_dat),

        .o_state(dat_trans_o_state)
    );
///////////////////////////////////////////////////////////////////////////////
    wire        cmd_trans_o_ready;
    wire        cmd_trans_i_cmd_en;
    wire [ 5:0] cmd_trans_i_cmd_no;
    wire [31:0] cmd_trans_i_cmd_arg;
    wire        cmd_trans_o_rsp_en;
    wire [47:0] cmd_trans_o_rsp_dat;
    wire        cmd_trans_o_rsp_bsy;
    wire [ 2:0] cmd_trans_o_state;
    sd_cmd_transceiver sd_cmd_trans(
        .i_clk(i_clk),
        .i_rst(i_rst),

        .o_ready(cmd_trans_o_ready),

        .i_cmd_en(cmd_trans_i_cmd_en),
        .i_cmd_no(cmd_trans_i_cmd_no),
        .i_cmd_arg(cmd_trans_i_cmd_arg),

        .o_rsp_en(cmd_trans_o_rsp_en),
        .o_rsp_dat(cmd_trans_o_rsp_dat),
        .o_rsp_bsy(cmd_trans_o_rsp_bsy),

        .sd_clk(sd_clk),
        .sd_cmd(sd_cmd),

        .o_state(cmd_trans_o_state)
    );
///////////////////////////////////////////////////////////////////////////////
    reg [ 5:0] cmd_no = 0;
    reg        cmd_en = 0;
    reg [31:0] cmd_arg = 0;

    assign cmd_trans_i_cmd_en = cmd_en;
    assign cmd_trans_i_cmd_no = cmd_no;
    assign cmd_trans_i_cmd_arg= cmd_arg;
///////////////////////////////////////////////////////////////////////////////
    localparam TRANS_IDLE = 2'b00;
    localparam TRANS_READ = 2'b01;
    localparam TRANS_WRITE = 2'b10;
    localparam TRANS_BWAIT = 2'b11;

    reg [ 1:0] trans_funct = TRANS_IDLE;
    reg [31:0] trans_blk_num = 0;

    assign dat_trans_i_funct = trans_funct;
    assign dat_trans_i_blk_num = trans_blk_num;
///////////////////////////////////////////////////////////////////////////////
    // for init
    localparam INIT = 0;
    localparam IDLE = 1;

    localparam EXEC_CMD = 2;
    localparam WAIT_BSY = 3;
    localparam WAIT_TRS = 4;
    localparam EXEC_WRT = 5;
    localparam WAIT_WRT = 6;
    localparam WAIT_INI = 7;
    localparam WAIT_STP = 8;


    localparam CMD00 = 16;
    localparam CMD08 = 17;
    localparam CMD55 = 18;
    localparam CMD41 = 19;
    localparam CHK41 = 20;
    localparam CMD02 = 21;
    localparam CMD03 = 22;
    localparam CMD07 = 23;
    localparam CMD17 = 24;
    localparam CMD18 = 25;
    localparam CMD24 = 26;
    localparam CMD25 = 27;
    localparam CMD12 = 28;
    localparam SETBW = 29;
    localparam CMD06 = 30;

    reg [ 4:0] state = INIT;
    reg [ 4:0] return_state;
    reg [31:0] boot_cnt = 32'd10_000_000;
    reg [31:0] blk_cnt = 0;
    reg [31:0] blk_adr = 0;
    reg [15:0] RCA = 0;

    assign o_ready = (state == IDLE);
    assign o_state = state;

    assign sd_rst = 0;
///////////////////////////////////////////////////////////////////////////////

    always @(posedge i_clk) begin
        sd_clk <= ~sd_clk;
        if(i_rst == 1) begin
            state <= INIT;
            return_state <= INIT;
            boot_cnt <= 32'd10_000_000; //  >= 1ms
            blk_cnt <= 0;
            blk_adr <= 0;

            cmd_no <= 0;
            cmd_en <= 0;
            cmd_arg<= 0;

            trans_funct <= TRANS_IDLE;
            trans_blk_num <= 0;

            RCA <= 0;
        end
        else begin
            case(state)
            INIT: begin
                if(boot_cnt == 0) begin
                    state <= CMD00;
                    boot_cnt <= 32'd10_000_000;
                end
                else begin
                    boot_cnt <= boot_cnt - 1;
                end
            end
            IDLE: begin
                if(i_ren == 1) begin
                    if(i_blk_num == 0) begin
                        state <= CMD17;
                        blk_cnt <= 1;
                        blk_adr <= i_adr;
                    end
                    else begin
                        state <= CMD18;
                        blk_cnt <= i_blk_num;
                        blk_adr <= i_adr;
                    end
                end
                else if(i_wen == 1) begin
                    if(i_blk_num == 0) begin
                        state <= CMD24;
                        blk_cnt <= 1;
                        blk_adr <= i_adr;
                    end
                    else begin
                        state <= CMD25;
                        blk_cnt <= i_blk_num;
                        blk_adr <= i_adr;
                    end
                end
            end
            EXEC_CMD: begin
                cmd_en <= 0;
                trans_funct <= TRANS_IDLE;
                if(cmd_trans_o_rsp_en) begin
                    if(cmd_trans_o_rsp_bsy) begin
                        state <= WAIT_BSY;
                        trans_funct <= WAIT_BSY;
                    end
                    else begin
                        state <= return_state;
                    end
                end
            end
            WAIT_BSY: begin
                trans_funct <= TRANS_IDLE;
                if(dat_trans_o_ready & (trans_funct == TRANS_IDLE)) begin
                    state <= return_state;
                end
            end
            WAIT_TRS: begin
                trans_funct <= TRANS_IDLE;
                if(dat_trans_o_ready & (trans_funct != TRANS_BWAIT)) begin
                    state <= IDLE;
                end
            end
            EXEC_WRT: begin
                if(dat_trans_o_ready) begin
                    trans_funct <= TRANS_WRITE;
                    trans_blk_num <= blk_cnt;
                    state <= WAIT_WRT;
                    return_state <= (cmd_no == 6'd24) ? IDLE : CMD12;
                end
            end
            WAIT_WRT: begin
                trans_funct <= TRANS_IDLE;
                if(cmd_trans_o_ready & dat_trans_o_ready & (trans_funct!=TRANS_WRITE)) begin
                    state <= return_state;
                end
            end
            WAIT_STP: begin
                if(dat_trans_o_ready) begin
                    trans_funct <= TRANS_BWAIT;
                    state <= WAIT_TRS;
                end
            end
            CMD00: begin
                if(cmd_trans_o_ready) begin
                    cmd_no <= 6'd00;
                    cmd_en <= 1;
                    cmd_arg<= 0;
                    return_state <= CMD08;
                    state <= EXEC_CMD;
                end
            end
            CMD08: begin
                if(cmd_trans_o_ready) begin
                    cmd_no <= 6'd08;
                    cmd_en <= 1;
                    cmd_arg<= 0;
                    return_state <= WAIT_INI;//CMD55;
                    state <= EXEC_CMD;
                end
            end
            WAIT_INI: begin
                if(boot_cnt == 0) begin
                    boot_cnt <= 32'd10_000_000;
                    state <= CMD55;
                    return_state <= CMD41;
                end
                else begin
                    boot_cnt <= boot_cnt - 1;
                end
            end
            CMD55: begin
                if(cmd_trans_o_ready) begin
                    cmd_no <= 6'd55;
                    cmd_en <= 1;
                    cmd_arg<= {RCA, 16'd0};
                    state <= EXEC_CMD;
                end
            end
            CMD41: begin
                if(cmd_trans_o_ready) begin
                    cmd_no <= 6'd41;
                    cmd_en <= 1;
                    cmd_arg<= 0;
                    return_state <= CHK41;
                    state <= EXEC_CMD;
                end
            end
            CHK41: begin
                if(cmd_trans_o_rsp_dat[39] == 1) begin
                    state <= CMD02;
                end
                else begin
                    state <= WAIT_INI;//CMD55;
                end
            end
            CMD02: begin
                if(cmd_trans_o_ready) begin
                    cmd_no <= 6'd02;
                    cmd_en <= 1;
                    cmd_arg<= 0;
                    return_state <= CMD03;
                    state <= EXEC_CMD;
                end
            end
            CMD03: begin
                if(cmd_trans_o_ready) begin
                    cmd_no <= 6'd03;
                    cmd_en <= 1;
                    cmd_arg<= 0;
                    return_state <= CMD07;
                    state <= EXEC_CMD;
                end
            end
            CMD07: begin
                if(cmd_trans_o_ready & dat_trans_o_ready) begin
                    cmd_no <= 6'd07;
                    cmd_en <= 1;
                    cmd_arg<= {cmd_trans_o_rsp_dat[39:24], 16'd0};
                    RCA <= cmd_trans_o_rsp_dat[39:24];
                    //trans_funct <= TRANS_BWAIT;
                    trans_blk_num <= 0;
                    return_state <= SETBW;//IDLE;
                    state <= EXEC_CMD;
                end
            end
            SETBW: begin
                state <= CMD55;
                return_state <= CMD06;
            end
            CMD06: begin
                if(cmd_trans_o_ready) begin
                    cmd_no <= 6'd06;
                    cmd_en <= 1;
                    cmd_arg<= 0;
                    return_state <= IDLE;
                    state <= EXEC_CMD;
                end
            end
            // single read
            CMD17: begin
                if(cmd_trans_o_ready & dat_trans_o_ready) begin
                    cmd_no <= 6'd17;
                    cmd_en <= 1;
                    cmd_arg<= blk_adr;
                    trans_funct <= TRANS_READ;
                    trans_blk_num <= blk_cnt;
                    state <= EXEC_CMD;
                    return_state <= WAIT_TRS;
                end
            end
            // mult read
            CMD18: begin
                if(cmd_trans_o_ready & dat_trans_o_ready) begin
                    cmd_no <= 6'd18;
                    cmd_en <= 1;
                    cmd_arg<= blk_adr;
                    trans_funct <= TRANS_READ;
                    trans_blk_num <= blk_cnt;
                    state <= EXEC_CMD;
                    return_state <= CMD12;
                end
            end
            // single write
            CMD24: begin
                if(cmd_trans_o_ready) begin
                    cmd_no <= 6'd24;
                    cmd_en <= 1;
                    cmd_arg<= blk_adr;
                    state <= EXEC_CMD;
                    return_state <= EXEC_WRT;
                end
            end
            // mult write
            CMD25: begin
                if(cmd_trans_o_ready) begin
                    cmd_no <= 6'd25;
                    cmd_en <= 1;
                    cmd_arg<= blk_adr;
                    state <= EXEC_CMD;
                    return_state <= EXEC_WRT;
                end
            end

            // stop mult
            CMD12: begin
                if(cmd_trans_o_ready & dat_trans_o_ready) begin
                    cmd_no <= 6'd12;
                    cmd_en <= 1;
                    cmd_arg<= blk_adr;
                    //trans_funct <= TRANS_BWAIT;
                    state <= EXEC_CMD;
                    return_state <= WAIT_STP;//WAIT_TRS;
                end
            end
            endcase
        end
    end
endmodule

module sd_dat_transceiver(
    input  wire        i_clk,
    input  wire        i_rst,

    output wire        o_ready,

    input  wire [ 1:0] i_funct, // 00 - idle, 01 - read, 10 - write, 11 - busy wait
    input  wire [31:0] i_blk_num, // >= 1

    output reg  [ 7:0] o_data,
    output reg         o_data_en,

    input  wire [ 7:0] i_data,
    output reg         o_data_ready,

    input  wire        sd_clk,
    inout  wire [ 3:0] sd_dat,

    output wire [ 3:0] o_state
    );

    localparam IDLE     = 4'b0111;
    localparam SEND_DAT = 4'b0001;
    localparam SEND_CRC = 4'b0010;
    localparam SEND_INI = 4'b0011;

    localparam WAIT_DAT = 4'b1000;
    localparam RECV_DAT = 4'b1001;

    localparam WAIT_RSP = 4'b1010;
    localparam RECV_RSP = 4'b1011;
    localparam WAIT_BSY = 4'b1100;

    reg  [ 3:0] state;

    reg  [47:0] rsp_out;
    reg  [ 7:0] rsp_cnt;
    reg  [15:0] dat_cnt;
    reg  [31:0] blk_cnt;

    reg  [15:0] dat_out_0 = 16'hFFFF;
    reg  [15:0] dat_out_1 = 16'hFFFF;
    reg  [15:0] dat_out_2 = 16'hFFFF;
    reg  [15:0] dat_out_3 = 16'hFFFF;

    assign o_ready = (state == IDLE);
    assign o_state = state;

    assign sd_dat[0] = state[3] ? 1'bz : dat_out_0[15];
    assign sd_dat[1] = state[3] ? 1'bz : dat_out_1[15];
    assign sd_dat[2] = state[3] ? 1'bz : dat_out_2[15];
    assign sd_dat[3] = state[3] ? 1'bz : dat_out_3[15];

    wire w_crc16_en  = (state == SEND_DAT) && (dat_cnt < 2*512) && !sd_clk;
    wire w_crc16_rst = i_rst | (state == SEND_CRC);

    wire [15:0] CRC16_0;
    sd_crc_16 dat_crc_0(
       .DAT(dat_out_0[15]),                            // Next input bit
       .EN(w_crc16_en),                            // Enable calculation
       .CLK(i_clk),                           // Current bit valid (Clock)
       .RST(w_crc16_rst),                             // Init CRC value
       .CRC(CRC16_0) // We need output registers
    );

    wire [15:0] CRC16_1;
    sd_crc_16 dat_crc_1(
       .DAT(dat_out_1[15]),                            // Next input bit
       .EN(w_crc16_en),                            // Enable calculation
       .CLK(i_clk),                           // Current bit valid (Clock)
       .RST(w_crc16_rst),                             // Init CRC value
       .CRC(CRC16_1) // We need output registers
    );

    wire [15:0] CRC16_2;
    sd_crc_16 dat_crc_2(
       .DAT(dat_out_2[15]),                            // Next input bit
       .EN(w_crc16_en),                            // Enable calculation
       .CLK(i_clk),                           // Current bit valid (Clock)
       .RST(w_crc16_rst),                             // Init CRC value
       .CRC(CRC16_2) // We need output registers
    );

    wire [15:0] CRC16_3;
    sd_crc_16 dat_crc_3(
       .DAT(dat_out_3[15]),                            // Next input bit
       .EN(w_crc16_en),                            // Enable calculation
       .CLK(i_clk),                           // Current bit valid (Clock)
       .RST(w_crc16_rst),                             // Init CRC value
       .CRC(CRC16_3) // We need output registers
    );

    localparam WRITE_DATA_SIZE = 513 * 2 - 1;

    always @ (posedge i_clk) begin
        if(i_rst) begin
            state <= IDLE;
            rsp_out <= 0;
            rsp_cnt <= 0;
            dat_cnt <= 0;
            blk_cnt <= 0;

            dat_out_0 <= 16'hFFFF;
            dat_out_1 <= 16'hFFFF;
            dat_out_2 <= 16'hFFFF;
            dat_out_3 <= 16'hFFFF;
        end
        else begin
                case(state)
                IDLE: begin
                case(i_funct)
                2'b00: begin

                end
                // READ
                2'b01: begin
                    state   <= WAIT_DAT;
                    blk_cnt <= i_blk_num;
                end
                // WRITE
                2'b10: begin
                    state   <= SEND_INI;//DAT;
                    blk_cnt <= i_blk_num;

                end
                // WAIT BUSY
                2'b11: begin
                    state   <= WAIT_BSY;
                    blk_cnt <= 0;
                end
                endcase
            end
            WAIT_DAT: begin
                if(sd_clk == 1 && sd_dat[0] == 0) begin
                    dat_cnt <= 512 * 2 + 16 - 1;
                    blk_cnt <= blk_cnt - 1;
                    state <= RECV_DAT;
                end
            end
            RECV_DAT: begin
                if(sd_clk == 1) begin
                    o_data <= {o_data[3:0], sd_dat[3:0]};
                    if(dat_cnt[0]==1'b0) begin
                        o_data_en <= (dat_cnt < 16) ? 0 : 1;
                        if(dat_cnt == 0) begin
                            state <= (blk_cnt == 0) ? IDLE : WAIT_DAT;
                        end
                        else begin
                            dat_cnt <= dat_cnt - 1;
                        end
                    end
                    else begin
                        o_data_en <= 0;
                        dat_cnt <= dat_cnt - 1;
                    end
                end
                else begin
                    o_data_en <= 0;
                end
            end
            SEND_INI: begin
                if(sd_clk == 1) begin
                    state   <= SEND_DAT;
                    dat_cnt <= WRITE_DATA_SIZE;
                    dat_out_0 <= 16'hBFFF;
                    dat_out_1 <= 16'hBFFF;
                    dat_out_2 <= 16'hBFFF;
                    dat_out_3 <= 16'hBFFF;
                end
            end
            SEND_DAT: begin
                if(sd_clk == 1) begin
                    if(dat_cnt[0] == 1'b0) begin
                        if(dat_cnt != 0) begin
                            dat_out_0 <= {8{i_data[4], i_data[0]}};
                            dat_out_1 <= {8{i_data[5], i_data[1]}};
                            dat_out_2 <= {8{i_data[6], i_data[2]}};
                            dat_out_3 <= {8{i_data[7], i_data[3]}};
                            o_data_ready <= 1;
                            dat_cnt <= dat_cnt - 1;
                        end
                        else begin
                            o_data_ready <= 0;
                            state <= SEND_CRC;
                            dat_cnt <= 16;
                            dat_out_0 <= CRC16_0;
                            dat_out_1 <= CRC16_1;
                            dat_out_2 <= CRC16_2;
                            dat_out_3 <= CRC16_3;
                        end
                    end
                    else begin
                        dat_out_0 <= {dat_out_0[14:0], 1'b1};
                        dat_out_1 <= {dat_out_1[14:0], 1'b1};
                        dat_out_2 <= {dat_out_2[14:0], 1'b1};
                        dat_out_3 <= {dat_out_3[14:0], 1'b1};
                        o_data_ready <= 0;
                        dat_cnt <= dat_cnt - 1;
                    end
                end
                else begin
                    o_data_ready <= 0;
                end
            end
            SEND_CRC: begin
                if(sd_clk) begin
                    if(dat_cnt == 0) begin
                        state <= WAIT_RSP;
                        blk_cnt <= blk_cnt - 1;
                    end
                    else begin
                        dat_out_0 <= {dat_out_0[14:0], 1'b1};
                        dat_out_1 <= {dat_out_1[14:0], 1'b1};
                        dat_out_2 <= {dat_out_2[14:0], 1'b1};
                        dat_out_3 <= {dat_out_3[14:0], 1'b1};
                        dat_cnt <= dat_cnt - 1;
                    end
                end
            end
            WAIT_RSP: begin
                if (sd_clk == 1) begin
                    if (sd_dat[0] == 0) begin
                        rsp_out <= 0;
                        state <= RECV_RSP;
                        rsp_cnt <= 47;
                    end
                end
            end
            RECV_RSP: begin
                if (sd_clk == 1) begin
                    rsp_out <= {rsp_out[46:0], sd_dat[0]};
                    if (rsp_cnt == 1) begin
                        state <= WAIT_BSY;
                    end
                    else begin
                        rsp_cnt <= rsp_cnt - 1;
                    end
                end
            end
            WAIT_BSY: begin
                if (sd_clk == 1) begin
                    if (sd_dat[0] == 1) begin
                        state <= (blk_cnt == 0) ? IDLE : SEND_INI;

                    end
                end
            end
            endcase
        end
    end
endmodule

module sd_cmd_transceiver(
    input  wire        i_clk,
    input  wire        i_rst,

    output wire        o_ready,

    input  wire        i_cmd_en,
    input  wire [ 5:0] i_cmd_no,
    input  wire [31:0] i_cmd_arg,

    output reg         o_rsp_en,
    output reg  [47:0] o_rsp_dat,
    output reg         o_rsp_bsy,

    input  wire        sd_clk,
    inout  wire        sd_cmd,

    output wire [ 2:0] o_state
    );

    localparam IDLE     = 3'b011;
    localparam SEND_CMD = 3'b001;
    localparam SEND_CRC = 3'b010;
    localparam EXEC_FIN = 3'b000;

    localparam WAIT_RSP = 3'b100;
    localparam RECV_RSP = 3'b101;

    reg  [ 2:0] state = IDLE;

    assign sd_cmd = state[2] ? 1'bz : cmd_out[47];

    reg  [ 7:0] cmd_cnt = 0;
    reg  [ 7:0] rsp_cnt = 0;
    reg  [47:0] cmd_out;

    assign o_ready = (state == IDLE);
    assign o_state = state;

    wire [47:0] w_cmd_dat;
    wire [ 7:0] w_rsp_bit;
    wire        w_rsp_bsy;
    cmd_rom cmd_rom0(
        .cmd_no(i_cmd_no),
        .arg(i_cmd_arg),

        .cmd_dat(w_cmd_dat),
        .rsp_bit(w_rsp_bit),
        .rsp_bsy(w_rsp_bsy)
    );

    wire w_crc7_en  = (state == SEND_CMD) && (cmd_cnt >= 8) && sd_clk;
    wire w_crc7_rst = i_rst | (state == SEND_CRC);
    wire w_crc7_dat = cmd_out[39];

    wire [ 6:0] CRC7;
    sd_crc_7 cmd_crc(
       .DAT(w_crc7_dat),                            // Next input bit
       .EN(w_crc7_en),                            // Enable calculation
       .CLK(i_clk),                           // Current bit valid (Clock)
       .RST(w_crc7_rst),                             // Init CRC value
       .CRC(CRC7) // We need output registers
    );

    always @ (posedge i_clk) begin
        if(i_rst) begin
            state <= IDLE;
            cmd_cnt <= 0;
            rsp_cnt <= 0;
            cmd_out <= {48{1'b1}};
            o_rsp_en  <= 0;
            o_rsp_dat <= 0;
            o_rsp_bsy <= 0;
        end
        else begin
            case(state)
            IDLE: begin
                if(i_cmd_en) begin
                    state <= SEND_CMD;
                    cmd_out <= w_cmd_dat;
                    cmd_cnt <= 47;
                    rsp_cnt <= w_rsp_bit;
                    o_rsp_bsy <= w_rsp_bsy;
                    o_rsp_en  <= 0;
                end
            end
            SEND_CMD: begin
                if (sd_clk == 1) begin
                    if(cmd_cnt == 0) begin
                        state <= SEND_CRC;
                        cmd_cnt <= 7;
                        cmd_out <= {CRC7, {41{1'b1}}};
                    end
                    else begin
                        cmd_cnt <= cmd_cnt - 1;
                        cmd_out <= {cmd_out[46:0], 1'b1};
                    end
                end
            end
            SEND_CRC: begin
                if (sd_clk == 1) begin
                    if(cmd_cnt == 0) begin
                        state <= (rsp_cnt == 0) ? EXEC_FIN : WAIT_RSP;
                        o_rsp_en <= (rsp_cnt == 0) ? 1 : 0;
                    end
                    else begin
                        cmd_cnt <= cmd_cnt - 1;
                        cmd_out <= {cmd_out[46:0], 1'b1};
                    end
                end
            end
            WAIT_RSP: begin
                if (sd_clk == 1) begin
                    if (sd_cmd == 0) begin
                        o_rsp_dat <= 0;
                        state <= RECV_RSP;
                    end
                end
            end
            RECV_RSP: begin
                if (sd_clk == 1) begin
                    o_rsp_dat <= {o_rsp_dat[46:0], sd_cmd};
                    if (rsp_cnt == 1) begin
                        state <= EXEC_FIN;//IDLE;
                        o_rsp_en <= 1;
                    end
                    else begin
                        rsp_cnt <= rsp_cnt - 1;
                    end
                end
            end
            EXEC_FIN: begin
                o_rsp_en <= 0;
                o_rsp_bsy <= 0;
                state <= IDLE;
            end
            endcase
        end
    end
endmodule

module cmd_rom(
    input  wire [ 5:0] cmd_no,
    input  wire [31:0] arg,

    output reg  [47:0] cmd_dat,
    output reg  [ 7:0] rsp_bit,
    output reg         rsp_bsy
    );

    always @ ( * ) begin
        case(cmd_no)
        6'd00  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd00, 32'h00_00_00_00}, 8'd000, 1'b0};
        6'd02  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd02, 32'h00_00_00_00}, 8'd135, 1'b0};
        6'd03  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd03, 32'h00_00_00_00}, 8'd047, 1'b0};
        6'd06  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd06, 32'h00_00_00_02}, 8'd047, 1'b0};
        6'd07  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd07, arg            }, 8'd047, 1'b1};
        6'd08  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd08, 32'h00_00_01_AA}, 8'd047, 1'b0};
        6'd12  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd12, 32'h00_00_00_00}, 8'd047, 1'b0};
        6'd17  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd17, arg            }, 8'd047, 1'b0};
        6'd18  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd18, arg            }, 8'd047, 1'b0};
        6'd24  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd24, arg            }, 8'd047, 1'b0};
        6'd25  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd25, arg            }, 8'd047, 1'b0};
        6'd41  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd41, 32'h50_FF_80_00}, 8'd047, 1'b0};
        6'd55  : {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b01, 6'd55, arg            }, 8'd047, 1'b0};
        default: {cmd_dat, rsp_bit, rsp_bsy} = {{8'hFF, 2'b11, 6'd63, 32'hFF_FF_FF_FF}, 8'd000, 1'b0};
        endcase
    end

endmodule

module sd_controller_spi(
    input  wire        i_clk,
    input  wire        i_rst,

    output wire        o_ready,

    input  wire        i_ren,
    output wire [ 7:0] o_data,
    output wire        o_data_en,

    input  wire        i_wen,
    input  wire [ 7:0] i_data,
    output wire        o_data_ready,

    input  wire [31:0] i_blk_num,
    input  wire [31:0] i_adr,

    output wire [ 4:0] o_state,

    input  wire        sd_cd,
    output wire        sd_rst,
    output reg         sd_clk,
    inout  wire        sd_cmd,
    inout  wire [ 3:0] sd_dat
);

    // for init
    localparam INIT = 0;
    localparam IDLE = 1;

    localparam SEND_CMD = 2;
    localparam WAIT_RSP = 3;
    localparam RECV_RSP = 4;
    localparam WAIT_BSY = 5;
    localparam SEND_TKN = 24;
    localparam SEND_CRC7 = 25;

    localparam WAIT_RECV_BLK = 6;
    localparam MAIN_RECV_BLK = 7;

    localparam INIT_SEND_BLK = 8;
    localparam MAIN_SEND_BLK = 9;
    localparam WAIT_SEND_BLK = 10;

    localparam CMD00 = 16;
    localparam CMD08 = 17;
    localparam CMD55 = 18;
    localparam CMD41 = 19;
    localparam CHECK_HCS = 20;
    localparam CMD58 = 21;
    localparam CHECK_CCS = 22;
    localparam CMD12 = 23;

    localparam WRITE_DATA_SIZE = 515;

    reg [ 4:0] state = INIT;
    reg [ 4:0] return_state;
    reg [47:0] cmd_out;
    reg [39:0] recv_data;
    reg        cmd_mode = 1;
    reg [ 7:0] data_sig = 8'hFF;
    reg [ 9:0] byte_counter;
    reg [13:0] send_bit_counter;
    reg [13:0] recv_bit_counter;
    reg [13:0] cmd_bit_counter;
    reg [31:0] block_counter;
    reg [26:0] boot_counter = 27'd100_000_000;
    reg [ 7:0] dout = 0;
    reg        byte_available = 0;
    reg        ready_for_next_byte = 0;
    reg        cs = 1;

    assign o_ready = (state == IDLE);
    assign o_data  = dout;
    assign o_data_en = byte_available;
    assign o_data_ready = ready_for_next_byte;
    assign o_state = state;

    assign mosi = cmd_mode ? cmd_out[47] : data_sig[7];

    wire miso = sd_dat[0];
    assign sd_cmd    = mosi;
    assign sd_rst    = 0;
    assign sd_dat[1] = 1;
    assign sd_dat[2] = 1;
    assign sd_dat[3] = cs;

    function [47:0] cmd_rom(input [ 5:0] cmd_no, input [31:0] arg);
    begin
        case(cmd_no)
        6'd00: cmd_rom = {8'hFF, 2'b01, 6'd00, 32'h00_00_00_00};//_95;
        6'd08: cmd_rom = {8'hFF, 2'b01, 6'd08, 32'h00_00_01_AA};//48'hFF_48_00_00_01_AA;//_87;
        6'd12: cmd_rom = {8'hFF, 2'b01, 6'd12, 32'h00_00_00_00};//56'hFF_4C_00_00_00_00;//_FF;
        6'd17: cmd_rom = {8'hFF, 2'b01, 6'd17, arg};//{16'hFF_51, arg};//,    8'hFF};
        6'd18: cmd_rom = {8'hFF, 2'b01, 6'd18, arg};//{16'hFF_52, arg};//,    8'hFF};
        6'd24: cmd_rom = {8'hFF, 2'b01, 6'd24, arg};//{16'hFF_58, arg};//,    8'hFF};
        6'd25: cmd_rom = {8'hFF, 2'b01, 6'd25, arg};//{16'hFF_59, arg};//,    8'hFF};
        6'd41: cmd_rom = {8'hFF, 2'b01, 6'd41, 32'h50_FF_80_00};//56'hFF_69_40_FF_80_00;//_17;
        6'd55: cmd_rom = {8'hFF, 2'b01, 6'd55, 32'h00_00_00_00};//56'hFF_77_00_00_00_00;//_65;
        6'd58: cmd_rom = {8'hFF, 2'b01, 6'd58, 32'h00_00_00_00};//56'hFF_7A_00_00_00_00;//_FD;
        default: cmd_rom = 48'hFF_FF_FF_FF_FF_FF;//_FF;
        endcase
    end
    endfunction

    wire w_crc7_en  = (state == SEND_CMD) && (cmd_bit_counter <= 39) && sclk;
    wire w_crc7_rst = i_rst | (state == SEND_CRC7);
    wire w_crc7_dat = cmd_out[39];

    wire [ 6:0] CRC7;
    sd_crc_7 cmd_crc(
       .DAT(w_crc7_dat),                            // Next input bit
       .EN(w_crc7_en),                            // Enable calculation
       .CLK(i_clk),                           // Current bit valid (Clock)
       .RST(w_crc7_rst),                             // Init CRC value
       .CRC(CRC7) // We need output registers
    );

    always @(posedge i_clk) begin
        sd_clk <= ~sd_clk;
        if(i_rst == 1) begin
            state <= INIT;
            boot_counter <= 27'd10_000_000; //  >= 1ms
            cmd_out <= {48{1'b1}};
            byte_counter <= 0;
            byte_available <= 0;
            ready_for_next_byte <= 0;
            cmd_mode <= 1;
            send_bit_counter <= 160;
            block_counter <= 0;
            cs <= 1;
            state <= INIT;
            dout <= 0;
            byte_available <= 0;
            ready_for_next_byte <= 0;
            cmd_bit_counter <= 0;
        end
        else begin
            case(state)
                INIT: begin
                    if(boot_counter == 0) begin
                        if(send_bit_counter == 0) begin
                            cs <= 0;
                            state <= CMD00;
                        end
                        else begin
                            send_bit_counter <= send_bit_counter - 1;
                        end
                    end
                    else begin
                        boot_counter <= boot_counter - 1;
                    end
                end
                CMD00: begin
                    cmd_out <= cmd_rom(6'd00, 0);
                    recv_bit_counter <= 6;
                    return_state <= CMD08;
                    state <= SEND_CMD;
                end
                CMD08: begin
                    cmd_out <= cmd_rom(6'd08, 0);
                    recv_bit_counter <= 38;
                    return_state <= CMD55;
                    state <= SEND_CMD;
                end
                CMD55: begin
                    cmd_out <= cmd_rom(6'd55, 0);
                    recv_bit_counter <= 6;
                    return_state <= CMD41;
                    state <= SEND_CMD;
                end
                CMD41: begin
                    cmd_out <= cmd_rom(6'd41, 0);
                    recv_bit_counter <= 6;
                    return_state <= CHECK_HCS;
                    state <= SEND_CMD;
                end
                CHECK_HCS: begin
                    if(recv_data[0] == 0) begin
                        state <= CMD58;
                    end
                    else begin
                        state <= CMD55;
                    end
                end
                CMD58: begin
                    cmd_out <= cmd_rom(6'd58, 0);
                    recv_bit_counter <= 38;
                    return_state <= CHECK_CCS;
                    state <= SEND_CMD;
                end
                CHECK_CCS: begin
                    if(recv_data[30] == 1) begin
                        state <= IDLE;
                    end
                    else begin
                        state <= CMD58;
                    end
                end
                IDLE: begin
                    if(i_ren == 1) begin
                        block_counter <= i_blk_num;
                        cmd_out <= (i_blk_num==0) ? cmd_rom(6'd17, i_adr) : cmd_rom(6'd18, i_adr);//{16'hFF_51, i_adr, 8'hFF} : {16'hFF_52, i_adr, 8'hFF};
                        recv_bit_counter <= 6;
                        return_state <= WAIT_RECV_BLK;
                        state <= SEND_CMD;
                    end
                    else if(i_wen == 1) begin
                        block_counter <= i_blk_num;
                        cmd_out <= (i_blk_num==0) ? cmd_rom(6'd24, i_adr) : cmd_rom(6'd25, i_adr);//{16'hFF_58, i_adr, 8'hFF} : {16'hFF_59, i_adr, 8'hFF};
                        recv_bit_counter <= 6;
                        return_state <= INIT_SEND_BLK;
                        state <= SEND_CMD;
                    end
                    else begin
                        state <= IDLE;
                    end
                end
                SEND_CMD: begin
                    if (sd_clk == 1) begin
                        if(cmd_bit_counter == 47) begin
                            state <= SEND_CRC7;
                            cmd_bit_counter <= 0;
                            cmd_out <= {CRC7, {41{1'b1}}};
                        end
                        else begin
                            cmd_bit_counter <= cmd_bit_counter + 1;
                            cmd_out <= {cmd_out[46:0], 1'b1};
                        end
                    end
                end
                SEND_CRC7: begin
                    if (sd_clk == 1) begin
                        if(cmd_bit_counter == 7) begin
                            state <= WAIT_RSP;
                            cmd_bit_counter <= 0;
                        end
                        else begin
                            cmd_bit_counter <= cmd_bit_counter + 1;
                            cmd_out <= {cmd_out[46:0], 1'b1};
                        end
                    end
                end
                WAIT_RSP: begin
                    if (sd_clk == 1) begin
                        if (miso == 0) begin
                            recv_data <= 0;
                            state <= RECV_RSP;
                        end
                    end
                end
                RECV_RSP: begin
                    byte_available <= 0;
                    if (sd_clk == 1) begin
                        recv_data <= {recv_data[38:0], miso};
                        if (recv_bit_counter == 0) begin
                            state <= return_state;
                        end
                        else begin
                            recv_bit_counter <= recv_bit_counter - 1;
                        end
                    end
                end
                WAIT_RECV_BLK: begin
                    if(sd_clk == 1 && miso == 0) begin
                        recv_bit_counter <= ((512 + 2) * 8)-1;
                        block_counter <= block_counter - 1;
                        return_state <= (block_counter == 0) ? IDLE : (block_counter == 1) ? CMD12 : WAIT_RECV_BLK;
                        state <= MAIN_RECV_BLK;
                    end
                end
                MAIN_RECV_BLK: begin
                    if(sd_clk == 1) begin
                        recv_data <= {recv_data[6:0], miso};
                        if(recv_bit_counter[2:0]==3'b000) begin
                            dout <= {recv_data[6:0], miso};
                            byte_available <= (recv_bit_counter < 16) ? 0 : 1;
                            if(recv_bit_counter == 0) begin
                                state <= return_state;
                            end
                            else begin
                                recv_bit_counter <= recv_bit_counter - 1;
                            end
                        end
                        else begin
                            byte_available <= 0;
                            recv_bit_counter <= recv_bit_counter - 1;
                        end
                    end
                    else begin
                        byte_available <= 0;
                    end
                end
                CMD12: begin
                    cmd_out <= cmd_rom(6'd12, 0);
                    recv_bit_counter <= 6;
                    return_state <= WAIT_BSY;
                    state <= SEND_CMD;
                end
                INIT_SEND_BLK: begin
                    cmd_mode <= 0;
                    ready_for_next_byte <= 0;
                    state <= MAIN_SEND_BLK;
                    data_sig <= (block_counter == 0) ? 8'hFE : 8'hFC;
                    send_bit_counter <= WRITE_DATA_SIZE * 8 - 1;
                end
                MAIN_SEND_BLK: begin
                    if(sd_clk == 1) begin
                        if(send_bit_counter[2:0] == 3'b000) begin
                            if(send_bit_counter == 0) begin
                                state <= RECV_RSP;
                                recv_bit_counter <= 7;
                                return_state <= WAIT_SEND_BLK;
                            end
                            else if(send_bit_counter <= 16) begin
                                data_sig <= 8'hFF;
                            end
                            else begin
                                data_sig <= i_data;
                                ready_for_next_byte <= 1;
                            end
                        end
                        else begin
                            data_sig <= {data_sig[6:0], 1'b1};
                            ready_for_next_byte <= 0;
                        end
                        send_bit_counter <= send_bit_counter - 1;
                    end
                    else begin
                        ready_for_next_byte <= 0;
                    end
                end
                WAIT_SEND_BLK: begin
                    if (sd_clk == 1) begin
                        if (miso == 1) begin
                            data_sig <= 8'hFD;
                            send_bit_counter <= 7;
                            recv_bit_counter <= 6;
                            state <= (block_counter == 0) ? IDLE : (block_counter == 1) ? SEND_TKN : INIT_SEND_BLK;
                            return_state <= WAIT_BSY;
                            block_counter <= block_counter - 1;
                            cmd_mode <= (block_counter == 1) ? 0 : 1;
                        end
                    end
                end
                SEND_TKN: begin
                    if (sd_clk == 1) begin
                        if (send_bit_counter == 0) begin
                            state <= WAIT_RSP;
                        end
                        else begin
                            send_bit_counter <= send_bit_counter - 1;
                            data_sig <= {data_sig[6:0], 1'b1};
                        end
                    end
                end
                WAIT_BSY: begin
                    if (sd_clk == 1) begin
                        if (miso == 1) begin
                            state <= IDLE;
                            cmd_mode <= 1;
                        end
                    end
                end
            endcase
        end
    end
endmodule


module sd_crc_7(
   input  wire       DAT,                            // Next input bit
   input  wire       EN,                            // Enable calculation
   input  wire       CLK,                           // Current bit valid (Clock)
   input  wire       RST,                             // Init CRC value
   output reg  [6:0] CRC // We need output registers
   );                               // Current output CRC value
   wire         inv;

   assign inv = DAT ^ CRC[6];                   // XOR required?

   always @(posedge CLK) begin
      if (RST) begin
         CRC <= 0;                                  // Init before calculation
         end
      else begin
         if (EN) begin
             CRC[6] <= CRC[5];
             CRC[5] <= CRC[4];
             CRC[4] <= CRC[3];
             CRC[3] <= CRC[2] ^ inv;
             CRC[2] <= CRC[1];
             CRC[1] <= CRC[0];
             CRC[0] <= inv;
             end
         end
      end

endmodule

module sd_crc_16(
    input  wire        DAT,                            // Next input bit
    input  wire        EN,                            // Enable calculation
    input  wire        CLK,                           // Current bit valid (Clock)
    input  wire        RST,                             // Init CRC value
    output reg  [15:0] CRC // We need output registers
);                                                   // Current output CRC value
                        // We need output registers
   wire         inv;

   assign inv = DAT ^ CRC[15];                   // XOR required?

   always @(posedge CLK) begin
      if (RST) begin
         CRC <= 0;                                  // Init before calculation
         end
      else begin
         if (EN) begin
             CRC[15] <= CRC[14];
             CRC[14] <= CRC[13];
             CRC[13] <= CRC[12];
             CRC[12] <= CRC[11] ^ inv;
             CRC[11] <= CRC[10];
             CRC[10] <= CRC[9];
             CRC[9] <= CRC[8];
             CRC[8] <= CRC[7];
             CRC[7] <= CRC[6];
             CRC[6] <= CRC[5];
             CRC[5] <= CRC[4] ^ inv;
             CRC[4] <= CRC[3];
             CRC[3] <= CRC[2];
             CRC[2] <= CRC[1];
             CRC[1] <= CRC[0];
             CRC[0] <= inv;
             end
         end
      end

endmodule
