module test_sdcram#(
    parameter TEST_SIZE = 2**27
    )(
    input  wire        CLK,
    input  wire        RST,

    input  wire        sd_cd,
    output wire        sd_rst,
    output wire        sd_clk,
    inout  wire        sd_cmd,
    inout  wire [ 3:0] sd_dat,

    input  wire        test_ren,
    input  wire        test_wen,
    output wire [31:0] test_dout,
    output wire [63:0] test_ccnt
    );

    wire clk_sd = CLK;

////////////////////////////////////////////////////////////////////////////////
    wire [40:0] sdcram_addr;
    wire        sdcram_ren;
    wire [ 3:0] sdcram_wen;
    wire [31:0] sdcram_wdata;
    wire [31:0] sdcram_rdata;
    wire        sdcram_busy;
    wire [ 8:0] sdcram_state;
    wire [ 2:0] sdi_state;
    wire [ 4:0] sdc_state;
    wire [ 2:0] ct_state;
    wire [ 3:0] cache_wen;
    wire        cache_en;

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
    reg  [40:0] r_sd_addr = 0;
    reg  [31:0] r_sd_data = 0;
    reg         r_sd_ren = 0;
    reg         r_sd_wen = 0;
    reg  [63:0] r_ccnt = 0;
    reg         r_start = 0;
    reg         r_rtest = 0;
    reg         r_wtest = 0;

    assign sdcram_addr  = r_sd_addr;
    assign sdcram_ren   = r_sd_ren;
    assign sdcram_wen   = {4{r_sd_wen}};
    assign sdcram_wdata = r_sd_addr[31:0];

    assign test_dout = r_sd_data;
    assign test_ccnt = r_ccnt;

    wire w_req = test_ren | test_wen;
    wire w_fin = (r_sd_addr == TEST_SIZE);

    localparam  INIT = 0;
    localparam  IDLE = 1;
    localparam  SEND = 2;
    localparam  WAIT = 3;
    reg [1:0] r_state = INIT;

    always @ (posedge CLK) begin
        r_ccnt <= (RST | !r_start) ? 0 : (w_fin) ? r_ccnt : r_ccnt + 1;
        r_rtest<= (RST) ? 0 : (w_fin) ? 0 : (r_rtest) ? 1 : test_ren;
        r_wtest<= (RST) ? 0 : (w_fin) ? 0 : (r_wtest) ? 1 : test_wen;
        if(RST) begin
            r_state <= INIT;
        end
        else begin
        case(r_state)
        INIT: begin
            r_state  <= IDLE;
            r_sd_ren <= 0;
            r_sd_wen <= 0;
            r_sd_addr<= 0;
            r_sd_data<= 0;
            r_start  <= 0;
        end
        IDLE: begin
            /*
            if(!sdcram_busy & w_req) begin
                r_state  <= SEND;
                r_sd_ren <= w_r_req;
                r_sd_wen <= w_w_req;
            end
            */
            if(!sdcram_busy & !w_fin) begin
                if(r_rtest) begin
                    r_state <= SEND;
                    r_start <= 1;
                    r_sd_ren<= 1;
                end
                else if(r_wtest) begin
                    r_state <= SEND;
                    r_start <= 1;
                    r_sd_wen<= 1;
                end
            end

        end
        SEND: begin
            r_state <= WAIT;
        end
        WAIT: begin
            if(!sdcram_busy) begin
                r_state  <= IDLE;
                r_sd_ren <= 0;
                r_sd_wen <= 0;
                r_sd_addr<= r_sd_addr + 4;
                r_sd_data <= sdcram_rdata;
            end
        end
        endcase
        end
    end

////////////////////////////////////////////////////////////////////////////////

endmodule
