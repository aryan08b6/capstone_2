module spi_slave_slam_offload (
    input wire clk,

    input wire sclk,
    input wire mosi,
    input wire cs_n,
    output reg miso
);

    // -----------------------------------------------------------------
    // Multi-stage Synchronizers
    // -----------------------------------------------------------------
    reg [2:0] sclk_sync = 3'b0;
    reg [1:0] cs_n_sync = 2'b11;
    reg [1:0] mosi_sync = 2'b0;

    always @(posedge clk) begin
        sclk_sync <= {sclk_sync[1:0], sclk};
        cs_n_sync <= {cs_n_sync[0], cs_n};
        mosi_sync <= {mosi_sync[0], mosi};
    end

    wire sclk_rising  = (sclk_sync[1] && !sclk_sync[2]);
    wire sclk_falling = (!sclk_sync[1] && sclk_sync[2]);
    wire secure_cs_n  = cs_n_sync[1];
    wire secure_mosi  = mosi_sync[1];

    // -----------------------------------------------------------------
    // FIFO Signals & Instantiations
    // -----------------------------------------------------------------
    reg         in_fifo_wr_en = 0;
    reg         in_fifo_rd_en = 0;
    wire [63:0] in_fifo_q;
    wire        in_fifo_empty;
    wire        in_fifo_full;

    reg  [63:0] out_fifo_data = 0;
    reg         out_fifo_wr_en = 0;
    reg         out_fifo_rd_en = 0;
    wire [63:0] out_fifo_q;
    wire        out_fifo_empty;
    wire        out_fifo_full;

    INPUT_FIFO in_fifo_inst(
        .Data(rx_shift_64),
        .WrClk(clk),
        .RdClk(clk),
        .WrEn(in_fifo_wr_en),
        .RdEn(in_fifo_rd_en),
        .Almost_Empty(),
        .Almost_Full(),
        .Q(in_fifo_q),
        .Empty(in_fifo_empty),
        .Full(in_fifo_full)
    );

    OUTPUT_FIFO out_fifo_inst(
        .Data(out_fifo_data),
        .WrClk(clk),
        .RdClk(clk),
        .WrEn(out_fifo_wr_en),
        .RdEn(out_fifo_rd_en),
        .Almost_Empty(),
        .Almost_Full(),
        .Q(out_fifo_q),
        .Empty(out_fifo_empty),
        .Full(out_fifo_full)
    );

    // -----------------------------------------------------------------
    // Block 1: SPI RX (Deserializer)
    // -----------------------------------------------------------------
    reg [63:0] rx_shift_64   = 64'd0;
    reg [5:0]  rx_bit_cnt    = 6'd0;
    reg        rx_done_pulse = 1'b0;

    always @(posedge clk) begin
        rx_done_pulse <= 0;
        in_fifo_wr_en <= 0;

        if (secure_cs_n) begin
            rx_bit_cnt <= 0;
        end else begin
            if (sclk_rising) begin
                rx_shift_64 <= {rx_shift_64[62:0], secure_mosi};
                rx_bit_cnt  <= rx_bit_cnt + 1;

                if (rx_bit_cnt == 6'd63) begin
                    rx_done_pulse <= 1;
                end
            end
        end

        if (rx_done_pulse) begin
            in_fifo_wr_en <= 1;
        end
    end

    // -----------------------------------------------------------------
    // Block 2: SLAM Computation Modules
    // -----------------------------------------------------------------

    // Storage for FAST corner detection
    reg [63:0] prev_row_data = 0;
    reg [63:0] next_row_data = 0;

    // Instantiate FAST corner detector
    wire [7:0] fast_flags;
    wire [7:0] fast_strength;
    fast_corner_detector fast_inst (
        .clk(clk),
        .pixel_data(in_fifo_q),
        .prev_row(prev_row_data),
        .next_row(next_row_data),
        .corner_flags(fast_flags),
        .corner_strength(fast_strength)
    );

    // -----------------------------------------------------------------
    // Block 2B: Processor FSM - FAST Corner Detection Only
    // -----------------------------------------------------------------
    reg [2:0] proc_state = 0;

    always @(posedge clk) begin
        in_fifo_rd_en  <= 0;
        out_fifo_wr_en <= 0;

        case (proc_state)
            0: begin // Idle - wait for input data
                if (!in_fifo_empty && !out_fifo_full) begin
                    in_fifo_rd_en <= 1;
                    proc_state    <= 1;
                end
            end
            1: begin // Data fetched, prepare for computation
                proc_state <= 2;
            end
            2: begin // Execute FAST corner detection
                out_fifo_data <= {fast_strength, fast_flags, 48'b0};
                out_fifo_wr_en <= 1;
                proc_state <= 0;
            end
        endcase

        // Store current and neighboring rows for FAST corner detection
        if (proc_state == 0) begin
            prev_row_data <= in_fifo_q;
        end
        if (proc_state == 1) begin
            next_row_data <= in_fifo_q;
        end
    end

    // -----------------------------------------------------------------
    // Block 3: SPI TX (Serializer)
    // -----------------------------------------------------------------
    reg [63:0] tx_shift_64 = 64'd0;
    reg [1:0]  tx_state    = 0;
    reg        tx_ready    = 0;

    initial begin
        miso = 1'b0;
    end

    always @(posedge clk) begin
        out_fifo_rd_en <= 0;

        if (!secure_cs_n) begin
            // Transaction active: shift data out on falling edges
            tx_state <= 0;
            tx_ready <= 0;

            if (sclk_falling) begin
                tx_shift_64 <= {tx_shift_64[62:0], 1'b0};
                miso        <= tx_shift_64[62];
            end
        end
        else begin
            // Bus Idle: Pre-fetch the next 64 bits from OUTPUT_FIFO
            case (tx_state)
                0: begin
                    if (!tx_ready) begin
                        if (!out_fifo_empty) begin
                            out_fifo_rd_en <= 1;
                            tx_state       <= 1;
                        end else begin
                            tx_shift_64 <= 64'h0;
                            miso        <= 1'b0;
                        end
                    end
                end
                1: begin
                    tx_state <= 2;
                end
                2: begin
                    tx_shift_64 <= out_fifo_q;
                    miso        <= out_fifo_q[63];
                    tx_ready    <= 1;
                    tx_state    <= 3;
                end
                3: begin
                    // Hold until CS goes low
                end
            endcase
        end
    end

endmodule
