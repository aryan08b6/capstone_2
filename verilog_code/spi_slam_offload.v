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
    // Block 2: SLAM Computation Modules with Pipeline Architecture
    // -----------------------------------------------------------------

    // Pipeline state machine states
    localparam STATE_IDLE    = 3'b000;
    localparam STATE_RECEIVE = 3'b001;
    localparam STATE_FAST    = 3'b010;
    localparam STATE_ORB     = 3'b011;
    localparam STATE_SAD     = 3'b100;
    localparam STATE_OUTPUT  = 3'b101;

    // Pipeline mode selection (from SPI command)
    localparam PIPELINE_FULL = 3'b101;  // Gaussian → FAST → ORB → SAD

    reg [2:0] proc_state = 0;
    reg [2:0] pipeline_mode = 0;

    // Row buffers for Gaussian filter context
    reg [63:0] gaussian_prev_row = 0;
    reg [63:0] gaussian_curr_row = 0;
    reg [63:0] gaussian_next_row = 0;

    // Intermediate pipeline results
    reg [63:0] gaussian_out_buf = 0;
    reg [7:0]  fast_flags_buf = 0;
    reg [7:0]  fast_strength_buf = 0;
    reg [63:0] orb_descriptor_buf = 0;
    reg [31:0] sad_value_buf = 0;
    reg [7:0]  disparity_buf = 0;

    // Storage for FAST corner detection (for non-pipeline mode compatibility)
    reg [63:0] prev_row_data = 0;
    reg [63:0] next_row_data = 0;

    // Instantiate Gaussian Filter
    wire [63:0] gaussian_filtered;
    gaussian_filter gauss_inst (
        .clk(clk),
        .current_row(gaussian_curr_row),
        .prev_row(gaussian_prev_row),
        .next_row(gaussian_next_row),
        .filtered_data(gaussian_filtered)
    );

    // Instantiate FAST corner detector
    wire [7:0] fast_flags;
    wire [7:0] fast_strength;
    fast_corner_detector fast_inst (
        .clk(clk),
        .pixel_data(gaussian_out_buf),  // Use filtered data for pipeline mode
        .prev_row(gaussian_prev_row),
        .next_row(gaussian_next_row),
        .corner_flags(fast_flags),
        .corner_strength(fast_strength)
    );

    // Instantiate ORB Descriptor
    wire [63:0] orb_descriptor_result;
    wire orb_descriptor_valid;
    orb_descriptor orb_inst (
        .clk(clk),
        .patch_data(gaussian_out_buf),
        .patch_data2(gaussian_out_buf),
        .descriptor(orb_descriptor_result),
        .descriptor_valid(orb_descriptor_valid)
    );

    // Instantiate SAD Block Matcher
    wire [31:0] sad_result;
    wire [7:0] best_disparity_result;
    sad_block_matcher sad_inst (
        .clk(clk),
        .left_block(orb_descriptor_buf),
        .right_block(gaussian_out_buf),
        .disparity_offset(8'd0),
        .sad_value(sad_result),
        .best_disparity(best_disparity_result)
    );

    // -----------------------------------------------------------------
    // Block 2B: Pipeline FSM - Gaussian → FAST → ORB → SAD
    // -----------------------------------------------------------------

    always @(posedge clk) begin
        in_fifo_rd_en  <= 0;
        out_fifo_wr_en <= 0;

        case (proc_state)
            STATE_IDLE: begin
                if (!in_fifo_empty && !out_fifo_full) begin
                    // Check mode from first byte of input
                    if ((in_fifo_q[7:5] == 3'b100) || (in_fifo_q[7:5] == 3'b101)) begin
                        // Pipeline mode (0x80-0x9F command range)
                        pipeline_mode <= in_fifo_q[2:0];
                        in_fifo_rd_en <= 1;
                        proc_state <= STATE_RECEIVE;
                    end else begin
                        // Fallback to existing FAST-only mode
                        in_fifo_rd_en <= 1;
                        proc_state <= STATE_RECEIVE;
                    end
                end
            end

            STATE_RECEIVE: begin
                // Shift row buffers for Gaussian context
                gaussian_prev_row <= gaussian_curr_row;
                gaussian_curr_row <= gaussian_next_row;
                gaussian_next_row <= in_fifo_q;

                // Apply Gaussian filter immediately
                gaussian_out_buf <= gaussian_filtered;

                proc_state <= STATE_FAST;
            end

            STATE_FAST: begin
                // FAST detection on buffered rows
                fast_flags_buf <= fast_flags;
                fast_strength_buf <= fast_strength;
                proc_state <= STATE_ORB;
            end

            STATE_ORB: begin
                // ORB descriptor extraction (run if corners detected)
                if (fast_flags_buf != 8'b0) begin
                    orb_descriptor_buf <= orb_descriptor_result;
                end else begin
                    orb_descriptor_buf <= 64'b0;
                end
                proc_state <= STATE_SAD;
            end

            STATE_SAD: begin
                // SAD block matching
                sad_value_buf <= sad_result;
                disparity_buf <= best_disparity_result;
                proc_state <= STATE_OUTPUT;
            end

            STATE_OUTPUT: begin
                // Format output: [disparity:8 | sad_hi:8 | sad_lo:8 | strength:8 | flags:8 | reserved:24]
                out_fifo_data <= {disparity_buf, sad_value_buf[15:8],
                                 sad_value_buf[7:0], fast_strength_buf, fast_flags_buf};
                out_fifo_wr_en <= 1;
                proc_state <= STATE_IDLE;
            end

            default: proc_state <= STATE_IDLE;
        endcase
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
