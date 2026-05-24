module spi_slave_string (
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

    // Both FIFOs run on the main 27MHz system clock
    INPUT_FIFO in_fifo_inst(
        .Data(rx_shift_64),     // Fed directly from RX shift register
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
        .Data(out_fifo_data),   // Fed from processor block
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
    reg [5:0]  rx_bit_cnt    = 6'd0; // Counts 0 to 63
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
                
                // When 64 bits are shifted in, trigger a write on the next cycle
                if (rx_bit_cnt == 6'd63) begin
                    rx_done_pulse <= 1;
                end
            end
        end

        // Wait 1 clock cycle for rx_shift_64 to fully update before writing
        if (rx_done_pulse) begin
            in_fifo_wr_en <= 1;
        end
    end

    // -----------------------------------------------------------------
    // Block 2: Data Processor (Inverts Bits)
    // -----------------------------------------------------------------
    reg [1:0] proc_state = 0;

    always @(posedge clk) begin
        in_fifo_rd_en  <= 0;
        out_fifo_wr_en <= 0;

        case (proc_state)
            0: begin
                // If data is available and output isn't blocked
                if (!in_fifo_empty && !out_fifo_full) begin
                    in_fifo_rd_en <= 1;
                    proc_state    <= 1;
                end
            end
            1: begin
                // Standard IP FIFOs take 1 cycle to present data after rd_en
                proc_state <= 2;
            end
            2: begin
                // Data is ready on in_fifo_q. Invert it and push to Output FIFO.
                out_fifo_data  <= ~in_fifo_q;
                out_fifo_wr_en <= 1;
                proc_state     <= 0;
            end
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
            // Bus Idle: Pre-fetch the next 64 bits from the OUTPUT_FIFO
            case (tx_state)
                0: begin
                    if (!tx_ready) begin
                        if (!out_fifo_empty) begin
                            out_fifo_rd_en <= 1;
                            tx_state       <= 1;
                        end else begin
                            // Nothing to send, pre-load 0s
                            tx_shift_64 <= 64'h0;
                            miso        <= 1'b0;
                        end
                    end
                end
                1: begin
                    // Wait for FIFO latency
                    tx_state <= 2;
                end
                2: begin
                    // Load the fetched data into the shift register
                    tx_shift_64 <= out_fifo_q;
                    miso        <= out_fifo_q[63]; // Pre-drive MSB
                    tx_ready    <= 1;
                    tx_state    <= 3;
                end
                3: begin
                    // Hold state until CS goes low
                end
            endcase
        end
    end

endmodule