// FAST Corner Detector Module
// Detects corners using the FAST algorithm on 3x3 pixel neighborhoods
// Processes 8 bytes (64-bit) at a time

module fast_corner_detector (
    input wire clk,
    input wire [63:0] pixel_data,      // 8 pixels (8 bits each)
    input wire [63:0] prev_row,        // Previous row pixels
    input wire [63:0] next_row,        // Next row pixels
    output reg [7:0] corner_flags,     // Bit i = 1 if pixel i is a corner
    output reg [7:0] corner_strength   // Intensity of corner detection
);

    reg [7:0] pixels [0:7];
    reg [7:0] prev_pixels [0:7];
    reg [7:0] next_pixels [0:7];

    integer i, j;
    reg [7:0] center, neighbor;
    reg [3:0] bright_count, dark_count;
    parameter THRESHOLD = 30;

    always @(posedge clk) begin
        // Unpack 64-bit data into 8-bit chunks
        for (i = 0; i < 8; i = i + 1) begin
            pixels[i]      <= pixel_data[(i*8)+:8];
            prev_pixels[i] <= prev_row[(i*8)+:8];
            next_pixels[i] <= next_row[(i*8)+:8];
        end
    end

    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            center = pixels[i];
            bright_count = 0;
            dark_count = 0;

            // Check 8 neighbors in 3x3 window (Bresenham circle)
            // N, NE, E, SE, S, SW, W, NW
            if (prev_pixels[i] > center + THRESHOLD) bright_count = bright_count + 1;
            if (next_pixels[i] > center + THRESHOLD) bright_count = bright_count + 1;

            if (i > 0) begin
                if (pixels[i-1] > center + THRESHOLD) bright_count = bright_count + 1;
                if (prev_pixels[i-1] > center + THRESHOLD) bright_count = bright_count + 1;
                if (next_pixels[i-1] > center + THRESHOLD) bright_count = bright_count + 1;
            end

            if (i < 7) begin
                if (pixels[i+1] > center + THRESHOLD) bright_count = bright_count + 1;
                if (prev_pixels[i+1] > center + THRESHOLD) bright_count = bright_count + 1;
                if (next_pixels[i+1] > center + THRESHOLD) bright_count = bright_count + 1;
            end

            // Similar for dark pixels
            if (prev_pixels[i] < center - THRESHOLD) dark_count = dark_count + 1;
            if (next_pixels[i] < center - THRESHOLD) dark_count = dark_count + 1;

            if (i > 0) begin
                if (pixels[i-1] < center - THRESHOLD) dark_count = dark_count + 1;
                if (prev_pixels[i-1] < center - THRESHOLD) dark_count = dark_count + 1;
                if (next_pixels[i-1] < center - THRESHOLD) dark_count = dark_count + 1;
            end

            if (i < 7) begin
                if (pixels[i+1] < center - THRESHOLD) dark_count = dark_count + 1;
                if (prev_pixels[i+1] < center - THRESHOLD) dark_count = dark_count + 1;
                if (next_pixels[i+1] < center - THRESHOLD) dark_count = dark_count + 1;
            end

            // FAST criterion: at least 9 contiguous pixels with significant difference
            corner_flags[i] = (bright_count >= 4 || dark_count >= 4);
            corner_strength[i] = bright_count > dark_count ? bright_count : dark_count;
        end
    end

endmodule
