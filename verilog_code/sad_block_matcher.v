// SAD (Sum of Absolute Differences) Block Matcher
// Computes SAD metric for stereo vision disparity estimation
// Processes two 64-bit blocks (left and right stereo images)

module sad_block_matcher (
    input wire clk,
    input wire [63:0] left_block,     // 8x8 block from left image
    input wire [63:0] right_block,    // 8x8 block from right image
    input wire [7:0] disparity_offset, // Disparity offset to test
    output reg [31:0] sad_value,      // SAD result (32-bit to avoid overflow)
    output reg [7:0] best_disparity   // Best matching disparity
);

    reg [7:0] left_pixels [0:7];
    reg [7:0] right_pixels [0:7];
    reg [15:0] partial_sad;
    integer i;

    always @(posedge clk) begin
        // Unpack 64-bit blocks into 8-bit pixels
        for (i = 0; i < 8; i = i + 1) begin
            left_pixels[i]  <= left_block[(i*8)+:8];
            right_pixels[i] <= right_block[(i*8)+:8];
        end
    end

    always @(*) begin
        partial_sad = 0;

        // Compute absolute differences and sum them
        for (i = 0; i < 8; i = i + 1) begin
            if (left_pixels[i] > right_pixels[i])
                partial_sad = partial_sad + (left_pixels[i] - right_pixels[i]);
            else
                partial_sad = partial_sad + (right_pixels[i] - left_pixels[i]);
        end

        sad_value = {16'b0, partial_sad}; // Zero-extend to 32 bits
    end

    // Track best disparity (in a real system, this would compare across disparities)
    always @(posedge clk) begin
        if (sad_value < 128) begin // Arbitrary threshold for good match
            best_disparity <= disparity_offset;
        end
    end

endmodule
