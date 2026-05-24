// Gaussian Blur / Low-Pass Filter Module
// Implements 3x3 Gaussian kernel for image smoothing
// Used for creating image pyramids in SLAM

module gaussian_filter (
    input wire clk,
    input wire [63:0] current_row,    // Current 8 pixels
    input wire [63:0] prev_row,       // Previous row pixels
    input wire [63:0] next_row,       // Next row pixels
    output reg [63:0] filtered_data   // Filtered output
);

    reg [7:0] curr [0:7];
    reg [7:0] prev [0:7];
    reg [7:0] next [0:7];
    reg [15:0] sum;
    reg [7:0] result;
    integer i;

    // Gaussian kernel (simplified 3x3):
    // [1 2 1]
    // [2 4 2] / 16
    // [1 2 1]

    always @(posedge clk) begin
        for (i = 0; i < 8; i = i + 1) begin
            curr[i] <= current_row[(i*8)+:8];
            prev[i] <= prev_row[(i*8)+:8];
            next[i] <= next_row[(i*8)+:8];
        end
    end

    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            sum = 0;

            // Center and vertical neighbors (weight 4 and 2)
            sum = sum + (curr[i] << 2);  // Center * 4
            sum = sum + (prev[i] << 1);  // Top * 2
            sum = sum + (next[i] << 1);  // Bottom * 2

            // Horizontal neighbors
            if (i > 0) begin
                sum = sum + (curr[i-1] << 1); // Left * 2
                sum = sum + prev[i-1];         // Top-left * 1
                sum = sum + next[i-1];         // Bottom-left * 1
            end

            if (i < 7) begin
                sum = sum + (curr[i+1] << 1); // Right * 2
                sum = sum + prev[i+1];         // Top-right * 1
                sum = sum + next[i+1];         // Bottom-right * 1
            end

            // Divide by 16 (shift right by 4)
            result = sum >> 4;
            filtered_data[(i*8)+:8] = result;
        end
    end

endmodule
