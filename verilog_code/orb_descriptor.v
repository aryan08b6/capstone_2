// ORB Descriptor Extraction Module
// Computes binary ORB-like descriptors from image patches
// Each pixel pair comparison becomes one bit of the descriptor

module orb_descriptor (
    input wire clk,
    input wire [63:0] patch_data,     // 8x8 patch (64 pixels)
    input wire [63:0] patch_data2,    // Second patch for comparison
    output reg [63:0] descriptor,     // 64-bit binary descriptor
    output reg descriptor_valid
);

    reg [7:0] patch1 [0:7];
    reg [7:0] patch2 [0:7];
    integer i;

    always @(posedge clk) begin
        for (i = 0; i < 8; i = i + 1) begin
            patch1[i] <= patch_data[(i*8)+:8];
            patch2[i] <= patch_data2[(i*8)+:8];
        end
        descriptor_valid <= 1;
    end

    // ORB descriptor: Compare pairs of pixels
    // Each comparison becomes one bit
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            // Simple pixel pair comparison
            // If patch1[i] > patch2[i], set bit to 1, else 0
            descriptor[i] = (patch1[i] > patch2[i]) ? 1'b1 : 1'b0;

            // Cross-comparisons for more bits
            descriptor[i+8]  = (patch1[i] > patch1[(i+1)%8]) ? 1'b1 : 1'b0;
            descriptor[i+16] = (patch2[i] > patch2[(i+1)%8]) ? 1'b1 : 1'b0;

            // Intensity gradient bits
            descriptor[i+24] = ({patch1[i], 1'b0} > {patch2[i], 1'b0}) ? 1'b1 : 1'b0;
            descriptor[i+32] = (patch1[i] > (patch2[i] >> 1)) ? 1'b1 : 1'b0;

            // Fill remaining bits with neighbor comparisons
            descriptor[i+40] = (patch1[i] > patch1[0]) ? 1'b1 : 1'b0;
            descriptor[i+48] = (patch2[i] < patch1[i]) ? 1'b1 : 1'b0;
            descriptor[i+56] = ((patch1[i] + patch2[i]) > 128) ? 1'b1 : 1'b0;
        end
    end

endmodule
