`timescale 1ns/1ps
// TB stub: sd_card_top 行为级模型（仅供仿真，不进 TD 工程）
// sd_init_done 在复位释放约 20 拍后置 1，其余接口悬置
module sd_card_top (
    input  wire        clk,
    input  wire        rst,
    output reg         SD_nCS,
    output reg         SD_DCLK,
    output reg         SD_MOSI,
    input  wire        SD_MISO,
    output reg         sd_init_done,
    input  wire        sd_sec_read,
    input  wire [31:0] sd_sec_read_addr,
    output reg  [7:0]  sd_sec_read_data,
    output reg         sd_sec_read_data_valid,
    output reg         sd_sec_read_end,
    input  wire        sd_sec_write,
    input  wire [31:0] sd_sec_write_addr,
    input  wire [31:0] sd_sec_write_data,
    output reg         sd_sec_write_data_req,
    output reg         sd_sec_write_end
);
    reg [7:0] init_cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sd_init_done <= 1'b0;
            init_cnt    <= 8'd0;
        end else if (!sd_init_done) begin
            if (init_cnt == 8'd20) sd_init_done <= 1'b1;
            else                   init_cnt <= init_cnt + 8'd1;
        end
    end

    always @* begin
        SD_nCS = 1'b1;
        SD_DCLK = 1'b0;
        SD_MOSI = 1'b0;
        sd_sec_read_data = 8'd0;
        sd_sec_read_data_valid = 1'b0;
        sd_sec_read_end = 1'b0;
        sd_sec_write_data_req = 1'b0;
        sd_sec_write_end = 1'b0;
    end
endmodule
