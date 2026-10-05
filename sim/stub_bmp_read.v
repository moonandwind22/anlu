`timescale 1ns/1ps
// TB stub: bmp_read 行为级模型（仅供仿真，不进 TD 工程）
// - 扫描：scan_start 后每 10 拍报一张图（sector=0x100*(i+1)），报完 IMG_COUNT 张置 scan_done
// - 装载：load_start 后忙 LOAD_CYCLES 拍，随后回 idle（ready=1 表示源图已送完 FIFO）
module bmp_read #(
    parameter integer IMG_COUNT  = 4,
    parameter integer LOAD_CYCLES = 30
)(
    input  wire        clk,
    input  wire        rst,
    output wire        ready,

    input  wire        scan_start,
    input  wire [31:0] scan_start_sector,
    input  wire [31:0] scan_max_sector,
    input  wire [2:0]  scan_target_count,
    output reg         scan_done,
    output reg         scan_found_valid,
    output reg  [31:0] scan_found_sector,
    output reg  [2:0]  scan_found_total,

    input  wire        load_start,
    input  wire [31:0] load_sector,

    input  wire        sd_init_done,
    output reg  [3:0]  state_code,
    output reg  [15:0] bmp_width,
    output reg  [15:0] bmp_height,
    output wire        write_req,
    input  wire        write_req_ack,
    output wire        sd_sec_read,
    output wire [31:0] sd_sec_read_addr,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end,
    output wire        bmp_data_wr_en,
    output wire [23:0] bmp_data,

    output reg         tb_load_done      // TB 专用：装载源图完成脉冲
);
    reg [1:0]  mode;   // 0=idle, 1=scan, 2=load
    reg [10:0] cnt;    // 11 位：支持 LOAD_CYCLES > 255（避免 [7:0] 截断）
    reg [2:0]  fidx;

    assign ready         = (mode == 2'd0);
    assign write_req     = 1'b0;
    assign sd_sec_read   = 1'b0;
    assign sd_sec_read_addr = 32'd0;
    assign bmp_data_wr_en = 1'b0;
    assign bmp_data       = 24'd0;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mode <= 2'd0; cnt <= 8'd0; fidx <= 3'd0;
            scan_done <= 1'b0; scan_found_valid <= 1'b0;
            scan_found_sector <= 32'd0; scan_found_total <= 3'd0;
            state_code <= 4'd0;
            bmp_width <= 16'd640; bmp_height <= 16'd480;
            tb_load_done <= 1'b0;
        end else begin
            tb_load_done     <= 1'b0;
            scan_found_valid <= 1'b0;

            if (mode == 2'd0) begin
                if (scan_start) begin
                    mode <= 2'd1; cnt <= 8'd0; fidx <= 3'd0;
                    scan_done <= 1'b0; state_code <= 4'd2;
                end else if (load_start) begin
                    mode <= 2'd2; cnt <= 8'd0; state_code <= 4'd3;
                end
            end else if (mode == 2'd1) begin
                if (cnt == 11'd10) begin
                    cnt <= 8'd0;
                    if (fidx < IMG_COUNT[2:0]) begin
                        scan_found_valid <= 1'b1;
                        scan_found_sector <= 32'h100 * (fidx + 3'd1);
                        fidx <= fidx + 3'd1;
                    end else begin
                        mode <= 2'd0;
                        scan_done <= 1'b1;
                        scan_found_total <= IMG_COUNT[2:0];
                        state_code <= 4'd0;
                    end
                end else cnt <= cnt + 8'd1;
            end else begin
                if (cnt == LOAD_CYCLES[10:0]) begin
                    mode <= 2'd0;
                    tb_load_done <= 1'b1;   // 源图已送完 FIFO（ready 将回 1）
                    state_code <= 4'd0;
                end else cnt <= cnt + 8'd1;
            end
        end
    end
endmodule
