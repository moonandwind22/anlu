
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
    // 自动轮播周期（第2课 3.2：默认 2.5s，官方为 1s）
    parameter [31:0]  PERIOD_CYCLES     = 32'd250_000_000,
    parameter [31:0]  MIN_PERIOD_CYCLES = 32'd50_000_000,    // 0.5s
    parameter [31:0]  MAX_PERIOD_CYCLES = 32'd3_000_000_000 // 30s
)(
    input                       clk,
    input                       rst,
    input                       key_next,       // 板上 KEY1：单击下一张（router 的 key2）
    input                       key_auto,       // 板上 KEY2：单击轮播开关 / 长按调节（router 的 key3）
    output [3:0]                state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    // 轮播周期运行时配置（暂无外部来源，顶层 tie 0，端口预留）
    input                       period_cfg_valid,
    input  [31:0]               period_cfg_cycles,

    output reg [7:0]            error_code,     // 0x00 正常 / 0x51 周期越界 / 0x52 请求忙 / 0x53 无资源
    output [7:0]                seg_n,          // 数码管段选，低有效 {dp,g,f,e,d,c,b,a}
    output [7:0]                dig_n,          // 数码管位选，低有效 one-hot（板卡 6 位，用 [5:0]）

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

// ---- 错误码体系（第2课 3.2） ----
localparam [7:0] ERR_NONE         = 8'h00;
localparam [7:0] ERR_PERIOD_RANGE = 8'h51;
localparam [7:0] ERR_REQUEST_BUSY = 8'h52;
localparam [7:0] ERR_NO_RESOURCE  = 8'h53;

// 默认周期换算成整秒（四舍五入，2.5s -> 3）
localparam [5:0] PERIOD_SEC_DEFAULT = (PERIOD_CYCLES + CLK_FREQ_HZ/2) / CLK_FREQ_HZ;

// ---- 推进状态机（策略层脉冲 -> 资源校验 -> 装载） ----
localparam [1:0] ADV_IDLE  = 2'd0;
localparam [1:0] ADV_PULSE = 2'd1;
localparam [1:0] ADV_CHECK = 2'd2;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             sd_init_done;
wire             bmp_ready;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;

// ---- 按键处理层（router + guard，替换原 key_press_debounce） ----
wire key2_event;         // KEY1 单击：下一张
wire key3_event;         // KEY2 单击：轮播开/关
wire key3_long_event;    // KEY2 长按 0.5s：调节值步进
wire save_event;         // KEY1+KEY2 同按 1s：保存设置（寄存器级）

// ---- 控制策略层 ----
wire [1:0] media_id;     // 策略层唯一决策"下一张切哪张"（模 4 推进）
wire player_enable_fast; // 首图开门：scan_done && 发现了第 1 张

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              next_req_pending;
reg              load_busy;

// ---- 推进状态机寄存器 ----
reg [1:0]        adv_state;
reg [1:0]        adv_retry;

// ---- 会话设置寄存器 ----
reg [7:0]        volume;           // KEY2 长按步进 +32，默认 128
reg [7:0]        saved_volume;     // 组合键锁存（断电即失，Flash 留作后续）
reg [5:0]        period_sec;       // 当前轮播周期整秒（四舍五入）
reg [5:0]        saved_period_sec;
reg              saved_auto_en;

// ---- 显示层事件 ----
reg              pic_event;        // 提交切图 -> PIC 页强制 2s
reg              vol_event;        // 调节步进 -> VOL 页强制 2s
reg              prd_event;        // 周期生效/保存确认 -> PRD 页强制 2s

// bmp_ready 先表示"源文件读取/送 FIFO 完成"；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

// 当前轮播周期（运行时可配置）
reg [31:0] period_cycles_r;

wire auto_tick;
wire [3:0] img_found_bits;

assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};

// 发现位图 -> 完成位图（发现顺序即编号顺序）：count=1 -> 0001，count=4 -> 1111
assign img_found_bits = 4'b1111 >> (4 - img_found_count);
assign auto_tick      = (auto_cnt == period_cycles_r - 32'd1);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];

// 推进脉冲（PULSE 态恰为 1 拍），策略层在其下一拍完成 media_id+1
wire adv_next_pulse = (adv_state == ADV_PULSE);

// 周期配置的整秒换算（四舍五入，常数除法）
wire [31:0] period_cfg_sec = (period_cfg_cycles + CLK_FREQ_HZ/2) / CLK_FREQ_HZ;

function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0;
    end
endfunction

// ===================== 按键处理层 =====================
// 消抖 20ms 对齐官方 key_press_debounce；组合键 1s；KEY3 长按 0.5s
dual_key_chord_router #(
    .DEBOUNCE_CYCLES  (CLK_FREQ_HZ/50),   // 20ms
    .HOLD_CYCLES      (CLK_FREQ_HZ),      // 1s
    .KEY3_LONG_CYCLES (CLK_FREQ_HZ/2)     // 0.5s
) u_key_router (
    .clk             (clk),
    .rst_n           (~rst),
    .save_enabled    (1'b1),
    .key2_n          (key_next),
    .key3_n          (key_auto),
    .key2_event      (key2_event),
    .key3_event      (key3_event),
    .key3_long_event (key3_long_event),
    .save_event      (save_event),
    .save_locked     (),
    .key2_stable_n   (),
    .key3_stable_n   ()
);

// ===================== 控制策略层 =====================
// 策略层是"下一张切哪张"的唯一决策者（模 4 推进），替代原 next_index_limited
media_policy u_media_policy (
    .clk           (clk),
    .rst_n         (~rst),
    .next_event    (adv_next_pulse),
    .menu_up       (1'b0),
    .menu_down     (1'b0),
    .menu_select   (1'b0),
    .image_complete(img_found_bits),
    .media_id      (media_id),
    .audio_id      (),                 // 本工程为固定测试音，图片-音频联动留作后续
    .menu_state    (),
    .linked_event  ()
);

// 首图开门：与原 scan_done && found>0 语义等价的架构化接法
// （2 缓冲下语义降级为首图开门，4 缓冲升级时自动恢复完整语义）
fast_boot_gate u_fast_boot_gate (
    .clk              (clk),
    .rst_n            (~rst),
    .ingest_done      (scan_done),
    .image_complete   (img_found_bits),
    .player_enable_raw(scan_done),
    .player_enable_fast(player_enable_fast),
    .first_image_ready()
);

// ===================== 显示输出层 =====================
// 三页面轮播 PIC/VOL/PRD，参数事件强制对应页 2s（6 位数码管适配版）
seg7_panel #(
    .CLK_HZ            (CLK_FREQ_HZ),
    .SCAN_STEP_CYCLES  (41667),          // 100MHz/41667/6 ≈ 每位 400Hz 刷新
    .PAGE_PERIOD_CYCLES(CLK_FREQ_HZ),   // 1s 页面轮播
    .HOLD_CYCLES       (2*CLK_FREQ_HZ)  // 强制页 2s
) u_seg7_panel (
    .clk       (clk),
    .rst_n     (~rst),
    .enable    (1'b1),
    .media_id  (media_id),
    .volume    (volume),
    .period_sec(period_sec),
    .pic_event (pic_event),
    .vol_event (vol_event),
    .prd_event (prd_event),
    .seg_n     (seg_n),
    .dig_n     (dig_n)
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;
        load_busy             <= 1'b0;
        source_done_seen      <= 1'b0;
        display_valid         <= 1'b0;
        adv_state             <= ADV_IDLE;
        adv_retry             <= 2'd0;
        volume                <= 8'd128;
        saved_volume          <= 8'd128;
        period_cycles_r       <= PERIOD_CYCLES;
        period_sec            <= PERIOD_SEC_DEFAULT;
        saved_period_sec      <= PERIOD_SEC_DEFAULT;
        saved_auto_en         <= 1'b0;
        pic_event             <= 1'b0;
        vol_event             <= 1'b0;
        prd_event             <= 1'b0;
        error_code            <= ERR_NONE;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        pic_event         <= 1'b0;
        vol_event         <= 1'b0;
        prd_event         <= 1'b0;

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            next_req_pending      <= 1'b0;
            load_busy             <= 1'b0;
            source_done_seen      <= 1'b0;
            display_valid         <= 1'b0;
            adv_state             <= ADV_IDLE;
            adv_retry             <= 2'd0;
            // 会话设置（volume/period/保存组/错误码）不随拔卡回退
        end else begin
            // 扫描阶段缓存前 4 张图的起始 sector
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    default: ;
                endcase

                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            // 记住 bmp_read 已经把源图送完 FIFO，但还不能切显示，得等整帧写完
            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            // 只有真正收到 write_finish_toggle 脉冲，才提交新图并切换显示缓冲区
            if (load_busy && source_done_seen && write_finish_pulse) begin
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                disp_buf_idx          <= pending_buf_idx;
                img_idx               <= load_idx;
                display_valid         <= 1'b1;
                first_image_committed <= 1'b1;
                pic_event             <= 1'b1;   // PIC 页强制 2s
            end

            // 上电后自动发起一次"扫描前 4 张 BMP"
            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                pending_buf_idx       <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                next_req_pending      <= 1'b0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
            end else begin
                // ---- 事件源（router）----
                // KEY1 单击：请求推进（已挂起/推进中则报忙，不累积，防连跳）
                if (key2_event && player_enable_fast) begin
                    if ((adv_state != ADV_IDLE) || next_req_pending)
                        error_code <= ERR_REQUEST_BUSY;
                    else begin
                        adv_state <= ADV_PULSE;
                        adv_retry <= 2'd0;
                    end
                end

                // KEY2 单击：轮播开/关
                if (key3_event && scan_done && (img_found_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                end

                // KEY2 长按：调节值 +32 步进（256 回绕），VOL 页强制 2s
                if (key3_long_event) begin
                    volume    <= volume + 8'd32;
                    vol_event <= 1'b1;
                end

                // KEY1+KEY2 组合长按：锁存设置（寄存器级保存），PRD 页强制 2s 作确认
                if (save_event) begin
                    saved_volume     <= volume;
                    saved_period_sec <= period_sec;
                    saved_auto_en    <= auto_play_en;
                    prd_event        <= 1'b1;
                end

                // 轮播周期运行时配置（0.5s~30s，越界报错保持旧值）
                if (period_cfg_valid) begin
                    if ((period_cfg_cycles >= MIN_PERIOD_CYCLES) &&
                        (period_cfg_cycles <= MAX_PERIOD_CYCLES)) begin
                        period_cycles_r <= period_cfg_cycles;
                        period_sec      <= period_cfg_sec[5:0];
                        prd_event       <= 1'b1;
                    end else begin
                        error_code <= ERR_PERIOD_RANGE;
                    end
                end

                // ---- 推进状态机：PULSE(发1拍) -> CHECK(资源校验/装载/挂起) ----
                case (adv_state)
                    ADV_PULSE: adv_state <= ADV_CHECK;
                    ADV_CHECK: begin
                        if (img_found_bits[media_id] && (media_id != img_idx)) begin
                            // 目标有效：空闲即装载，忙则挂起 1 次（目标=media_id，挂起期间稳定）
                            if (!load_busy && bmp_ready && display_valid) begin
                                load_idx         <= media_id;
                                load_sector      <= sector_lut(media_id);
                                pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                                write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                                load_start_pulse <= 1'b1;
                                load_busy        <= 1'b1;
                                source_done_seen <= 1'b0;
                                auto_cnt         <= 32'd0;
                            end else begin
                                next_req_pending <= 1'b1;
                            end
                            adv_state <= ADV_IDLE;
                            adv_retry <= 2'd0;
                        end else if (adv_retry < 2'd3) begin
                            // 目标无效：重试推进（<=3 次；4 次回绕后 media_id 回到原显示值，无失步）
                            adv_retry <= adv_retry + 2'd1;
                            adv_state <= ADV_PULSE;
                        end else begin
                            // 回绕一周仍无有效目标：报无资源，不切换、不卡死
                            error_code <= ERR_NO_RESOURCE;
                            adv_state  <= ADV_IDLE;
                            adv_retry  <= 2'd0;
                        end
                    end
                    default: ;
                endcase

                // ---- 自动轮播计时（周期可配置）----
                // 只有当前没有写图任务、首图已提交且至少 2 张图时才计时
                if (scan_done && auto_play_en && display_valid && !load_busy &&
                    first_image_committed && (img_found_count > 3'd1)) begin
                    if (auto_tick && (adv_state == ADV_IDLE) && !next_req_pending) begin
                        auto_cnt  <= 32'd0;
                        adv_state <= ADV_PULSE;   // 到期与手动走同一条推进状态机
                        adv_retry <= 2'd0;
                    end else if (!auto_tick) begin
                        auto_cnt <= auto_cnt + 32'd1;
                    end
                    // auto_tick 且推进忙：保持在 tick 上，推进空闲后立即触发
                end else begin
                    auto_cnt <= 32'd0;
                end

                // 首图自动加载到 buffer0（策略层复位值 0 与首图一致，无失步）
                if (scan_done && !first_image_committed && bmp_ready && !load_busy && (img_found_count != 3'd0)) begin
                    load_idx         <= 2'd0;
                    load_sector      <= img_sector0;
                    pending_buf_idx  <= 2'd0;
                    write_buf_idx    <= 2'd0;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 挂起的推进请求：目标 = 策略层已校验的 media_id
                else if (scan_done && bmp_ready && display_valid && !load_busy && next_req_pending && (img_found_count != 3'd0)) begin
                    load_idx         <= media_id;
                    load_sector      <= sector_lut(media_id);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
            end
        end
    end
end

bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule
