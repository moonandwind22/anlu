`timescale 1ns/1ps
// TB: sd_card_bmp 控制核心闭环冒烟（推进状态机/挂起/防连跳/自动轮播/回绕/无资源）
// 时间参数缩小：CLK_FREQ_HZ=1000 -> 消抖 20 拍、组合键 1000 拍、KEY3 长按 500 拍
module tb_ctrl #(parameter integer IMG_COUNT = 4);
    reg clk = 0;
    reg rst = 1;
    reg key_next = 1, key_auto = 1;
    wire [3:0]  state_code;
    wire        display_valid;
    reg         wfin_tgl = 0;
    wire [1:0]  wbuf, dbuf;
    wire [7:0]  error_code;
    wire [7:0]  seg_n, dig_n;
    wire        write_req, write_en;
    wire [31:0] write_data;
    wire        SD_nCS, SD_DCLK, SD_MOSI;
    reg         sd_miso = 1'b1;
    // DUT 内部实例绑定到 stub 模块：注入场景图片数 + 取装载完成脉冲
    // LOAD_CYCLES=400：拉长"装载忙"窗口，使用例 C 的两次补按分别落在
    // 忙区间（->挂起）与挂起保持期（->0x52），且不与提交时刻竞争
    defparam dut.bmp_read_m0.IMG_COUNT  = IMG_COUNT;
    defparam dut.bmp_read_m0.LOAD_CYCLES = 400;
    wire stub_load_done = dut.bmp_read_m0.tb_load_done;

    sd_card_bmp #(
        .CLK_FREQ_HZ      (1000),
        .PERIOD_CYCLES    (32'd2000),
        .MIN_PERIOD_CYCLES(32'd100),
        .MAX_PERIOD_CYCLES(32'd100000)
    ) dut (
        .clk                (clk),
        .rst                (rst),
        .key_next           (key_next),
        .key_auto           (key_auto),
        .state_code         (state_code),
        .bmp_width          (),
        .bmp_height         (),
        .display_valid      (display_valid),
        .write_finish_toggle(wfin_tgl),
        .write_buf_idx      (wbuf),
        .disp_buf_idx       (dbuf),
        .period_cfg_valid   (1'b0),
        .period_cfg_cycles  (32'd0),
        .error_code         (error_code),
        .seg_n              (seg_n),
        .dig_n              (dig_n),
        .write_req          (write_req),
        .write_req_ack      (1'b0),
        .write_en           (write_en),
        .write_data         (write_data),
        .SD_nCS             (SD_nCS),
        .SD_DCLK            (SD_DCLK),
        .SD_MOSI            (SD_MOSI),
        .SD_MISO            (sd_miso)
    );

    always #5 clk = ~clk;

    // 模拟 mem 域：帧写完成在 load 源图完成 15 拍后翻转 toggle（帧原子提交）
    reg [3:0] fin_cnt = 0;
    always @(posedge clk) begin
        if (rst) begin
            fin_cnt <= 4'd0;
        end else if (stub_load_done) begin
            fin_cnt <= 4'd15;
        end else if (fin_cnt != 4'd0) begin
            fin_cnt <= fin_cnt - 4'd1;
            if (fin_cnt == 4'd1) wfin_tgl <= ~wfin_tgl;
        end
    end

    // 统计提交次数（每帧写完成一次）
    integer commits = 0;
    always @(wfin_tgl) commits = commits + 1;

    // 全局周期计数
    integer cycnow = 0;
    always @(posedge clk) cycnow = cycnow + 1;

    integer errors = 0;
    task chk(input cond, input [8*40-1:0] name);
        begin
            if (cond !== 1'b1) begin
                $display("FAIL %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    task cyc(input integer n);
        repeat (n) @(posedge clk);
    endtask

    task press_next;   // 消抖 20 拍：按 60 拍，松开后事件在 ~22 拍内出现
        begin key_next = 0; cyc(60); key_next = 1; cyc(40); end
    endtask
    task press_auto;
        begin key_auto = 0; cyc(60); key_auto = 1; cyc(40); end
    endtask

    initial begin
        cyc(10); rst = 0;

        // ---- 用例A：上电 -> 扫描 -> 首图装载 -> 提交 ----
        wait (display_valid == 1'b1);
        chk (commits == 1,   "A: one commit after boot");
        chk (dbuf == 2'd0,   "A: first image on buffer0");
        chk (error_code == 8'h00, "A: no error");
        $display("PASS A: boot -> scan -> first image committed");
        $display("      (IMG_COUNT=%0d)", IMG_COUNT);

        if (IMG_COUNT >= 2) begin
            // ---- 用例B：单击下一张 ----
            press_next;
            wait (commits == 2);
            cyc(10);   // dbuf 在 wfin 翻换后 ~3 拍才提交，留余量避开竞态
            chk (dbuf == 2'd1, "B: dbuf flipped to 1");
            chk (error_code == 8'h00, "B: no error");
            $display("PASS B: KEY1 single click -> advance to image 2");
        end

        if (IMG_COUNT == 4) begin
            // ---- 用例C：装载忙时按键挂起 1 次；已挂起再按报 0x52 ----
            // stub 装载 150 拍 + 帧写 15 拍：press_next 返回后忙窗口仍有 ~65 拍
            press_next;                    // 触发推进+装载（装载期 150 拍）
            cyc(10);
            key_next = 0; cyc(60); key_next = 1; cyc(30);  // 装载中补按 -> 挂起
            key_next = 0; cyc(60); key_next = 1; cyc(30);  // 已挂起再按 -> 0x52
            chk (error_code == 8'h52, "C: busy error latched");
            wait (commits == 4);           // 两次提交：当前 + 挂起目标
            cyc(10);
            chk (dbuf == 2'd1, "C: dbuf back to 1 (0->1->0->1)");
            $display("PASS C: pending once while busy, second press -> 0x52");

            // ---- 用例D：KEY2 单击开轮播，~2000 拍后自动切一张 ----
            press_auto;
            t0 = cycnow;
            wait (commits == 5);
            t1 = cycnow;
            // 周期起点是轮播开启（事件）时刻，终点含 400 拍装载 + 15 拍帧写 + 同步延迟
            chk ((t1 - t0) >= 2000 && (t1 - t0) <= 2600, "D: auto period ~2000 cycles");
            chk (error_code == 8'h52, "D: no new error");
            press_auto;                    // 关轮播
            $display("PASS D: auto play toggles, period ~2000 cycles (measured %0d)", t1 - t0);
            cyc(3000);
            chk (commits == 5, "D: auto off -> no more advance");

            // ---- 用例E：KEY3 长按不误触发轮播开关 ----
            key_auto = 0; cyc(700); key_auto = 1; cyc(40);   // >500 拍长按
            cyc(3000);
            chk (commits == 5, "E: long press -> no auto toggle, no spurious load");
            $display("PASS E: KEY3 long hold -> volume step only, no toggle");

            // ---- 用例F：组合键 1300 拍 -> save，不产生误事件 ----
            key_next = 0; key_auto = 0; cyc(1300); key_next = 1; key_auto = 1; cyc(40);
            cyc(3000);
            chk (commits == 5, "F: chord -> save only, no spurious load");
            $display("PASS F: chord hold -> save, no spurious advance");
        end

        if (IMG_COUNT == 2) begin
            // ---- 用例H：2 张图 0<->1 回绕 ----
            press_next;
            wait (commits == 3);
            cyc(10);
            chk (dbuf == 2'd0, "H: wrap back to buffer0");
            chk (error_code == 8'h00, "H: no error after wrap");
            $display("PASS H: 2-image card wraps 0->1->0");
        end

        if (IMG_COUNT == 1) begin
            // ---- 用例G：1 张图，按键不切换（0x53），不卡死 ----
            press_next;
            cyc(200);
            chk (error_code == 8'h53, "G: no-resource error");
            chk (commits == 1, "G: no load attempted");
            chk (dbuf == 2'd0, "G: buffer unchanged");
            press_next;                    // 再按一次：仍 0x53，系统不卡死
            cyc(200);
            chk (error_code == 8'h53, "G2: still no-resource, not stuck");
            chk (commits == 1, "G2: still no load");
            $display("PASS G: 1-image card -> 0x53, no switch, no deadlock");
            // 轮播开关在 1 张图时应被拒绝
            press_auto;
            cyc(3000);
            chk (commits == 1, "G3: auto rejected with 1 image");
            $display("PASS G3: auto toggle rejected on 1-image card");
        end

        if (errors == 0) $display("ALL CTRL CASES PASS (IMG_COUNT=%0d)", IMG_COUNT);
        else             $display("CTRL ERRORS: %0d (IMG_COUNT=%0d)", errors, IMG_COUNT);
        $finish;
    end

    integer t0, t1;

    // 看门狗
    initial begin
        #200_000_000;
        $display("TIMEOUT!");
        $finish;
    end
endmodule
