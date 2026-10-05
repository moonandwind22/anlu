`timescale 1ns/1ps
// TB: dual_key_chord_router 行为冒烟（参数缩小加速仿真，不进 TD 工程）
module tb_router;
    reg clk = 0;
    reg rst_n = 0;
    reg key2_n = 1, key3_n = 1;
    wire key2_event, key3_event, key3_long_event, save_event, save_locked;

    localparam integer DEB = 100;    // 100 cycles debounce
    localparam integer HOLD = 2000;  // 2000 cycles chord hold
    localparam integer LONG3 = 1000; // 1000 cycles key3 long

    dual_key_chord_router #(
        .DEBOUNCE_CYCLES(DEB),
        .HOLD_CYCLES(HOLD),
        .KEY3_LONG_CYCLES(LONG3)
    ) dut (
        .clk(clk), .rst_n(rst_n), .save_enabled(1'b1),
        .key2_n(key2_n), .key3_n(key3_n),
        .key2_event(key2_event), .key3_event(key3_event),
        .key3_long_event(key3_long_event),
        .save_event(save_event), .save_locked(save_locked),
        .key2_stable_n(), .key3_stable_n()
    );

    always #5 clk = ~clk; // 100MHz, 周期 10ns

    integer n_key2 = 0, n_key3 = 0, n_long = 0, n_save = 0;
    always @(posedge clk) begin
        if (key2_event) n_key2 = n_key2 + 1;
        if (key3_event) n_key3 = n_key3 + 1;
        if (key3_long_event) n_long = n_long + 1;
        if (save_event) n_save = n_save + 1;
    end

    task cyc(input integer n);   // 等 n 个时钟周期
        repeat (n) @(posedge clk);
    endtask

    initial begin
        rst_n = 0; cyc(2*DEB); rst_n = 1; cyc(2*DEB);

        // 用例1: KEY2 单击 -> 1 个 key2_event
        key2_n = 0; cyc(2*DEB); key2_n = 1; cyc(2*DEB);
        if (n_key2 !== 1) $display("FAIL case1: n_key2=%0d", n_key2);
        else $display("PASS case1: KEY2 single click -> key2_event");

        // 用例2: KEY3 单击 -> 1 个 key3_event
        key3_n = 0; cyc(2*DEB); key3_n = 1; cyc(2*DEB);
        if (n_key3 !== 1) $display("FAIL case2: n_key3=%0d", n_key3);
        else $display("PASS case2: KEY3 single click -> key3_event");

        // 用例3: KEY3 长按超过 LONG3 再松开 -> 1 个 key3_long_event，0 个新增 key3_event
        key3_n = 0; cyc(2*LONG3); key3_n = 1; cyc(2*DEB);
        if (n_long !== 1 || n_key3 !== 1) $display("FAIL case3: n_long=%0d n_key3=%0d", n_long, n_key3);
        else $display("PASS case3: KEY3 long hold -> key3_long_event once, no toggle");

        // 用例4: KEY3 长按 3 倍时长不松 -> 仍只 1 次（marked 防重复）
        key3_n = 0; cyc(3*LONG3); key3_n = 1; cyc(2*DEB);
        if (n_long !== 2) $display("FAIL case4: n_long=%0d", n_long);
        else $display("PASS case4: KEY3 held 3x duration -> exactly one long event");

        // 用例5: 双键同按超过 HOLD -> 1 个 save_event
        key2_n = 0; key3_n = 0; cyc(2*HOLD); key2_n = 1; key3_n = 1; cyc(2*DEB);
        if (n_save !== 1) $display("FAIL case5: n_save=%0d", n_save);
        else $display("PASS case5: chord hold -> save_event once");
        if (!save_locked) $display("FAIL case5b: save_locked not asserted");
        else $display("PASS case5b: save_locked latched");

        // 用例6: save 之后组合键不再触发（locked，一次上电一次保存）
        key2_n = 0; key3_n = 0; cyc(3*HOLD); key2_n = 1; key3_n = 1; cyc(2*DEB);
        if (n_save !== 1) $display("FAIL case6: n_save=%0d (should stay 1)", n_save);
        else $display("PASS case6: no second save after locked");

        // 用例7: 单击后快速补按另一键（无 CHORD 误触发 save）
        key2_n = 0; cyc(2*DEB); key2_n = 1; cyc(2*DEB);
        key3_n = 0; cyc(2*DEB); key3_n = 1; cyc(2*DEB);
        if (n_save !== 1) $display("FAIL case7: n_save=%0d", n_save);
        else $display("PASS case7: sequential presses produce no save");

        $display("ALL CASES DONE");
        $finish;
    end
endmodule
