`timescale 1ns/1ps
// TB: seg7_panel 6 位适配冒烟（不进 TD 工程）
module tb_panel;
    reg clk = 0;
    reg rst_n = 0;
    wire [7:0] seg_n, dig_n;
    reg pic_event = 0, vol_event = 0, prd_event = 0;

    // 参数缩小：每 digit 10 cycles，采样 6 位约需 360 cycles，页窗口要足够长
    seg7_panel #(
        .CLK_HZ(100000000),
        .DIGITS(6),
        .SCAN_STEP_CYCLES(10),
        .PAGE_PERIOD_CYCLES(5000),
        .HOLD_CYCLES(5000)
    ) dut (
        .clk(clk), .rst_n(rst_n), .enable(1'b1),
        .media_id(2'd0), .volume(8'd128), .period_sec(6'd3),
        .pic_event(pic_event), .vol_event(vol_event), .prd_event(prd_event),
        .seg_n(seg_n), .dig_n(dig_n)
    );

    always #5 clk = ~clk;

    task cyc(input integer n);
        repeat (n) @(posedge clk);
    endtask

    integer errors = 0;
    reg [7:0] d0, d1, d2, d3, d4, d5;

    task sample_digits;
        begin
            wait (dig_n[0] == 1'b0); #1 d0 = seg_n;
            wait (dig_n[1] == 1'b0); #1 d1 = seg_n;
            wait (dig_n[2] == 1'b0); #1 d2 = seg_n;
            wait (dig_n[3] == 1'b0); #1 d3 = seg_n;
            wait (dig_n[4] == 1'b0); #1 d4 = seg_n;
            wait (dig_n[5] == 1'b0); #1 d5 = seg_n;
        end
    endtask

    task chk(input [7:0] got, input [7:0] exp, input [8*8-1:0] name);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got %h exp %h", name, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        cyc(20); rst_n = 1; cyc(20);

        // ---- PIC 页（media_id=0 -> "PIC 1"）----
        pic_event = 1; cyc(2); pic_event = 0;
        cyc(2); sample_digits;
        chk(d0, 8'hF9, "PIC d0=1");   // '1'
        chk(d1, 8'hC6, "PIC d1=C");   // 'C'
        chk(d2, 8'hF9, "PIC d2=I");   // 'I'
        chk(d3, 8'h8C, "PIC d3=P");   // 'P'
        chk(d4, 8'hFF, "PIC d4=blank");
        chk(d5, 8'hFF, "PIC d5=blank");
        if (errors == 0) $display("PASS PIC page symbols (6-digit layout)");

        // ---- VOL 页（volume=128 -> "VOL 128"）----
        vol_event = 1; cyc(2); vol_event = 0;
        cyc(2); sample_digits;
        chk(d0, 8'h80, "VOL d0=8");
        chk(d1, 8'hA4, "VOL d1=2");
        chk(d2, 8'hF9, "VOL d2=1");
        chk(d3, 8'hC7, "VOL d3=L");
        chk(d4, 8'hC0, "VOL d4=O");
        chk(d5, 8'hC1, "VOL d5=V");
        if (errors == 0) $display("PASS VOL page symbols + BCD(128)");

        // ---- PRD 页（period_sec=3 -> "PRD 03S"）----
        prd_event = 1; cyc(2); prd_event = 0;
        cyc(2); sample_digits;
        chk(d0, 8'h92, "PRD d0=S");
        chk(d1, 8'hB0, "PRD d1=3");
        chk(d2, 8'hC0, "PRD d2=0");
        chk(d3, 8'hA1, "PRD d3=d");
        chk(d4, 8'hAF, "PRD d4=R");
        chk(d5, 8'h8C, "PRD d5=P");
        if (errors == 0) $display("PASS PRD page symbols (period 3s)");

        // ---- 6 位回绕：dig_n[7:6] 永不为 0 ----
        fork
            begin : wrap_check
                reg ok;
                integer i;
                ok = 1'b1;
                repeat (1000) begin
                    @(posedge clk);
                    if (dig_n[7] == 1'b0 || dig_n[6] == 1'b0) ok = 1'b0;
                end
                if (!ok) begin
                    $display("FAIL wrap: dig_n[7:6] went active");
                    errors = errors + 1;
                end else
                    $display("PASS 6-digit wrap (dig_n[7:6] never active)");
            end
            begin
                cyc(1000);
            end
        join

        if (errors == 0) $display("ALL PANEL CASES PASS");
        else $display("TOTAL ERRORS: %0d", errors);
        $finish;
    end
endmodule
