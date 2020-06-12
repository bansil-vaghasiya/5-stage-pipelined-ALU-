module mips_tb();

  reg clk_90,rst_90;

  initial begin
    clk_90 = 0;
    forever begin
      #5 clk_90 = !clk_90;
    end
  end

  initial begin
    rst_90 = 0;
    #25;
    rst_90 = 1;
  end

  initial begin
    #200;
    $finish();
  end

  initial begin
    $dumpfile("wave.vcd"); $dumpvars;
  end

  mipscode mipstb(clk_90,rst_90);

endmodule
