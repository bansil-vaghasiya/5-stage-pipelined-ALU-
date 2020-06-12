module mipscode(input clk_90, rst_90);

  parameter reg0_90 = 5'd0;
  parameter reg1_90 = 5'd1;
  parameter reg2_90 = 5'd2;
  parameter reg3_90 = 5'd3;
  parameter reg4_90 = 5'd4;
  parameter reg5_90 = 5'd5;
  parameter reg7_90 = 5'd7;

  parameter Itype_90 = 2'b1;
  parameter Rtype_90 = 2'd2;
  parameter Jtype_90 = 2'd3;

  parameter AdduOpcode_90 = 6'b0;
  parameter Addufunct_90 = 6'h21;

  parameter BeqOpcode_90 = 6'd4;
  parameter LwOpcode_90 = 6'h23;

  parameter MulOpcode_90 = 6'h0;
  parameter Mulfunct_90 = 6'h18;

  parameter AddiuOpcode_90 = 6'h9;

  parameter JOpcode_90 = 6'h2;

  parameter JROpcode_90 = 6'h0;
  parameter JRfunct_90 = 6'h8;

  wire wr_enable_inst_90,rd_enable_inst_90;
  wire [7:0] addr_inst_90;
  wire [31:0] data_in_inst_90,dataout_inst_90;

  wire [5:0] opc_90;
  wire [2:0] Insttype_90;
  wire [5:0] funct_90;
  wire [5:0] rs_90,rd_90,rt_90;
  wire [15:0] imm;
  wire [25:0] jaddr_90; 

  wire [31:0] isign_extended_90;

  wire wren_data_90,rden_data_90;
  wire [7:0] adata_90;
  wire [31:0] datain_data_90,dataout_data_90;

  wire [31:0] Jaddr_90,Jaddr_PC_90;

  wire ALUOp_flag_90; 
  wire aluzeroflag_90;
  reg mem2reg_flag_d,mem2regflag_90;
  reg mem2reg_flag_stage1_d,mem2regflag_stage1_90;
  reg mem2reg_flag_stage2_d,mem2regflag_stage2_90;
  reg WriteBack_flag_d,wbflag_90;
  reg WriteBack_flag_d1_d,wbflag_d1_90;

  reg isBranch,isBranch_d;
  wire stall_branch;

  reg [3:0] stallBranch_cnt_90,stallBranch_cnt_d;

  wire [31:0] curr_inst_90;

  reg [31:0] r0,r0_d;
  reg [31:0] r1,r1_d;
  reg [31:0] r2,r2_d;
  reg [31:0] r3,r3_d;
  reg [31:0] r4,r4_d;
  reg [31:0] r5,r5_d;
  reg [31:0] r7,r7_d;
  reg [31:0] r31,r31_d;

  reg [31:0] register[32];

  reg [31:0] PC_90,PC_d;

  //reg [31:0] curr_inst_22,curr_inst_d;

  reg [31:0] data1_90,data1_d;
  reg [31:0] data2_90,data2_d;
  reg [31:0] data_res_90,data_res_d;
  reg [5:0] alu_opcode,alu_opcode_d;
  reg [5:0] alu_opcode_stage1,alu_opcode_stage1_d;
  reg [5:0] alu_opcode_stage2,alu_opcode_stage2_d;
  reg [5:0] alu_funct,alu_funct_d;

  reg [31:0] alu_res_90,alu_res_d;
  reg [31:0] alu_res_stage1,alu_res_stage1_d;
  reg [31:0] mem_add_90,mem_add_d;
  reg [31:0] result_reg_90,result_reg_d;
  reg [31:0] result_reg_d1,result_reg_d1_d;

  reg [31:0] load_data;//,load_data_d;


  assign addr_inst_90 = PC_90;
  assign rd_enable_inst_90 = (isBranch_d) ? 1'b0 : 1'b1;
  assign PC_d = (isBranch_d) ? ((opc_90 == JOpcode_90) ? 32'd4 : Jaddr_PC_90) : PC_90 + 4;

  //Fetch

  //assign curr_inst_d = (rd_enable_inst_90) ? dataout_inst_90 : curr_inst_90;
  assign curr_inst_90 = dataout_inst_90;


  //Decode 
  assign opc_90 = curr_inst_90[31:26];
  assign funct_90 = curr_inst_90[5:0];
  assign Insttype_90 = (opc_90 == 6'b0) ? Rtype_90 :((opc_90 == 6'd2) ? Jtype_90 : Itype_90 );
  assign rs_90 = curr_inst_90[25:21];
  assign rt_90 = curr_inst_90[20:16];
  assign rd_90 = curr_inst_90[15:11];
  assign imm = curr_inst_90[15:0];
  assign jaddr_90 = curr_inst_90[25:0];

  assign stall_branch = (opc_90 == BeqOpcode_90 ||opc_90 == JOpcode_90 || opc_90 == JROpcode_90 && funct_90 == JRfunct_90);
  assign stallBranch_cnt_d = (stall_branch& stallBranch_cnt_90 != 1 )? stallBranch_cnt_90 + 1'd1 : 4'd0;
  assign isBranch_d = stall_branch& (stallBranch_cnt_90 != 4'd1);

  assign ALUOp_flag_90 = !(Insttype_90 == Rtype_90);
  assign mem2reg_flag_d = (opc_90 == LwOpcode_90);
  assign WriteBack_flag_d = (Insttype_90 == Rtype_90 | opc_90 == AddiuOpcode_90 | opc_90 == LwOpcode_90);
  assign isign_extended_90 = (imm[15]) ? (32'hFFFF0000 | imm) : (32'h0 | imm);
  //assign data1_d = (Insttype_90 == Rtype_90 | Insttype_90 == Itype_90) ? rs_90 : 31'd0;
  assign data1_d = ( rs_90 == reg0_90) ? r0 : 
                   ((rs_90 == reg1_90) ? r1 :
                    ((rs_90 == reg2_90) ? r2 : 
                     ((rs_90 == reg3_90) ? r3 : 
                      ((rs_90 == reg4_90) ? r4 :
                       ((rs_90 == reg5_90) ? r5 :
                        ((rs_90 == reg7_90) ? r7 : r31))))));
  assign data2_d = (ALUOp_flag_90)  ? isign_extended_90 :
                   ((rt_90 == reg0_90) ? r0 : 
                    ((rt_90 == reg1_90) ? r1 :
                     ((rt_90 == reg2_90) ? r2 : 
                      ((rt_90 == reg3_90) ? r3 : 
                       ((rt_90 == reg4_90) ? r4 :
                        ((rt_90 == reg5_90) ? r5 :
                         ((rt_90 == reg7_90) ? r7 : r31)))))));
  //assign data2_d = (ALUOp_flag_90) ? isign_extended_90 : rt_90 ;
  assign data_res_d = (Insttype_90 == Rtype_90) ? rd_90 : ((Insttype_90 == Itype_90) ? rt_90 : 32'd0);
  assign alu_opcode_d = opc_90;
  assign alu_funct_d = funct_90;


  //Execute 

  assign alu_res_d = (alu_opcode == AddiuOpcode_90 | ((alu_opcode == AdduOpcode_90) &(alu_funct == Addufunct_90)) | alu_opcode == LwOpcode_90) ? data1_90 + data2_90 : data1_90 * data2_90;
  assign aluzeroflag_90 = (alu_opcode == BeqOpcode_90 & (data1_90 == data2_90)) ? 1'b1: 1'b0;  
  assign result_reg_d = data_res_90;
  assign WriteBack_flag_d1_d = wbflag_90;
  assign Jaddr_90 = PC_90 + (data2_90 << 2);
  assign Jaddr_PC_90 = (aluzeroflag_90 &isBranch)? Jaddr_90 : PC_90;
  assign alu_opcode_stage1_d = alu_opcode;
  assign mem2reg_flag_stage1_d = mem2regflag_90;


  //datamem_22

  assign mem2reg_flag_stage2_d = mem2regflag_stage1_90;
  assign alu_res_stage1_d = alu_res_90;
  assign rden_data_90 = (alu_opcode_stage1 == LwOpcode_90);
  assign adata_90 = alu_res_90;

  assign load_data = (mem2regflag_stage2_90) ? dataout_data_90: alu_res_stage1;
  assign result_reg_d1_d = result_reg_90;

  //WriteBack

  assign r0_d = r0;
  assign r1_d = (result_reg_d1 == reg1_90 && wbflag_d1_90 ) ? load_data : r1;
  assign r2_d = (result_reg_d1 == reg2_90 && wbflag_d1_90 ) ? load_data : r2;
  assign r3_d = (result_reg_d1 == reg3_90 && wbflag_d1_90 ) ? load_data : r3;
  assign r4_d = (result_reg_d1 == reg4_90 && wbflag_d1_90 ) ? load_data : r4;
  assign r5_d = (result_reg_d1 == reg5_90 && wbflag_d1_90 ) ? load_data : r5;
  assign r7_d = (result_reg_d1 == reg7_90 && wbflag_d1_90 ) ? load_data : r7;
 // assign r31_d = (result_reg_d1 == reg31 && wbflag_d1_90 ) ?load_data : r31;



 InstructionMemory U1 (clk_90,rst_90,wr_enable_inst_90,rd_enable_inst_90,addr_inst_90,data_in_inst_90,dataout_inst_90);
  DataMemory D1 (clk_90,rst_90,wren_data_90,rden_data_90,adata_90,datain_data_90,dataout_data_90);

    always @(posedge clk_90 or negedge rst_90) begin
    if(!rst_90) begin
      r0 <= 1;
      r1 <= 0;
      r2 <= 1;
      r3 <= 0;
      r4 <= 0;
      r5 <= 4;
      r7 <= 9;
      r31<= 0;
      PC_90 <= 0;

      // Fetch 
      //curr_inst_22 <= 0;


      //Decode 
      data1_90 <= 0;
      data2_90 <= 0;
      data_res_90 <= 0;
alu_opcode<= 0;
alu_funct<= 0;
isBranch<= 0;
      stallBranch_cnt_90 <= 0;
      mem2regflag_90 <= 0;


      //Execute 

      alu_res_90 <= 0;
      result_reg_90 <= 0;
      mem2regflag_stage1_90 <= 0;
      wbflag_90 <= 0;
      alu_opcode_stage1 <= 0;


      //datamem_90
      result_reg_d1 <= 0;
      alu_res_stage1 <= 0;
    //  load_data<= 0;
      wbflag_d1_90 <= 0;
      mem2regflag_stage2_90 <= 0;


      //WriteBack

    end else begin
      r0 <= r0_d;
      r1 <= r1_d;
      r2 <= r3_d;
      r3 <= r3_d;
      r4 <= r4_d;
      r5 <= r5_d;
      r7 <= r7_d;
      r31<= r31_d;

      PC_90 <=  PC_d;

      //Fetch
     // curr_inst_90 <= curr_inst_d;      


      //Decode 
      data1_90 <= data1_d;
      data2_90 <= data2_d;
      data_res_90 <= data_res_d;
alu_opcode<= alu_opcode_d;
alu_funct<= alu_funct_d;    
isBranch<= isBranch_d;
      stallBranch_cnt_90 <= stallBranch_cnt_d;
      mem2regflag_90 <= mem2reg_flag_d;

      //Execute 

      alu_res_90 <= alu_res_d;
      result_reg_90 <= result_reg_d;
      mem2regflag_stage1_90 <= mem2reg_flag_stage1_d;
      wbflag_90 <= WriteBack_flag_d;
      alu_opcode_stage1 <= alu_opcode_stage1_d;



      result_reg_d1 <= result_reg_d1_d;
      alu_res_stage1 <= alu_res_stage1_d;
     // load_data<= load_data_d;
      wbflag_d1_90 <= WriteBack_flag_d1_d;
      mem2regflag_stage2_90 <= mem2reg_flag_stage2_d;


      //WriteBack

    end
  end

endmodule

module InstructionMemory(input clk_90,rst_90,wren_90,rd_enable,[7:0] addr,[31:0] data_in_90,output [31:0] data_out_90);

  bit [7:0] datamem_90[255:0];
  reg [31:0] dataout_90;

  assign data_out_90 = dataout_90;

  initial begin

    //Addu
    datamem_90[0] = 8'b00100001;
    datamem_90[1] = 8'b00001000;
    datamem_90[2] = 8'b00000000;
    datamem_90[3] = 8'b00000000;

    // beq
    datamem_90[4] = 8'b00001000;
    datamem_90[5] = 8'b00000000;
    datamem_90[6] = 8'b11100000;
    datamem_90[7] = 8'b00010000;

    //lw
    datamem_90[8]  = 8'b00000000;
    datamem_90[9]  = 8'b00000000;
    datamem_90[10] = 8'b01100010;
    datamem_90[11] = 8'b10001100;

    //lw
    datamem_90[12] = 8'b00000000;
    datamem_90[13] = 8'b00000000;
    datamem_90[14] = 8'b10100100;
    datamem_90[15] = 8'b10001100;

    //mul
    datamem_90[16] = 8'b00011000;
    datamem_90[17] = 8'b00010000;
    datamem_90[18] = 8'b01000100;
    datamem_90[19] = 8'b00000000;

    //Addu
    datamem_90[20] = 8'b00100001;
    datamem_90[21] = 8'b00001000;
    datamem_90[22] = 8'b00100010;
    datamem_90[23] = 8'b00000000;

    //addiu
    datamem_90[24] = 8'b00000100;
    datamem_90[25] = 8'b00000000;
    datamem_90[26] = 8'b01100011;
    datamem_90[27] = 8'b00100100;

    //addiu
    datamem_90[28] = 8'b00000100;
    datamem_90[29] = 8'b00000000;
    datamem_90[30] = 8'b10100101;
    datamem_90[31] = 8'b00100100;

    //addiu
    datamem_90[32] = 8'b11111111; // 
    datamem_90[33] = 8'b11111111;
    datamem_90[34] = 8'b11100111;
    datamem_90[35] = 8'b00100100;


    //J
    datamem_90[36] = 8'b00000001;
    datamem_90[37] = 8'b00000000;
    datamem_90[38] = 8'b00000000;
    datamem_90[39] = 8'b00001000;

    //JR
    datamem_90[40] = 8'b00001000;
    datamem_90[41] = 8'b00000000;
    datamem_90[42] = 8'b00000000;
    datamem_90[43] = 8'b00000010;


  end

  always @(posedge clk_90 or negedge rst_90) begin
    if(!rst_90) begin
      dataout_90 <= 0;
    end else begin
      if(wren_90) begin
        datamem_90[addr] <= data_in_90[7:0];
        datamem_90[addr + 1] <= data_in_90[15:8];
        datamem_90[addr + 2] <= data_in_90[23:16];
        datamem_90[addr + 3] <= data_in_90[31:24];
      end 
      if(rd_enable) begin
        dataout_90[7:0] <= datamem_90[addr];
        dataout_90[15:8] <= datamem_90[addr + 1];
        dataout_90[23:16] <= datamem_90[addr + 2];
        dataout_90[31:24] <= datamem_90[addr + 3];
      end
    end
  end

endmodule

module DataMemory(input clk_90,rst_90,wren_90,rd_enable,[7:0] addr,[31:0] data_in_90,output [31:0] data_out_90);

  bit [7:0] datamem_90[255:0];

  reg [31:0] dataout_90;

  assign data_out_90 = dataout_90;

  initial begin
    datamem_90[0] = 0;
    datamem_90[8] = 1;
    datamem_90[16] = 4;
    datamem_90[24] = 5;
    datamem_90[32] = 5;
    datamem_90[40] = 2;
    datamem_90[48] = 5;
    datamem_90[56] = 9;
    datamem_90[64] = 0;

    datamem_90[4] = 2;
    datamem_90[12] = 3;
    datamem_90[20] = 6;
    datamem_90[28] = 7;
    datamem_90[36] = 7;
    datamem_90[44] = 4;
    datamem_90[52] = 7;
    datamem_90[60] = 11;
    datamem_90[68] = 2;
  end

  always @(posedge clk_90 or negedge rst_90) begin
    if(!rst_90) begin
      dataout_90 <= 0;
    end else begin
      if(wren_90) begin
        datamem_90[addr] <= data_in_90[7:0];
        datamem_90[addr + 1] <= data_in_90[15:8];
        datamem_90[addr + 2] <= data_in_90[23:16];
        datamem_90[addr + 3] <= data_in_90[31:24];
      end 
      if(rd_enable) begin
        dataout_90[7:0] <= datamem_90[addr];
        dataout_90[15:8] <= datamem_90[addr + 1];
        dataout_90[23:16] <= datamem_90[addr + 2];
        dataout_90[31:24] <= datamem_90[addr + 3];
      end
    end
  end

endmodule

