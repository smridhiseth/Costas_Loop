`timescale 1ns/10ps

/*module costas( input reg clk, input reg reset, 
input reg signed [9:0] ADC, 
input reg pushADC, 
output reg signed [7:0] Byte, 
output reg pushByte, 
output reg Sync, 
output reg lastByte, 
input reg stopIn);*/

module costas(clk,reset,pushADC,ADC,pushByte,Byte,Sync,lastByte,stopIn);

input clk, reset, pushADC, stopIn;
input signed [9:0] ADC;
output reg pushByte, lastByte, Sync;
output reg [7:0] Byte;

`include "tables.v"

reg pushin, pushin_mixer, pushin_nco, pushin_matchfilter, pushin_fir_shift, pushin_fir, sync_enable;
reg signed [31:0] Phase, sctable_out;
reg signed [31:0] NCO, NCO_new, Phase_new, mixer_out;
reg signed [31:0] Cos_out, Sin_out;
reg signed [31:0] ADC1;
reg signed [31:0] sin_nco, cos_nco;
reg signed [31:0] fir_sin_out, fir_cos_out;
real fir_sin_out_deci;
reg signed [31:0] fir_cos_temp [42:0];
reg signed [31:0] fir_sin_temp [42:0]; 
reg signed [31:0] shft_cos_reg [42:0];
reg signed [31:0] shft_sin_reg [42:0];
reg signed [31:0] mixer_out_temp,firc_out;

wire integer f [42:0]; 

assign f[0] = 2;
assign f[1] = 4;
assign f[2] = 4;
assign f[3] = 2;
assign f[4] = -3;
assign f[5] = -10;
assign f[6] = -14;
assign f[7] = -14;
assign f[8] = -6;
assign f[9] = 7;
assign f[10] = 19;
assign f[11] = 22;
assign f[12] = 12;
assign f[13] = -11;
assign f[14] = -36;
assign f[15] = -48;
assign f[16] = -35;
assign f[17] = 11;
assign f[18] = 83;
assign f[19] = 161;
assign f[20] = 221;
assign f[21] = 244;
assign f[22] = 221;
assign f[23] = 161;
assign f[24] = 83;
assign f[25] = 11;
assign f[26] = -35;
assign f[27] = -48;
assign f[28] = -36;
assign f[29] = -11;
assign f[30] = 12;
assign f[31] = 22;
assign f[32] = 19;
assign f[33] = 7;
assign f[34] = -6;
assign f[35] = -14;
assign f[36] = -14;
assign f[37] = -10;
assign f[38] = -3;
assign f[39] = 2;
assign f[40] = 4;
assign f[41] = 4;
assign f[42] = 2;

integer peak, count;


integer count1 = 0;
always @ (*)
begin
	NCO_new = NCO;
	Phase_new = Phase;
	if(pushin)
		begin
			shft_cos_reg[0] = (ADC1 * cos_nco)/512;
			shft_sin_reg[0] = (ADC1 * sin_nco)/512;
			if(ADC1 == 3)
			begin
			count1 = 1;
			end
			Cos_out = (ADC1 * cos_nco)/512;
			Sin_out = (ADC1 * sin_nco)/512;
		end
	if(pushin_mixer)
		begin
			mixer_out_temp = fir_cos_out * fir_sin_out;
			//mixer_out_temp = 2 * fir_sin_out;
			mixer_out = {{16{mixer_out_temp[31]}}, mixer_out_temp[31:16]};
		end
	if(pushin_nco)
		begin
			Phase_new = 32'd429496729 + (mixer_out << 14);
			NCO_new = NCO + Phase;
		end
	if(pushin_matchfilter)
		begin
			count = count + 1;
			if(fir_cos_out[31] == 1'b1)
				begin
					peak = peak - 1;
				end
				
			if(fir_cos_out[31] == 1'b0)
				begin
					peak = peak + 1;
				end
			if(count == 1047 ) //1047
				begin
					sync_enable = 1'b1;
				end
		
		end
		
		
end
integer i, j, k;
integer l;
reg send_out;
/*function real deci;
input reg [31:0] opa;
real res;
begin
 res = ((opa[31]*(2**21)) + (opa[30]*(2**20)) + (opa[29]*(2**19)) + (opa[28]*(2**18))+
        (opa[27]*(2**17)) + (opa[26]*(2**16)) + (opa[25]*(2**15)) + (opa[24]*(2**14))+
        (opa[23]*(2**13)) + (opa[22]*(2**12)) + (opa[21]*(2**11)) + (opa[20]*(2**10))+
        (opa[19]*(2**9)) + (opa[18]*(2**8)) + (opa[17]*(2**7)) + (opa[16]*(2**6))  +
        (opa[15]*(2**5)) + (opa[14]*(2**4)) + (opa[13]*(2**3)) + (opa[12]*(2**2))+
        (opa[11]*(2**1)) + (opa[10]*(1)) + (opa[9]*(0.5)) + (opa[8]*(0.25))+
         (opa[7]*(0.125)) + (opa[6]*(0.0625)) + (opa[5]*(0.03125)) + (opa[4]*(0.015625)) +  
         (opa[3]*(0.0078125)) + (opa[2]*(0.00390625)) + (opa[1]*(0.001953125)) + (opa[0]*(0.0009765625)));

  
 //$display("value of %b is %22.10f", opa, res);
 deci = res;
end
endfunction*/


always @ (negedge clk or posedge(reset))
	begin
		if(reset)
			begin
				fir_cos_out <= 0;
				fir_sin_out <= 0;
				//firc_out <= 0;
				l = 0;
				for(k=0; k<43; k=k+1)
				    begin
				        shft_cos_reg[k] <= 32'b0;
			            shft_sin_reg[k] <= 32'b0;
			        end
			end
		else
			begin
				if(pushin_fir)
					begin
						//shft_cos_reg[0] <= Cos_out;
						//shft_sin_reg[0] <= Sin_out;
						//fir_cos_out <= ((f[0]*shft_cos_reg[0])/1024);
						for (j=0; j<43; j=j+1)
						  begin
						      fir_cos_temp[j] <= ((f[j]*shft_cos_reg[j])/1024);
						      fir_sin_temp[j] <= ((f[j]*shft_sin_reg[j])/1024);
						      //fir_cos_out <= ((f[j]*shft_cos_reg[j])/1024);
						      //fir_sin_out <= fir_sin_out + ((f[j]*shft_sin_reg[j])/1024);
						  end
						
					end
				if(pushin_fir_shift)
					begin
        				fir_cos_out <= fir_cos_temp[0] + fir_cos_temp[1] + fir_cos_temp[2] + fir_cos_temp[3] + fir_cos_temp[4] +
        				            fir_cos_temp[5] + fir_cos_temp[6] + fir_cos_temp[7] + fir_cos_temp[8] + fir_cos_temp[9] +
        				            fir_cos_temp[10] + fir_cos_temp[11] + fir_cos_temp[12] + fir_cos_temp[13] + fir_cos_temp[14] +
        				            fir_cos_temp[15] + fir_cos_temp[16] + fir_cos_temp[17] + fir_cos_temp[18] + fir_cos_temp[19] +
        				            fir_cos_temp[20] + fir_cos_temp[21] + fir_cos_temp[22] + fir_cos_temp[23] + fir_cos_temp[24] +
        				            fir_cos_temp[25] + fir_cos_temp[26] + fir_cos_temp[27] + fir_cos_temp[28] + fir_cos_temp[29] +
        				            fir_cos_temp[30] + fir_cos_temp[31] + fir_cos_temp[32] + fir_cos_temp[33] + fir_cos_temp[34] +
        				            fir_cos_temp[35] + fir_cos_temp[36] + fir_cos_temp[37] + fir_cos_temp[38] + fir_cos_temp[39] +
        				            fir_cos_temp[40] + fir_cos_temp[41] + fir_cos_temp[42];
        				            
        				fir_sin_out <= fir_sin_temp[0] + fir_sin_temp[1] + fir_sin_temp[2] + fir_sin_temp[3] + fir_sin_temp[4] +
        				            fir_sin_temp[5] + fir_sin_temp[6] + fir_sin_temp[7] + fir_sin_temp[8] + fir_sin_temp[9] +
        				            fir_sin_temp[10] + fir_sin_temp[11] + fir_sin_temp[12] + fir_sin_temp[13] + fir_sin_temp[14] +
        				            fir_sin_temp[15] + fir_sin_temp[16] + fir_sin_temp[17] + fir_sin_temp[18] + fir_sin_temp[19] +
        				            fir_sin_temp[20] + fir_sin_temp[21] + fir_sin_temp[22] + fir_sin_temp[23] + fir_sin_temp[24] +
        				            fir_sin_temp[25] + fir_sin_temp[26] + fir_sin_temp[27] + fir_sin_temp[28] + fir_sin_temp[29] +
        				            fir_sin_temp[30] + fir_sin_temp[31] + fir_sin_temp[32] + fir_sin_temp[33] + fir_sin_temp[34] +
        				            fir_sin_temp[35] + fir_sin_temp[36] + fir_sin_temp[37] + fir_sin_temp[38] + fir_sin_temp[39] +
        				            fir_sin_temp[40] + fir_sin_temp[41] + fir_sin_temp[42];
        				            
        				//fir_sin_out_deci <= deci(fir_sin_out);
						for (i=1; i<43; i=i+1)
							begin
								shft_cos_reg[i] <= shft_cos_reg[i-1];
								shft_sin_reg[i] <= shft_sin_reg[i-1];
							end
					end
			end
		
end

reg pushByte1;
always @ (negedge clk or posedge(reset)) 
	begin
        	if(reset) 
			begin
				pushin <= #1 0;
				NCO <= #1 0;
				Phase <= #1 32'd429496729;
				sin_nco <= #1 0;
				cos_nco <= #1 0;
				ADC1 <= #1 0;
		        peak <= 0;
		        count <= 0;
			end

		else 
			begin
				if(pushADC) ADC1 <= #1 {{22{ADC[9]}},ADC}; 
				pushin <= #1 pushADC;
				pushin_fir <= #1 pushin;
				pushin_fir_shift  <= #1 pushin_fir;
				pushin_matchfilter <= #1 pushin_fir_shift;
				pushin_mixer <= #1 pushin_fir_shift;
				pushin_nco <= #1 pushin_mixer;
				NCO <= #1 NCO_new;
				Phase <= #1 Phase_new;
				//if(send_out == 1) pushByte <= #1 pushin_nco; 
				sctable_out <= #1 scTable(NCO_new[31:22]);
				sin_nco <= #1 {{16{sctable_out[31]}}, sctable_out[31:16]}; 
				cos_nco <= #1 {{16{sctable_out[15]}}, sctable_out[15:0]}; 
		        //{sin_nco,cos_nco} <= #1 scTable(NCO[31:22]);
			end
		
	end

//module fsm(clk,reset,push,start,fcos,out_byte,last_byte);


  //input clk,reset,push,start;
  //input [31:0]fcos;
  //output[7:0]out_byte;
  //output last_byte;
  
  parameter [2:0] wait_start = 3'b000;
  parameter [2:0] count11_s  = 3'b001;
  parameter [2:0] put_d1     = 3'b010;  
  parameter [2:0] count13_s  = 3'b011;
  parameter [2:0] decode_op  = 3'b100;
  parameter [2:0] count13_s_last = 3'b101;
  //parameter [2:0] decode_op  = 3'b101;
  
  reg [3:0] count10,count10_d;
  reg [3:0] count11,count11_d;
  reg [3:0] count13,count13_d;
  reg [31:0] Byte1 [10:0];
  reg [2:0] ps,ns;
  integer m =0;
  reg[9:0] store_d,store;
  reg st, nxt_st;
  always@(negedge clk or posedge reset) //. negative clock, active low reset 
    begin
      if(reset)
        begin
          ns     <= wait_start;
          nxt_st <= 1;
          count10<=0;
          count11<=0;
          count13<=0;
        //  #1 $display("ns is is %b",ns);
          store<=0;
        end
      else
        begin
          ps      <= #1 ns;
          st <= #1 nxt_st;
          count10 <= #1 count10_d;
          count11 <= #1 count11_d;
          count13 <= #1 count13_d;
          store   <= #1 store_d;
        end  
    end
  
  always@(*)
    begin    
      count10_d = count10;
      count11_d = count11;
      count13_d = count13;
      store_d   = store ;
    //  $display("ps is %d",ps);
      if(pushin_matchfilter)
      begin
      //send_out = 0;
      pushByte = 0;
      case(ps)
       wait_start : begin
                     if(sync_enable)
                       begin
                         ns = count11_s;
                        // $display("h");
                       end
         	    else
                       begin
                     count10_d =3'd0;
         	     count11_d =4'd0;
         	     count13_d =4'd0;
                       store_d =10'd0;    
                            ns =ps;
                       end  
       		  end
        
       count11_s  : begin
         	      count13_d = 0;
         	      
         	      
         	     count11_d = count11+1;
         	     send_out = 0;
               //      $display("count11_d is %d",count11); 
         	     if(count11==4'd10)
                       begin
                         ns = put_d1;
                       end   
           	     else
                        ns=ps;
       	            end
       put_d1     : begin
                    
         	     store_d=store>>1;
                     store_d[9]= fir_cos_out[31];
         	     count11_d=0;
                     count10_d=count10+1;
                     if(count10==4'd09)
                       begin
                         ns=decode_op;
                       end
         	     else
                       ns=count13_s;
                    
      		     end
 
       count13_s  : begin
                    count13_d = count13+1;
                   // $display("count13_d is %d",count13);
                    if(count13==4'd12)
                      begin
                        ns=count11_s;
                      end  
         			
                    end
       decode_op  : begin
         	         count10_d=0;
                     ns=count13_s_last;
                     
                    Byte1[m] = deco(store);
                    m = m + 1; 
                    
                    end
       count13_s_last : begin
                        count13_d = count13+1;
                        
                        
                   // $display("count13_d is %d",count13);
                    if(count13==4'd12)
                      begin
                        ns=count11_s;
                        send_out = 1;
                        
                      end
                      end
       default    : ns=ps; 
         
      endcase   
          
    end   
  end
  
  always @(pushADC)
  begin
  if(send_out == 1)
    begin
    case(st)
        0: begin
          //pushByte = 1;
          if(m==1)
          begin
          Byte = Byte1[m-1];
          Sync = 1;
          lastByte = 0;
          end
          if(m==3)
          begin
          Byte = Byte1[m-2];
          Sync = 0;
          lastByte = 0;
          end
          if(m==5)
          begin
          Byte = Byte1[m-3];
          Sync = 0;
          lastByte = 1;
          end
          if(m==6)
          begin
          Byte = 8'h55;
          Sync = 1;
          lastByte = 1;
          end
          nxt_st = 1;
          end
        1: begin
          pushByte = 0;
          nxt_st = 0;
          end
     endcase          
   end
 end  
   
 function [7:0] deco;
   input [9:0]data;
   reg [3:0]data4; // 4b to 3b
   reg [2:0]data_op4;	// 4b to 3b
   reg [5:0]data6; // 6b to 5b
   reg [4:0]data_op6; // 6b to 5b

   begin
      data4=data[9:6]; // assign 4 bits. 
      case(data4)
      
       4'b1101: data_op4=3'h0;
       4'b1001: data_op4=3'h1;
       4'b1010: data_op4=3'h2;
       4'b0011: data_op4=3'h3;
       4'b1011: data_op4=3'h4;
       4'b0101: data_op4=3'h5;
       4'b0110: data_op4=3'h6;
       4'b0111: data_op4=3'h7;
       4'b1000: data_op4=3'h7;


       4'b0010: data_op4=3'h0;
       4'b1100: data_op4=3'h3;
       4'b0100: data_op4=3'h4;
       4'b1110: data_op4=3'h7;
       4'b0001: data_op4=3'h7;

       default :data_op4=3'h0;
   
  endcase

      data6=data[5:0];
      case(data6)
      6'b111001: data_op6=5'h0;
      6'b101110: data_op6=5'h1;
      6'b101101: data_op6=5'h2;
      6'b100011: data_op6=5'h3;
      6'b101011: data_op6=5'h4;
      6'b100101: data_op6=5'h5;
      6'b100110: data_op6=5'h6;
      6'b000111: data_op6=5'h7;
      6'b100111: data_op6=5'h8;
      6'b101001: data_op6=5'h9;
      6'b101010: data_op6=5'h10;
      6'b001011: data_op6=5'h11;
      6'b101100: data_op6=5'h12;
      6'b001101: data_op6=5'h13;
      6'b001110: data_op6=5'h14;
      6'b111010: data_op6=5'h15;
      6'b000101: data_op6=5'h15;
      6'b110110: data_op6=5'h16;
      6'b110001: data_op6=5'h17;
      6'b110010: data_op6=5'h18;
      6'b010011: data_op6=5'h19;
      6'b110100: data_op6=5'h20;
      6'b010101: data_op6=5'h21;
      6'b010110: data_op6=5'h22;
      6'b010111: data_op6=5'h23;
      6'b110011: data_op6=5'h24;
      6'b011001: data_op6=5'h25;
      6'b011010: data_op6=5'h26;
      6'b011011: data_op6=5'h27;
      6'b011100: data_op6=5'h28;
      6'b011101: data_op6=5'h29;
      6'b011110: data_op6=5'h30;
      6'b110101: data_op6=5'h31;

      6'b000110: data_op6=5'h0;
      6'b010001: data_op6=5'h1;
      6'b010010: data_op6=5'h2;
      6'b010100: data_op6=5'h4;
      6'b111000: data_op6=5'h7;
      6'b011000: data_op6=5'h8;
      6'b001001: data_op6=5'h16;
      6'b101000: data_op6=5'h23;
      6'b001100: data_op6=5'h24;
      6'b100100: data_op6=5'h27;
      6'b100010: data_op6=5'h29;
      6'b100001: data_op6=5'h30;
      6'b001010: data_op6=5'h31;

     default: data_op6=5'h0;
  endcase

deco =  {data_op4,data_op6};	
	
end
endfunction       	
   

initial begin
$dumpfile("costas.vcd");
$dumpvars(0,costas);
end

endmodule
