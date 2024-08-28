module AdcProject(
    ///////// Clocks /////////
    input logic              ADC_CLK_10,
    input logic              MAX10_CLK1_50,
    input logic              MAX10_CLK2_50,

    ///////// KEY /////////
    input logic    [ 1: 0]   KEY,

    ///////// SW /////////
    input logic    [ 9: 0]   SW,

    ///////// LEDR /////////
    output logic   [ 9: 0]   LEDR,

    ///////// HEX /////////
    output logic   [ 7: 0]   HEX0,
    output logic   [ 7: 0]   HEX1,
    output logic   [ 7: 0]   HEX2,
    output logic   [ 7: 0]   HEX3,
    output logic   [ 7: 0]   HEX4,
    output logic   [ 7: 0]   HEX5,

	 //////// GPIO - 0 ////////////////
    output logic motorPin,

    ///////// ARDUINO /////////
    inout    [15: 0]   ARDUINO_IO,
    inout              ARDUINO_RESET_N 
);



logic reset_n;
logic sys_clk;

assign reset_n = 1'b1;

// Instantiating ADC IP core
adc u0 (
    .clk_clk                              (MAX10_CLK1_50),                              
    .reset_reset_n                        (reset_n),                        
    .modular_adc_0_command_valid          (command_valid),          
    .modular_adc_0_command_channel        (command_channel),        
    .modular_adc_0_command_startofpacket  (command_startofpacket),  
    .modular_adc_0_command_endofpacket    (command_endofpacket),    
    .modular_adc_0_command_ready          (command_ready),          
    .modular_adc_0_response_valid         (response_valid),         
    .modular_adc_0_response_channel       (response_channel),       
    .modular_adc_0_response_data          (response_data),          
    .modular_adc_0_response_startofpacket (response_startofpacket), 
    .modular_adc_0_response_endofpacket   (response_endofpacket),   
    .clock_bridge_0_out_clk_clk           (sys_clk)            
);



logic command_valid;
logic [4:0] command_channel;
logic command_startofpacket;
logic command_endofpacket;
logic command_ready;


assign command_startofpacket = 1'b1;
assign command_endofpacket = 1'b1;
assign command_valid = 1'b1;
assign command_channel = SW[2:0]+1;


logic response_valid;
logic [4:0] response_channel;
logic [11:0] response_data;
logic response_startofpacket;
logic response_endofpacket;
logic [4:0] cur_adc_ch;
logic [11:0] adc_sample_data;

// FSM State Definitions
typedef enum logic [1:0] {COLD = 2'b00, HOT = 2'b01} state_t;
state_t current_state, next_state;

// Initialization of the FSM and motorPin
initial begin
    current_state = COLD;
    motorPin = 1'b0;
end

// State Transition Logic
always_ff @(posedge sys_clk) begin
    if (response_valid) begin
        adc_sample_data <= response_data;
        cur_adc_ch <= response_channel;
        
        case (current_state)
            COLD: if (adc_sample_data > 12'h9F8) next_state = HOT; // temp > 27 C
            HOT: if (adc_sample_data <= 12'h9E8) next_state = COLD; // temp <= 22
        endcase
    end
end            

// State update
always_ff @(posedge sys_clk) begin
    current_state <= next_state;
    motorPin <= (current_state == HOT);
end    

assign LEDR[9:0] = adc_sample_data[11:2];

assign HEX5[7] = 1'b1;
assign HEX4[7] = 1'b0;
assign HEX3[7] = 1'b1;
assign HEX2[7] = 1'b1;
assign HEX1[7] = 1'b1;
assign HEX0[7] = 1'b1;

SEG7_LUT SEG7_LUT_ch (
    .oSEG(HEX5),
    .iDIG(SW[2:0])
);

assign HEX4 = 8'b01111111;
assign HEX0 = 8'b11111111;

SEG7_LUT SEG7_LUT_v (
    .oSEG(HEX3),
    .iDIG(adc_sample_data[11:8])
);

SEG7_LUT SEG7_LUT_v_1 (
    .oSEG(HEX2),
    .iDIG(adc_sample_data[7:4])
);

SEG7_LUT SEG7_LUT_v_2 (
    .oSEG(HEX1),
    .iDIG(adc_sample_data[3:0])
);



endmodule
