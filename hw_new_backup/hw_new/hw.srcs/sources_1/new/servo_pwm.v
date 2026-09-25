module servo_pwm(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [22:0] pulse_ticks_pan,
    input  wire [21:0] pulse_ticks_tilt,       
    output reg         pwm_pan,
    output reg         pwm_tilt,
    output wire        laser_out
);

    // 100MHz / 50Hz = 2,000,000 cycles per period
    localparam integer PERIOD_TICKS = 2_000_000;

    reg [21:0] cnt;

    wire [21:0] pan_ticks = pulse_ticks_pan[21:0];
    wire        laser_en  = pulse_ticks_pan[22];
    assign laser_out = laser_en;
    
    
    // period counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cnt <= 22'd0;
        else if (cnt >= PERIOD_TICKS - 1)
            cnt <= 22'd0;
        else
            cnt <= cnt + 1'b1;
    end

    // PWM output (two channels)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_pan  <= 1'b0;
            pwm_tilt <= 1'b0;
        end else begin
            pwm_pan  <= (cnt < pan_ticks);
            pwm_tilt <= (cnt < pulse_ticks_tilt);
        end
    end
    

endmodule