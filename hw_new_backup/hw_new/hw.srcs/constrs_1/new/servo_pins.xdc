# Pmod JE - Zybo Z7-20
# pwm_pan  → JE pin 1 (V12)
# pwm_tilt → JE pin 2 (W16)
# laser    → JE pin 3 (J15)
# GND     → JE pin 5
# Vcc     → JE pin 6 (사용 안 함, 외부 6V 따로)

set_property -dict { PACKAGE_PIN V12   IOSTANDARD LVCMOS33 } [get_ports { pwm_pan_0 }]
set_property -dict { PACKAGE_PIN W16   IOSTANDARD LVCMOS33 } [get_ports { pwm_tilt_0 }]
set_property -dict { PACKAGE_PIN J15   IOSTANDARD LVCMOS33 } [get_ports { laser_out_0 }]