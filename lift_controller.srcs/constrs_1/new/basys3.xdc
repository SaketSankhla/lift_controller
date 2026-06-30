## basys3.xdc
## Pin Constraints for 4-Floor Elevator Controller
## Target Board: Digilent Basys 3 (Xilinx Artix-7 XC7A35T-1CPG236C)

## Clock Signal (100 MHz On-Board Oscillator)
set_property PACKAGE_PIN W5 [get_ports clk]							
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]
 
## Reset Button (Center Push Button BTNC)
set_property PACKAGE_PIN U18 [get_ports rst]						
set_property IOSTANDARD LVCMOS33 [get_ports rst]

## Slide Switches (Input Mapping)

# Floor Position Sensors (Switches SW0 to SW3)
set_property PACKAGE_PIN V17 [get_ports {floor_sensor[0]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {floor_sensor[0]}]
set_property PACKAGE_PIN V16 [get_ports {floor_sensor[1]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {floor_sensor[1]}]
set_property PACKAGE_PIN W16 [get_ports {floor_sensor[2]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {floor_sensor[2]}]
set_property PACKAGE_PIN W17 [get_ports {floor_sensor[3]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {floor_sensor[3]}]

# Hall UP Destination Buttons (Switches SW4 to SW7)
set_property PACKAGE_PIN W15 [get_ports {hall_req_up[0]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_up[0]}]
set_property PACKAGE_PIN V15 [get_ports {hall_req_up[1]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_up[1]}]
set_property PACKAGE_PIN W14 [get_ports {hall_req_up[2]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_up[2]}]
set_property PACKAGE_PIN W13 [get_ports {hall_req_up[3]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_up[3]}]

# Hall DOWN Destination Buttons (Switches SW8 to SW11)
set_property PACKAGE_PIN V2 [get_ports {hall_req_dn[0]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_dn[0]}]
set_property PACKAGE_PIN T3 [get_ports {hall_req_dn[1]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_dn[1]}]
set_property PACKAGE_PIN T2 [get_ports {hall_req_dn[2]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_dn[2]}]
set_property PACKAGE_PIN R3 [get_ports {hall_req_dn[3]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_dn[3]}]

# Cabin Floor Buttons (Switches SW12 to SW15)
set_property PACKAGE_PIN W2 [get_ports {cabin_req[0]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {cabin_req[0]}]
set_property PACKAGE_PIN U1 [get_ports {cabin_req[1]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {cabin_req[1]}]
set_property PACKAGE_PIN T1 [get_ports {cabin_req[2]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {cabin_req[2]}]
set_property PACKAGE_PIN R2 [get_ports {cabin_req[3]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {cabin_req[3]}]

# Safety Sensors (Push Buttons)
# Upper Button (BTNU) -> door_obstruction
set_property PACKAGE_PIN T18 [get_ports door_obstruction]						
set_property IOSTANDARD LVCMOS33 [get_ports door_obstruction]
# Lower Button (BTND) -> cabin_overload
set_property PACKAGE_PIN U17 [get_ports cabin_overload]						
set_property IOSTANDARD LVCMOS33 [get_ports cabin_overload]

## LEDs (Output Mapping)
# Motor state and brake release (LEDs LD0 to LD2)
set_property PACKAGE_PIN U16 [get_ports motor_up]					
set_property IOSTANDARD LVCMOS33 [get_ports motor_up]
set_property PACKAGE_PIN E19 [get_ports motor_dn]					
set_property IOSTANDARD LVCMOS33 [get_ports motor_dn]
set_property PACKAGE_PIN U19 [get_ports brake_release]					
set_property IOSTANDARD LVCMOS33 [get_ports brake_release]

# Door states (LEDs LD3 to LD4)
set_property PACKAGE_PIN V19 [get_ports door_open]					
set_property IOSTANDARD LVCMOS33 [get_ports door_open]
set_property PACKAGE_PIN W18 [get_ports door_close]					
set_property IOSTANDARD LVCMOS33 [get_ports door_close]
