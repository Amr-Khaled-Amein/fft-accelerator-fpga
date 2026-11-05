# command line build, same thing as Run All in the IDE
# from the fft folder:  D:\gowin\Gowin_V1.9.11.03_Education_x64\IDE\bin\gw_sh.exe build.tcl

set_device -name GW2AR-18C GW2AR-LV18QN88C8/I7

add_file src/top.v
add_file src/fft_core.v
add_file src/sample_ram.v
add_file src/twiddle_rom.v
add_file src/uart_tx.v
add_file src/uart_rx.v
add_file src/top.cst
add_file src/top.sdc

set_option -top_module top
set_option -output_base_name fft
set_option -verilog_std v2001

run all
