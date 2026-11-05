set here [file dirname [file normalize [info script]]]
set root [file dirname $here]
cd $root
set project_name [expr {[info exists ::env(WM_HLS_PROJECT)] ? $::env(WM_HLS_PROJECT) : "wm_hls_build"}]
if {[file exists $project_name]} {error "Preserve existing WM HLS build"}
open_project $project_name
set_top wm_trajectory
add_files [file join $here wm_trajectory.cpp] -cflags "-std=c++0x -ffp-contract=off"
add_files -tb [file join $here test_wm.cpp] -cflags "-std=c++0x -ffp-contract=off"
add_files -tb [file join $root verification wm_cases.bin]
open_solution solution1
set_part xc7z020clg400-1
create_clock -period 10 -name default
config_compile -unsafe_math_optimizations=false
csim_design
csynth_design
cosim_design -rtl verilog -tool xsim
export_design -format ip_catalog -rtl verilog -vendor xilinx.com -library hls -version 1.0
exit
