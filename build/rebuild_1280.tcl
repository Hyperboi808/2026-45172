# rebuild_1280.tcl —— 只重建 1280 RM（preprocess 新 RTL：TILE=8/MAX_ROWS=72）
# 复用：config_full_routed.dcp（静态区+路由已锁）、stereo1280_ooc.dcp（stereo RTL 未变）
# 只重新 OOC 综合 preprocess，swap 后 place/route/pr_verify/partial 位流。
set dfx_package [file normalize [file dirname [info script]]]
set dist     [file join $dfx_package out_vision_dfx]
set rtl1280  [file join $dfx_package hls_gen rtl_1280]

if {![file exists [file join $dist config_full_routed.dcp]]} {error "config_full_routed.dcp missing"}
if {![file exists [file join $dist stereo1280_ooc.dcp]]}    {error "stereo1280_ooc.dcp missing"}
if {![file exists [file join $rtl1280 preprocess hdl verilog preprocess_accel.v]]} {error "new preprocess RTL missing"}

# ---- proc：synth_rm_ooc（改顶层模块名后 OOC 综合，匹配 BD 命名） ----
proc synth_rm_ooc {rtl_dir topfile old_module new_module outdcp} {
    set src [file join $rtl_dir $topfile]
    set f [open $src r]; set txt [read $f]; close $f
    regsub -all "module\\s+$old_module\\s*\\(" $txt "module $new_module (" txt
    set tmpsrc [file join /tmp "rm_${new_module}.v"]
    set f [open $tmpsrc w]; puts $f $txt; close $f
    set files [list]
    foreach fl [glob -nocomplain [file join $rtl_dir *.v]] {
        if {[file tail $fl] eq $topfile} {lappend files $tmpsrc} else {lappend files $fl}
    }
    read_verilog {*}$files
    synth_design -mode out_of_context -top $new_module -part xc7z020clg400-1 -flatten_hierarchy none
    write_checkpoint -force $outdcp
    close_design
    remove_files [get_files -quiet]
    puts "DFX1280: synth_rm_ooc $old_module -> $new_module -> $outdcp"
}

# ---- proc：swap_rm_pixel ----
proc swap_rm_pixel {full_dcp stereo_dcp pre_dcp} {
    open_checkpoint $full_dcp
    if {[catch {lock_design -unlock -level routing} u1]} {puts "DFX1280: unlock routing: $u1"}
    update_design -black_box -cell [lindex [get_cells -hierarchical -filter {NAME =~ *stereo_rt_accel_0}] 0]
    read_checkpoint -cell [lindex [get_cells -hierarchical -filter {NAME =~ *stereo_rt_accel_0}] 0] $stereo_dcp
    update_design -black_box -cell [lindex [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}] 0]
    read_checkpoint -cell [lindex [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}] 0] $pre_dcp
    lock_design -level routing
    return [list \
        [lindex [get_cells -hierarchical -filter {NAME =~ *stereo_rt_accel_0}] 0] \
        [lindex [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}] 0]]
}

# ============ 1) 重新 OOC 综合 preprocess（新 RTL） ============
# pre_ref 从 config_full 里取真实 REF_NAME（与 partition_def 一致）
open_checkpoint [file join $dist config_full_routed.dcp]
set pre_ref [get_property REF_NAME [lindex [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}] 0]]
close_design
puts "DFX1280: pre_ref=$pre_ref"
synth_rm_ooc [file join $rtl1280 preprocess hdl verilog] preprocess_accel.v preprocess_accel $pre_ref [file join $dist pre1280_ooc.dcp]
puts "DFX1280: pre1280_ooc.dcp ready (TILE=8/MAX_ROWS=72)"

# ============ 2) swap 1280 RM + place/route ============
set swapped [swap_rm_pixel [file join $dist config_full_routed.dcp] [file join $dist stereo1280_ooc.dcp] [file join $dist pre1280_ooc.dcp]]
set stereo_cell [lindex $swapped 0]
set pre_cell    [lindex $swapped 1]
puts "DFX1280: implementing config_1280"
place_design -directive Explore
route_design -directive Explore
report_timing_summary -file [file join $dist timing_summary_1280.rpt]
set setup_1280 [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold_1280  [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup_1280 eq "" || $hold_1280 eq "" || $setup_1280<0 || $hold_1280<0} {error "config_1280 timing failed: setup=$setup_1280 hold=$hold_1280"}
puts "DFX1280: timing OK setup=$setup_1280 hold=$hold_1280"
write_checkpoint -force [file join $dist config_1280_routed.dcp]
write_bitstream -force -cell $stereo_cell [file join $dist partial_stereo_1280.bit]
write_bitstream -force -cell $pre_cell    [file join $dist partial_pre_1280.bit]
if {[file size [file join $dist partial_stereo_1280.bit]] < 10000} {error "partial_stereo_1280.bit too small"}
if {[file size [file join $dist partial_pre_1280.bit]] < 10000}    {error "partial_pre_1280.bit too small"}
puts "DFX1280: partial_stereo_1280.bit + partial_pre_1280.bit written"
if {[catch {pr_verify [file join $dist config_full_routed.dcp] [file join $dist config_1280_routed.dcp]} pv]} {
    error "DFX1280: pr_verify full-vs-1280 FAILED: $pv"
} else {
    puts "DFX1280: pr_verify PASSED (full vs 1280)"
}
close_design
puts "DFX1280_BUILD_OK setup_1280=$setup_1280 hold_1280=$hold_1280"
