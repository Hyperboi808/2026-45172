# dfx_vision.tcl —— PYNQ-Z2 四-overlay DFX 构建（3 像素档 + UART）
# Vivado 2023.2+ batch 模式（Windows）:
#   vivado -mode batch -source dfx_vision.tcl
# （必须 2023.2+：2022.1/2022.2 对 7 系列 DFX 有 0 帧 partial 工具 bug）
#
# 目标：四个可独立快速加载/卸载的 overlay（DFX partial，~200ms 级）:
#   - 像素 320（preprocess320 + stereo320，320x240，深度 + 点云）
#   - 像素 640（preprocess640 + stereo640，640x480，深度 + 点云；整片默认）
#   - 像素 1280（preprocess1280 + stereo1280，1280x720/720p，深度 + 点云；需 HLS 综合）
#   - UART（dual_pmod_uart，舵机协议，独立加载/卸载）
#   任一像素档可与 UART 并存（各自独立 RP，同时加载时两 LED 同亮）。
#
# 架构（双 RP）:
#   静态区: PS + control_smc + bm_smc + aux_smc + pointcloud_accel（分辨率无关）
#           + status_led_gpio（4 LED）+ model_rst_gpio（软件复位 smartconnect）
#   RP_A（像素槽）: stereo_rt_accel + preprocess_accel 两 cell 共享 pblock_pixel
#       RM 变体: 640 / 320 / 1280 / blank（stereo+preprocess 一起切）
#   RP_B（UART 槽）: dual_pmod_uart，独立 pblock_uart
#       RM 变体: uart / blank
#
# 地址表（沿用现有软件约定）:
#   pointcloud 0x40000000   stereo 0x40010000   preprocess 0x40020000
#   uart       0x40030000   rst_gpio 0x40050000  led      0x40060000
#
# LED 映射（4 个物理 LED，status_led[3:0]）:
#   LED[0]=320 已加载  LED[1]=640 已加载  LED[2]=1280 已加载  LED[3]=UART 已加载
#
# 产出（out_vision_dfx/）:
#   vision_base.bit            完整位流（静态 + 640 + uart；功能 = 640 深度 + UART 模式）
#   partial_pixel_640.bit      像素槽 RM640 部分位流（stereo+preprocess 合并）
#   partial_pixel_320.bit      像素槽 RM320 部分位流
#   partial_pixel_1280.bit     像素槽 RM1280 部分位流（若 HLS 网表存在）
#   partial_pixel_blank.bit    像素槽 blank 部分位流（卸载像素）
#   partial_uart.bit           UART 槽 RM uart 部分位流
#   partial_uart_blank.bit     UART 槽 blank 部分位流（卸载 UART）
#   partial_manifest.json      sha256 清单（板端校验用）
#   *.rpt                      时序/DRC/利用率报告
#
# 前提: 与本脚本同目录下
#   hw/hw320/            —— 320 版网表 ip_repo + board_files + xdc + dual_pmod_uart.v
#   hardware_sources/hardware/  —— 640 版网表 ip_repo（stereo/preprocess）
#   hls_gen/rtl_1280/    —— （阶段 2 可选）1280 版 stereo/preprocess RTL
#
# 注意:
#   - PBLOCK_PIXEL / PBLOCK_UART 是占位初值；如 place/pr_verify 失败，
#     按 utilization_hierarchical.rpt 里 stereo/preprocess/uart 的实际资源调整。
#   - 1280 档（720p）资源/时序能否收敛须实际综合+实现才知道，脚本检测到
#     hls_gen/rtl_1280 下的网表才会构建，否则自动跳过。
#   - 所有部分位流都从同一 config_full 静态导出；任何静态修改必须全量重建。
#   - blank 后软件层绝不读该 RP 的地址（见 README 安全规则）。

set dfx_package [file normalize [file dirname [info script]]]
set hw320    [file join $dfx_package hw hw320]
set hw640    [file join $dfx_package hardware_sources hardware]
set build    [file join $dfx_package build_vision_dfx]
set dist     [file join $dfx_package out_vision_dfx]
set rtl1280  [file join $dfx_package hls_gen rtl_1280]
if {[file exists $build] || [file exists $dist]} {error "Preserve existing build; use a fresh copy of this package"}

# config_full 的 640 baseline IP + 常驻 pointcloud（分辨率无关，只此一份在 hw320）
set repos [list \
    [file join $hw640 ip_repo stereo] \
    [file join $hw640 ip_repo preprocess] \
    [file join $hw320 ip_repo pointcloud]]
foreach repo $repos {if {![file exists [file join $repo component.xml]]} {error "Missing bundled HLS IP: $repo"}}
file mkdir $build
file mkdir $dist

# ---- RP Pblock（xc7z020 器件网格，get_rp_rects 实测方法）----
# 器件为阶梯形楼层图：中间列（SLICE_X27-67，tile X19-43）Y0-149 全高，
# 两侧列只有 Y0-49。像素 RP 是资源大头（stereo640 的 StereoBM 为 BRAM 密集），
# 取中间大块；UART RP 极小（2 通道 UART + FIFO），取左侧上小块。
# 占位初值，按 utilization_hierarchical.rpt 调整。
proc get_rp_rects {xmin xmax ymin ymax} {
    set tiles [list]
    foreach tl [get_tiles -filter "TILE_TYPE =~ CLB* || TILE_TYPE =~ BRAM* || TILE_TYPE =~ DSP*"] {
        if {[string match *INT* $tl]} {continue}
        regexp {X(\d+)Y(\d+)$} $tl -> tx ty
        if {$tx >= $xmin && $tx <= $xmax && $ty >= $ymin && $ty <= $ymax} {lappend tiles $tl}
    }
    set sites [list]
    foreach tl $tiles {
        foreach s [get_sites -quiet -of_objects $tl] {
            if {[regexp {^(SLICE|RAMB36|RAMB18|FIFO18E1|RAMBFIFO36E1|DSP48)_} $s]} {lappend sites $s}
        }
    }
    set cols [dict create]
    foreach s $sites {
        regexp {^(.*_X\d+)Y\d+$} $s -> prefix
        dict lappend cols $prefix $s
    }
    set rects [list]
    foreach prefix [lsort [dict keys $cols]] {
        set ss [lsort -dictionary [dict get $cols $prefix]]
        lappend rects "[lindex $ss 0]:[lindex $ss end]"
    }
    puts "DFX: RP rects ($xmin-$xmax x $ymin-$ymax): $rects"
    return $rects
}
# ================= Phase 0: UART RM 网表（RTL module，无 BD OOC 产物） =================
# dual_pmod_uart 是 -type module 的 RTL（非 IP），synth_1 里被并进 wrapper，不单独 OOC。
# 在 create_project 前用非项目模式自建 OOC 网表（顶层名 = dual_pmod_uart），
# 供 config_full 换入（避免项目模式 remove_files 污染 sources_1 fileset）。
puts "DFX: synthesizing uart RM netlist (OOC from dual_pmod_uart.v)"
read_verilog [file join $hw320 dual_pmod_uart.v]
synth_design -mode out_of_context -top dual_pmod_uart -part xc7z020clg400-1 -flatten_hierarchy none
write_checkpoint -force [file join $dist uart_ooc.dcp]
close_design
puts "DFX: uart RM OOC netlist ready"

# ================= Phase 1: 建工程 + BD（config_full = 640 + uart） =================
set_param board.repoPaths [list [file join $hw320 board_files pynq-z2]]
create_project pa_vision $build -part xc7z020clg400-1
set_property board_part tul.com.tw:pynq-z2:part0:1.0 [current_project]
set_property PR_FLOW 1 [current_project]
set_property ip_repo_paths $repos [current_project]
update_ip_catalog
add_files -norecurse [file join $hw320 dual_pmod_uart.v]
update_compile_order -fileset sources_1

create_bd_design stereo_design
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external {FIXED_IO, DDR} apply_board_preset {1} Master {Disable} Slave {Disable}} $ps
set_property -dict [list CONFIG.PCW_USE_M_AXI_GP0 {1} CONFIG.PCW_USE_S_AXI_HP0 {1} CONFIG.PCW_USE_S_AXI_HP1 {1} CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100.000000}] $ps

# 像素 RP 的两个 cell（640 baseline）+ 常驻 pointcloud + UART RP
set bm    [create_bd_cell -type ip -vlnv xilinx.com:hls:stereo_rt_accel:1.0 stereo_rt_accel_0]
set pre   [create_bd_cell -type ip -vlnv xilinx.com:hls:preprocess_accel:1.0 preprocess_accel_0]
set cloud [create_bd_cell -type ip -vlnv xilinx.com:hls:pointcloud_accel:1.0 pointcloud_accel_0]
set uart  [create_bd_cell -type module -reference dual_pmod_uart dual_pmod_uart_0]

# 互联
set control [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 control_smc]
set bmdata  [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 bm_smc]
set auxdata [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 aux_smc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {6}] $control
set_property -dict [list CONFIG.NUM_SI {4} CONFIG.NUM_MI {1} CONFIG.STRATEGY {1}] $bmdata
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1} CONFIG.STRATEGY {1}] $auxdata

# 控制面（GP0 -> 各 IP s_axi_control）
connect_bd_intf_net [get_bd_intf_pins $ps/M_AXI_GP0] [get_bd_intf_pins $control/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins $control/M00_AXI] [get_bd_intf_pins $bm/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins $control/M01_AXI] [get_bd_intf_pins $pre/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins $control/M02_AXI] [get_bd_intf_pins $cloud/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins $control/M03_AXI] [get_bd_intf_pins $uart/S_AXI]

# 数据面（HP -> m_axi）
for {set i 0} {$i<4} {incr i} {connect_bd_intf_net [get_bd_intf_pins $bm/m_axi_gmem$i] [get_bd_intf_pins $bmdata/S0${i}_AXI]}
connect_bd_intf_net [get_bd_intf_pins $cloud/m_axi_memory] [get_bd_intf_pins $auxdata/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins $pre/m_axi_memory] [get_bd_intf_pins $auxdata/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins $bmdata/M00_AXI] [get_bd_intf_pins $ps/S_AXI_HP0]
connect_bd_intf_net [get_bd_intf_pins $auxdata/M00_AXI] [get_bd_intf_pins $ps/S_AXI_HP1]

# 复位 + 时钟
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_100m]
set zero [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const0]
set one [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const1]
set_property CONFIG.CONST_VAL 0 $zero
set_property CONFIG.CONST_VAL 1 $one

# 四个状态 LED（物理 LD0..LD3）
set led [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 status_led_gpio]
set_property -dict [list CONFIG.C_GPIO_WIDTH {4} CONFIG.C_ALL_OUTPUTS {1} CONFIG.C_DOUT_DEFAULT {0x00000000}] $led

# 复位 GPIO（1 bit 输出，默认 1 = 不复位）：partial 重配置后软件写 0 脉冲复位
# stereo/pre/uart 的 ap_rst_n + bm_smc/aux_smc，清 DFX 期间 m_axi 翻转导致的
# smartconnect 死锁（UG909 Avoiding Deadlock）。
set mr_gpio [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 model_rst_gpio]
set_property -dict [list CONFIG.C_GPIO_WIDTH {1} CONFIG.C_ALL_OUTPUTS {1} CONFIG.C_DOUT_DEFAULT {0x00000001}] $mr_gpio

# 复位 AND：model_rst_n = peripheral_aresetn AND gpio_io_o。默认 1 跟随，软件写 0 强制复位。
set mr_and [create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic:2.0 model_rst_and]
set_property -dict [list CONFIG.C_OPERATION {and} CONFIG.C_SIZE {1}] $mr_and

# 时钟（FCLK_CLK0 100MHz -> 全部）
connect_bd_net [get_bd_pins $ps/FCLK_CLK0] [get_bd_pins $ps/M_AXI_GP0_ACLK] [get_bd_pins $ps/S_AXI_HP0_ACLK] [get_bd_pins $ps/S_AXI_HP1_ACLK] [get_bd_pins $bm/ap_clk] [get_bd_pins $pre/ap_clk] [get_bd_pins $cloud/ap_clk] [get_bd_pins $uart/s_axi_aclk] [get_bd_pins $control/aclk] [get_bd_pins $led/s_axi_aclk] [get_bd_pins $mr_gpio/s_axi_aclk] [get_bd_pins $rst/slowest_sync_clk]
foreach cell [list $bmdata $auxdata] {
    foreach pin [get_bd_pins -of_objects $cell -filter {NAME =~ *ACLK}] {connect_bd_net [get_bd_pins $ps/FCLK_CLK0] $pin}
}

# 复位输入（proc_sys_reset）
connect_bd_net [get_bd_pins $ps/FCLK_RESET0_N] [get_bd_pins $rst/ext_reset_in]
connect_bd_net [get_bd_pins $zero/dout] [get_bd_pins $rst/mb_debug_sys_rst]
connect_bd_net [get_bd_pins $one/dout] [get_bd_pins $rst/aux_reset_in] [get_bd_pins $rst/dcm_locked]

# 静态区复位（直接 peripheral_aresetn）：pointcloud / control / led / rst_gpio
connect_bd_net [get_bd_pins $rst/peripheral_aresetn] [get_bd_pins $cloud/ap_rst_n] [get_bd_pins $control/aresetn] [get_bd_pins $led/s_axi_aresetn] [get_bd_pins $mr_gpio/s_axi_aresetn]

# 软件可控复位（AND 门）：RP 的 ap_rst_n/s_axi_aresetn + 数据面 smartconnect 全部 ARESETN。
# 像素槽 stereo/pre 与 UART 槽 uart 重配置后 m_axi/s_axi 信号翻转，须软件脉冲复位清死锁。
connect_bd_net [get_bd_pins $rst/peripheral_aresetn] [get_bd_pins $mr_and/Op1]
connect_bd_net [get_bd_pins $mr_gpio/gpio_io_o] [get_bd_pins $mr_and/Op2]
connect_bd_net [get_bd_pins $mr_and/Res] [get_bd_pins $bm/ap_rst_n] [get_bd_pins $pre/ap_rst_n] [get_bd_pins $uart/s_axi_aresetn]
foreach cell [list $bmdata $auxdata] {
    foreach pin [get_bd_pins -of_objects $cell -filter {NAME =~ *ARESETN}] {connect_bd_net [get_bd_pins $mr_and/Res] $pin}
}

# 顶层端口
make_bd_pins_external [get_bd_pins $uart/uart_rx] [get_bd_pins $uart/uart_tx]
set_property name uart_rx [get_bd_ports uart_rx_0]
set_property name uart_tx [get_bd_ports uart_tx_0]
make_bd_pins_external [get_bd_pins $led/gpio_io_o]
set_property name status_led [get_bd_ports gpio_io_o_0]

# 控制面地址分配（沿用现有软件约定）
connect_bd_intf_net [get_bd_intf_pins $control/M04_AXI] [get_bd_intf_pins $led/S_AXI]
connect_bd_intf_net [get_bd_intf_pins $control/M05_AXI] [get_bd_intf_pins $mr_gpio/S_AXI]
assign_bd_address -offset 0x40000000 -range 64K -target_address_space [get_bd_addr_spaces $ps/Data] [get_bd_addr_segs $cloud/s_axi_control/Reg]
assign_bd_address -offset 0x40010000 -range 64K -target_address_space [get_bd_addr_spaces $ps/Data] [get_bd_addr_segs $bm/s_axi_control/Reg]
assign_bd_address -offset 0x40020000 -range 64K -target_address_space [get_bd_addr_spaces $ps/Data] [get_bd_addr_segs $pre/s_axi_control/Reg]
set uart_segs [get_bd_addr_segs -of_objects [get_bd_intf_pins $uart/S_AXI]]
if {[llength $uart_segs]!=1} {error "Unexpected UART address segment"}
assign_bd_address -offset 0x40030000 -range 4K -target_address_space [get_bd_addr_spaces $ps/Data] $uart_segs
assign_bd_address -offset 0x40060000 -range 4K -target_address_space [get_bd_addr_spaces $ps/Data] [get_bd_addr_segs $led/S_AXI/Reg]
assign_bd_address -offset 0x40050000 -range 4K -target_address_space [get_bd_addr_spaces $ps/Data] [get_bd_addr_segs $mr_gpio/S_AXI/Reg]

assign_bd_address
validate_bd_design
save_bd_design
set bd [get_files stereo_design.bd]
generate_target all $bd
make_wrapper -files $bd -top
add_files -norecurse [file join $build pa_vision.gen sources_1 bd stereo_design hdl stereo_design_wrapper.v]
add_files -fileset constrs_1 -norecurse [file join $hw320 pmod_uart.xdc]
add_files -fileset constrs_1 -norecurse [file join $hw320 status_led.xdc]
set_property top stereo_design_wrapper [current_fileset]
update_compile_order -fileset sources_1

set_param general.maxThreads 4
set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} -object [get_runs synth_1] -value {-flatten_hierarchy none}

# ================= Phase 2: 综合 config_full + DFX 元数据 =================
# 标准综合 run（BD 每个 IP 由 launch_runs 自动 OOC 综合成 DCP 后读入）
puts "DFX: synthesizing config_full via synth_1 run"
launch_runs synth_1 -jobs 4
wait_on_run synth_1
open_run synth_1
write_checkpoint -force [file join $dist synth_full.dcp]
puts "DFX: synth_full + uart_ooc done; stopping here (resume_dfx.tcl handles Phase 3-7)"
return

# 三个独立 RP pblock（每个 partition 一个 pblock，DRC HDPR-12）
# pre（preprocess_accel）实测只占 20 个 RAMB36，Y0-49 时钟区域 tile 6/17/22 各 10 个=30 个
# 足够。之前误给 pre 整列 RAMB（tile22 Y0-149），Y50-149 的 RAMB 把 pre 配置帧拉宽到 Y50-99
# 覆盖 X26-30 SLICE，静态逻辑落进去换 RM 后报 HDPR-6/7。故 pre RAMB 收窄到 Y0-49。
# pre 的 CLB 须覆盖 tile 0-21 与 RAMB tile22 连续（配置帧连续覆盖到 tile22）。
# stereo = X23-43(5400 slice + BRAM X36 + DSP X25) + 右下 X68-79(600) + 右上 X80-92(1950)
#   = 7950 slice 覆盖实测 7553；SLICEM 2250 覆盖 2097。
set PBLOCK_PRE    [concat [get_rp_rects 0 21 0 49] [get_rp_rects 22 22 0 49]]
set PBLOCK_STEREO [concat [get_rp_rects 23 43 0 149] \
    [list {SLICE_X68Y0:SLICE_X79Y49} {SLICE_X80Y0:SLICE_X92Y149}]]
# uart 有 96 LUT 分布式 RAM（RAMD64E，必须 SLICEM）。右下 X68-79 全 SLICEL（无 SLICEM），
# 故 uart 放右上 SLICE_X93-99（含 SLICEM X94/X96/X98）。X100-113 留给静态区 pointcloud。
set PBLOCK_UART   [list {SLICE_X93Y0:SLICE_X99Y149}]

# 定位三个 RP cell，黑盒化 + create_partition_def（DFX 元数据，0 帧根因修复）
set scells [get_cells -hierarchical -filter {NAME =~ *stereo_rt_accel_0}]
set pcells [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}]
set ucells [get_cells -hierarchical -filter {NAME =~ *dual_pmod_uart_0}]
if {[llength $scells] != 1} {error "stereo_rt_accel_0 cells: $scells"}
if {[llength $pcells] != 1} {error "preprocess_accel_0 cells: $pcells"}
if {[llength $ucells] != 1} {error "dual_pmod_uart_0 cells: $ucells"}
set stereo_cell [lindex $scells 0]
set pre_cell    [lindex $pcells 0]
set uart_cell   [lindex $ucells 0]
puts "DFX: RP cells stereo=$stereo_cell pre=$pre_cell uart=$uart_cell"

# 黑盒化三个 cell + create_partition_def（dfx320 实测顺序：记录 ref -> black_box -> 重取 -> HD.RECONFIGURABLE -> partition_def）
set stereo_ref_orig [get_property REF_NAME $stereo_cell]
set pre_ref_orig    [get_property REF_NAME $pre_cell]
set uart_ref_orig   [get_property REF_NAME $uart_cell]
update_design -black_box -cell $stereo_cell
update_design -black_box -cell $pre_cell
update_design -black_box -cell $uart_cell
set stereo_cell [lindex [get_cells -hierarchical -filter {NAME =~ *stereo_rt_accel_0}] 0]
set pre_cell    [lindex [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}] 0]
set uart_cell   [lindex [get_cells -hierarchical -filter {NAME =~ *dual_pmod_uart_0}] 0]
set_property HD.RECONFIGURABLE 1 $stereo_cell
set_property HD.RECONFIGURABLE 1 $pre_cell
set_property HD.RECONFIGURABLE 1 $uart_cell
create_partition_def -name pd_stereo     -module $stereo_ref_orig
create_partition_def -name pd_preprocess -module $pre_ref_orig
create_partition_def -name pd_uart       -module $uart_ref_orig
puts "DFX: partition_defs = [get_partition_defs]"

# ---- blank RM stub 生成器（写 synth_stub + 输出拉安全电平） ----
# AXI-Lite 从机握手（*AWREADY/*WREADY/*ARREADY/*BVALID/*RVALID，大小写不敏感）拉 1
# （读返回 0，PS 访问不挂总线）；其余输出（m_axi *_VALID/*ADDR 等）拉 0（master 空闲）。
proc make_blank_stub {cell outfile} {
    write_verilog -force -mode synth_stub -cell $cell $outfile
    set f [open $outfile r]; set txt [read $f]; close $f
    set outs [list]
    foreach line [split $txt "\n"] {
        if {[regexp {^\s*output\s+(\[\d+:\d+\])?\s*(\w+)\s*;} $line -> bw name]} {lappend outs $name}
    }
    if {[llength $outs] == 0} {error "make_blank_stub: no output ports in $outfile"}
    set assigns ""
    foreach name $outs {
        if {[regexp {(?i)(AWREADY|WREADY|ARREADY|BVALID|RVALID)$} $name]} {
            append assigns "  assign $name = 1'b1;\n"
        } else {
            append assigns "  assign $name = 1'b0;\n"
        }
    }
    set idx [string last "endmodule" $txt]
    if {$idx < 0} {error "make_blank_stub: endmodule not found"}
    set txt "[string range $txt 0 [expr {$idx - 1}]]$assigns[string range $txt $idx end]"
    set f [open $outfile w]; puts $f $txt; close $f
    puts "DFX: blank stub -> $outfile ([llength $outs] outputs)"
}

# ================= Phase 3: config_full 实现（640 + uart） =================
# 递归找 640 stereo/preprocess RM 的 OOC DCP（BD IP OOC 综合产物，在 .gen 下）
proc find_rm_dcp {root pat} {
    set out [list]
    foreach f [glob -nocomplain -directory $root *] {
        if {[file isdirectory $f]} {set out [concat $out [find_rm_dcp $f $pat]]} \
        elseif {[string match $pat $f]} {lappend out $f}
    }
    return $out
}
set stereo640_dcps [glob -nocomplain [file join $build pa_vision.runs *stereo_rt_accel*_synth_1 *.dcp]]
set pre640_dcps    [glob -nocomplain [file join $build pa_vision.runs *preprocess_accel*_synth_1 *.dcp]]
puts "DFX: 640 RM DCP candidates stereo=[llength $stereo640_dcps] pre=[llength $pre640_dcps]"
if {[llength $stereo640_dcps] == 0} {error "stereo 640 RM DCP not found"}
if {[llength $pre640_dcps] == 0}    {error "preprocess 640 RM DCP not found"}
# 每个 read_checkpoint 前重新 get_cells（read_checkpoint 使旧句柄失效，Vivado 12-12435）
read_checkpoint -cell [lindex [get_cells -hierarchical -filter {NAME =~ *stereo_rt_accel_0}] 0] [lindex $stereo640_dcps 0]
read_checkpoint -cell [lindex [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}] 0] [lindex $pre640_dcps 0]
read_checkpoint -cell [lindex [get_cells -hierarchical -filter {NAME =~ *dual_pmod_uart_0}] 0] [file join $dist uart_ooc.dcp]
# read_checkpoint 重建 cell，旧句柄失效，重新获取
set stereo_cell [lindex [get_cells -hierarchical -filter {NAME =~ *stereo_rt_accel_0}] 0]
set pre_cell    [lindex [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}] 0]
set uart_cell   [lindex [get_cells -hierarchical -filter {NAME =~ *dual_pmod_uart_0}] 0]

# 三个独立 RP pblock（每个 partition 一个 pblock，DRC HDPR-12）
create_pblock pblock_stereo
add_cells_to_pblock [get_pblocks pblock_stereo] [list $stereo_cell]
foreach r $PBLOCK_STEREO {resize_pblock [get_pblocks pblock_stereo] -add $r}
set_property SNAPPING_MODE ON [get_pblocks pblock_stereo]
set_property RESET_AFTER_RECONFIG TRUE [get_pblocks pblock_stereo]

create_pblock pblock_pre
add_cells_to_pblock [get_pblocks pblock_pre] [list $pre_cell]
foreach r $PBLOCK_PRE {resize_pblock [get_pblocks pblock_pre] -add $r}
set_property SNAPPING_MODE ON [get_pblocks pblock_pre]
set_property RESET_AFTER_RECONFIG TRUE [get_pblocks pblock_pre]

create_pblock pblock_uart
add_cells_to_pblock [get_pblocks pblock_uart] [list $uart_cell]
foreach r $PBLOCK_UART {resize_pblock [get_pblocks pblock_uart] -add $r}
set_property SNAPPING_MODE ON [get_pblocks pblock_uart]
set_property RESET_AFTER_RECONFIG TRUE [get_pblocks pblock_uart]

puts "DFX: implementing config_full (640 + uart)"
opt_design -directive ExploreArea
place_design -directive Explore
phys_opt_design -directive ExploreWithAggressiveHoldFix
route_design -directive Explore
report_timing_summary -file [file join $dist timing_summary_full.rpt]
report_drc -file [file join $dist drc_full.rpt]
report_utilization -hierarchical -file [file join $dist utilization_hierarchical.rpt]
set setup [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold  [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup eq "" || $hold eq "" || $setup<0 || $hold<0} {error "config_full timing failed: setup=$setup hold=$hold"}
puts "DFX: config_full timing OK setup=$setup hold=$hold"

lock_design -level routing
write_checkpoint -force [file join $dist config_full_routed.dcp]
write_bitstream -force [file join $dist vision_base.bit]
puts "DFX: full bitstream written"
write_bitstream -force -cell $stereo_cell -cell $pre_cell [file join $dist partial_pixel_640.bit]
write_bitstream -force -cell $uart_cell [file join $dist partial_uart.bit]
if {[file size [file join $dist partial_pixel_640.bit]] < 10000} {error "partial_pixel_640.bit too small: DFX frame derivation broken"}
if {[file size [file join $dist partial_uart.bit]] < 10000} {error "partial_uart.bit too small"}
puts "DFX: partial_pixel_640.bit + partial_uart.bit written"

# 提前准备三个 blank stub（stereo/preprocess/uart）
make_blank_stub $stereo_cell [file join $dist blank_stereo_stub.v]
make_blank_stub $pre_cell    [file join $dist blank_pre_stub.v]
make_blank_stub $uart_cell   [file join $dist blank_uart_stub.v]
# UART blank 时 TX 保持空闲高电平（2'b11），否则 TX 线被拉低
set fu [open [file join $dist blank_uart_stub.v] r]; set tu [read $fu]; close $fu
set tu [regsub -all {assign uart_tx = 1'b0;} $tu {assign uart_tx = 2'b11;}]
set fu [open [file join $dist blank_uart_stub.v] w]; puts $fu $tu; close $fu
set stereo_ref [get_property REF_NAME $stereo_cell]
set pre_ref    [get_property REF_NAME $pre_cell]
set uart_ref   [get_property REF_NAME $uart_cell]

# blank RM OOC 综合（非项目模式）
close_design
close_project
proc synth_stub_ooc {stub top outdcp} {
    read_verilog $stub
    synth_design -mode out_of_context -top $top -part xc7z020clg400-1 -flatten_hierarchy none
    write_checkpoint -force $outdcp
    close_design
    remove_files [get_files -quiet]
}
synth_stub_ooc [file join $dist blank_stereo_stub.v] $stereo_ref [file join $dist blank_stereo_ooc.dcp]
synth_stub_ooc [file join $dist blank_pre_stub.v]    $pre_ref    [file join $dist blank_pre_ooc.dcp]
synth_stub_ooc [file join $dist blank_uart_stub.v]   $uart_ref   [file join $dist blank_uart_ooc.dcp]
puts "DFX: blank RM OOC netlists ready"

# ---- RM 换入流程（照搬 dfx320 实测：unlock -> black_box -> read_checkpoint -> lock -> place/route）----
proc swap_rm_pixel {full_dcp stereo_dcp pre_dcp} {
    open_checkpoint $full_dcp
    if {[catch {lock_design -unlock -level routing} u1]} {puts "DFX: unlock routing: $u1"}
    if {[catch {lock_design -unlock -level placement} u2]} {puts "DFX: unlock placement: $u2"}
    # 每个 read_checkpoint 前重新 get_cells（read_checkpoint 使旧句柄失效）
    update_design -black_box -cell [lindex [get_cells -hierarchical -filter {NAME =~ *stereo_rt_accel_0}] 0]
    read_checkpoint -cell [lindex [get_cells -hierarchical -filter {NAME =~ *stereo_rt_accel_0}] 0] $stereo_dcp
    update_design -black_box -cell [lindex [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}] 0]
    read_checkpoint -cell [lindex [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}] 0] $pre_dcp
    lock_design -level routing
    return [list \
        [lindex [get_cells -hierarchical -filter {NAME =~ *stereo_rt_accel_0}] 0] \
        [lindex [get_cells -hierarchical -filter {NAME =~ *preprocess_accel_0}] 0]]
}

proc swap_rm_uart {full_dcp uart_dcp} {
    open_checkpoint $full_dcp
    set uc [lindex [get_cells -hierarchical -filter {NAME =~ *dual_pmod_uart_0}] 0]
    if {[catch {lock_design -unlock -level routing} u1]} {puts "DFX: unlock routing: $u1"}
    if {[catch {lock_design -unlock -level placement} u2]} {puts "DFX: unlock placement: $u2"}
    update_design -black_box -cell [get_cells $uc]
    read_checkpoint -cell [get_cells $uc] $uart_dcp
    lock_design -level routing
    return [lindex [get_cells -hierarchical -filter {NAME =~ *dual_pmod_uart_0}] 0]
}

# ================= Phase 4: RM320（真实网表换入） =================
puts "DFX: synthesizing 320 RM netlists (OOC from hw320 hdl)"
read_verilog {*}[glob -nocomplain [file join $hw320 ip_repo stereo hdl verilog *.v]]
synth_design -mode out_of_context -top $stereo_ref -part xc7z020clg400-1 -flatten_hierarchy none
write_checkpoint -force [file join $dist stereo320_ooc.dcp]
close_design
remove_files [get_files -quiet]
read_verilog {*}[glob -nocomplain [file join $hw320 ip_repo preprocess hdl verilog *.v]]
synth_design -mode out_of_context -top $pre_ref -part xc7z020clg400-1 -flatten_hierarchy none
write_checkpoint -force [file join $dist pre320_ooc.dcp]
close_design
remove_files [get_files -quiet]
puts "DFX: 320 RM OOC netlists ready"

set swapped [swap_rm_pixel [file join $dist config_full_routed.dcp] [file join $dist stereo320_ooc.dcp] [file join $dist pre320_ooc.dcp]]
set stereo_cell [lindex $swapped 0]
set pre_cell    [lindex $swapped 1]
puts "DFX: implementing config_320"
place_design -directive Explore
route_design -directive Explore
report_timing_summary -file [file join $dist timing_summary_320.rpt]
set setup_320 [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold_320  [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup_320 eq "" || $hold_320 eq "" || $setup_320<0 || $hold_320<0} {error "config_320 timing failed: setup=$setup_320 hold=$hold_320"}
write_checkpoint -force [file join $dist config_320_routed.dcp]
write_bitstream -force -cell $stereo_cell -cell $pre_cell [file join $dist partial_pixel_320.bit]
if {[file size [file join $dist partial_pixel_320.bit]] < 10000} {error "partial_pixel_320.bit too small"}
puts "DFX: partial_pixel_320.bit written"
if {[catch {pr_verify [file join $dist config_full_routed.dcp] [file join $dist config_320_routed.dcp]} pv]} {error "pr_verify full-vs-320 FAILED: $pv"}
puts "DFX: pr_verify PASSED (full vs 320)"
close_design

# ================= Phase 5: RM1280（可选，若 HLS 网表存在） =================
set has_1280 [expr {[file exists [file join $rtl1280 stereo hdl verilog stereo_rt_accel.v]] && \
                     [file exists [file join $rtl1280 preprocess hdl verilog preprocess_accel.v]]}]
if {$has_1280} {
    puts "DFX: synthesizing 1280 RM netlists (OOC from hls_gen/rtl_1280)"
    read_verilog {*}[glob -nocomplain [file join $rtl1280 stereo hdl verilog *.v]]
    synth_design -mode out_of_context -top $stereo_ref -part xc7z020clg400-1 -flatten_hierarchy none
    write_checkpoint -force [file join $dist stereo1280_ooc.dcp]
    close_design
    remove_files [get_files -quiet]
    read_verilog {*}[glob -nocomplain [file join $rtl1280 preprocess hdl verilog *.v]]
    synth_design -mode out_of_context -top $pre_ref -part xc7z020clg400-1 -flatten_hierarchy none
    write_checkpoint -force [file join $dist pre1280_ooc.dcp]
    close_design
    remove_files [get_files -quiet]
    set swapped [swap_rm_pixel [file join $dist config_full_routed.dcp] [file join $dist stereo1280_ooc.dcp] [file join $dist pre1280_ooc.dcp]]
    set stereo_cell [lindex $swapped 0]
    set pre_cell    [lindex $swapped 1]
    puts "DFX: implementing config_1280"
    place_design -directive Explore
    route_design -directive Explore
    report_timing_summary -file [file join $dist timing_summary_1280.rpt]
    set setup_1280 [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
    set hold_1280  [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
    if {$setup_1280 eq "" || $hold_1280 eq "" || $setup_1280<0 || $hold_1280<0} {error "config_1280 timing failed: setup=$setup_1280 hold=$hold_1280"}
    write_checkpoint -force [file join $dist config_1280_routed.dcp]
    write_bitstream -force -cell $stereo_cell -cell $pre_cell [file join $dist partial_pixel_1280.bit]
    if {[file size [file join $dist partial_pixel_1280.bit]] < 10000} {error "partial_pixel_1280.bit too small"}
    puts "DFX: partial_pixel_1280.bit written"
    if {[catch {pr_verify [file join $dist config_full_routed.dcp] [file join $dist config_1280_routed.dcp]} pv]} {error "pr_verify full-vs-1280 FAILED: $pv"}
    puts "DFX: pr_verify PASSED (full vs 1280)"
    close_design
} else {
    puts "DFX: WARNING 1280 RM netlist not found under $rtl1280; skipping 1280 config"
}

# ================= Phase 6: blank RM（像素 blank + uart blank） =================
# 像素 blank（stereo + preprocess 都换 blank）
set swapped [swap_rm_pixel [file join $dist config_full_routed.dcp] [file join $dist blank_stereo_ooc.dcp] [file join $dist blank_pre_ooc.dcp]]
set stereo_cell [lindex $swapped 0]
set pre_cell    [lindex $swapped 1]
puts "DFX: implementing config_pixel_blank"
place_design
route_design
report_timing_summary -file [file join $dist timing_summary_pixel_blank.rpt]
set setup_pb [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold_pb  [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup_pb eq "" || $hold_pb eq "" || $setup_pb<0 || $hold_pb<0} {error "config_pixel_blank timing failed"}
write_checkpoint -force [file join $dist config_pixel_blank_routed.dcp]
write_bitstream -force -cell $stereo_cell -cell $pre_cell [file join $dist partial_pixel_blank.bit]
puts "DFX: partial_pixel_blank.bit written"
if {[catch {pr_verify [file join $dist config_full_routed.dcp] [file join $dist config_pixel_blank_routed.dcp]} pv]} {error "pr_verify full-vs-pixel_blank FAILED: $pv"}
puts "DFX: pr_verify PASSED (full vs pixel blank)"
close_design

# UART blank
set uart_cell [swap_rm_uart [file join $dist config_full_routed.dcp] [file join $dist blank_uart_ooc.dcp]]
puts "DFX: implementing config_uart_blank"
place_design
route_design
report_timing_summary -file [file join $dist timing_summary_uart_blank.rpt]
set setup_ub [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold_ub  [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup_ub eq "" || $hold_ub eq "" || $setup_ub<0 || $hold_ub<0} {error "config_uart_blank timing failed"}
write_checkpoint -force [file join $dist config_uart_blank_routed.dcp]
write_bitstream -force -cell $uart_cell [file join $dist partial_uart_blank.bit]
puts "DFX: partial_uart_blank.bit written"
if {[catch {pr_verify [file join $dist config_full_routed.dcp] [file join $dist config_uart_blank_routed.dcp]} pv]} {error "pr_verify full-vs-uart_blank FAILED: $pv"}
puts "DFX: pr_verify PASSED (full vs uart blank)"
close_design

# ================= Phase 7: manifest（板端 switch_fast.py 校验） =================
proc sha256_file {path} {
    # Linux: sha256sum 输出 "<64位hex>  <文件名>"
    if {[catch {exec sha256sum $path} out]} {
        error "sha256sum failed for $path: $out"
    }
    set hex ""
    if {[regexp {^([0-9a-fA-F]{64})\s} $out -> h]} {set hex [string tolower $h]}
    if {$hex eq ""} {error "sha256sum parse failed for $path"}
    return $hex
}
set entries [list full vision_base.bit pixel640 partial_pixel_640.bit pixel320 partial_pixel_320.bit pixelblank partial_pixel_blank.bit uart partial_uart.bit uartblank partial_uart_blank.bit]
if {$has_1280} {lappend entries pixel1280 partial_pixel_1280.bit}
set jbody ""
foreach {k f} $entries {
    set h [sha256_file [file join $dist $f]]
    append jbody [format {  "%s": "%s",} $k $h]
}
set jbody [string range $jbody 0 end-1]
set f [open [file join $dist partial_manifest.json] w]
puts $f "{\n$jbody\n}"
close $f
puts "DFX: manifest written -> [file join $dist partial_manifest.json]"
set okline "DFX_BUILD_OK setup_full=$setup setup_320=$setup_320"
if {$has_1280} {append okline " setup_1280=$setup_1280"}
puts $okline
puts "DFX: outputs in $dist"
