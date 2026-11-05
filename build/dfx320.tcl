# dfx320.tcl —— PYNQ-Z2 320 位流的 DFX 构建（单 RP 双 RM + blank RM）
# Vivado 2023.2+ batch 模式（Windows）:
#   vivado -mode batch -source dfx320.tcl
# （必须 2023.2+：2022.1/2022.2 对 7 系列 DFX 有 0 帧 partial 工具 bug；
#   包内浮点核心为 rev15，2022.2 加密、2023.2+ 可解密）
#
# 目标：四个 overlay 中最快的加载路径 ——
#   静态区（常驻，与 320 视觉位流同构）:
#     stereo_rt_accel + preprocess_accel + pointcloud_accel + dual_pmod_uart
#     + status_led + PS + 各 interconnect（相机/舵机/点云全程可用）
#   重配置区 RP（单个，两个模型 RM 共享同一槽位——dense 与 wm 顶层端口完全一致）:
#     RP model cell  —— RM1 dense（项目内实现）/ RM2 wm（改名网表换入）/ RM3 blank（stub）
#
# 产出（out320dfx/）:
#   vision_base_320_dfx.bit    完整 320 位流（静态 + dense RM；功能等价现 320，
#                              但布局不同——上板须按新位流重新验收）
#   partial_dense_fp32.bit     dense RM 部分位流（~百 KB 级，毫秒级加载）
#   partial_wm_trajectory.bit  wm RM 部分位流
#   partial_models_blank.bit   blank RM 部分位流（"卸载模型加速器"）
#   partial_manifest.json      sha256 清单（板端 switch_fast.py 校验用）
#   *.rpt                      时序/DRC/利用率报告
#
# 前提: 与本脚本同目录 hw320/（含 ip_repo 全部 HLS IP 源码 + board_files + xdc），
#       Vivado 2022.1 标准版（Zynq-7020 的 DFX 免费支持），磁盘 >= 10GB，
#       构建时长预估 60-120 分钟。
#
# 注意（必读 README_DFX.md）:
#   - PBLOCK_MODEL 是占位初值；如 place/pr_verify 失败，按 utilization_hierarchical.rpt
#     实际资源调整（改本文件顶部变量后重跑）。
#   - 若静态区利用率过高（如 SLICE>75%）DFX 无法收敛，需先评估裁减静态区。
#   - 部分位流与整片位流的静态区必须完全一致：所有部分位流都从同一
#     config_full 静态导出；任何静态修改（BD/约束/版本）必须全量重建。
#   - **wm 的控制寄存器地址从 0x40070000 变为 0x40040000**（与 dense 同槽位），
#     板端 WM 模型服务必须适配新地址。

set dfx_package [file normalize [file dirname [info script]]]
set here [file join $dfx_package hw320]
set build [file join $dfx_package b320dfx]
set dist [file join $dfx_package out320dfx]
if {[file exists $build] || [file exists $dist]} {error "Preserve existing build; use a fresh copy of this package"}
set repos [list [file join $here ip_repo stereo] [file join $here ip_repo preprocess] [file join $here ip_repo pointcloud] [file join $here ip_repo dense] [file join $here ip_repo wm]]
foreach repo $repos {if {![file exists [file join $repo component.xml]]} {error "Missing bundled HLS IP"}}
file mkdir $build
file mkdir $dist

# ---- 单 RP Pblock（probe_dfx.tcl 实测 xc7z020 器件网格，2026-11-04）----
# 器件为阶梯形楼层图：中间列（SLICE_X27-67，tile X19-43）Y0-149 全高，
# 两侧列只有 Y0-49。RP 取中间区 tile X20-38 x tileY0-99：
#   覆盖 BRAM tile22+36（RAMB36 site X2/X3）、DSP tile25（DSP48 site X2）；CMT tile33 无 slice 自动排除。
#   容量 Y0-129：~3900 slices / 52 RAMB36 / 104 RAMB18 / 52 DSP48（tile X20-38 x tileY0-129）。
# 尺寸依据：dense_fp32 是 DSP 瓶颈（44 DSP48E1 / 6737 LUT / 12.66%），首次构建 Y0-99 只有
#   40 DSP → place 报 UTLZ-1（44 vs 40）。扩 Y 到 129 拿 52 DSP（余量 8，防 phys_opt DSP 复制）。
#   wm（7551 LUT / 8 BRAM / 3 DSP）与 blank 均远小于 dense，slice/BRAM 不受限。
# pblock 以 tile 矩形给出（按列分组、端点用真实 tile 名）：整列全类型站点
# （含 FIFO18E1/RAMBFIFO36E1 变体）自动纳入，避免 site 矩形跨类型 Y 跨度不一致
# 导致的 18-993 列劈裂（mini 探针实测教训）。resize_pblock 不收 tile 对象、只收矩形。
proc get_rp_rects {xmin xmax ymin ymax} {
    set tiles [list]
    foreach tl [get_tiles -filter "TILE_TYPE =~ CLB* || TILE_TYPE =~ BRAM* || TILE_TYPE =~ DSP*"] {
        if {[string match *INT* $tl]} {continue}
        regexp {X(\d+)Y(\d+)$} $tl -> tx ty
        if {$tx >= $xmin && $tx <= $xmax && $ty >= $ymin && $ty <= $ymax} {lappend tiles $tl}
    }
    # BRAM/DSP tile 只在 Y=5 倍数存在（1 个 RAMB36/DSP48 site 占 5 tile 行），不能直接拼 tile 矩形。
    # tile -> get_sites -of_objects 反查站点，按站点类型分列拼 site 矩形：
    # 各类型窗口与 tile Y 窗口严格一致，FIFO18E1/RAMBFIFO36E1 变体自动包含。
    set sites [list]
    foreach tl $tiles {
        foreach s [get_sites -quiet -of_objects $tl] {
            # 只保留六种可布站点类型；TIEOFF 等稀疏/非布类型会产出非法矩形
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

# ================= Phase 1: 建工程 + BD（单模型 cell，同 build320.tcl 静态区） =================
set_param board.repoPaths [list [file join $here board_files pynq-z2]]
create_project pa320 $build -part xc7z020clg400-1
set_property board_part tul.com.tw:pynq-z2:part0:1.0 [current_project]
# DFX 总开关（0 帧根因之一：缺此 create_partition_def 报 "Enable PR_FLOW"）
set_property PR_FLOW 1 [current_project]
set_property ip_repo_paths $repos [current_project]
update_ip_catalog
add_files -norecurse [file join $here dual_pmod_uart.v]
update_compile_order -fileset sources_1
create_bd_design stereo_design
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external {FIXED_IO, DDR} apply_board_preset {1} Master {Disable} Slave {Disable}} $ps
set_property -dict [list CONFIG.PCW_USE_M_AXI_GP0 {1} CONFIG.PCW_USE_S_AXI_HP0 {1} CONFIG.PCW_USE_S_AXI_HP1 {1} CONFIG.PCW_USE_S_AXI_HP2 {1} CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100.000000}] $ps
set bm [create_bd_cell -type ip -vlnv xilinx.com:hls:stereo_rt_accel:1.0 stereo_rt_accel_0]
set cloud [create_bd_cell -type ip -vlnv xilinx.com:hls:pointcloud_accel:1.0 pointcloud_accel_0]
set pre [create_bd_cell -type ip -vlnv xilinx.com:hls:preprocess_accel:1.0 preprocess_accel_0]
# 单 RP cell：dense 为 RM1；wm 作为 RM2 网表在 Phase 4 换入同一槽位
set nn [create_bd_cell -type ip -vlnv xilinx.com:hls:dense_fp32:1.0 dense_fp32_0]
set nnlink [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 nn_smc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $nnlink
set uart [create_bd_cell -type module -reference dual_pmod_uart dual_pmod_uart_0]
set control [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 control_smc]
set bmdata [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 bm_smc]
set auxdata [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 aux_smc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {6}] $control
set_property -dict [list CONFIG.NUM_SI {4} CONFIG.NUM_MI {1} CONFIG.STRATEGY {1}] $bmdata
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1} CONFIG.STRATEGY {1}] $auxdata
connect_bd_intf_net [get_bd_intf_pins $ps/M_AXI_GP0] [get_bd_intf_pins $control/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins $control/M00_AXI] [get_bd_intf_pins $bm/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins $control/M01_AXI] [get_bd_intf_pins $cloud/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins $control/M02_AXI] [get_bd_intf_pins $pre/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins $control/M03_AXI] [get_bd_intf_pins $uart/S_AXI]
connect_bd_intf_net [get_bd_intf_pins $control/M04_AXI] [get_bd_intf_pins $nn/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins $nn/m_axi_memory] [get_bd_intf_pins $nnlink/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins $nnlink/M00_AXI] [get_bd_intf_pins $ps/S_AXI_HP2]
for {set i 0} {$i<4} {incr i} {connect_bd_intf_net [get_bd_intf_pins $bm/m_axi_gmem$i] [get_bd_intf_pins $bmdata/S0${i}_AXI]}
connect_bd_intf_net [get_bd_intf_pins $cloud/m_axi_memory] [get_bd_intf_pins $auxdata/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins $pre/m_axi_memory] [get_bd_intf_pins $auxdata/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins $bmdata/M00_AXI] [get_bd_intf_pins $ps/S_AXI_HP0]
connect_bd_intf_net [get_bd_intf_pins $auxdata/M00_AXI] [get_bd_intf_pins $ps/S_AXI_HP1]
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_100m]
set zero [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const0]
set one [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const1]
set_property CONFIG.CONST_VAL 0 $zero
set_property CONFIG.CONST_VAL 1 $one
connect_bd_net [get_bd_pins $ps/FCLK_CLK0] [get_bd_pins $ps/M_AXI_GP0_ACLK] [get_bd_pins $ps/S_AXI_HP0_ACLK] [get_bd_pins $ps/S_AXI_HP1_ACLK] [get_bd_pins $bm/ap_clk] [get_bd_pins $cloud/ap_clk] [get_bd_pins $pre/ap_clk] [get_bd_pins $uart/s_axi_aclk] [get_bd_pins $control/aclk] [get_bd_pins $nn/ap_clk] [get_bd_pins $nnlink/aclk] [get_bd_pins $ps/S_AXI_HP2_ACLK] [get_bd_pins $rst/slowest_sync_clk]
connect_bd_net [get_bd_pins $ps/FCLK_RESET0_N] [get_bd_pins $rst/ext_reset_in]
connect_bd_net [get_bd_pins $zero/dout] [get_bd_pins $rst/mb_debug_sys_rst]
connect_bd_net [get_bd_pins $one/dout] [get_bd_pins $rst/aux_reset_in] [get_bd_pins $rst/dcm_locked]
connect_bd_net [get_bd_pins $rst/peripheral_aresetn] [get_bd_pins $bm/ap_rst_n] [get_bd_pins $cloud/ap_rst_n] [get_bd_pins $pre/ap_rst_n] [get_bd_pins $uart/s_axi_aresetn] [get_bd_pins $control/aresetn] [get_bd_pins $nn/ap_rst_n] [get_bd_pins $nnlink/aresetn]
foreach cell [list $bmdata $auxdata] {
 foreach pin [get_bd_pins -of_objects $cell -filter {NAME =~ *ACLK}] {connect_bd_net [get_bd_pins $ps/FCLK_CLK0] $pin}
 foreach pin [get_bd_pins -of_objects $cell -filter {NAME =~ *ARESETN}] {connect_bd_net [get_bd_pins $rst/peripheral_aresetn] $pin}
}
make_bd_pins_external [get_bd_pins $uart/uart_rx] [get_bd_pins $uart/uart_tx]
set_property name uart_rx [get_bd_ports uart_rx_0]
set_property name uart_tx [get_bd_ports uart_tx_0]
assign_bd_address -offset 0x40000000 -range 64K -target_address_space [get_bd_addr_spaces $ps/Data] [get_bd_addr_segs $cloud/s_axi_control/Reg]
assign_bd_address -offset 0x40010000 -range 64K -target_address_space [get_bd_addr_spaces $ps/Data] [get_bd_addr_segs $bm/s_axi_control/Reg]
assign_bd_address -offset 0x40020000 -range 64K -target_address_space [get_bd_addr_spaces $ps/Data] [get_bd_addr_segs $pre/s_axi_control/Reg]
assign_bd_address -offset 0x40040000 -range 64K -target_address_space [get_bd_addr_spaces $ps/Data] [get_bd_addr_segs $nn/s_axi_control/Reg]
set uart_segs [get_bd_addr_segs -of_objects [get_bd_intf_pins $uart/S_AXI]]
puts "UART_ADDRESS_SEGMENTS=$uart_segs"
if {[llength $uart_segs]!=1} {error "Unexpected UART address segment"}
assign_bd_address -offset 0x40030000 -range 4K -target_address_space [get_bd_addr_spaces $ps/Data] $uart_segs

# 四个状态 LED（同 build320.tcl）
set led [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 status_led_gpio]
set_property -dict [list CONFIG.C_GPIO_WIDTH {4} CONFIG.C_ALL_OUTPUTS {1} CONFIG.C_DOUT_DEFAULT {0x00000000}] $led
connect_bd_intf_net [get_bd_intf_pins $control/M05_AXI] [get_bd_intf_pins $led/S_AXI]
connect_bd_net [get_bd_pins $ps/FCLK_CLK0] [get_bd_pins $led/s_axi_aclk]
connect_bd_net [get_bd_pins $rst/peripheral_aresetn] [get_bd_pins $led/s_axi_aresetn]
make_bd_pins_external [get_bd_pins $led/gpio_io_o]
set_property name status_led [get_bd_ports gpio_io_o_0]
assign_bd_address -offset 0x40060000 -range 4K -target_address_space [get_bd_addr_spaces $ps/Data] [get_bd_addr_segs $led/S_AXI/Reg]

assign_bd_address
validate_bd_design
save_bd_design
set bd [get_files stereo_design.bd]
generate_target all $bd
make_wrapper -files $bd -top
add_files -norecurse [file join $build pa320.gen sources_1 bd stereo_design hdl stereo_design_wrapper.v]
add_files -fileset constrs_1 -norecurse [file join $here pmod_uart.xdc]
add_files -fileset constrs_1 -norecurse [file join $here status_led.xdc]
set_property top stereo_design_wrapper [current_fileset]
update_compile_order -fileset sources_1

# 标准综合 run（默认 Hierarchical synth_checkpoint_mode）：BD 每个 IP（含
# floating_point 加密核心、auto_pc 等 nested IP）由 launch_runs 自动 OOC 综合成
# DCP 后读入。不要 synth_checkpoint_mode None + 手动 synth_design：加密核心无
# RTL 可综合（Synth 8-439 floating_point_v7_1_15 not found 的根因）。
# 实现阶段仍手动跑（opt/place/phys_opt/route）：open_run 后设置的 RP 属性/pblock
# 只在内存设计里，交给 impl_1 run 会因重新读 synth DCP 而丢失。
set_param general.maxThreads 4
set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} -object [get_runs synth_1] -value {-flatten_hierarchy none}

# ================= Phase 2: 综合 =================
# 0 帧根因（2026-11-05 diag19 实证）：缺 PR_FLOW 1（DFX 总开关）+ create_partition_def
# （partition 元数据从未建立）。HD.PARTITION 在 DFX 里恒空（HDPR-49：与 HD.RECONFIGURABLE 互斥），
# 旧脚本"HD.PARTITION 非空"自检必然失败。正确流程见 open_run synth_1 之后的 black_box +
# create_partition_def 块。
puts "DFX: synthesizing config_full via synth_1 run (all BD IPs OOC, dense=RP)"
launch_runs synth_1 -jobs 4
wait_on_run synth_1
open_run synth_1
write_checkpoint -force [file join $dist synth_full.dcp]

set dcells [get_cells -hierarchical -filter {NAME =~ *dense_fp32_0}]
if {[llength $dcells] != 1} {error "dense_fp32_0 hierarchical cells: $dcells (expect exactly 1)"}
set dense_cell [lindex $dcells 0]
puts "DFX: RP cell = $dense_cell"
puts "DFX: dense IS_BLACKBOX after synth = [get_property IS_BLACKBOX $dense_cell]"
# 0 帧根因修复（2026-11-05 diag19 实证）：
#   open_run 后 dense 是完整逻辑（IS_BLACKBOX=0），须先黑盒化回到静态视角；
#   记录黑盒前原始 REF_NAME（create_partition_def -module 用原始名，非 _bb）；
#   set HD.RECONFIGURABLE + create_partition_def 建立 DFX partition 元数据。
#   HD.PARTITION 在 DFX 里恒空（HDPR-49 互斥），严禁自检它。
set dense_ref_orig [get_property REF_NAME $dense_cell]
update_design -black_box -cell $dense_cell
set dense_cell [lindex [get_cells -hierarchical -filter {NAME =~ *dense_fp32_0}] 0]
set_property HD.RECONFIGURABLE 1 $dense_cell
create_partition_def -name pd_dense -module $dense_ref_orig
puts "DFX: partition_def created = [get_partition_defs]"

# blank RM stub 生成器：从综合网表导出精确端口声明，输出拉安全电平——
# s_axi_control 五个握手输出拉 1（AXI-Lite 从机全应答、读返回 0，PS 访问不会挂总线），
# 其余输出（m_axi_memory 各 *_VALID、*ADDR 等）拉 0（AXI master 完全空闲）。
proc make_blank_stub {cell outfile} {
    # 2022.1 弃用 write_verilog -mode port（12-3349）；改用 -mode synth_stub 生成
    # 完整 stub（module 名 = REF_NAME，端口已聚合总线），再插入 assign 拉安全电平。
    write_verilog -force -mode synth_stub -cell $cell $outfile
    set f [open $outfile r]
    set txt [read $f]
    close $f
    # synth_stub 端口声明格式："  output [63:0]name;" 或 "  output name;"（无空格、带分号）
    set outs [list]
    foreach line [split $txt "\n"] {
        if {[regexp {^\s*output\s+(\[\d+:\d+\])?(\w+)\s*;} $line -> bw name]} {
            lappend outs $name
        }
    }
    if {[llength $outs] == 0} {error "make_blank_stub: no output ports found in $outfile"}
    # AXI-Lite 握手信号拉 1（从设备可响应读），其余输出（m_axi_memory *_VALID/*ADDR 等）拉 0（master 空闲）
    set ack {s_axi_control_AWREADY s_axi_control_WREADY s_axi_control_ARREADY s_axi_control_BVALID s_axi_control_RVALID}
    set assigns ""
    foreach name $outs {
        if {$name in $ack} {
            append assigns "  assign $name = 1'b1;\n"
        } else {
            append assigns "  assign $name = 1'b0;\n"
        }
    }
    set idx [string last "endmodule" $txt]
    if {$idx < 0} {error "make_blank_stub: endmodule not found in $outfile"}
    set txt "[string range $txt 0 [expr {$idx - 1}]]$assigns[string range $txt $idx end]"
    set f [open $outfile w]
    puts $f $txt
    close $f
    puts "DFX: blank stub written -> $outfile ([llength $outs] outputs)"
}

# ================= Phase 3: config_full（RM1 = dense） =================
# Phase 2 尾已对黑盒 dense 设 HD.RECONFIGURABLE、partition 已建立（见 line 200 附近自检）。
# 这里 read_checkpoint -cell 换入 dense RM 的 OOC 综合 DCP（.gen 目录），再 pblock + 实现。
# Y 范围改 0-149（完整 3 个时钟区域），Y129 跨时钟区域边界会导致帧不对齐。
set PBLOCK_MODEL [get_rp_rects 20 38 0 149]

# 递归查找 dense RM 的 OOC DCP（标准 BD IP OOC 综合产物，在 pa320.gen 下）
proc find_dense_rm_dcp {root} {
    set out [list]
    foreach f [glob -nocomplain -directory $root *] {
        if {[file isdirectory $f]} {
            set out [concat $out [find_dense_rm_dcp $f]]
        } elseif {[string match *stereo_design_dense_fp32_0_0*.dcp $f]} {
            lappend out $f
        }
    }
    return $out
}
set dense_dcps [find_dense_rm_dcp $build]
puts "DFX: dense RM DCP candidates: $dense_dcps"
if {[llength $dense_dcps] == 0} {error "dense RM OOC DCP not found under $build"}
set dense_dcp [lindex $dense_dcps 0]
read_checkpoint -cell $dense_cell $dense_dcp
# read_checkpoint 重建 cell，旧句柄失效，重新获取；partition 应随 cell 属性保留
set dense_cell [lindex [get_cells -hierarchical -filter {NAME =~ *dense_fp32_0}] 0]
puts "DFX: after read_checkpoint (dense RM) HD.RECONFIGURABLE=[get_property HD.RECONFIGURABLE $dense_cell]"

create_pblock pblock_model
add_cells_to_pblock [get_pblocks pblock_model] [list $dense_cell]
foreach r $PBLOCK_MODEL {resize_pblock [get_pblocks pblock_model] -add $r}
set_property SNAPPING_MODE ON [get_pblocks pblock_model]
set_property RESET_AFTER_RECONFIG TRUE [get_pblocks pblock_model]

puts "DFX: implementing config_full (manual, on synth result)"
opt_design -directive ExploreArea
place_design -directive Explore
phys_opt_design -directive ExploreWithAggressiveHoldFix
route_design -directive Explore
report_timing_summary -file [file join $dist timing_summary_full.rpt]
report_drc -file [file join $dist drc_full.rpt]
report_utilization -hierarchical -file [file join $dist utilization_hierarchical.rpt]
set setup [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup eq "" || $hold eq "" || $setup<0 || $hold<0} {error "config_full timing failed: setup=$setup hold=$hold"}
puts "DFX: config_full timing OK setup=$setup hold=$hold"

lock_design -level routing
write_checkpoint -force [file join $dist config_full_routed.dcp]
write_bitstream -force [file join $dist vision_base_320_dfx.bit]
puts "DFX: full bitstream written"
write_bitstream -force -cell $dense_cell [file join $dist partial_dense_fp32.bit]
set psize [file size [file join $dist partial_dense_fp32.bit]]
puts "DFX: partial_dense_fp32.bit size = $psize bytes"
if {$psize < 10000} {error "partial_dense_fp32.bit too small ($psize bytes): DFX partition/frame derivation still broken"}
puts "DFX: partial_dense_fp32.bit written"

# 提前准备 blank stub（设计仍打开；RM 参考名 = cell 的 REF_NAME）
set dense_ref [get_property REF_NAME $dense_cell]
puts "DFX: RP REF_NAME = $dense_ref"
make_blank_stub $dense_cell [file join $dist blank_stub.v]

# blank RM OOC 综合（非项目模式，避开项目文件管理）
close_design
close_project
read_verilog [file join $dist blank_stub.v]
synth_design -mode out_of_context -top $dense_ref -part xc7z020clg400-1 -flatten_hierarchy none
write_checkpoint -force [file join $dist blank_ooc.dcp]
close_design
remove_files [get_files -quiet]
puts "DFX: blank RM OOC netlist ready"

# ================= Phase 4: RM2 = wm（改名网表 + 换入锁定静态） =================
# 递归查找 wm_rm_ip 的 OOC 综合 DCP
proc find_wm_dcp {root} {
    set out [list]
    foreach f [glob -nocomplain -directory $root *] {
        if {[file isdirectory $f]} {
            set out [concat $out [find_wm_dcp $f]]
        } elseif {[string match *dense_fp32*.dcp $f]} {
            lappend out $f
        }
    }
    return $out
}
# 打开 DCP 校验顶层名
proc check_dcp_top {dcp} {
    open_checkpoint $dcp
    set top [get_property TOP [current_design]]
    if {$top eq ""} {
        set names [list]
        foreach c [get_cells -quiet -hierarchical -filter {IS_PRIMITIVE == 0}] {
            lappend names [get_property REF_NAME $c]
        }
        set top [join [lsort -unique $names] ","]
    }
    close_design
    return $top
}
# wm 顶层与 dense 端口一致；以 dense 的 REF_NAME 作为 module_name 生成包装器，
# OOC 综合出的 DCP 顶层名 = REF_NAME，才能被 read_checkpoint -cell 换入 RP。
create_project wmproj [file join $build wmproj] -part xc7z020clg400-1
set_property ip_repo_paths [list [file join $here ip_repo wm]] [current_project]
update_ip_catalog
# 2022.1 create_ip 无位置参数：-module_name 即 IP 名（改成 dense 的 REF_NAME，
# OOC 综合出的 DCP 顶层名 = REF_NAME，才能被 read_checkpoint -cell 换入 RP）
# -dir 目录必须预先存在（探针 run15 实证：目录不存在则 Common 17-69）
file mkdir [file join $build wmproj_ip]
create_ip -vlnv xilinx.com:hls:wm_trajectory:1.0 -module_name $dense_ref -dir [file join $build wmproj_ip]
puts "DFX: synthesizing wm RM netlist (synth_ip, module_name=$dense_ref)"
synth_ip [get_ips $dense_ref]
set wmdcps [find_wm_dcp $build]
if {[llength $wmdcps] == 0} {error "wm_rm_ip synthesized DCP not found under $build"}
set wmdcp ""
foreach d $wmdcps {
    set t [check_dcp_top $d]
    puts "DFX: candidate wm RM DCP = $d (top=$t)"
    if {$t eq $dense_ref} {set wmdcp $d; break}
}
if {$wmdcp eq ""} {error "no wm RM DCP with top=$dense_ref among: $wmdcps"}
file copy -force $wmdcp [file join $dist wm_rm_synth.dcp]
puts "DFX: wm RM DCP selected -> [file join $dist wm_rm_synth.dcp]"
close_project

# 换入 wm：解锁 -> black_box -> read_checkpoint -cell（探针 T2-T4 实测：
# read_checkpoint 只认黑盒；black_box 在锁定设计上因 GND/VCC/clk 网被锁而失败。
# 换入后立即重新 lock_design -level routing：black_box 已 unroute 的 RP 端口网段
# 不受锁，但 cfg1 已布的静态弧被锁住，place/route 只能动 RP 内段。
# 探针 run14 HDPRVerify-11 实证：不重新 lock，route 会重布穿越 RP 边界的
# 静态走线（rp_i/y[14]、b_IBUF[4] 等），pr_verify 必然失败。
# 2022.1 无 unlock_design 命令；lock_design -unlock 必须带 -level）
open_checkpoint [file join $dist config_full_routed.dcp]
# 重新获取 dense_cell（close_design/create_project 会使旧 cell 对象句柄失效为 null）
set dense_cell [lindex [get_cells -hierarchical -filter {NAME =~ *dense_fp32_0}] 0]
puts "DFX: config_full dense HD.RECONFIGURABLE=[get_property HD.RECONFIGURABLE $dense_cell]"
if {[catch {lock_design -unlock -level routing} u1]} {puts "DFX: unlock routing failed: $u1"}
if {[catch {lock_design -unlock -level placement} u2]} {puts "DFX: unlock placement failed: $u2"}
update_design -black_box -cell [get_cells $dense_cell]
read_checkpoint -cell [get_cells $dense_cell] [file join $dist wm_rm_synth.dcp]
# read_checkpoint 会重建 cell，旧 dense_cell 句柄失效，重新获取
set dense_cell [lindex [get_cells -hierarchical -filter {NAME =~ *dense_fp32_0}] 0]
puts "DFX: after wm swap HD.RECONFIGURABLE=[get_property HD.RECONFIGURABLE $dense_cell]"
lock_design -level routing
puts "DFX: implementing config_wm (RM2 swapped in, static locked)"
place_design -directive Explore
route_design -directive Explore
report_timing_summary -file [file join $dist timing_summary_wm.rpt]
set setup_w [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold_w [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup_w eq "" || $hold_w eq "" || $setup_w<0 || $hold_w<0} {error "config_wm timing failed: setup=$setup_w hold=$hold_w"}
puts "DFX: config_wm timing OK setup=$setup_w hold=$hold_w"
write_checkpoint -force [file join $dist config_wm_routed.dcp]
write_bitstream -force -cell [get_cells $dense_cell] [file join $dist partial_wm_trajectory.bit]
puts "DFX: partial_wm_trajectory.bit written"
close_design

puts "DFX: pr_verify config_full vs config_wm"
# 2022.1 实测 pr_verify 仅三种格式（12-3499）：(1) pr_verify DCP1 DCP2
# (2) -initial/-additional (3) -in_memory -additional；无 -full_check
if {[catch {pr_verify [file join $dist config_full_routed.dcp] [file join $dist config_wm_routed.dcp]} pv_w]} {
    error "pr_verify full-vs-wm FAILED: $pv_w"
}
puts "DFX: pr_verify PASSED (static region identical, full vs wm)"

# ================= Phase 5: RM3 = blank（stub，全输出拉安全电平） =================
open_checkpoint [file join $dist config_full_routed.dcp]
# 重新获取 dense_cell（close_design/create_project 会使旧 cell 对象句柄失效为 null）
set dense_cell [lindex [get_cells -hierarchical -filter {NAME =~ *dense_fp32_0}] 0]
puts "DFX: config_full dense HD.RECONFIGURABLE=[get_property HD.RECONFIGURABLE $dense_cell]"
if {[catch {lock_design -unlock -level routing} u1]} {puts "DFX: unlock routing failed: $u1"}
if {[catch {lock_design -unlock -level placement} u2]} {puts "DFX: unlock placement failed: $u2"}
update_design -black_box -cell [get_cells $dense_cell]
read_checkpoint -cell [get_cells $dense_cell] [file join $dist blank_ooc.dcp]
# read_checkpoint 会重建 cell，旧 dense_cell 句柄失效，重新获取
set dense_cell [lindex [get_cells -hierarchical -filter {NAME =~ *dense_fp32_0}] 0]
puts "DFX: after blank swap HD.RECONFIGURABLE=[get_property HD.RECONFIGURABLE $dense_cell]"
lock_design -level routing
puts "DFX: implementing config_blank (stub RM swapped in, static locked)"
place_design
route_design
report_timing_summary -file [file join $dist timing_summary_blank.rpt]
set setup_b [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold_b [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup_b eq "" || $hold_b eq "" || $setup_b<0 || $hold_b<0} {error "config_blank timing failed: setup=$setup_b hold=$hold_b"}
puts "DFX: config_blank timing OK setup=$setup_b hold=$hold_b"
write_checkpoint -force [file join $dist config_blank_routed.dcp]
write_bitstream -force -cell [get_cells $dense_cell] [file join $dist partial_models_blank.bit]
puts "DFX: partial_models_blank.bit written"
close_design

puts "DFX: pr_verify config_full vs config_blank"
if {[catch {pr_verify [file join $dist config_full_routed.dcp] [file join $dist config_blank_routed.dcp]} pv_b]} {
    error "pr_verify full-vs-blank FAILED: $pv_b"
}
puts "DFX: pr_verify PASSED (static region identical, full vs blank)"

# ================= Phase 6: manifest（板端 switch_fast.py 校验） =================
proc sha256_file {path} {
    if {[catch {exec certutil -hashfile $path SHA256} out]} {
        error "certutil failed for $path: $out"
    }
    set lines [split $out "\r\n"]
    # certutil 输出: 空行 / "SHA256 的 <path> 哈希:" / 哈希值 / "CertUtil: ..."
    set hex ""
    foreach ln $lines {
        set ln [string trim $ln]
        if {[regexp {^[0-9a-fA-F]{64}$} $ln]} {set hex [string tolower $ln]}
    }
    if {$hex eq ""} {error "certutil parse failed for $path"}
    return $hex
}

set sha_dense [sha256_file [file join $dist partial_dense_fp32.bit]]
set sha_wm [sha256_file [file join $dist partial_wm_trajectory.bit]]
set sha_blank [sha256_file [file join $dist partial_models_blank.bit]]
set sha_full [sha256_file [file join $dist vision_base_320_dfx.bit]]
set f [open [file join $dist partial_manifest.json] w]
puts $f "{\"dense\":\"$sha_dense\",\"wm\":\"$sha_wm\",\"blank\":\"$sha_blank\",\"full\":\"$sha_full\"}"
close $f
puts "DFX: manifest written"
puts "DFX_BUILD_OK setup_full=$setup setup_wm=$setup_w setup_blank=$setup_b"
puts "DFX: outputs in $dist"
