$ipmap = @{
  'floating_point_v7_1_rfs.vhd'      = 'floating_point_v7_1\hdl\floating_point_v7_1_rfs.vhd'
  'xbip_pipe_v3_0_vh_rfs.vhd'        = 'xbip_pipe_v3_0\hdl\xbip_pipe_v3_0_vh_rfs.vhd'
  'xbip_utils_v3_0_vh_rfs.vhd'       = 'xbip_utils_v3_0\hdl\xbip_utils_v3_0_vh_rfs.vhd'
  'xbip_dsp48_wrapper_v3_0_vh_rfs.vhd' = 'xbip_dsp48_wrapper_v3_0\hdl\xbip_dsp48_wrapper_v3_0_vh_rfs.vhd'
  'xbip_bram18k_v3_0_vh_rfs.vhd'     = 'xbip_bram18k_v3_0\hdl\xbip_bram18k_v3_0_vh_rfs.vhd'
  'axi_utils_v2_0_vh_rfs.vhd'        = 'axi_utils_v2_0\hdl\axi_utils_v2_0_vh_rfs.vhd'
  'xbip_dsp48_addsub_v3_0_vh_rfs.vhd' = 'xbip_dsp48_addsub_v3_0\hdl\xbip_dsp48_addsub_v3_0_vh_rfs.vhd'
  'xbip_dsp48_multadd_v3_0_vh_rfs.vhd' = 'xbip_dsp48_multadd_v3_0\hdl\xbip_dsp48_multadd_v3_0_vh_rfs.vhd'
  'mult_gen_v12_0_vh_rfs.vhd'        = 'mult_gen_v12_0\hdl\mult_gen_v12_0_vh_rfs.vhd'
}
$base = 'C:\Users\Niiii\Desktop\dfx320_run6\dfx320_build\hw320\ip_repo'
foreach ($ip in @('dense','wm')) {
  $dst = "$base\$ip\hdl\ip\floating_point_lib\hdl"
  foreach ($f in $ipmap.Keys) {
    $src = "E:\Vivado\2023.2\data\ip\xilinx\$($ipmap[$f])"
    if (Test-Path $src) {
      Copy-Item $src "$dst\$f" -Force
    } else {
      Write-Output ("MISSING: " + $src)
    }
  }
}
Write-Output 'REPLACED_FP2023'
