#set project_directory   [file dirname [info script]]
#append project_name "vivado_" $guinness_dir
set project_name "vivado"
set project_directory "."

#cd $project_directory
create_project -force -part "xc7a100tcsg324-1" $project_name $project_directory/$project_name


if {[string equal [get_filesets -quiet sources_1] ""]} {
    create_fileset -srcset sources_1
}

if {[string equal [get_filesets -quiet constrs_1] ""]} {
    create_fileset -constrset constrs_1
}

set synth_1_flow     "Vivado Synthesis 2019"
set synth_1_strategy "Vivado Synthesis Defaults"
if {[string equal [get_runs -quiet synth_1] ""]} {
    create_run -name synth_1 -flow $synth_1_flow -strategy $synth_1_strategy -constrset constrs_1
} else {
    set_property flow     $synth_1_flow     [get_runs synth_1]
    set_property strategy $synth_1_strategy [get_runs synth_1]
}
current_run -synthesis [get_runs synth_1]

set impl_1_flow      "Vivado Implementation 2019"
set impl_1_strategy  "Vivado Implementation Defaults"
if {[string equal [get_runs -quiet impl_1] ""]} {
    create_run -name impl_1 -flow $impl_1_flow -strategy $impl_1_strategy -constrset constrs_1 -parent_run synth_1
} else {
    set_property flow     $impl_1_flow      [get_runs impl_1]
    set_property strategy $impl_1_strategy  [get_runs impl_1]
}
current_run -implementation [get_runs impl_1]

add_files -fileset constrs_1 -norecurse "src/constraint/nexys4ddr.xdc"

add_files -fileset sources_1 -norecurse "src/verilog/module"
add_files -fileset sources_1 -norecurse "src/verilog/boards/top_nexys4ddr.v"

set_property source_mgmt_mode None [current_project]
set_property top top_nexys4ddr [current_fileset]
set_property top_file {src/verilog/boards/top_nexys4ddr.v} [current_fileset]

create_ip -name clk_wiz -vendor xilinx.com -library ip -version "6.0" -module_name clk_wiz_0 -dir "vivado/ip" -force
set_property -dict [list CONFIG.PRIM_SOURCE "No_buffer" CONFIG.PRIM_IN_FREQ 100.000 \
    CONFIG.CLKOUT1_USED {true} CONFIG.CLKOUT1_REQUESTED_OUT_FREQ 50.000 \
    CONFIG.USE_LOCKED "true" CONFIG.LOCKED_PORT "locked" CONFIG.USE_RESET "true" \
    CONFIG.RESET_TYPE "ACTIVE_HIGH" CONFIG.RESET_PORT "reset"] [get_ips clk_wiz_0]


launch_runs synth_1
wait_on_run synth_1

launch_runs impl_1
#wait_on_run impl_1 -lsf {sbatch -N 1 -n 4 -t 48:00:00}
wait_on_run impl_1

open_run    impl_1
report_utilization -hierarchical -file [file join $project_directory "project.rpt" ]
report_timing      -file [file join $project_directory "project.rpt" ] -append

launch_runs impl_1 -to_step write_bitstream
wait_on_run impl_1

close_project