onerror {quit -code 1 -force}
onbreak {resume}
vlib work
vlog -sv ../user_source/hdl_source/control/gf_follow_controller.sv tb_gimbal_reference.sv
vsim -onfinish stop work.tb_gimbal_reference
run -all
if {[examine -radix unsigned sim:/tb_gimbal_reference/test_done] != 1} {
    error "Reference-point test did not complete"
}
quit -code 0 -force
