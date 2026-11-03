# Run in the generated camera_to_dsi_display_Runs/phy_1 directory.
# First process: set gf_optimize 1; source this file.
# Fresh process: set gf_optimize 0; source this file to verify the saved DB.
# No bitstream is generated if the reloaded final Setup/Hold report fails.
if {![info exists gf_optimize]} { set gf_optimize 0 }
if {[catch {
    set gf_project camera_to_dsi_display
    if {$gf_optimize} {
        set gf_input ${gf_project}_pr.db
        set gf_output ${gf_project}_optimized
        set gf_report opt_final
    } else {
        set gf_input ${gf_project}_optimized.db
        set gf_output ${gf_project}_verified
        set gf_report verify_final
    }
    if {![file exists $gf_input]} { error "Missing input database $gf_input" }
    import_device ph1_35p.db -package PH1P35MDG324 -speed 3
    open_project ${gf_project}.prj -noanalyze
    import_db $gf_input
    clear_sdc -user
    read_sdc ../../${gf_project}.sdc
    set gf_arm [get_ports -quiet {I_gimbal_arm}]
    set gf_pwm [get_ports {O_gimbal_pan_pwm O_gimbal_tilt_pwm}]
    if {[llength $gf_arm] != 0 || [llength $gf_pwm] != 2} {
        error "Expected mode-only control with two PWM ports and no external ARM port"
    }
    puts "GIMBAL_MODE_ONLY_PORT_CHECK_PASS"
    if {$gf_optimize} {
        phys_opt -post_route -effort ultra
        fix_hold
    }
    update_timing -mode final
    report_timing_status -file ${gf_report}.ts
    report_timing_summary -file ${gf_report}.timing
    report_timing_exception -file ${gf_report}_exceptions.timing
    export_sdc -file ${gf_report}_constraints.sdc
    report_area -io_info -file ${gf_report}.area
    report_clock_utilization -file ${gf_report}_clocks.txt
    flow_status -file ${gf_report}_flow.status
    set gf_channel [open ${gf_report}.ts r]
    set gf_status [read $gf_channel]
    close $gf_channel
    if {![regexp {SWNS:\s*([-0-9.]+)ns} $gf_status gf_match gf_setup] ||
        ![regexp {HWNS:\s*([-0-9.]+)ns} $gf_status gf_match gf_hold]} {
        error "Cannot parse final Setup/Hold status"
    }
    if {$gf_setup < 0 || $gf_hold < 0} {
        error "Final timing failed: Setup=$gf_setup ns Hold=$gf_hold ns"
    }
    export_db ${gf_output}.db
    bitgen -bit ${gf_output}.bit
    puts "GIMBAL_FINAL_PASS optimize=$gf_optimize setup_ns=$gf_setup hold_ns=$gf_hold bit=${gf_output}.bit"
} gf_error]} {
    puts "GIMBAL_FINAL_ERROR: $gf_error"
    exit 1
}
exit
