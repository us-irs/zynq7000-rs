	set outdir [lindex $argv 1]
	set xsa [lindex $argv 0]
	sdtgen set_dt_param -xsa $xsa -dir $outdir -board_dts zedboard
	sdtgen generate_sdt
