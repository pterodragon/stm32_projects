openocd -d0 -f ./openocd/tcl/board/st_nucleo_f4.cfg -c "init;targets;halt;flash write_image erase build/ov7670.hex;shutdown"
