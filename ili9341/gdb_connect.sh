elf=./build/ili9341.elf
cmd_file=cmd.gdb

gdb=$([ `command -v gdb-multiarch` ] && echo "gdb-multiarch" || echo "arm-none-eabi-gdb")
$gdb -x $cmd_file $elf
