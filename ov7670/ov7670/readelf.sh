elf_file=$1
if [ -z "$elf_file" ]
then
    elf_file=./build/ili9341.elf
fi
arm-none-eabi-readelf -h $elf_file
