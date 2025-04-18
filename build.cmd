@echo off

"c:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin\arm-none-eabi-gcc.exe" -T link.ld -nostartfiles -O3 -DNDEBUG -mthumb -mcpu=cortex-m4 main.c -o blob_intermediate
"c:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin\arm-none-eabi-objcopy.exe" -O binary blob_intermediate blob.bin
dir blob.bin

powershell -Command "[Convert]::ToBase64String([IO.File]::ReadAllBytes('.\blob.bin'))"
