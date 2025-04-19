@echo off
REM
REM Build the flash algorithms for the Winbond W25M512 SPI NOR part then
REM convert it to a raw binary (no ELF header) then into a base64-encoded
REM string for inclusion in the probe-rs chip YAML file.
REM
arm-none-eabi-gcc.exe -I./include -T link.ld -nostartfiles -O3 -DNDEBUG -mthumb -mcpu=cortex-m4 main.c -o blob
arm-none-eabi-objcopy.exe -O binary blob blob.bin
dir blob.bin

powershell -command "[Convert]::ToBase64String([IO.File]::ReadAllBytes('.\blob.bin'))"
