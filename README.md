# bluepill-flash
A simple cross-platform tool able to flash bluepills (STM32F10xx) using the ST bootloader through serial port

## Compiling
You need [MGPCL](https://github.com/montoyo/mgpcl) in order to build bluepill-flash. The following commands should work:
```sh
git clone https://github.com/montoyo/mgpcl.git
cd mgpcl
cmake -DMGPCL_ENABLE_GUI=OFF -DMGPCL_ENABLE_SSL=OFF .
make
cd ..
git clone https://github.com/montoyo/bluepill-flash.git
cd bluepill-flash
cmake -DMGPCL_INCLUDE_DIR=../mgpcl/include -DMGPCL_LIB_DIR=../mgpcl/build .
make
```

## Usage
./stm32-flash [--options...] commands...

Examples:
* ./stm32-flash --device /dev/ttyACM0 write my_file.bin start
* ./stm32-flash --device /dev/ttyACM0 --baud-rate 115200 go 0x8001234

### Options
* --device DEVICE is used to select the serial port (something like COM3 on Windows and /dev/ttyACM0 on Linux)
* --baud-rate BAUDRATE is used to change the serial baud rate (default is 9600)
* --no-pid-check can be used to disable product ID checking (this is used to make sure the target chip is really a bluepill)

### Command: write FILE.bin
Erases the flash and writes FILE.bin at the beginning of flash (0x8000000) to the bluepill. Blocks are verified by default.
If you don't want to erase the flash before writing, use the "--no-erase" switch. If you want to skip verification, use the "--no-verify" switch.

### Command: erase
Erases the flash.

### Command: write-unprotect
Disables the write protection.

### Command: go ADDRESS
Jumps to the specified code address and proceed to normal execution. You **must** use the "0x" prefix if your address
is a hexadecimal address. Without this, the address will be interpreted as decimal.

### Command: start
Same as "go 0x8000000".

