# Summary

This is an ESP-IDF UART speed test that shoves as many bytes over the pipe as it possibly can at the highest baud that seems feasible

Tracks bytes sent/received in total per second (roughly), and counts how many of the bytes received were as expected.

Constantly sends a pattern of "abababab"

Not the most scientific test, but maybe good for gauging maximum theoretical raw throughput.

# Build

This repo has convenience scripts to build for different targets into different build directories without having to full clean between switching targets and without wiping the sdkconfig every time.

For best results, use two separate terminals.

Prepare your build environment:
```sh
. ~/esp/esp-idf/export.sh
```

Get this repo:

```sh
git clone https://github.com/ftab/esp32-uart-speed-test.git
cd esp32-uart-speed-test
```

Set for your desired target: (note the extra dot and space; set-target sets an environment variable and needs to be sourced)
```sh
. ./set-target esp32
```

To build, flash, and monitor:

```sh
./build flash monitor
```

If it did not find the correct tty:

```sh
./build flash monitor -p /dev/ttyUSB0
```

Substitute with the right port as needed

Just monitor:
```sh
./idf monitor
```

Just menuconfig:
```sh
./idf menuconfig
```
