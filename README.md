## I3E-D3 Blupill implementation
~/.platformio/packages/framework-mbed/targets/TARGET_STM/TARGET_STM32F1/TARGET_BLUEPILL_F103C8/PinNames.h

Change USBTX and USBRX values:
```c++
USBTX       = PA_9,
USBRX       = PA_10
```

Connect bluepill using a USB/TTL converter. In the first terminal, run roscore:
```bash
$ roscore
```

In the second terminal, run the Serial Node:
```bash
$ rosrun rosserial_python serial_node.py
```