To flash STM32 over USb:

```
2D-Spacecraft\flight-controller > cargo dfu --release
```

To flash ESP32 over USB:

```
2D-Spacecraft\rcs-controller > cargo run --release
```

## Setting up STM32 for flashing

No probe-rs because DFU

`cargo dfu` to build and flash.

Hold the boot button down when plugging in to enter DFU mode (required for flashing).

You need to use Zadig to swap usb drivers. Plug in the device into DFU mode, make sure the device is selected in the dropdown. May need to go into the menu to refresh available devices.

- cargo dfu needs libusb-win32
- betaflight needs WinUSB https://betaflight.com/docs/development/USB-Flashing

Configuration for the board I'm using:
https://github.com/betaflight/config/blob/master/configs/OMNIBUSF4SD/config.h

embassy_usb_logger

Probably want to use internal oscillator
