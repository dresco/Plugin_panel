# grblPANEL - a control panel plugin for grblHAL

This grblHAL plugin provides support for a hardware control panel, using either **Modbus RTU** or **CAN bus** (work in progress) for the connection. It provides support for issuing commands, spindle/feed overrides, jogging etc, and for displaying current position & status.

(A typical control panel would provide physical buttons, encoders, and a display - and would be used in addition to a software sender).

---
Note: This plugin is under development - and is subject to change as the [reference control panel](https://github.com/dresco/grblPANEL) implementation is further developed.

It offers support for up to 88 input keys, and up to 4 quadrature encoders. (Somewhat arbitrary, but based on current development hardware).

The current modbus register descriptions, along with the keypad bitfields, can be found in the docs folder. Note that not all inputs are handled as yet.

## To enable the plugin for testing;

Copy the source to a new "panel" folder in the grblHAL source tree, and define as a source folder in your IDE if necessary.

Add the following lines to plugins_init.h to load the plugin (in the 3rd party plugins section).

    #if PANEL_ENABLE
        extern void panel_init (void);
        panel_init();
    #endif

The following definitions need to be added, either in your IDE or in my_machine.h.

For Modbus;

    PANEL_ENABLE=1
    MODBUS_ENABLE=1
    MODBUS_MAX_ADU_SIZE=64

For CAN bus;

    PANEL_ENABLE=2
    CANBUS_ENABLE=1

Note that to use CAN, both the [CAN bus plugin](https://github.com/dresco/Plugin_canbus) and supporting CAN driver code for your platform are needed. Drivers for STM32F4xx and STM32H7xx are currently in development.
