# Modbus RTU control panel plugin
This grblHAL plugin provides support for a Modbus RTU connection to a hardware control panel. (A typical control panel would provide physical buttons, encoders, and a display - and would be used in addition to a software sender).

It provides support for issuing commands, spindle/feed overrides, jogging etc, and for displaying current position & status.

---
Note: This plugin is under development - and is likely to change as a reference control panel implementation is developed. 

It offers support for up to 88 input keys, and up to 4 quadrature encoders. (Somewhat arbitrary, but based on current development hardware).

The current modbus register descriptions can be found in the docs folder, note that not all inputs are handled as yet.

---
To enable the plugin for testing;

Copy the source to a new "panel" folder in the grblHAL source tree, and define as a source folder in your IDE if necessary.

The following definitions need to be added, either in your IDE or in my_machine.h.

    PANEL_ENABLE=1
    MODBUS_ENABLE=1
    MODBUS_MAX_ADU_SIZE=64

Add the following lines to plugins_init.h to load the plugin (in the 3rd party plugins section).

    #if PANEL_ENABLE
        extern void panel_init (void);
        panel_init();
    #endif
