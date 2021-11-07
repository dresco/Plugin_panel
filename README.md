# Modbus RTU control panel plugin
This grblHAL plugin provides support for a Modbus RTU connection to a hardware control panel. (It is anticipated that such a control panel would provide physical buttons, encoders, and a display - and would be used in addition to a software sender).

It provides support for issuing commands, spindle/feed overrides, jogging etc, and for displaying current position & status.

---
Note: This plugin is under development - and is likely to change as a reference control panel implementation is developed. 

It offers support for up to 80 input keys, and up to 4 quadrature encoders. (Somewhat arbitrary, but based on current development hardware). 

The current modbus register descriptions can be found in the docs folder, note that not all inputs are handled as yet.

---
To enable the plugin for testing;

Copy the source to a new "panel" folder in the grblHAL source tree, and define as a source folder in your IDE if necessary.

Define PANEL_ENABLE=1, either in your IDE or in my_machine.h.

Edit plugins_init.h to load the plugin.

    diff --git a/plugins_init.h b/plugins_init.h
    index bca7c28..015bc32 100644
    --- a/plugins_init.h
    +++ b/plugins_init.h
    @@ -53,6 +53,11 @@
         webui_init();
     #endif
     
    +#if PANEL_ENABLE
    +    extern void panel_init (void);
    +    panel_init();
    +#endif
    +
         my_plugin_init();
     
     #if ODOMETER_ENABLE


Increase the size of the Modbus ADU buffer in modbus.h

    diff --git a/modbus.h b/modbus.h
    index 277c27c..7e1d170 100644
    --- a/modbus.h
    +++ b/modbus.h
    @@ -24,7 +24,7 @@
     #ifndef _MODBUS_H_
     #define _MODBUS_H_
     
    -#define MODBUS_MAX_ADU_SIZE 10
    +#define MODBUS_MAX_ADU_SIZE 40
     #define MODBUS_QUEUE_LENGTH 8
     
     typedef enum {


