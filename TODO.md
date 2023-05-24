# todo

CAN bus
 - add inital support for canbus plugin - WIP
 - use PANEL_ENABLE value to compile for either modbus or canbus

Modbus
 - how to handle situation where modbus data overflows receive buffer?

Misc
 - change constants to plugin settings nvr data
 - retrieve panel software version, to display with $I
 - reduce TX fequency if panel not responding
 - set init_ok to false if comms lost (avoid any encoder jump on restart of panel cpu)
 - add handlers for remaining keydata
 - ignore but save encoder jog position changes if received while not idle? else potential big difference at end of job..
 - add support for multi-axis kepypad jogging
 - add support for up to 8 axis

Issues
 - occasionally getting stuck in jog mode on keypad/encoder jog
   seen on Nucleo dev board only, not seen on Teensy..

# done

- no crc check on received data?
- huanyang checksum support - or not?
- check length of write NAK response - shorter, will hit timeout
- support multiple encoders, common functions
- Teensy carrier board for rs485 module 
- output register definitions
- handlers for output data
- sys.position is abosulte machine coords, how to deal with wcs offsets?
- add rapid  override encoder processing
- add acceleration ramp to smooth keypad jog
- MODBUS_MAX_ADU_SIZE should be defined in IDE, update readme file accordingly..
- unlock does not work, as enqueuing gcode not supported in alarm state
- agree how to access additional spindle data (power etc)
- unlock not working (HLP prompt on console)
- frequent spindle alarm 14 when changing spindle override on router, from either gui or panel
  does not happen on bench setup with or without panel present..
  electrical noise - resolved
- modbus issues if using common processDisplayData() code
