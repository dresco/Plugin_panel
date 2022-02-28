todo
----

Modbus
  - reduce TX fequency if panel not responding
  - set init_ok to false if comms lost (avoid any jump on restart of panel cpu)

Misc
 - unlock does not work, as enqueuing gcode not supported in alarm state
 - agree how to access additional spindle data (power etc)
 - add handlers for remaining keydata
 - change constants to plugin settings data
 - how to handle situation where modbus data overflows receive buffer?
 - ignore but save  encoder jog position changes if received while not idle? else potential big difference at end of job..

done
----
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
