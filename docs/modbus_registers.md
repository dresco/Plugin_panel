**16 bit InputRegisters - keypress and encoder information sent to grbl controller**

Address | Type | Description
--|--|--
100 | unsigned | debug (ticks high)
101 | unsigned | debug (ticks low)
102  | unsigned | Encoder_1
103  | unsigned | Encoder_2
104  | unsigned | Encoder_3
105  | unsigned | Encoder_4
106   | bitfield | Keypad_1
107   | bitfield | Keypad_2
108   | bitfield | Keypad_3
109   | bitfield | Keypad_4
110   | bitfield | Keypad_5
111   | bitfield | Keypad_6
<br>

**16 bit HoldingRegisters - data from grbl controller to be displayed on panel**

Address | Type | Description
--|--|--
100 | unsigned | grbl state
101 ||-
102 | unsigned | spindle speed
103 | unsigned | spindle load
104 | 2 x 8bit unsigned | spindle override & active wcs
105 | 2 x 8bit unsigned | feed override & rapid override
106 | 2 x 8bit unsigned | jog mode & mpg mode
107 |16bits of 32bit float data| x axis position
108 |16bits of 32bit float data| x axis position
109 |16bits of 32bit float data| y axis position
110 |16bits of 32bit float data| y axis position
111 |16bits of 32bit float data| z axis position
112 |16bits of 32bit float data| z axis position
113 |16bits of 32bit float data| a axis position
114 |16bits of 32bit float data| a axis position
115 |16bits of 32bit float data| b axis position
116 |16bits of 32bit float data| b axis position
