# B&G Network and 2020

## Nmea 0183 to Fastnet Protocol Convertor

bench testing

nano reading nema0183 and sending it on fastnet Network display


## Nema0183 to Fastnet Network instrments converter 
### Nano Wiring

### Conponents
RS-232 
Treedix 2pcs Mini RS232 to TTL MAX3232 Convert Adapter Board Transceiver Breakout Board 3v-5v
https://www.amazon.co.uk/dp/B09BCKWL8V?ref=ppx_yo2ov_dt_b_fed_asin_title

Nano
ELEGOO Nano Board 3pcs CH 340 Compatible with Arduino Nano V3.0
https://www.amazon.co.uk/dp/B072BMYZ18?ref=ppx_yo2ov_dt_b_fed_asin_title


Rs323   --> Nano
```
R1 in  - Nema0183 feed

Neg    --> GND (b4)

+5v    --> 5v (A4)

R1 out --> D8 (B11)
```

Nano

```
Gnd (A2) common ground
5v (a4) to Rs232
Tx1 (b2) to Fastnet
GND (b4) to rs232
D8 to rs232 R1 out
```

Reset optional 
```
RST (a3) to gnd to reset
```



### 2020CD Display RS485 input

RS-232 
Treedix 2pcs Mini RS232 to TTL MAX3232 Convert Adapter Board Transceiver Breakout Board 3v-5v
https://www.amazon.co.uk/dp/B09BCKWL8V?ref=ppx_yo2ov_dt_b_fed_asin_title

RS485
Fasizi 10pcs 5V MAX485 / RS485 Module TTL to RS-485 MCU Development Board
https://www.amazon.co.uk/dp/B09Z26ZY1X?ref=ppx_yo2ov_dt_b_fed_asin_title

Nano TBD

ELEGOO Nano Board 3pcs CH 340 Compatible with Arduino Nano V3.0
https://www.amazon.co.uk/dp/B072BMYZ18?ref=ppx_yo2ov_dt_b_fed_asin_title

### Mega ATM

### RS232 (nmea input) 
```
R1 in  - Nema0183 feed
Neg    --> GND (a4)
+3v    --> 3v (b4)
R1 out --> 48 
```

### RS485

```
GND  -- > GND B6
+5v  -- > 5v  B5
A    --> display White
B    --> display green

D1  --> Tx1 18 (B-4)
DE	--> 13 flow control (b5)
RE  --^
R0  --> Rx1 19 (b-3)
```

### Mega 
Shared grounds all input/output through the two boards


## Nano wiring TBD...
```
R1 in  - Nema0183 feed
Neg    --> GND (a4)
+3v    --> 5v (b4)
R1 out --> 
```

RS485
```
GND  -- > GND 
+5v  -- > 5v  
A    --> display White
B    --> display green

D1  --> Tx1 
DE	--> flow control 
RE  --^
R0  --> Rx1 
```

Nano TBD




### NOTES
Read nmea using MAX3232 board

   SoftwareSerial nmeaSerial(8, 9);  // RX = 8, TX = 9 (TX normally unused)
   
   
   voltage
   awa
   aws
   depth
   speed SOG
   
   water temp C
   VMG
   
   
   Sent: $IIVMG,5.4,N*3D

Sent: $WIMWV,85.0,R,19.5,N,A*23

Sent: $IIDPT,36.7,0.0*72

Sent: $IIVTG,,T,,M,10.9,N,20.2,K*51

Sent: $IIXDR,U,13.0,V,MAIN*5A

Sent: $IIXDR,C,20.9,C,WATER*0E

Sent: $IIVMG,5.4,N*3D

Sent: $WIMWV,90.0,R,20.0,N,A*28

Sent: $IIDPT,37.0,0.0*74

Sent: $IIVTG,,T,,M,11.0,N,20.4,K*5F

Sent: $IIXDR,U,13.1,V,MAIN*5B

end B&G wiring


### Bridge wiring code
```
wiring sheild -
blue -
brown + 
yellow green Fastnet
```




