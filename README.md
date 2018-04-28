# c2000_OLED_SSD1306

A Texas Instruments C2000 (F28027F, F28069M,...) code that displays on a small 0.98" OLED SSD1306 variables and text

Simply connect the I2C bus SDA/SCL and 3.3V / GND pins
**More info**
  - https://e2e.ti.com/support/microcontrollers/c2000/f/171/t/676395

You have to adapt the main.c to your code

UPDATE: 28/04/2018

Better I2C timing
now I use RM=1 mode so I generate manually STOP bit, I still use the 4-level FIFO
**More info**
  - https://e2e.ti.com/support/microcontrollers/c2000/f/171/p/682830/2517100#2517100
