This has been made possible by multiple MFL reading examples by pazi88 and alkaba at the ms4x discord and speeduino code found
Also by muki01, piersholt and curious.ninja/ for their l/k-bus research

https://github.com/piersholt/wilhelm-docs
https://github.com/muki01/I-K_Bus
https://curious.ninja/blog/arduino-bmw-i-bus-interface-messages/
https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu-m54_PnP/Dev/MFL/MFL.ino
https://github.com/just-oblivious/arduino-ibustrx



Pin mapping ESP32-c3
0-7 in, buttons using the buttonmapping
8 out, PWM for brightness of the button's leds
9 out, Cruise control single wire out
10 in, mode switch
20-21, i/k Bus rx tx

PWM for the leds
              +12V
                |
      +----------+----------+----------+----------+
      |          |          |          |          |
    LED1+      LED2+      LED3+      LED4+        (x8)
    LED1-      LED2-      LED3-      LED4-
      |          |          |          |
      +----------+----------+----------+----------+
                         |
                       D  IRLB8721
                       S ───────── GND
                       G ──[100Ω]── ESP32 PWM pin
                              |
                             10k
                              |
                             GND
