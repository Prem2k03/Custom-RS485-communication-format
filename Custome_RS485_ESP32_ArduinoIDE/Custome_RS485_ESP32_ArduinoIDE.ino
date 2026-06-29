#include "Custom_RS_485.h"

// 1. Create the interrupt callback function
void onRs485Receive() {
  // slave_process() handles one byte per call, so we loop 
  // until the hardware buffer is empty.
  while (Serial2.available()) {
    slave_process();
  }
}

void setup() {
  Serial.begin(115200);
  
  // 2. Initialize your custom RS-485 implementation
  rs485_begin(Serial2, 4, 115200, 16, 17);  // RS-485 on UART2, RE_DE=4 
  slave_begin(2, 4); 

  // 3. Attach the interrupt callback to Serial2
  // This tells the ESP32 to run onRs485Receive() whenever data arrives.
  Serial2.onReceive(onRs485Receive); 
}

void loop() {
  // 4. Your loop is now completely free of polling!
  slave_write_register(0, 5);
  Serial.print("Reg Value : ");
  Serial.println(slave_read_register(1));
  
}