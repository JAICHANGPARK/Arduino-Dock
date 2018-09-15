
#define DEVICE_NAME                       "BLE Serial"
#define TXRX_BUF_LEN                      20

static uint8_t rx_buf[TXRX_BUF_LEN];
static uint8_t rx_buf_num;
static uint8_t rx_state = 0;

// handle when rx event
void uart_handle(uint32_t id, SerialIrq event) {
  Serial.print("Serial event Occuer");
  // Serial receive byte event
  //  if (event == RxIrq) {
  //    // print the receive byte
  //    if (Serial.available()) {
  //      Serial.print("Serial event Occuer");
  //      Serial.write(Serial.read());
  //      Serial.println("End");
  //    }
  //  }
  if (event == RxIrq) {
    if (rx_state == 0) {
      rx_state = 1;
      //      timeout.attach_us(m_uart_rx_handle, 100000);
      rx_buf_num = 0;
    }
    while (Serial.available()) {
      if (rx_buf_num < 20) {
        rx_buf[rx_buf_num] = Serial.read();
        rx_buf_num++;
      }
      else {
        Serial.read();
      }
    }
    for (int i = 0; i < rx_buf_num; i++) {
      Serial.println(rx_buf[i], HEX);
    }

  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.attach(uart_handle);
  Serial.println("Serial IRQ Handle demo ");
  Serial.println("Please input string to Serial com :");
  pinMode(D13, OUTPUT);
}

void loop() {
  uint8_t serialNumberReadCommand[7] = {0x8b, 0x11, 0x20, 0x13, 0x24, 0x10, 0x2a};
  uint8_t savaDataCountGetCommand[7] = {0x8b, 0x11, 0x20, 0x18, 0x26, 0x10, 0x22};


  // put your main code here, to run repeatedly:
  digitalWrite(D13, HIGH);
  Serial.write(0x80);
  delay(2000);
  digitalWrite(D13, LOW);
  Serial.write(serialNumberReadCommand, 7);
  delay(2000);
  digitalWrite(D13, HIGH);
  Serial.write(savaDataCountGetCommand, 7);
  delay(2000);
  digitalWrite(D13, LOW);
  //  Serial.write(serialNumberReadCommand, 7);
  delay(2000);
}

