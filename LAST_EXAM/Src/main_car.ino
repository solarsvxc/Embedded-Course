#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


struct Packet {
  int16_t x;
  int16_t y;
  int16_t z;
  char msg[6]; 
};

Packet rx_packet;

RF24 radio(7, 8); // CE, CSN
uint8_t nrf_addr[5] = {0x31, 0x32, 0x33, 0x34, 0x35}; 

const int ENA = 3; const int IN1 = 2; const int IN2 = 4;
const int ENB = 9; const int IN3 = 5; const int IN4 = 6;


const int deadzone = 1500; 

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  if (!radio.begin()) {
    Serial.println("nRF24L01 không hoạt động!");
    while (1);
  }
  radio.openReadingPipe(1, nrf_addr);
  radio.setChannel(80);           
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  
  Serial.println("Đang chờ tín hiệu từ STM32...");
}

void loop() {
  if (radio.available()) {
    radio.read(&rx_packet, sizeof(Packet));

    // Raw data của ICM-20948 khoảng +/- 16384 cho 1G
    int speed = map(rx_packet.x, -16000, 16000, -255, 255);
    int steering = map(rx_packet.y, -16000, 16000, -255, 255);

    // Áp dụng khoảng chết cho raw data trước khi tính toán
    if (abs(rx_packet.x) < deadzone) speed = 0;
    if (abs(rx_packet.y) < deadzone) steering = 0;

    // Tính toán tốc độ từng bên
    int leftMotor = speed + steering;
    int rightMotor = speed - steering;

    moveMotors(leftMotor, rightMotor);

    
    Serial.print("MSG: "); Serial.print(rx_packet.msg);
    Serial.print(" | X: "); Serial.print(rx_packet.x);
    Serial.print(" | Y: "); Serial.println(rx_packet.y);
  } else {
    moveMotors(0, 0);
  }
}

void moveMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Motor trái
  if (left > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (left < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, abs(left));

  // Motor phải
  if (right > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (right < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, abs(right));
}