// Copyright 2023-2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ESP32
  #error This example runs on ESP32
#endif

#define CONFIG_LOG_DEFAULT_LEVEL CONFIG_LOG_DEFAULT_LEVEL_ERROR

#include <LDS_RPLIDAR_A1.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>

const uint8_t LDS_MOTOR_EN_PIN = 19; // ESP32 Dev Kit LDS enable pin
const uint8_t LDS_MOTOR_PWM_PIN = 15; // LDS motor speed control using PWM
#define LDS_MOTOR_PWM_FREQ    1
#define LDS_MOTOR_PWM_BITS    11
#define LDS_MOTOR_PWM_CHANNEL    2 // ESP32 PWM channel for LDS motor speed control

HardwareSerial LdSerial(2); // TX 17, RX 16
LDS_RPLIDAR_A1 lds;

const char *ssid = "ESP32RPLidarA1";
const char *password = "esp32lidar";

const char *udp_send_address = "192.168.4.255";
const int udp_port = 3333;

WiFiUDP udp;

unsigned int connected_clients = 0;

void setup() {
  Serial.begin(115200);
  lds.setScanPointCallback(lds_scan_point_callback);
  lds.setPacketCallback(lds_packet_callback);
  lds.setSerialWriteCallback(lds_serial_write_callback);
  lds.setSerialReadCallback(lds_serial_read_callback);
  lds.setMotorPinCallback(lds_motor_pin_callback);
  lds.init();

  wifiInit();

  Serial.println("Start LDS...");
  Serial.println(LdSerial.setRxBufferSize(1024)); // must be before .begin()
  Serial.print("LDS RX buffer size "); // default 128 hw + 256 sw
  uint32_t baud_rate = lds.getSerialBaudRate();
  Serial.print("LDS baud rate ");
  Serial.println(baud_rate);

  LdSerial.begin(baud_rate);
  while (LdSerial.read() >= 0);

  LDS::result_t result = lds.start();
  Serial.print("LDS init() result: ");
  Serial.println(lds.resultCodeToString(result));

  if (result < 0)
    Serial.println("WARNING: is LDS connected to ESP32?");


}

int wifiInit() {

  Serial.println("Configuring access point...");
  if (!WiFi.softAP(ssid, password)) {
    Serial.println("Soft AP creation failed.");
    while(1);
    
  }
    
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  Serial.println("Access point started!");

  Serial.println("Initialize UDP...");
  //initializes the UDP state
  //This initializes the transfer buffer
  udp.beginMulticast(IPAddress(*udp_send_address), udp_port);

  Serial.println("Configure WiFi Event Handler...");
  WiFi.onEvent(WiFiEventHandler);
  
  return 0;
}

#define UDP_MAX_CLIENTS 10
uint32_t udp_client_ips[UDP_MAX_CLIENTS] = {0};

void WiFiEventHandler(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch(event) {
      case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
          //When connected set 
          Serial.println("Client connected");
          connected_clients++;
          break;
      case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED: {
          ip_event_ap_staipassigned_t *ip_info = (ip_event_ap_staipassigned_t*) &info;
          Serial.print("Assigned IP address to client: ");
          Serial.println();
          Serial.println(ip_info->ip.addr);
          for (uint32_t idx = 0; idx < UDP_MAX_CLIENTS; idx++) {
            if (udp_client_ips[idx] == 0) {
              udp_client_ips[idx] = ip_info->ip.addr;
              break;
            }
          }
      } break;
      case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
          Serial.println("Client disconnected");
          if (connected_clients) connected_clients--;
          break;
      default: break;
    }
}

int lds_serial_read_callback() {
  return LdSerial.read();
}

size_t lds_serial_write_callback(const uint8_t * buffer, size_t length) {
  return LdSerial.write(buffer, length);
}

void udp_send_packet(uint32_t ip_addr, uint32_t i, float angle_deg, float distance_mm, float quality, bool scan_completed) {
    uint8_t tx_buffer[17] = {0};
    
    // measurement index
    tx_buffer[0] = i >> 24;
    tx_buffer[1] = i >> 16;
    tx_buffer[2] = i >> 8;
    tx_buffer[3] = i;

    Serial.printf("idx: %d\n", i);

    // distance_mm
    uint32_t distance_bytes = float_to_uint32(distance_mm);
    tx_buffer[4] = distance_bytes >> 24;
    tx_buffer[5] = distance_bytes >> 16;
    tx_buffer[6] = distance_bytes >> 8;
    tx_buffer[7] = distance_bytes;
    
    // angle_deg
    uint32_t angle_bytes = float_to_uint32(angle_deg);
    tx_buffer[8] = angle_bytes >> 24;
    tx_buffer[9] = angle_bytes >> 16;
    tx_buffer[10] = angle_bytes >> 8;
    tx_buffer[11] = angle_bytes;
    
    // quality
    uint32_t quality_bytes = (uint32_t)quality;
    tx_buffer[12] = quality_bytes >> 24;
    tx_buffer[13] = quality_bytes >> 16;
    tx_buffer[14] = quality_bytes >> 8;
    tx_buffer[15] = quality_bytes;
    
    // completed
    tx_buffer[16] = scan_completed ? 0xff : 0x00;

    udp.beginPacket(ip_addr, udp_port);
    udp.write(tx_buffer, 17);
    udp.endPacket();
    udp.flush();
}

void lds_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed) {
  static uint32_t i=0;

  /*
   *  Packet layout
   *  everything is MSB!
   *  Byte:        | 0              ...          3 | 4        ...          7 | 8        ...          11 | 12           ...             15 |          16          |
   *  Description: | measurement index as uint32_t | distance in mm as float | angle in degree as float | measurement quality as uint32_t | completed as boolean |
   */

  if (!connected_clients) return;
  for (uint32_t idx = 0; idx < UDP_MAX_CLIENTS; idx++) {
    if (udp_client_ips[idx] != 0)
      udp_send_packet(udp_client_ips[idx], i, angle_deg, distance_mm, quality, scan_completed);
  }
  i++;
}

uint32_t float_to_uint32(float f) {
  union {
    float f;
    uint32_t u;
  } fu = { .f = f };

  return fu.u;
}

void lds_info_callback(LDS::info_t code, String info) {
  Serial.print("LDS info ");
  Serial.print(lds.infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lds_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("LDS error ");
  Serial.print(lds.resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

void lds_motor_pin_callback(float value, LDS::lds_pin_t lds_pin) {
  int pin = (lds_pin == LDS::LDS_MOTOR_EN_PIN) ?
    LDS_MOTOR_EN_PIN : LDS_MOTOR_PWM_PIN;

  if (value <= LDS::DIR_INPUT) {
    // Configure pin direction
    if (value == LDS::DIR_OUTPUT_PWM) {
      ledcSetup(LDS_MOTOR_PWM_CHANNEL, LDS_MOTOR_PWM_FREQ, LDS_MOTOR_PWM_BITS);
      ledcAttachPin(pin, LDS_MOTOR_PWM_CHANNEL);
    } else
      pinMode(pin, (value == LDS::DIR_INPUT) ? INPUT : OUTPUT);
    return;
  }

  if (value < LDS::VALUE_PWM) // set constant output
    digitalWrite(pin, (value == LDS::VALUE_HIGH) ? HIGH : LOW);
  else { // set PWM duty cycle
    int pwm_value = ((1<<LDS_MOTOR_PWM_BITS)-1)*value;
    ledcWrite(LDS_MOTOR_PWM_CHANNEL, pwm_value);
  }
}

void lds_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {
  return;
}

void loop() {
  lds.loop();
}
