#include "fastnet.h"

#define FASTNET_ENABLE 19
#define FASTNET_BAUD   28800

Fastnet *fastnet;

bool display_registered = false;
uint8_t display_id = 0xFF;

enum readstate_t { LOST, SEEKING, HEADER, FILLING, DATA };
readstate_t read_state = LOST;

uint8_t rbuf[256];
uint8_t *p = rbuf;
uint8_t *q = rbuf;
uint32_t frame_time = 0;

Frame last_rec;


uint8_t checksum(uint8_t *data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += data[i];
  return ~sum + 1;
}

void fastnet_set_backlight(uint8_t level) {
  buf[0] = 0xFF;          // BROADCAST
  buf[1] = addr;          // source
  buf[2] = 0x02;          // payload length
  buf[3] = 0xF6;          // CM_SET_BACKLIGHT
  buf[4] = checksum(buf, 4);

  buf[5] = level;         // 0–15
  buf[6] = 0x20;          // magic constant (from repo)
  buf[7] = checksum(buf + 5, 2);

  digitalWrite(FASTNET_ENABLE, HIGH);
  Serial1.write(buf, 8);
  Serial1.flush();
  digitalWrite(FASTNET_ENABLE, LOW);

  seq++;
}


uint8_t data[256] = { 0 };
uint8_t rbuf[256] = { 0 };
uint8_t *p = rbuf;
uint8_t *q = rbuf;
uint64_t time = 0;

Frame last_rec;

enum readstate_t { LOST, SEEKING, HEADER, FILLING, DATA };
readstate_t read_state = LOST;


void process_frame() {
  char buf[32];

  Serial.print(last_rec.print());

  if(last_rec.command() == CM_SET_BACKLIGHT) {
    if(last_rec[0] == 0) Serial.println("Backlight Off");
    else if(last_rec[0] == 1) Serial.println("Backlight Low");
    else if(last_rec[0] == 2) Serial.println("Backlight Med");
    else if(last_rec[0] == 4) Serial.println("Backlight High");
  } else if(last_rec.command() == CM_CHANGE_PAGE) {
    Serial.print("Seq ID: ");
    Serial.println(last_rec[0]);
    if(last_rec[1] == 0) Serial.println("Next page");
    else if(last_rec[1] == 1) Serial.println("Previous page");
    else if(last_rec[1] == 0xff) Serial.println("Select page");
  } else if(last_rec.command() == CM_CURRENT_PAGE) {
    Serial.print("Seq ID: ");
    Serial.println(last_rec[0]);
    Serial.print("Page: ");
    Serial.println(last_rec[1]);
    fastnet->set_page(last_rec[1]);
    Serial.print("Channel: ");
    Channel::label(last_rec[2], buf);
    Serial.println(buf);
    Serial.print("Node: ");
    Node::name(last_rec[3], buf);
    Serial.println(buf);
    Serial.print("\nLabel: ");
    Serial.write(&last_rec[4], 8);
    Serial.print("\nUnits: ");
    Serial.write(&last_rec[12], 2);
  } else if(last_rec.command() == CM_HELLO) {
    fastnet->register_device(last_rec.from());
  }
}



void read() {
  static uint8_t l;

  digitalWrite(ENABLE, LOW);

  if(Serial1.available())
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);

  switch(read_state) {
    case LOST:
      if(!Serial1.available()) {
        read_state = SEEKING;
      } else if(Serial1.peek() == 0xff) {
        memset(rbuf, 0, 256);
        read_state = SEEKING;
      } else {
        Serial1.read();
      }

      break;
    case SEEKING:
      if(Serial1.available() > 4) {
        p = rbuf;
        q = rbuf;
        p += Serial1.readBytes(p, 5);

        if(fastnet->checksum(q, 5)) {
          read_state = LOST;
          break;
        }

        read_state = HEADER;
      }

      break;
    case HEADER:
      last_rec.set_header(q[0], q[1], q[2], q[3]);
      l = q[2];

      if(l == 0xff) {
        read_state = LOST;
        break;
      } else if(l == 0) {
        p = q;
        process_frame();
        read_state = SEEKING;
      } else {
        read_state = FILLING;
        time = millis();
        q += 5;
      }

      break;
    case FILLING:
      if(abs(millis() - time) > 1000) {
        read_state = SEEKING;
        break;
      }

      if(Serial1.available() > l)
        p += Serial1.readBytes(p, l + 1);

      if(p >= q + l)
        read_state = DATA;

      break;
    case DATA:
      read_state = SEEKING;
      last_rec.set_data(q);
      process_frame();

      break;
    default:
      break;
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(ENABLE, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);

  fastnet = new Fastnet(0x2F);
  fastnet->who();
  delay(100);
}

static uint16_t true_wind_ang = 3500 ;


void loop() {
    Serial.print("Setting true wind ");
    Serial.println(true_wind_ang);

    fastnet->true_wind(true_wind_ang,22);
    true_wind_ang += 10;
    if(true_wind_ang > 3600){
         Serial.print("Reset true wind ");
        true_wind_ang = 0;
    } 
    read();

    fastnet->backlight(4);
    delay(500);

    Serial.print("read_state: ");
    Serial.println(read_state);
 
    //fastnet->set_page(1);
    delay(500);

    if (last_rec.command() == CM_HELLO) {
        Serial.print("Display ID detected: 0x");
        Serial.println(last_rec.from(), HEX);
    }
    delay(500);
   
}
