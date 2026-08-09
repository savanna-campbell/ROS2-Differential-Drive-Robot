#include <Wire.h>
#include <TinyGPSPlus.h>

// SETUP MOTORS
int PWMB = 11;
int PWMA = 10;
int BIn1 = 45;
int BIn2 = 44;
int AIn1 = 36;
int AIn2 = 37;
int STBY = 41;
// setup encoders
const byte LEFT_PHASE_A = 3;
const byte LEFT_PHASE_B = 5;

const byte RIGHT_PHASE_A = 2;
const byte RIGHT_PHASE_B = 4;

volatile long left_ticks = 0;
volatile long right_ticks = 0;

// SETUP IMU
const int IMU_ADDR = 0x68;
int16_t ax, ay, az; // accelerometer data
int16_t gx, gy, gz; // gyroscope data
char logBuffer[150];


// SETUP SERIAL COMMANDS
const byte numChars = 32;
char receivedChars[numChars];
bool newData = false;

// SETUP GPS
const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

// streaming variables
bool streaming = false;
unsigned long stream_interval = 100; // current default, given in milliseconds
unsigned long last_stream_time = 0;

void setup() {
  // START SERIAL 
  Serial.begin(115200);
  // dedicated gps serial
  Serial2.begin(GPSBaud);
  Serial.println("<Serial ready>");

  // START IMU
  startIMU();

  // START MOTORS
  startMotors();

  

}

void loop() {
  
  
  recvEndWithMarker();
  processCommand();

  // UPDATE GPS
  update_gps();

  // stream
  stream_state();

}


// STARTING COMMANDS
void startMotors() {
  // general motor startup
  pinMode(PWMB, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIn1, OUTPUT);
  pinMode(BIn2, OUTPUT);
  pinMode(AIn1, OUTPUT);
  pinMode(AIn2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // general encoder startup
  pinMode(LEFT_PHASE_A, INPUT_PULLUP);
  pinMode(LEFT_PHASE_B, INPUT_PULLUP);
  pinMode(RIGHT_PHASE_A, INPUT_PULLUP);
  pinMode(RIGHT_PHASE_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_PHASE_A), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_PHASE_A), readRightEncoder, RISING);
}

void startIMU() {
  // initialize I2C
  Wire.begin();

  // Wake IMU
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

// SERIAL COMMANDS
void recvEndWithMarker()
{
  static byte idx = 0;
  char endMarker='\n';
  char rc;

  while(Serial.available() > 0 && newData == false){
    rc = Serial.read();

    if (rc != endMarker){
      receivedChars[idx] = rc;
      idx++;
      if (idx >= numChars) {
        idx = numChars - 1;
      }
    }
    else {
      receivedChars[idx] = '\0';
      idx = 0;
      newData = true;
    }
  }
}

void processCommand() {
  if (newData == true) {

    char* command = strtok(receivedChars, " ");

    Serial.println(command);
    if(strcmp(command, "PING") == 0) {
      ping();
    }
    else if(strcmp(command, "STOP") == 0) {
      stop();
    }
    else if(strcmp(command, "STATUS") == 0) {
      status();
    }
    else if(strcmp(command, "SET_MOTORS") == 0) {
      int left_motor = atoi(strtok(NULL, " "));
      int right_motor = atoi(strtok(NULL, " "));
      set_motors(left_motor, right_motor);
    }
    else if(strcmp(command, "F") == 0) {
      int motor_speed = atoi(strtok(NULL, " "));
      set_motors(motor_speed, motor_speed);
    }

    else if(strcmp(command, "B") == 0) {
      int motor_speed = atoi(strtok(NULL, " "));
      set_motors(-motor_speed, -motor_speed);
    }

    else if(strcmp(command, "GET_IMU") == 0) {
      get_imu();
    }

    else if(strcmp(command, "GET_GPS") == 0) {
      get_gps();
    }
    
    else if(strcmp(command, "GET_ENCODERS") == 0) {
      get_encoders();
    }
    
    else if(strcmp(command, "RESET_ENCODERS") == 0) {
      reset_encoders();
    }

    else if(strcmp(command, "GET_STATE") == 0) {
      get_state();
    }

    else if(strcmp(command, "START_STREAM") == 0) {
      int ms = atoi(strtok(NULL, " "));
      start_stream(ms);
    }
    else if(strcmp(command, "STOP_STREAM") == 0) {
      streaming = false;
    }

    newData = false;
  }
}
void ping() {
  Serial.println("PONG");
}

void stop(){
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  Serial.println("stopped");
}

void status(){
  
  Serial.println("Status is...");
}

void set_motors(int left, int right) {
  digitalWrite(STBY, HIGH);

  if(right < 0) {
    digitalWrite(BIn1, HIGH);
    digitalWrite(BIn2, LOW);
  }
  else {
    digitalWrite(BIn1, LOW);
    digitalWrite(BIn2, HIGH);
  }
  if (left < 0){
    digitalWrite(AIn1, LOW);
    digitalWrite(AIn2, HIGH);
  }
  else {
    digitalWrite(AIn1, HIGH);
    digitalWrite(AIn2, LOW);
  }
  
  
  analogWrite(PWMA, abs(left));
  analogWrite(PWMB, abs(right));
}

void read_imu() {

  // grab starting register
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  // grab consecutive bytes to see accelerometer and gyroscope data
  Wire.requestFrom(IMU_ADDR, 14, true);

  // combine the high and low data
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  int16_t rawTemp = (Wire.read() << 8) | Wire.read(); // skip temperature values

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  

}
void get_imu(){
  read_imu();
  sprintf(logBuffer, "Accel X Y Z: %d %d %d | Gyro X Y Z: %d %d %d\n", ax, ay, az, gx, gy, gz);
  Serial.println(logBuffer);

}

void update_gps() {
  while(Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
}

void get_gps() {
  uint32_t satellitesConnected = gps.satellites.value();

  if (gps.location.isValid()){
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    float hdop = gps.hdop.value() / 100.0;
    
    uint32_t fixAge = gps.location.age();

    sprintf(logBuffer, "GPS: lat lon sat fixAge hdop: %s %s %lu %lums %s\n", 
            String(latitude,6).c_str(), 
            String(longitude, 6).c_str(), 
            satellitesConnected, 
            fixAge, 
            String(hdop, 2).c_str());
    Serial.println(logBuffer);
  }
  else {
    sprintf(logBuffer, "No lock. Tracking %d satellites", satellitesConnected);
    Serial.println(logBuffer);
  }
}

void readLeftEncoder() {
  if (digitalRead(LEFT_PHASE_B) == HIGH) {
    left_ticks--;
  }
  else {
    left_ticks++;
  }
}

void readRightEncoder() {
  if(digitalRead(RIGHT_PHASE_B) == LOW) {
    right_ticks--;
  }
  else {
    right_ticks++;
  }
  
}

void get_encoders() {
  noInterrupts();
  long lt = left_ticks;
  long rt = right_ticks;
  interrupts();
  Serial.print("Right encoder: "); Serial.println(rt);
  Serial.print("Left encoder: "); Serial.println(lt);
}

void reset_encoders() {
  noInterrupts();
  right_ticks = 0;
  left_ticks = 0;
  interrupts();
}

// STATE time_ms left_count right_count ax ay az gx gy gz lat lon gps_fix
void get_state() {
  unsigned long time_ms = millis();

  noInterrupts();
  long lt = left_ticks;
  long rt = right_ticks;
  interrupts();

  read_imu();

  bool fix = gps.location.isValid();
  float lat = fix ? gps.location.lat() : 0.0;
  float lon = fix ? gps.location.lng() : 0.0;
  

  Serial.print("STATE ");
  Serial.print(time_ms);    Serial.print(",");
  Serial.print(lt);         Serial.print(",");
  Serial.print(rt);         Serial.print(",");
  Serial.print(ax);         Serial.print(",");
  Serial.print(ay);         Serial.print(",");
  Serial.print(az);         Serial.print(",");
  Serial.print(gx);         Serial.print(",");
  Serial.print(gy);         Serial.print(",");
  Serial.print(gz);         Serial.print(",");
  Serial.print(lat, 6);     Serial.print(",");
  Serial.print(lon, 6);     Serial.print(",");
  Serial.println(fix ? 1 : 0);

}

void start_stream(int ms) {
  streaming = true;
  stream_interval = ms;
  last_stream_time = millis();

}

void stream_state() {
  if(!streaming) return;

  unsigned long now = millis();
  if (abs(now - last_stream_time) > stream_interval) {
      last_stream_time = now;
      get_state();
  }
}
