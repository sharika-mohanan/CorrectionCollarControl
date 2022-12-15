#include <SPI.h> // include SPI library for encoder
#include <math.h>

# define baud 115200 // set baud rate here
// encoder i/p, o/p pins
#define DIO 12 // Input from encoder
#define CS  10 // chip select
#define CLK 13 // clock, default
// stepper motor i/p, o/p pins
#define dirPin 8 // Direction
#define stepPin 9 // Step

#define MS1 6 // Microstepping pins
#define MS2 5
#define MS3 4

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
int Dir;
int Spd;
float Hme = 0.0;
float MinLim = 0.0;
float MaxLim = 0.0;
int Move;
int NoS; //number of steps to be moved back once limit reached

float encoder_angle;
int lim;
float Lmt;

unsigned long previousMicro = 0; 
unsigned long currentMicro = 0; // time check

boolean newData = false;

//============

void setup() {
  Serial.begin(baud);

  // Stepper Motor
  pinMode(dirPin, OUTPUT); //Direction pin
  pinMode(stepPin, OUTPUT); // Step pin

  pinMode(MS1, OUTPUT);
  digitalWrite(MS1, LOW);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS2, HIGH);
  pinMode(MS3, OUTPUT);
  digitalWrite(MS3, LOW);
  
  // encoder
  pinMode(CLK, OUTPUT); // clock output
  pinMode(DIO, INPUT); // Input from encoder
  pinMode(CS, OUTPUT); // chip select output
  digitalWrite(CS, HIGH); // Ensure chip starts inactive
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz clock

  SPI.begin(); // start SPI bus

  En_Angle();
  Serial.println(encoder_angle);
}

//============

void loop() {

  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    newData = false;
  }

  if (Move == 1) {
    En_Angle();
    Serial.println(encoder_angle);
    MoveSM();
  }

 
  Move = 0;

}

//============

// Command string should start with '<' and end with '>'

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

//============

// Command string should be seperated by ':'

void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ":");     // get the first part - Direction
  Dir = atoi(strtokIndx); //

  strtokIndx = strtok(NULL, ":"); // this continues where the previous call left off
  Spd = atoi(strtokIndx);     // convert Speed option to integer

  strtokIndx = strtok(NULL, ":");
  Hme = atof(strtokIndx);     // Home position

  strtokIndx = strtok(NULL, ":");
  MinLim = atof(strtokIndx);     // Min Lmt

  strtokIndx = strtok(NULL, ":");
  MaxLim = atof(strtokIndx);     // Max Lmt

  strtokIndx = strtok(NULL, ":");
  Move = atoi(strtokIndx);     // Move?

}

//============

void MoveSM() {
  if (Dir == 1) {
    digitalWrite(dirPin, HIGH);
  } else if (Dir == 2) {
    digitalWrite(dirPin, LOW);
  }

  if (MinLim == 0 || MaxLim == 0) {

    if (Spd == 1) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }

    if (Spd == 2) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000);
    }
  }

  if (MinLim != 0 && MaxLim != 0) {

    if (Spd == 1) {
      lim = lim_check();
      if (lim == 0) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
      }

      lim = lim_check();
      if (lim == 1) {
        delay(100);
        

        if (Dir == 1) {
          digitalWrite(dirPin, HIGH);
        } else if (Dir == 2) {
          digitalWrite(dirPin, LOW);
        }
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(500);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(500);
      }


    }

    if (Spd == 2) {
      lim = lim_check();
      if (lim == 0) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
      }

      lim = lim_check();
      if (lim == 1) {
        delay(100);
        
        if (Dir == 1) {
          digitalWrite(dirPin, HIGH);
        } else if (Dir == 2) {
          digitalWrite(dirPin, LOW);
        }
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(500);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(500);
      }


    }
  }
}

//===========

void En_Angle() {

  int buf;
  int encoder_val;

  digitalWrite(CS , LOW); // encoder active
  delayMicroseconds(0.5); // 500ns delay before starting acquisition

  buf = SPI.transfer16(0x00);  // read the 1st byte
  //buf <<= 8;                      // shift the byte to the high 8 bits
  //buf |= SPI.transfer(0x00); // read the 2nd byte and OR to the low 8 bits of data

  delayMicroseconds(1); // spec sheet says min 500ns delay before stopping acquisition
  digitalWrite(CS, HIGH); // set encoder inactive

  //int checksum = buf << 9; // bit mask first 10 bits
  encoder_val = buf >> 5; // bit mask last 6 bits to get 10 bit absolute encoder value
  encoder_angle = encoder_val * 0.3515; // 1024 resolution

}

//===========

int lim_check() {

  En_Angle();

  lim = 0;

  if (MinLim < MaxLim) {
    if (encoder_angle > MinLim && encoder_angle < MaxLim) {
      lim = 1;

      if (abs(encoder_angle - MaxLim) < abs(encoder_angle - MinLim)) {
        Dir = 2;
      }

      if (abs(encoder_angle - MaxLim) > abs(encoder_angle - MinLim)) {
        Dir = 1;
      }
    }
  }

  if (MinLim > MaxLim) {
    if (encoder_angle > MinLim || encoder_angle < MaxLim) {
      lim = 1;
      if (encoder_angle < MaxLim) {
        Dir = 2;
      }
      if (encoder_angle > MinLim) {
        Dir = 1;
        if (abs(encoder_angle - MaxLim) > 350) {
          Dir = 2;
        }
      }
    }
  }

  // if (encoder_angle == MinLim || encoder_angle == MaxLim) {
  //  lim = 0;
  // }

  // if (abs(encoder_angle - MinLim) < 5 || abs(encoder_angle - MinLim) < 5) {
  //  lim = 0;
  // }

  return lim;

}
