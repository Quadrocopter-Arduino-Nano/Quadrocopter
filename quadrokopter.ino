#include <Arduino.h>
#include <Wire.h>

/* Channel Pins */
#define ch1_pin 2
#define ch2_pin 3
#define ch3_pin 4
#define ch4_pin 5
#define ch5_pin 6
#define ch6_pin 7

/* Debugging */
int i;
int lastOutput;

/* Calibrated constants */
float angleGyroRatio = 0.9996;
float oldValueRatio = 0.9;

/* Raw gyro values */
int temp;
int gyroX, gyroY, gyroZ;
long accX, accY, accZ;

/* Gyro offset */
long gyroXOffset, gyroYOffset, gyroZOffset;


/* Processing values */
bool firstRun = true;
float gyroRoll, gyroPitch, gyroYaw;
float pitchAngleCalc, rollAngleCalc;
long accNorm;
float accPitchAngle, accRollAngle;
long timer;
int ch1_state, ch2_state, ch3_state, ch4_state, ch5_state, ch6_state;
long ch1_timer, ch2_timer, ch3_timer, ch4_timer, ch5_timer, ch6_timer;
int throttle;
boolean motorsOn;

float pitchAngle, rollAngle;
int ch1_input ,ch2_input, ch3_input, ch4_input, ch5_input, ch6_input;

float tempDifference;
float rollMemI, rollReceiverInput, rollGyroInput, rollOutput, rollLastD;
float pitchMemI, pitchReceiverInput, pitchGyroInput, pitchOutput, pitchLastD;
float yawMemI, yawReceiverInput, yawGyroInput, yawOutput, yawLastD;
int esc1, esc2, esc3, esc4;
long esc1Timer, esc2Timer, esc3Timer, esc4Timer;
long escLoopTimer;


/* ESC values */
int minESC = 1220;
int maxESC = 1800;
int minThrottle = 1000;
int maxThrottle = 1800;

/* PID calibration */
float rollGainP = 1.4;
float rollGainI = 0.04;
float rollGainD = 15.0;
int rollMaxChange = 400;

float pitchGainP = 1.4;
float pitchGainI = 0.04;
float pitchGainD = 15.0;
int pitchMaxChange = 400;

float yawGainP = 1.4;
float yawGainI = 0.04;
float yawGainD = 15.0;
int yawMaxChange = 400;


void setup() {
    Serial.begin(56700);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(13, OUTPUT);

    DDRB |= B00011110;

    Serial.print("ESCs...");
    
    for (int i = 0; i < 1250; i++) {
      PORTB |= B011110;
      delayMicroseconds(1000);
      PORTB &= B100001;
      delayMicroseconds(3000);
    }

    Serial.println("fertig!");
    
    Wire.begin(); // Start I2C

    TWBR = 12; // 400kHZ
    
    Wire.beginTransmission(0x68); // Begin communciation with gyro
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    ch1_state = 0;
    ch2_state = 0;
    ch3_state = 0;
    ch4_state = 0;
    ch5_state = 0;
    ch6_state = 0;

    rollMemI = 0;
    rollLastD = 0;
    pitchMemI = 0;
    pitchLastD = 0;
    yawMemI = 0;
    yawLastD = 0;

    cli();
    PCICR |= 0b00000100;
    PCMSK2 |= 0b11111100;
    sei();

    Serial.print("Kalibriere Gyroskop...");

    motorsOn = false;
    for (int i = 0; i < 2500; i++) {
        updateRawValues();
        gyroXOffset += gyroX;
        gyroYOffset += gyroY;
        gyroZOffset += gyroZ;
    }

    gyroXOffset /= 2500;
    gyroYOffset /= 2500;
    gyroZOffset /= 2500;
    
    Serial.println("fertig!");

    /* // Debugging
    Serial.println("ESC1,ESC2,ESC3,ESC4");
    Serial.println("throttle,pitch,roll,yaw");
    Serial.println("rollReceiverInput,gyroRoll,tempDifference,rollMemI,rollLastD,rollOutput");
    */
    digitalWrite(LED_BUILTIN, HIGH);
    timer = micros();
}

void updateRawValues() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    while (Wire.available() < 14);
    accX = Wire.read() << 8 | Wire.read();
    accY = Wire.read() << 8 | Wire.read();
    accZ = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
}

void loop() {
    /* // Debugging
    i++;

    if (millis() - lastOutput > 1000) {
        lastOutput = millis();
        Serial.println(i);
    }
    */

    gyroX -= gyroXOffset;
    gyroY -= gyroYOffset;
    gyroZ -= gyroZOffset;

    // Gyro PID input is deg/sec
    gyroRoll = (gyroRoll * 0.7) + ((gyroX / 65.5) * 0.3);
    gyroPitch = (gyroPitch * 0.7) + ((gyroY / 65.5) * 0.3);
    gyroYaw = (gyroYaw * 0.7) + ((gyroZ / 65.5) * 0.3);

    // 65.5 (datasheet)
    pitchAngleCalc += gyroX * ((1.0 / 250.0) / 65.5);
    rollAngleCalc += gyroY * ((1.0 / 250.0) / 65.5);

    // Correct roll and pitch for yaw (trigonometry)
    pitchAngleCalc += rollAngleCalc * sin(gyroZ * (((1.0 / 250.0) / 65.5) * (PI / 180.0)));
    rollAngleCalc += pitchAngleCalc * sin(gyroZ * (((1.0 / 250.0) / 65.5) * (PI / 180.0)));

    accNorm = sqrt(sq(accX) + sq(accY) + sq(accZ));
    accPitchAngle = asin((float) accY / (float) accNorm) * (1.0 / (PI / 180.0));
    accRollAngle = asin((float) accX / (float) accNorm) * (1.0 / (PI / 180.0));

    if (firstRun) {
        firstRun = false;
        pitchAngleCalc = accPitchAngle;
        rollAngleCalc = accRollAngle;
    } else {
        pitchAngleCalc = pitchAngleCalc * angleGyroRatio + accPitchAngle * (1.0 - angleGyroRatio);
        rollAngleCalc = rollAngleCalc * angleGyroRatio + accRollAngle * (1.0 - angleGyroRatio);
    }

    rollAngle = rollAngle * oldValueRatio + rollAngleCalc * (1.0 - oldValueRatio);
    pitchAngle = pitchAngle * oldValueRatio + pitchAngleCalc * (1.0 - oldValueRatio);

    rollReceiverInput = 0;
    if (ch2_input > 1508) rollReceiverInput = ch2_input - 1508;
    else if (ch2_input < 1492) rollReceiverInput = ch2_input - 1492;
    rollReceiverInput /= 3.0;

    pitchReceiverInput = 0;
    if (ch1_input > 1508) pitchReceiverInput = ch1_input - 1508;
    else if (ch1_input < 1492) pitchReceiverInput = ch1_input - 1492;
    pitchReceiverInput /= 3.0;

    yawReceiverInput = 0;
    if (ch4_input > 1508) yawReceiverInput = ch4_input - 1508;
    else if (ch4_input < 1492) yawReceiverInput = ch4_input - 1492;
    yawReceiverInput /= 3.0;
    
    calculatePID();

    throttle = map(ch3_input, 1000, 2000, minThrottle, maxThrottle);

    /*esc1 = throttle - pitchOutput + rollOutput - yawOutput;
    esc2 = throttle + pitchOutput + rollOutput + yawOutput;
    esc3 = throttle + pitchOutput - rollOutput - yawOutput;
    esc4 = throttle - pitchOutput - rollOutput + yawOutput;*/
    
    esc1 = throttle - pitchOutput + rollOutput - yawOutput;
    esc2 = throttle + pitchOutput + rollOutput + yawOutput;
    esc3 = throttle + pitchOutput - rollOutput - yawOutput;
    esc4 = throttle - pitchOutput - rollOutput + yawOutput;

    esc1 = map(esc1, 1000, 2000, minESC, maxESC);
    esc2 = map(esc2, 1000, 2000, minESC, maxESC);
    esc3 = map(esc3, 1000, 2000, minESC, maxESC);
    esc4 = map(esc4, 1000, 2000, minESC, maxESC);

    if (esc1 < minESC) esc1 = minESC;
    if (esc2 < minESC) esc2 = minESC;
    if (esc3 < minESC) esc3 = minESC;
    if (esc4 < minESC) esc4 = minESC;
    if (esc1 > maxESC) esc1 = maxESC;
    if (esc2 > maxESC) esc2 = maxESC;
    if (esc3 > maxESC) esc3 = maxESC;
    if (esc4 > maxESC) esc4 = maxESC;

    // esc1 = throttle; // Debugging

    if (ch5_input < 1250) { // Kill signal
        motorsOn = false;
        rollMemI = 0;
        rollLastD = 0;
        pitchMemI = 0;
        pitchLastD = 0;
        yawMemI = 0;
        yawLastD = 0;
    } else if (ch5_input >= 1250 && ch5_input < 1750) { // Failsafe signal
        /*esc1 = 2000;
        esc2 = 2000;
        esc3 = 2000;
        esc4 = 2000;*/
    } else {
        motorsOn = true;
    }
    
    if (!motorsOn) {
        esc1 = 1000;
        esc2 = 1000;
        esc3 = 1000;
        esc4 = 1000;
    }

    if (micros() - timer > 4050) Serial.println("Loop takes too long");
    while (micros() - timer < 4000); // ToDo: Fix overflow after 70 minutes

    timer = micros();

    PORTB |= B011110;
    
    esc1Timer = esc1 + timer;
    esc2Timer = esc2 + timer;
    esc3Timer = esc3 + timer;
    esc4Timer = esc4 + timer;
    updateRawValues();

    while ((PORTB & B011110) != B000000) {
        escLoopTimer = micros();
        if (esc1Timer <= escLoopTimer) PORTB &= B111101;
        if (esc2Timer <= escLoopTimer) PORTB &= B111011;
        if (esc3Timer <= escLoopTimer) PORTB &= B110111;
        if (esc4Timer <= escLoopTimer) PORTB &= B101111;
    }

    /*
    Serial.print(esc1);
    Serial.print(";");
    Serial.print(esc2);
    Serial.print(";");
    Serial.print(esc3);
    Serial.print(";");
    Serial.println(esc4);
    */
}

void calculatePID() {
    // Roll
    tempDifference = gyroRoll - rollReceiverInput;
    rollMemI += rollGainI * tempDifference;
    if (rollMemI > rollMaxChange) rollMemI = rollMaxChange;
    else if (rollMemI < rollMaxChange * -1) rollMemI = rollMaxChange * -1;
    rollOutput = rollGainP * tempDifference;// + rollMemI + rollGainD * (tempDifference - rollLastD);
    /*if (rollOutput > rollMaxChange) rollOutput = rollMaxChange;
    else if (rollOutput < rollMaxChange * -1) rollOutput = rollMaxChange * -1;
    rollLastD = tempDifference;*/

    // Pitch
    tempDifference = gyroPitch - pitchReceiverInput;
    pitchMemI += pitchGainI * tempDifference;
    if (pitchMemI > pitchMaxChange) pitchMemI = pitchMaxChange;
    else if (pitchMemI < pitchMaxChange * -1) pitchMemI = pitchMaxChange * -1;
    pitchOutput = pitchGainP * tempDifference;// + pitchMemI + pitchGainD * (tempDifference - pitchLastD);
    /*if (pitchOutput > pitchMaxChange) pitchOutput = pitchMaxChange;
    else if (pitchOutput < pitchMaxChange * -1) pitchOutput = pitchMaxChange * -1;
    pitchLastD = tempDifference;*/

    // Yaw
    tempDifference = gyroYaw - yawReceiverInput;
    yawMemI += yawGainI * tempDifference;
    if (yawMemI > yawMaxChange) yawMemI = yawMaxChange;
    else if (yawMemI < yawMaxChange * -1) yawMemI = yawMaxChange * -1;
    yawOutput = yawGainP * tempDifference;// + yawMemI + yawGainD * (tempDifference - yawLastD);
    /*if (yawOutput > yawMaxChange) yawOutput = yawMaxChange;
    else if (yawOutput < yawMaxChange * -1) yawOutput = yawMaxChange * -1;
    yawLastD = tempDifference;*/
}

ISR(PCINT2_vect) {
    long currentMicros = micros();
    if (ch1_state == 0 && (PIND & B00000100)) {
        ch1_state = 1;
        ch1_timer = currentMicros;
    } else if (ch1_state == 1 && !(PIND & B00000100)) {
        ch1_state = 0;
        ch1_input = currentMicros - ch1_timer;
    }

    if (ch2_state == 0 && (PIND & B00001000)) {
        ch2_state = 1;
        ch2_timer = currentMicros;
    } else if (ch2_state == 1 && !(PIND & B00001000)) {
        ch2_state = 0;
        ch2_input = currentMicros - ch2_timer;
    }

    if (ch3_state == 0 && (PIND & B00010000)) {
        ch3_state = 1;
        ch3_timer = currentMicros;
    } else if (ch3_state == 1 && !(PIND & B00010000)) {
        ch3_state = 0;
        ch3_input = currentMicros - ch3_timer;
    }

    if (ch4_state == 0 && (PIND & B00100000)) {
        ch4_state = 1;
        ch4_timer = currentMicros;
    } else if (ch4_state == 1 && !(PIND & B00100000)) {
        ch4_state = 0;
        ch4_input = currentMicros - ch4_timer;
    }

    if (ch5_state == 0 && (PIND & B01000000)) {
        ch5_state = 1;
        ch5_timer = currentMicros;
    } else if (ch5_state == 1 && !(PIND & B01000000)) {
        ch5_state = 0;
        ch5_input = currentMicros - ch5_timer;
    }

    if (ch6_state == 0 && (PIND & B10000000)) {
        ch6_state = 1;
        ch6_timer = currentMicros;
    } else if (ch6_state == 1 && !(PIND & B10000000)) {
        ch6_state = 0;
        ch6_input = currentMicros - ch6_timer;
    }
}
