#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Servo.h>

const byte TEENSY_LED_PIN = 13; // used for debugging break beam sensor

// SENSORS
const byte IR_RECEIVER_PIN = 12;
const byte BREAK_BEAM_PIN = 10;



// ACTUATION
const byte SERVO_PIN = 6;

// camera stepper motor
const byte CAM_STEPPER_EN_PIN = 0;
const byte CAM_STEPPER_DIR_PIN = 1;
const byte CAM_STEPPER_STEP_PIN = 2;

// staging stepper motor
const byte STAGE_STEPPER_EN_PIN = 3;
const byte STAGE_STEPPER_DIR_PIN = 4;
const byte STAGE_STEPPER_STEP_PIN = 5;


// BLDC motor control (flywheel and drivetrain)
const uint32_t CAN_PACKET_DUTY_CYCLE = 0;
const uint32_t CAN_PACKET_CURRENT = 1;

const byte TOP_FLY_CAN_ADDR = 27;
const byte BOT_FLY_CAN_ADDR = 91;

const byte DRIVE_FR_CAN_ADDR = 84;
const byte DRIVE_BL_CAN_ADDR = 79;
const byte DRIVE_FL_CAN_ADDR = 94;
const byte DRIVE_BR_CAN_ADDR = 72;

float duty_drive_FR = 0.0;
float duty_drive_BL = 0.0;
float duty_drive_FL = 0.0;
float duty_drive_BR = 0.0;

bool flywheels_running = false;

// durations in milliseconds
const uint16_t FLYWHEEL_START_DELAY = 26000;
const unsigned long STAGING_FORWARD_DURATION = 33000;

uint8_t staging_mode = 0;
unsigned long staging_forward_stop_time = 0; // the time in the future when the staging motor should stop

uint8_t camera_mode = 0;
unsigned long last_camera_step_time = 0;

unsigned long last_step_time = 0;
const unsigned long STEP_INTERVAL = 800;

// CAN communication with the BLDC motors
unsigned long lastCanSend = 0;
const unsigned long CAN_INTERVAL = 50;


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_bus; // setting CAN1 uses the first set of CAN communication pins, 22 and 23

// gate servo control
int8_t servo_angle = 90;
Servo gateServo;

// break beam sensor states
int last_beam_state = HIGH;

// IR REMOTE CODES
const unsigned long IR_0 = 0xFF6897;
const unsigned long IR_POWER = 0xFFA25D;
const unsigned long IR_FAST_BACK = 0xFF22DD;
const unsigned long IR_FAST_FWD = 0xFFC23D;
const unsigned long IR_VOL_PLUS = 0xFF629D;
const unsigned long IR_VOL_MINUS = 0xFFA857;
const unsigned long IR_UP = 0xFF906F;
const unsigned long IR_DOWN = 0xFFE01F;
const unsigned long IR_PLAY = 0xFF02FD;
const unsigned long IR_FUNC_STOP = 0xFFE21D;
const unsigned long IR_7 = 0xFF42BD;
const unsigned long IR_8 = 0xFF4AB5;
const unsigned long IR_9 = 0xFF52AD;


// returns the length of a specified IR signal, length determines if the incoming signals represent a header
unsigned long readPulse(int level, unsigned long timeout = 20000) {
    unsigned long start = micros();
    while (digitalRead(IR_RECEIVER_PIN) == level) {
        // if the length of the signal is longer than 20000 microseconds, return 0
        // this prevents the code from hanging indefinitely if no state change occurs
        if (micros() - start > timeout) {
            return 0;
        }
    }
    return micros() - start;
}


void setVescDuty(uint8_t id, float duty) {
    CAN_message_t msg;

    msg.id = (CAN_PACKET_DUTY_CYCLE << 8) | id; // unique CAN id
    msg.flags.extended = 1;
    msg.len = 4;

    int32_t sendValue = (int32_t)(duty * 100000.0);

    // slice the message into four pieces to be able to fit into the msg array, MSB first
    msg.buf[0] = (sendValue >> 24) & 0xFF;
    msg.buf[1] = (sendValue >> 16) & 0xFF;
    msg.buf[2] = (sendValue >> 8) & 0xFF;
    msg.buf[3] = sendValue & 0xFF;

    can_bus.write(msg);
}

void setVescCurrent(uint8_t id, float current) {
  CAN_message_t msg;

  msg.id = (CAN_PACKET_CURRENT << 8) | id;
  msg.flags.extended = 1;
  msg.len = 4;

  int32_t sendValue = (int32_t)(current * 1000.0);

  msg.buf[0] = (sendValue >> 24) & 0xFF;
  msg.buf[1] = (sendValue >> 16) & 0xFF;
  msg.buf[2] = (sendValue >> 8) & 0xFF;
  msg.buf[3] = sendValue & 0xFF;

  can_bus.write(msg);
}

void stopAllDriveMotors() {
    duty_drive_FR = 0.0;
    duty_drive_BL = 0.0;
    duty_drive_FL = 0.0;
    duty_drive_BR = 0.0;
}

void sendMotorDuty(uint8_t id, float duty) {
    if (duty != 0.0) {
        setVescDuty(id, duty);
    } else {
        setVescCurrent(id, 0.0); // allow the motor to coast rather than actively fight movement
    }
}


void setup() {

    pinMode(IR_RECEIVER_PIN, INPUT);
    pinMode(BREAK_BEAM_PIN, INPUT_PULLUP);
    pinMode(TEENSY_LED_PIN, OUTPUT);

    pinMode(CAM_STEPPER_EN_PIN, OUTPUT);
    pinMode(CAM_STEPPER_DIR_PIN, OUTPUT);
    pinMode(CAM_STEPPER_STEP_PIN, OUTPUT);
    digitalWrite(CAM_STEPPER_EN_PIN, LOW);

    pinMode(STAGE_STEPPER_EN_PIN, OUTPUT);
    pinMode(STAGE_STEPPER_DIR_PIN, OUTPUT);
    pinMode(STAGE_STEPPER_STEP_PIN, OUTPUT);
    digitalWrite(STAGE_STEPPER_EN_PIN, LOW);

    gateServo.attach(SERVO_PIN);
    gateServo.write(servo_angle);

    can_bus.begin();
    can_bus.setBaudRate(500000);
}

void loop() {
    // IR signal detection
    if (digitalRead(IR_RECEIVER_PIN) == LOW) {
        unsigned long pulse_low_time = readPulse(LOW);
        unsigned long pulse_high_time = readPulse(HIGH);

        // a repeat code uses a shorter 2250us high pulse
        if (pulse_low_time > 8000 && pulse_high_time > 2000 && pulse_high_time < 3000) {
        } else if (pulse_low_time > 8000 && pulse_high_time > 4000) {
            // system knows that a new command is about to come in
            unsigned long code = 0;

            // now decode the bits
            // the "code" variable is what cumulatively stores the ir code coming in
            for (int i = 0; i < 32; i++) {
                readPulse(LOW); // every bit starts with a space, so we ignore that, what we want is in the high signal
                unsigned long bitHigh = readPulse(HIGH);
                code <<= 1; // make room for the new bit at the end
                if (bitHigh > 1000) {
                code |= 1;  // add a 1 to the end of the code
                }
            }


            switch (code)
            {
            case IR_FAST_FWD:
                float val = (duty_drive_FL != 0.0) ? 0.0 : 0.1;
                duty_drive_FL = val;
                duty_drive_BR = val;
                break;
            
            case IR_FAST_BACK:
                float val = (duty_drive_FL != 0.0) ? 0.0 : -0.1;
                duty_drive_FL = val;
                duty_drive_BR = val;
                break;

            case IR_VOL_PLUS:
                float val = (duty_drive_FR != 0.0) ? 0.0 : 0.1;
                duty_drive_FR = val;
                duty_drive_BL = (val == 0.0) ? 0.0 : -0.1;
                break;

            case IR_VOL_MINUS:
                float val = (duty_drive_FR != 0.0) ? 0.0 : -0.1;
                duty_drive_FR = val;
                duty_drive_BL = (val == 0.0) ? 0.0 : 0.1;
                break;

            case IR_UP:
                float val = (duty_drive_FL != 0.0) ? 0.0 : 0.1;
                duty_drive_FL = (val == 0.0) ? 0.0 : -val;
                duty_drive_BR = val;
                break;

            case IR_DOWN:
                float val = (duty_drive_FR != 0.0) ? 0.0 : 0.1;
                duty_drive_FR = val;
                duty_drive_BL = val;
                break;

            case IR_PLAY:
                // HANDLE BEGINNING THE AUTOMATIC STAGING
                if (digitalRead(BREAK_BEAM_PIN) == LOW) {
                    staging_mode = 1;
                    flywheels_running = false;
                    staging_forward_stop_time = millis() + STAGING_FORWARD_DURATION;
                    digitalWrite(STAGE_STEPPER_DIR_PIN, LOW);

                    // open the gate
                    servo_angle = 0;
                    gateServo.write(servo_angle);
                }
                break;
            
            case IR_FUNC_STOP:
                if (staging_mode == 2) {
                    staging_mode = 0;
                } else {
                    staging_mode = 2;
                    digitalWrite(STAGE_STEPPER_DIR_PIN, HIGH);
                }
                break;

            case IR_7:
                // toggling gate servo angle
                servo_angle = (servo_angle == 90) ? 0 : 90;
                gateServo.write(servo_angle);
                break;

            case IR_8:
                if (camera_mode == 1) {
                    camera_mode = 0;
                } else {
                    camera_mode = 1;
                    digitalWrite(STAGE_STEPPER_DIR_PIN, LOW);
                }
                break;

            case IR_9:
                if (camera_mode == 2) {
                    camera_mode = 0;
                } else {
                    camera_mode = 2;
                    digitalWrite(STAGE_STEPPER_DIR_PIN, HIGH);
                }
                break;

            case IR_POWER:
                flywheels_running = false;
                break;

            case IR_0:
                stopAllDriveMotors();
                break;

            default:
                break;
            }
                   

        }
    }


    if (staging_mode == 1 && millis() >= staging_forward_stop_time) {
        // staging motor completes the 33 second run
        staging_mode = 0;
    }

    if (staging_mode == 1 && !flywheels_running && millis() >= staging_forward_stop_time - (STAGING_FORWARD_DURATION - FLYWHEEL_START_DELAY)) {
        flywheels_running = true;
    }

    // protect the CAN bus from behing overhwelmend, but also make sure that it stays active
    // send commands to the VESCs every 50 milliseconds
    if (millis() - lastCanSend >= CAN_INTERVAL) {
        lastCanSend = millis();

        // set to 20% speed if we want the flywheels to be running
        flywheels_running ? setVescDuty(TOP_FLY_CAN_ADDR, -0.2) : setVescCurrent(TOP_FLY_CAN_ADDR, 0.0);
        flywheels_running ? setVescDuty(BOT_FLY_CAN_ADDR, -0.2) : setVescCurrent(BOT_FLY_CAN_ADDR, 0.0);

        sendMotorDuty(DRIVE_FL_CAN_ADDR, duty_drive_FL);
        sendMotorDuty(DRIVE_BR_CAN_ADDR, duty_drive_BR);
        sendMotorDuty(DRIVE_FR_CAN_ADDR, duty_drive_FR);
        sendMotorDuty(DRIVE_BL_CAN_ADDR, duty_drive_BL);
    }


    // 1600 total pulse cycle 
    if (staging_mode != 0 && micros() - last_step_time >= STEP_INTERVAL) {
        last_step_time = micros();

        digitalWrite(STAGE_STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(400);

        digitalWrite(STAGE_STEPPER_STEP_PIN, LOW);
        delayMicroseconds(400);
    }


    if (camera_mode != 0 && micros() - last_camera_step_time >= STEP_INTERVAL) {
        last_camera_step_time = micros();

        digitalWrite(CAM_STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(400);

        digitalWrite(CAM_STEPPER_STEP_PIN, LOW);
        delayMicroseconds(400);
    }

    int beamState = digitalRead(BREAK_BEAM_PIN);

    // debugging
    if (beamState != last_beam_state) {
        if (beamState == LOW) {
            digitalWrite(TEENSY_LED_PIN, HIGH);
        } else {
            digitalWrite(TEENSY_LED_PIN, LOW);
        }
        last_beam_state = beamState;
    }
}
