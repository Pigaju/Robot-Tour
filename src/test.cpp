#ifdef TEST_BUILD

#include <M5Dial.h>
#include <Wire.h>

// Motor controller I2C addresses
#define MOTOR_RIGHT_ADDR 0x20
#define MOTOR_LEFT_ADDR  0x21

// Encoder I2C addresses
#define ENCODER_LEFT_ADDR  0x58
#define ENCODER_RIGHT_ADDR 0x59

// Test states
enum TestState {
    TEST_IDLE,
    TEST_I2C_SCAN,
    TEST_MOTOR_SPIN,
    TEST_ENCODER_READ,
    TEST_COMPLETE
};

TestState testState = TEST_IDLE;
unsigned long testStartTime = 0;
int32_t encoderLeftStart = 0, encoderRightStart = 0;
int32_t encoderLeftCurrent = 0, encoderRightCurrent = 0;

// Function to read encoder value from I2C
int32_t readEncoderValue(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(0x00);  // Register 0 for position
    Wire.endTransmission();
    
    Wire.requestFrom(addr, 4);
    if (Wire.available() >= 4) {
        int32_t value = 0;
        value |= (int32_t)Wire.read() << 24;
        value |= (int32_t)Wire.read() << 16;
        value |= (int32_t)Wire.read() << 8;
        value |= (int32_t)Wire.read();
        return value;
    }
    return 0;
}

// Function to send motor command via I2C
void setMotorSpeed(uint8_t addr, int16_t speed) {
    Wire.beginTransmission(addr);
    Wire.write(0x01);  // Command register
    Wire.write((speed >> 8) & 0xFF);
    Wire.write(speed & 0xFF);
    Wire.endTransmission();
}

// Function to scan I2C bus and display found devices
void scanI2CBus() {
    M5Dial.Display.fillScreen(BLACK);
    M5Dial.Display.setTextSize(1);
    M5Dial.Display.setCursor(10, 10);
    M5Dial.Display.println("I2C SCAN:");
    
    int deviceCount = 0;
    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            M5Dial.Display.printf("Found: 0x%02X\n", addr);
            deviceCount++;
        }
    }
    
    if (deviceCount == 0) {
        M5Dial.Display.println("No devices found!");
    }
    
    M5Dial.Display.println("\nPress to test motors");
}

// Function to test motor spinning
void testMotorSpinning() {
    M5Dial.Display.fillScreen(BLACK);
    M5Dial.Display.setTextSize(2);
    M5Dial.Display.setCursor(10, 20);
    M5Dial.Display.println("MOTOR TEST");
    
    unsigned long elapsed = millis() - testStartTime;
    
    if (elapsed < 2000) {
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setCursor(10, 60);
        M5Dial.Display.println("Spinning RIGHT motor forward...");
        setMotorSpeed(MOTOR_RIGHT_ADDR, 200);
        setMotorSpeed(MOTOR_LEFT_ADDR, 0);
    } 
    else if (elapsed < 4000) {
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setCursor(10, 60);
        M5Dial.Display.println("Spinning LEFT motor forward...");
        setMotorSpeed(MOTOR_RIGHT_ADDR, 0);
        setMotorSpeed(MOTOR_LEFT_ADDR, 200);
    } 
    else if (elapsed < 6000) {
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setCursor(10, 60);
        M5Dial.Display.println("Both motors forward...");
        setMotorSpeed(MOTOR_RIGHT_ADDR, 200);
        setMotorSpeed(MOTOR_LEFT_ADDR, 200);
    } 
    else {
        setMotorSpeed(MOTOR_RIGHT_ADDR, 0);
        setMotorSpeed(MOTOR_LEFT_ADDR, 0);
        M5Dial.Display.setTextSize(2);
        M5Dial.Display.setCursor(10, 60);
        M5Dial.Display.println("Motor test done!");
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setCursor(10, 110);
        M5Dial.Display.println("Press for encoder test");
        testState = TEST_ENCODER_READ;
        testStartTime = millis();
    }
}

// Function to test encoder reading
void testEncoderReading() {
    M5Dial.Display.fillScreen(BLACK);
    M5Dial.Display.setTextSize(2);
    M5Dial.Display.setCursor(10, 20);
    M5Dial.Display.println("ENCODER TEST");
    
    unsigned long elapsed = millis() - testStartTime;
    
    if (elapsed < 1000) {
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setCursor(10, 60);
        M5Dial.Display.println("Reading encoder baselines...");
        encoderLeftStart = readEncoderValue(ENCODER_LEFT_ADDR);
        encoderRightStart = readEncoderValue(ENCODER_RIGHT_ADDR);
    } 
    else if (elapsed < 5000) {
        // Spin motors and read encoders
        setMotorSpeed(MOTOR_RIGHT_ADDR, 150);
        setMotorSpeed(MOTOR_LEFT_ADDR, 150);
        
        encoderLeftCurrent = readEncoderValue(ENCODER_LEFT_ADDR);
        encoderRightCurrent = readEncoderValue(ENCODER_RIGHT_ADDR);
        
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setCursor(10, 60);
        M5Dial.Display.println("Reading encoders...");
        M5Dial.Display.printf("L Delta: %ld\n", encoderLeftCurrent - encoderLeftStart);
        M5Dial.Display.printf("R Delta: %ld\n", encoderRightCurrent - encoderRightStart);
    } 
    else {
        setMotorSpeed(MOTOR_RIGHT_ADDR, 0);
        setMotorSpeed(MOTOR_LEFT_ADDR, 0);
        
        M5Dial.Display.setTextSize(2);
        M5Dial.Display.setCursor(10, 60);
        
        // Check if encoders moved
        int32_t leftDelta = encoderLeftCurrent - encoderLeftStart;
        int32_t rightDelta = encoderRightCurrent - encoderRightStart;
        
        if (abs(leftDelta) > 10 && abs(rightDelta) > 10) {
            M5Dial.Display.println("PASS!");
            M5Dial.Display.setTextSize(1);
            M5Dial.Display.setCursor(10, 110);
            M5Dial.Display.printf("L: %ld  R: %ld\n", leftDelta, rightDelta);
        } else {
            M5Dial.Display.println("FAIL!");
            M5Dial.Display.setTextSize(1);
            M5Dial.Display.setCursor(10, 110);
            M5Dial.Display.println("Encoders not moving");
        }
        
        testState = TEST_COMPLETE;
    }
}

void setup() {
    auto cfg = M5.config();
    M5Dial.begin(cfg);
    Wire.begin();  // Initialize I2C bus

    M5Dial.Display.setTextSize(2);
    M5Dial.Display.setCursor(20, 100);
    M5Dial.Display.println("Motor/Encoder Test");
    M5Dial.Display.setTextSize(1);
    M5Dial.Display.setCursor(20, 140);
    M5Dial.Display.println("Press button to start");
}

void loop() {
    M5Dial.update();

    if (M5Dial.BtnA.wasPressed()) {
        if (testState == TEST_IDLE) {
            testState = TEST_I2C_SCAN;
            testStartTime = millis();
        } 
        else if (testState == TEST_I2C_SCAN) {
            testState = TEST_MOTOR_SPIN;
            testStartTime = millis();
        } 
        else if (testState == TEST_MOTOR_SPIN) {
            testState = TEST_ENCODER_READ;
            testStartTime = millis();
        }
        else if (testState == TEST_COMPLETE) {
            testState = TEST_IDLE;
        }
    }
    
    // Run appropriate test based on state
    if (testState == TEST_IDLE) {
        M5Dial.Display.setTextSize(2);
        M5Dial.Display.setCursor(20, 100);
        M5Dial.Display.println("Motor/Encoder Test");
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setCursor(20, 140);
        M5Dial.Display.println("Press button to start");
    }
    else if (testState == TEST_I2C_SCAN) {
        scanI2CBus();
    }
    else if (testState == TEST_MOTOR_SPIN) {
        testMotorSpinning();
    }
    else if (testState == TEST_ENCODER_READ) {
        testEncoderReading();
    }
}

#endif // TEST_BUILD
