#include "MicroBit.h"

// --- Pin Definitions ---
#define TX_PIN_1 6
#define TX_PIN_2 4
#define BIT_DELAY_US 9
#define COMPARE_DELAY_MS 240

#define ACCEL_SENSITIVITY 0.001 // ±2g range, 1 mg/LSB
#define ACCEL_SCALE_FACTOR 1000 

#define OUT_X_L_A 0x28

#define TWI_SCL_PIN 8
#define TWI_SDA_PIN 16
#define TWI_ADDRESS 0x19

#define BUTTON_A_PIN 14
#define BUTTON_B_PIN 23

#define PWM_SPEAKER_PIN 0
#define PWM_FREQUENCY_BASE 500
#define PWM_FREQUENCY_MAX 5000

static bool initialized = false;
static bool accelerometerInitialized = false;

extern "C" char* itoa(int value, char* str, int base);

// --- Function Prototypes ---
void bitBangSerial(const char *string);
void voteForChocolate(void);
void showAccelerometerSamples(void);
void makeNoise(void);

// --- Initialization Prototypes ---
void initGPIOandTimer();
void initTWI();
void initPWM();
void setPWMFrequency(uint16_t frequency);
void buttons();
void delayMicroseconds(int us);
uint16_t calculateFrequencyFromY();
int16_t getAccelerometerSample(char axis);
bool waitForEvent(volatile uint32_t *event, uint32_t timeout);

// --- Helper: Microsecond Delay ---
void delayMicroseconds(int us) {
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->PRESCALER = 4;
    NRF_TIMER0->CC[1] = us;
    NRF_TIMER0->EVENTS_COMPARE[1] = 0;
    NRF_TIMER0->TASKS_START = 1;
    while (!NRF_TIMER0->EVENTS_COMPARE[1]);
}

// --- Helper: Wait for Event ---
bool waitForEvent(volatile uint32_t *event, uint32_t timeout) {
    uint32_t count = 0;
    while (!(*event) && count++ < timeout);
    if (*event) {
        *event = 0; // Clear the event
        return true;
    }
    return false; // Timed out
}

// --- Subtask 1: Bit-Bang Serial ---

//initialises gpio's and timers
void initGPIOandTimer() {
    if (!initialized) {
        NRF_P0->DIRSET = (1 << TX_PIN_1) | (1 << TX_PIN_2);
        NRF_P0->OUTSET = (1 << TX_PIN_1) | (1 << TX_PIN_2);

        NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
        NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
        NRF_TIMER1->PRESCALER = 4;
        NRF_TIMER1->CC[0] = COMPARE_DELAY_MS * 1000;
        NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled;
        NRF_TIMER1->TASKS_START = 1;

        initialized = true;
    }
}


//transmits the bytes 
void transmitByte(char byte) {
    NRF_P0->OUTCLR = (1 << TX_PIN_1) | (1 << TX_PIN_2);
    delayMicroseconds(BIT_DELAY_US);

    for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            NRF_P0->OUTSET = (1 << TX_PIN_1) | (1 << TX_PIN_2);
        } else {
            NRF_P0->OUTCLR = (1 << TX_PIN_1) | (1 << TX_PIN_2);
        }
        delayMicroseconds(BIT_DELAY_US);
    }

    NRF_P0->OUTSET = (1 << TX_PIN_1) | (1 << TX_PIN_2);
    delayMicroseconds(BIT_DELAY_US);
}

void bitBangSerial(const char *string) {
    initGPIOandTimer();
    while (*string) {
        transmitByte(*string);
        string++;
    }
}

void voteForChocolate(void) {
    initGPIOandTimer();
    const char *message = "Twix\n\r";
    while (1) {
        bitBangSerial(message);
        delayMicroseconds(100000);
    }
}

// --- Subtask 2: Accelerometer ---
void initTWI() {
    NRF_TWI1->ENABLE = TWI_ENABLE_ENABLE_Disabled; // Disable TWI before configuration

    // Configure SDA and SCL pins as per the schematic
    NRF_TWI1->PSEL.SCL = 8; // P0.8 for SCL //MICROBIT_PIN_INT_SCL
    NRF_TWI1->PSEL.SDA = 16;

    NRF_TWI1->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K100; // 100 kHz frequency
    NRF_TWI1->ENABLE = TWI_ENABLE_ENABLE_Enabled;      // Enable TWI peripheral

    // Clear any pending events
    NRF_TWI1->EVENTS_STOPPED = 0;
    NRF_TWI1->EVENTS_ERROR = 0;
    NRF_TWI1->EVENTS_TXDSENT = 0;
    NRF_TWI1->EVENTS_RXDREADY = 0;
}

// --- Write to Register ---
bool writeRegister(uint8_t reg, uint8_t value) {
    NRF_TWI1->ADDRESS = TWI_ADDRESS;

    // Start TX
    NRF_TWI1->TXD = reg;
    NRF_TWI1->TASKS_STARTTX = 1;
    if (!waitForEvent(&NRF_TWI1->EVENTS_TXDSENT, 100000)) {
        bitBangSerial("Error: TWI TX timeout while sending register address.\r\n");
        NRF_TWI1->TASKS_STOP = 1;
        return false;
    }

    // Send value
    NRF_TWI1->TXD = value;
    if (!waitForEvent(&NRF_TWI1->EVENTS_TXDSENT, 100000)) {
        bitBangSerial("Error: TWI TX timeout while sending data.\r\n");
        NRF_TWI1->TASKS_STOP = 1;
        return false;
    }

    // Stop TX
    NRF_TWI1->TASKS_STOP = 1;
    return waitForEvent(&NRF_TWI1->EVENTS_STOPPED, 100000);
}

uint8_t readRegister(uint8_t reg) {
    NRF_TWI1->ADDRESS = TWI_ADDRESS;

    // Send register address
    NRF_TWI1->TXD = reg;
    NRF_TWI1->TASKS_STARTTX = 1;
    if (!waitForEvent(&NRF_TWI1->EVENTS_TXDSENT, 100000)) {
        bitBangSerial("Error: TWI TX timeout while sending register address.\r\n");
        NRF_TWI1->TASKS_STOP = 1;
        return 0;
    }

    // Switch to RX mode
    NRF_TWI1->TASKS_STARTRX = 1;
    if (!waitForEvent(&NRF_TWI1->EVENTS_RXDREADY, 100000)) {
        bitBangSerial("Error: TWI RX timeout.\r\n");
        NRF_TWI1->TASKS_STOP = 1;
        return 0;
    }

    uint8_t value = NRF_TWI1->RXD;
    NRF_TWI1->TASKS_STOP = 1;
    waitForEvent(&NRF_TWI1->EVENTS_STOPPED, 100000);
    return value;
}

// --- Accelerometer Initialization ---
void initializeAccelerometer() {
    if (!accelerometerInitialized) {
        bitBangSerial("Initializing accelerometer...\r\n");

        // Set up TWI peripheral if not already done
        initTWI();

        // Read WHO_AM_I register
        uint8_t whoAmI = readRegister(0x0F); // WHO_AM_I register address
        if (whoAmI != 0x33) {
            bitBangSerial("Error: WHO_AM_I failed.\r\n");
            return;
        }

        // Enable accelerometer: 100Hz data rate, all axes enabled (CTRL_REG1_A)
        if (!writeRegister(0x20, 0x57)) { // Normal mode, 100Hz, all axes enabled
            bitBangSerial("Error: Failed to write to CTRL_REG1_A.\r\n");
            return;
        }

        // Configure scale: ±2g, normal mode (CTRL_REG4_A)
        if (!writeRegister(0x23, 0x00)) { // Continuous update, ±2g scale
            bitBangSerial("Error: Failed to write to CTRL_REG4_A.\r\n");
            return;
        }

        accelerometerInitialized = true;
        bitBangSerial("Accelerometer initialized successfully.\r\n");
    }
}

// --- Get Accelerometer Sample ---
int16_t getAccelerometerAxis(uint8_t lowReg) {
    uint8_t lowByte = readRegister(lowReg);
    uint8_t highByte = readRegister(lowReg + 1);

    // Combine high and low bytes to form a signed 16-bit value
    return ((int16_t)((highByte << 8) | lowByte)>>6);
}

int16_t getAccelerometerSample(char axis) {
    uint8_t reg;
    switch (axis) {
        case 'X': reg = OUT_X_L_A; break;
        case 'Y': reg = OUT_X_L_A + 2; break;
        case 'Z': reg = OUT_X_L_A + 4; break;
        default:
            bitBangSerial("Error: Invalid axis specified.\r\n");
            return 0;
    }

    return getAccelerometerAxis(reg);
}

// --- Show Accelerometer Samples ---
void showAccelerometerSamples() {
    char xBuffer[10], yBuffer[10], zBuffer[10]; // Buffers for individual axis values

    initializeAccelerometer(); // Ensure accelerometer is initialized

    // Inform that the subtask is running
    bitBangSerial("... Subtask 2 running ...\r\n");

    while (1) {
        // Read raw accelerometer data for each axis
        int16_t rawX = getAccelerometerSample('X');
        int16_t rawY = getAccelerometerSample('Y');
        int16_t rawZ = getAccelerometerSample('Z');

        // Scale down to fit the range -512 to +511
        int16_t adjustedX = (rawX >> 4) & 0x3FF; // Limit to 10-bit signed range
        int16_t adjustedY = (rawY >> 4) & 0x3FF;
        int16_t adjustedZ = (rawZ >> 4) & 0x3FF;

        // Ensure values are within the signed 10-bit range
        if (adjustedX > 511) adjustedX -= 1024;
        if (adjustedY > 511) adjustedY -= 1024;
        if (adjustedZ > 511) adjustedZ -= 1024;

        // Convert adjusted values to strings
        itoa(adjustedX, xBuffer, 10);
        itoa(adjustedY, yBuffer, 10);
        itoa(adjustedZ, zBuffer, 10);

        // Send formatted string in parts
        bitBangSerial("[X: ");
        bitBangSerial(xBuffer);
        bitBangSerial("] [Y: ");
        bitBangSerial(yBuffer);
        bitBangSerial("] [Z: ");
        bitBangSerial(zBuffer);
        bitBangSerial("]\r\n");

        // Delay to maintain ~5Hz update rate
        delayMicroseconds(500000); // 200ms delay
    }
}

// --- Subtask 3: Make Noise ---
void buttons() {
    // Configure Button A (P0.14) as input with pull-up and connect
    NRF_P0->PIN_CNF[BUTTON_A_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                     (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                     (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);

    // Configure Button B (P0.23) as input with pull-up and connect
    NRF_P0->PIN_CNF[BUTTON_B_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                     (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                     (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
}

// --- Debounce Delay ---
void debounceDelay() {
    for (volatile int i = 0; i < 10000; i++); // Simple loop delay for debouncing
}

// --- PWM Initialization ---
void initPWM() {
    // Connect the speaker pin to PWM
    NRF_PWM0->PSEL.OUT[0] = (PWM_SPEAKER_PIN << PWM_PSEL_OUT_PIN_Pos) |
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

    // Enable PWM
    NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);

    // Set mode to up counting
    NRF_PWM0->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);

    // Set prescaler to divide clock by 1 for high-resolution timing
    NRF_PWM0->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);

    // Set the period (COUNTERTOP) to 1 ms (16000 counts for a 16 MHz clock)
    NRF_PWM0->COUNTERTOP = (16000 << PWM_COUNTERTOP_COUNTERTOP_Pos);

    // Disable looping
    NRF_PWM0->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);

    // Set decoder to load a common value for all channels
    NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) |
                        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);

    // Configure sequence 0
    static uint16_t seq0_ram[1] = {8000}; // Default 50% duty cycle
    NRF_PWM0->SEQ[0].PTR = ((uint32_t)(seq0_ram) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[0].CNT = ((sizeof(seq0_ram) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;
}


// --- Calculate Frequency Based on Y-Axis ---
// --- Map Y-Axis Value to Frequency ---
uint16_t calculateFrequencyFromY(int16_t yValue) {
    // Map Y value (-512 to +511) to frequency range (500 Hz to 5 kHz)
    return (PWM_FREQUENCY_BASE + ((yValue + 512) * (PWM_FREQUENCY_MAX - PWM_FREQUENCY_BASE) / 1024));
}

// --- Play Sound at a Given Frequency ---
void playSound(uint16_t frequency) {
    // Calculate COUNTERTOP for the given frequency
    uint16_t countertop = 16000000 / frequency;

    // Ensure COUNTERTOP fits within the allowable range
    if (countertop > 0xFFFF) countertop = 0xFFFF;

    // Update COUNTERTOP and duty cycle
    NRF_PWM0->COUNTERTOP = (countertop << PWM_COUNTERTOP_COUNTERTOP_Pos);

    // Adjust duty cycle to 50%
    static uint16_t seq0_ram[1];
    seq0_ram[0] = countertop / 2;  // 50% duty cycle
    NRF_PWM0->SEQ[0].PTR = ((uint32_t)(seq0_ram) << PWM_SEQ_PTR_PTR_Pos);

    // Start the sequence
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

// --- Stop Sound ---
void stopSound() {
    NRF_PWM0->TASKS_STOP = 1;  // Stop PWM
    while (!NRF_PWM0->EVENTS_STOPPED);  // Wait for STOPPED event
    NRF_PWM0->EVENTS_STOPPED = 0;  // Clear the STOPPED event
}

// --- Subtask 3 Implementation ---
void makeNoise(void) {
    // Initialize PWM and buttons
    initPWM();
    buttons();
    initializeAccelerometer(); // Initialize accelerometer during setup

    bitBangSerial("... Subtask 3 running ...\r\n");

    while (1) {
        // Check if Button A is pressed
        if (!(NRF_P0->IN & (1 << BUTTON_A_PIN))) { // Active low
            debounceDelay(); // Debounce delay
            if (!(NRF_P0->IN & (1 << BUTTON_A_PIN))) { // Confirm press
                bitBangSerial("Button A pressed.\r\n"); // Feedback for Button A
                
                // Fetch Y-axis value and calculate frequency
                int16_t yValue = getAccelerometerSample('Y');
                uint16_t freq = calculateFrequencyFromY(yValue);

                playSound(freq); // Play sound at calculated frequency
            }
        }

        // Check if Button B is pressed
        if (!(NRF_P0->IN & (1 << BUTTON_B_PIN))) { // Active low
            debounceDelay(); // Debounce delay
            if (!(NRF_P0->IN & (1 << BUTTON_B_PIN))) { // Confirm press
                bitBangSerial("Button B pressed.\r\n"); // Feedback for Button B
                
                int16_t yValue = getAccelerometerSample('Y');
                uint16_t freq = calculateFrequencyFromY(yValue);

                playSound(freq); // Play sound at calculated frequency
            }
        }

        // Stop sound if no buttons are pressed
        if ((NRF_P0->IN & (1 << BUTTON_A_PIN)) && (NRF_P0->IN & (1 << BUTTON_B_PIN))) {
            NRF_PWM0->TASKS_STOP = 1; // Stop PWM
            bitBangSerial("No button pressed. Sound stopped.\r\n");
        }

        delayMicroseconds(500); // Delay to reduce polling rate
    }
}