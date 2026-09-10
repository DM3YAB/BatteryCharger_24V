/*
    Name:       BatteryCharger_24V.ino
    Author:     Andreas

    ATMega328P 16MHz (ProMini)

	I2C 0x40	INA238	HIGH_POWER
	I2C 0x41	INA238	MID_POWER
	I2C 0x43	INA238	LOW_POWER
	I2C 0x44	INA238	DCDC
	I2C 0x45	INA238	Battery
	I2C 0x48	ADS1015	ADC-2xTemp-Fan
	I2C 0x60	MCP4725	DAC
	
	PD5			SW-MID_POWER
	PD4			SW_LOW_POWER
	PD7			SW-HIGH_POWER
	PD6			SW-Charge    HIGH active, LOW = OFF
	PC1			DCDC_EN
	
	Input
	PC0			DCDC_PG
	
	PD2			Button_Set
	ADC6		Button_Down 460
	ADC6		Button_Up   703
	ADC6		Button_OK	383
	
	ADC7		LightIntensity Display
	
	Display		GMG12864-06D    ST7565
	// CS  = PB2
	// RSE = PB0   // Reset
	// RS  = PB1   // A0 / Data-Command
	// SCL = PB5
	// SI  = PB3
	
	UART		debug / commands
*/

#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>

// ============================================================
// Debug
// ============================================================

bool debugEnabled = false;

#define DBG_BEGIN(x)      Serial.begin(x)
#define DBG_PRINT(x)      Serial.print(x)
#define DBG_PRINTLN(x)    Serial.println(x)
#define DBG_PRINT_HEX(x)  Serial.print(x, HEX)

// ============================================================
// System Parameters
// ============================================================

#define UART_BAUDRATE              115200
#define I2C_CLOCK_HZ               100000UL
#define DEBUG_INTERVAL_MS          100UL		// UART log interval.
#define DEBUG_OFF_CHANGE_ONLY      1		// OFF state: print only when selected values change.
#define MEASURE_INTERVAL_MS        100UL
#define DISPLAY_INTERVAL_MS        500UL		// Slow display refresh for readable values.

// ============================================================
// I2C Addresses
// ============================================================

#define INA238_HIGH_POWER_ADDR         0x40
#define INA238_MID_POWER_ADDR        0x41
#define INA238_LOW_POWER_ADDR         0x43
#define INA238_DCDC_ADDR           0x44
#define INA238_BATTERY_ADDR        0x45

#define ADS1015_ADDR               0x48
#define MCP4725_ADDR               0x60


// ============================================================
// MCP4725 DAC Parameters
// ============================================================

#define DAC_CODE_MIN               0
#define DAC_CODE_MAX               4095
#define DAC_VREF_mV                5000L

// DCDC output model from measurement:
// DCDC_mV = 30113 - 2.404 * DAC_Code
#define DCDC_DAC_OFFSET_mV         30113L		// Measured 24V hardware DAC offset.
#define DCDC_DAC_SLOPE_uV_CODE      2404L		// Measured: 2.404mV per DAC code.

#define DCDC_VOUT_MIN_mV           20250L		// Minimum commanded DC/DC output voltage.
#define DCDC_VOUT_MAX_mV           29000L		// Maximum commanded DC/DC output voltage; DC/DC can reach about 32.9V. Fix!Me

#define DAC_RAMP_STEP_CODE         2000		// Fast step for open Battery-DCDC switch test.
#define DAC_RAMP_STEP_DELAY_MS     0UL

// ============================================================
// DCDC Converter Parameters
// ============================================================

#define DCDC_TASK_INTERVAL_MS          100UL		// Interval of the DC/DC state machine.
#define DCDC_TARGET_TOLERANCE_mV        50L		// Allowed voltage error around a DC/DC target.
#define DCDC_TARGET_OFFSET_mV            0L		// Additional offset used by the normal target calculation.
#define DCDC_SAFE_START_CODE          DAC_CODE_MAX		// Highest DAC code commands the lowest DC/DC output voltage.
#define DCDC_MAX_POWER_W               350L		// Configurable continuous limit; 350W = about 16.7A at 21V, battery current limit has priority. Fix!Me

/* ============================================================
   Input Source Parameters
   ============================================================ */

#define LOW_POWER_ON_mV              15000L		// Minimum voltage required before the low-power source may be selected.
#define MID_POWER_ON_mV              18000L		// Minimum voltage required before the mid-power source may be selected.
#define HIGH_POWER_ON_mV             24000L		// Minimum voltage required before the high-power source may be selected.

#define LOW_POWER_INPUT_MAX_mA        2000L		// Configurable source limit; 50mOhm shunt measurement range about 3.27A.
#define MID_POWER_INPUT_MAX_mA        6000L		// Configurable source limit; 9mOhm shunt measurement range about 18.2A.
#define HIGH_POWER_INPUT_MAX_mA       9000L		// Configurable source limit; 9mOhm shunt measurement range about 18.2A.

#define INPUT_CURRENT_REG_INTERVAL_MS  300UL		// Interval for source-current limiting.
#define INPUT_CURRENT_HYST_mA           200L		// Current margin used to prevent source-limit oscillation.
#define INPUT_CURRENT_STEP_UP_mA        100L		// Slow increase of requested battery current when source margin is available.
#define INPUT_CURRENT_STEP_DOWN_mA      300L		// Normal reduction of requested battery current at the source limit.
#define INPUT_CURRENT_FAST_DOWN_mA     1000L		// Fast reduction after a clear source overcurrent.

/* ============================================================
   Battery Charging Parameters
   ============================================================ */

#define ENABLE_FLOAT                      1		// Enables maintenance charging after a completed full-charge cycle.
#define BATTERY_RESTART_mV            25200L		// Starts a new full-charge cycle when the rested battery falls to this voltage.
#define BATTERY_ABSORPTION_mV         28400L		// Voltage maintained during absorption charging. Fix!Me
#define BATTERY_FLOAT_mV              27800L		// Voltage maintained while float charging is enabled.
#define BATTERY_FULL_CURRENT_mA         100L		// End-current criterion used after the minimum absorption time. Fix!Me
#define BATTERY_ABSORB_TOLERANCE_mV      50L		// Battery voltage must be this close to absorption voltage for end-current detection.
#define BATTERY_VOLTAGE_TAPER_mV        100L		// Range below the active voltage target in which charge current is tapered.
#define BATTERY_MAX_CHARGE_CURRENT_mA 15000L		// Configurable battery-current limit; 5mOhm shunt measurement range about 32.7A. Fix!Me
#define BATTERY_REST_TIME_MS          60000UL		// Charger remains disconnected this long after FULL before restart voltage is evaluated.
#define ABS_MIN_TIME_MS              900000UL		// Minimum 15-minute absorption time before end-current may finish charging.
#define ABS_MAX_TIME_MS            10800000UL		// Maximum 3-hour absorption time before the charge cycle is completed.

/* ============================================================
   Battery Protection Limits
   ============================================================ */

#define BATTERY_MIN_VOLTAGE_mV       21000L		// Minimum valid 24V battery voltage. Fix!Me
#define BATTERY_MAX_VOLTAGE_mV       28800L		// Battery overvoltage protection limit. Fix!Me

#define CHARGE_CURRENT_HYST_mA           10L		// Dead band around the active battery-current target.
#define CHARGE_MAX_OFFSET_mV           1000L		// DC/DC output may not rise more than this above actual battery voltage.
#define CHARGE_RAMP_STEP_CODE             1		// Normal DAC step; lower DAC code raises DC/DC voltage.
#define CHARGE_RAMP_INTERVAL_MS         100UL		// Interval of the battery-current controller.
#define CHARGE_POSITIVE_START_MS       1500UL		// Initial period after connection in which control searches only toward positive current.
#define CHARGE_CURRENT_FILTER_DIV         4L		// Low-pass divisor for battery-current regulation.
#define CHARGE_OVERCURRENT_mA            250L		// Margin above target current that triggers a fast reduction.
#define CHARGE_FAST_STEP_CODE              8		// DAC step used for fast current reduction.
#define CHARGE_STABLE_COUNT                5		// Stable samples required before the DC/DC state becomes READY.

/* ============================================================
   DC/DC Pre-Charge and Reverse-Current Protection
   ============================================================ */

#define SOURCE_ON_DELAY_MS              1000UL		// Delay before a newly selected source switch is closed.
#define DCDC_ON_DELAY_MS                1000UL		// Delay before the DC/DC converter is enabled.
#define CHARGE_ON_DELAY_MS              1000UL		// Settling time before the pre-charge output is evaluated.
#define DCDC_TEST_MIN_OUTPUT_mV         18000L		// Minimum plausible DC/DC output after converter startup.
#define DCDC_TEST_RAISE_OFFSET_mV          80L		// Test level above battery voltage used to prove converter response. Fix!Me
#define DCDC_TEST_RISE_MIN_mV              40L		// Minimum measured rise required for a successful function test. Fix!Me
#define DCDC_TEST_MATCH_TOL_mV              20L		// Allowed error while matching real DC/DC output to battery voltage.
#define DCDC_TEST_DAC_STEP_CODE               2U		// Fine DAC step used during the pre-charge function test.
#define DCDC_CONNECT_OFFSET_mV               30L		// DC/DC target above battery voltage when the charge switch closes. Fix!Me
#define DCDC_CONNECT_SETTLE_MS              300UL		// Settling time after commanding the battery connection level.
#define DCDC_TEST_STABLE_COUNT                3U		// Consecutive stable samples required in the pre-charge test.
#define DCDC_TEST_PHASE_TIMEOUT_MS         5000UL		// Maximum time allowed for one pre-charge test phase.
#define DCDC_REVERSE_CURRENT_LIMIT_mA       -50L		// Negative battery current below this level starts reverse-current timing.
#define DCDC_REVERSE_CURRENT_TIME_MS       4000UL		// Reverse current must persist this long before the charger disconnects.
#define DCDC_REVERSE_RETRY_MS             60000UL		// Wait before our charger may try again after another charger takes over.

/* ============================================================
   Temperature / Fan Parameters
   ============================================================ */

#define FAN_CHECK_INTERVAL_MS             1000UL		// Temperature sampling interval while supervising the converter.
#define FAN_AFTER_RUN_OFF_C                  35		// Fan stops after charging only when FET and coil are both below this temperature.
#define DCDC_DERATING_START_C                65		// Full configured power is allowed up to this FET/coil temperature. Fix!Me
#define DCDC_DERATING_70C_POWER_W           260L		// Allowed power at 70C. Fix!Me
#define DCDC_DERATING_75C_POWER_W           175L		// Allowed power at 75C. Fix!Me
#define DCDC_DERATING_80C_POWER_W           100L		// Allowed power at 80C. Fix!Me
#define DCDC_SHUTDOWN_TEMP_C                 85		// Charging stops when either FET or coil reaches this temperature. Fix!Me
#define DCDC_RESTART_TEMP_C                  55		// Charging may restart after thermal shutdown when both sensors are below this value. Fix!Me

// ============================================================
// ST7565 Display Pins - Port B
// ============================================================

#define LCD_RST                    PB0
#define LCD_A0                     PB1
#define LCD_CS                     PB2
#define LCD_SDA                    PB3
#define LCD_SCK                    PB5

#define LCD_RST_M                  _BV(LCD_RST)
#define LCD_A0_M                   _BV(LCD_A0)
#define LCD_CS_M                   _BV(LCD_CS)
#define LCD_SDA_M                  _BV(LCD_SDA)
#define LCD_SCK_M                  _BV(LCD_SCK)

#define LCD_WIDTH                  128
#define LCD_PAGES                  8

// ============================================================
// Port D Outputs
// ============================================================
// Source switches are active LOW.
// Charge switch PD6 is active HIGH.


#define PIN_DISPLAY_BACKLIGHT      PD3
#define PIN_SW_MID_POWER             PD5
#define PIN_SW_LOW_POWER              PD4
#define PIN_SW_HIGH_POWER              PD7
#define PIN_SW_CHARGE              PD6

#define MASK_DISPLAY_BACKLIGHT     _BV(PIN_DISPLAY_BACKLIGHT)
#define MASK_SW_MID_POWER            _BV(PIN_SW_MID_POWER)
#define MASK_SW_LOW_POWER             _BV(PIN_SW_LOW_POWER)
#define MASK_SW_HIGH_POWER             _BV(PIN_SW_HIGH_POWER)
#define MASK_SW_CHARGE             _BV(PIN_SW_CHARGE)

#define PORTD_OUTPUT_MASK          (MASK_DISPLAY_BACKLIGHT | MASK_SW_MID_POWER | MASK_SW_LOW_POWER | MASK_SW_HIGH_POWER | MASK_SW_CHARGE)

// ============================================================
// Port C Inputs / Outputs
// ============================================================

#define PIN_EN_DCDC                PC1
#define PIN_PG_DCDC                PC0

#define MASK_EN_DCDC               _BV(PIN_EN_DCDC)
#define MASK_PG_DCDC               _BV(PIN_PG_DCDC)

// ============================================================
// Button / ADC Parameters
// ============================================================

#define PIN_BUTTON_SET             PD2
#define MASK_BUTTON_SET            _BV(PIN_BUTTON_SET)

#define ADC_BUTTONS                6
#define ADC_LIGHT                  7

#define ADC_BTN_OK_VALUE           383
#define ADC_BTN_DOWN_VALUE         460
#define ADC_BTN_UP_VALUE           703
#define BUTTON_ADC_TOLERANCE       60

#define BUTTON_NONE                0
#define BUTTON_OK                  1
#define BUTTON_DOWN                2
#define BUTTON_UP                  3

// ============================================================
// Display Backlight
// ============================================================

#define DISPLAY_LIGHT_TIME_MS      60000UL

// ============================================================
// Display Layout
// ============================================================

#define DISPLAY_LEFT_X             2
#define DISPLAY_RIGHT_X            66
#define DISPLAY_COLUMN_W           60
#define DISPLAY_LABEL_Y            0
#define DISPLAY_SEPARATOR_Y       10		// Small line below the battery headline.
#define DISPLAY_VOLTAGE_Y          16
#define DISPLAY_CURRENT_Y          40		// Leave one empty text line between voltage and current.
#define DISPLAY_TEMP_Y             56		// Bottom line, right side.
#define DISPLAY_STATE_Y            56		// Bottom line, left side.
#define DISPLAY_CURRENT_MIN_mA     50L		// Hide current below this value.

// ============================================================
// INA238 Parameters
// ============================================================

#define INA238_CURRENT_LSB_uA      1000L		// 1mA/bit, supports up to about 32A

#define SHUNT_HIGH_POWER_mOHM          9		// 0.009 Ohm
#define SHUNT_MID_POWER_mOHM         9		// 0.009 Ohm
#define SHUNT_DCDC_mOHM            5		// 0.005 Ohm
#define SHUNT_BATTERY_mOHM         5		// 0.005 Ohm
#define SHUNT_LOW_POWER_mOHM          50		// 0.050 Ohm

// INA238 current correction.
// 1000 = no correction.
// 2000 = current value x 2.000.
// Use this for final calibration with a lab power supply or current meter.
#define INA238_CORR_HIGH_POWER_PERMILLE   2000L		// Input current calibration factor.
#define INA238_CORR_MID_POWER_PERMILLE    1000L		// Input current calibration factor.
#define INA238_CORR_LOW_POWER_PERMILLE    2000L		// Input current calibration factor.
#define INA238_CORR_DCDC_PERMILLE         1000L		// DC/DC current calibration factor.
#define INA238_CORR_BATTERY_PERMILLE      1000L		// Battery current calibration factor.

#define INA238_REG_CONFIG          0x00
#define INA238_REG_ADC_CONFIG      0x01
#define INA238_REG_SHUNT_CAL       0x02
#define INA238_REG_VSHUNT          0x04
#define INA238_REG_VBUS            0x05
#define INA238_REG_CURRENT         0x07
#define INA238_REG_DEVICE_ID       0x3F

#define INA238_CONFIG_VALUE        0x0000
#define INA238_ADC_CONFIG_VALUE    0xFB68

#define CH_HIGH_POWER                  0
#define CH_MID_POWER                 1
#define CH_DCDC                    2
#define CH_BATTERY                 3
#define CH_LOW_POWER                  4

struct INA238_Channel
{
	uint8_t address;
	uint16_t shunt_mOhm;
	int32_t currentCorrPermille;
	int32_t voltage_mV;
	int32_t current_mA;
	bool online;
};

INA238_Channel ina238[] =
{
	{ INA238_HIGH_POWER_ADDR,  SHUNT_HIGH_POWER_mOHM,  INA238_CORR_HIGH_POWER_PERMILLE,  0, 0, false },
	{ INA238_MID_POWER_ADDR, SHUNT_MID_POWER_mOHM, INA238_CORR_MID_POWER_PERMILLE, 0, 0, false },
	{ INA238_DCDC_ADDR,    SHUNT_DCDC_mOHM,    INA238_CORR_DCDC_PERMILLE,    0, 0, false },
	{ INA238_BATTERY_ADDR, SHUNT_BATTERY_mOHM, INA238_CORR_BATTERY_PERMILLE, 0, 0, false },
	{ INA238_LOW_POWER_ADDR,  SHUNT_LOW_POWER_mOHM,  INA238_CORR_LOW_POWER_PERMILLE,  0, 0, false }
};

#define INA238_COUNT (sizeof(ina238) / sizeof(ina238[0]))

// ============================================================
// DCDC State Machine
// ============================================================

enum DCDC_State
{
	DCDC_STATE_OFF = 0,
	DCDC_STATE_START,
	DCDC_STATE_CHARGE_ON,
	DCDC_STATE_RAMP_CURRENT,
	DCDC_STATE_READY
};

enum InputSource_Id
{
	INPUT_SOURCE_NONE = 0,
	INPUT_SOURCE_HIGH,
	INPUT_SOURCE_MID,
	INPUT_SOURCE_LOW
};

enum DCDC_PreChargeTest_State
{
	DCDC_TEST_IDLE = 0,
	DCDC_TEST_MATCH_BATTERY,
	DCDC_TEST_RAISE_OUTPUT,
	DCDC_TEST_CONNECT_LEVEL,
	DCDC_TEST_PASSED,
	DCDC_TEST_FAILED
};

// Battery charge state:
// WAIT: no charge until the battery is low enough.
// BULK: charge with current regulation up to absorption voltage.
// ABS: hold absorption voltage until battery current is below the full threshold.
// FULL: charger off until restart voltage is reached.
enum BatteryCharge_State
{
	BATTERY_CHARGE_WAIT = 0,
	BATTERY_CHARGE_BULK,
	BATTERY_CHARGE_ABS,
	BATTERY_CHARGE_FULL,
	BATTERY_CHARGE_FLOAT
};

bool DCDC_ControlOneStepToCurrent(uint32_t now_ms);
void DCDC_ResetCurrentControl();
void ChargeCurrent_Task(uint32_t now_ms);
int32_t ChargeCurrent_GetSet_mA();
int32_t ChargeCurrent_GetMax_mA();
int32_t InputCurrentLimit_Task(uint32_t now_ms, uint8_t source, int32_t requestedSet_mA);
int32_t InputCurrent_Get_mA(uint8_t source);
int32_t InputCurrent_GetMax_mA(uint8_t source);
void BatteryCharge_Task();
bool BatteryCharge_IsAllowed();
const char* BatteryCharge_StateText(BatteryCharge_State state);
bool DCDC_SetStartVoltageFromBattery();
int32_t DCDC_GetStartTargetFromBattery_mV();
int32_t DCDC_GetMaxTargetFromBattery_mV();
int32_t DCDC_GetTargetFromBattery_mV();
void DCDC_AllOff();
void Debug_Task(uint32_t now_ms);
bool Debug_ShouldPrintLine();
void Debug_PrintLine(uint32_t now_ms);
void Debug_UpdateLastOffValues();
int32_t Debug_VoltageBucket100mV(int32_t voltage_mV);
const char* Debug_SourceText();
void Charge_mAh_Task(uint32_t now_ms);
void Debug_Print_mAh_x1000(int64_t value_x1000);
void ButtonBacklight_Task(uint32_t now_ms);
void SerialCommand_Task();
void Debug_PrintStatusOnce();
void ButtonOK_Task(uint32_t now_ms);
void DCDC_PreChargeTest_Reset();
bool DCDC_PreChargeTest_Task(uint32_t now_ms);
bool DCDC_PreChargeTest_AdjustToVoltage(int32_t target_mV);
void DCDC_PreChargeTest_Fail();
void DCDC_ReverseCurrentCheck_Reset();
bool DCDC_ReverseCurrentCheck_Task(uint32_t now_ms);
int32_t DCDC_GetAllowedPower_W();
int32_t DCDC_GetPowerLimitedCurrent_mA();
void TemperatureProtection_Task();
void BatteryVoltageProtection_Task();

// ============================================================
// ADS1015 / NTC Parameters
// ============================================================

#define ADS1015_REG_CONVERSION     0x00
#define ADS1015_REG_CONFIG         0x01
#define ADS1015_REG_LO_THRESH      0x02
#define ADS1015_REG_HI_THRESH      0x03

#define ADS1015_OS_SINGLE          0x8000
#define ADS1015_PGA_6V144          0x0000
#define ADS1015_MODE_SINGLE        0x0100
#define ADS1015_MODE_CONTINUOUS    0x0000
#define ADS1015_DR_1600SPS         0x0080
#define ADS1015_COMP_DISABLE       0x0003

#define ADS1015_MUX_A0_A1          0x0000
#define ADS1015_MUX_A2_A3          0x3000

#define ADS1015_COMP_QUE_1         0x0000
#define ADS1015_COMP_MODE_TRAD     0x0000
#define ADS1015_COMP_POL_LOW       0x0000
#define ADS1015_COMP_LAT_NON       0x0000

// ============================================================
// NTC Conversion Table
// ============================================================

struct NTC_TableEntry
{
	int16_t temp_C;
	int16_t raw;
};

const NTC_TableEntry ntcTable[] PROGMEM =
{
	{ -10, 880 },
	{   0, 790 },
	{  10, 690 },
	{  20, 585 },
	{  25, 535 },
	{  30, 490 },
	{  40, 405 },
	{  50, 330 },
	{  60, 270 },
	{  70, 220 },
	{  80, 180 }
};

#define NTC_TABLE_COUNT (sizeof(ntcTable) / sizeof(ntcTable[0]))

// ============================================================
// Runtime Variables
// ============================================================

// Timing variables are separated by task.
uint32_t lastMeasure_ms = 0;             // INA238 measurement interval.
uint32_t lastControl_ms = 0;             // Charge current control interval.
uint32_t lastLog_ms = 0;                 // UART debug log interval.
uint32_t lastDisplay_ms = 0;             // LCD display refresh interval.
uint32_t lastChargeAccumulator_ms = 0;   // mAh integration interval.
uint32_t lastDcdcTask_ms = 0;            // DCDC state machine interval.
uint32_t dcdcDelayStart_ms = 0;          // Delay timer for DCDC start sequence.
uint32_t sourceDelayStart_ms = 0;        // Delay timer before source switch ON.
uint8_t inputSourcePending = INPUT_SOURCE_NONE; // Source waiting for SOURCE_ON_DELAY_MS.
uint32_t lastInputCurrentRegulation_ms = 0; // Input current limit regulation interval.
uint32_t lastFanCheck_ms = 0;            // Fan temperature check interval.
uint32_t lightTimer_ms = 0;              // Display backlight timeout.
bool lastBacklightSetPressed = false;      // Last SET button state for backlight trigger.
uint8_t lastBacklightAdcButton = BUTTON_NONE; // Last ADC button state for backlight trigger.

int64_t charge_mA_ms = 0;

int32_t debugLastHigh_100mV = 0x7FFFFFFFL;
int32_t debugLastMid_100mV = 0x7FFFFFFFL;
int32_t debugLastLow_100mV = 0x7FFFFFFFL;
int32_t debugLastBattery_100mV = 0x7FFFFFFFL;
int16_t debugLastFetTemperature_C = 32767;
int16_t debugLastCoilTemperature_C = 32767;
BatteryCharge_State debugLastBatteryChargeState = BATTERY_CHARGE_WAIT;


bool stateMidPower = false;
bool stateLowPower = false;
bool stateHighPower = false;
bool stateCharge = false;
bool stateDCDC = false;
bool fanState = false;

int16_t fetTemperature_C = 0;
int16_t coilTemperature_C = 0;

uint16_t dacCode = 0;
uint32_t chargePositiveStart_ms = 0;
int32_t chargeCurrentFiltered_mA = 0;
int32_t activeChargeCurrentSet_mA = 0L;
int32_t activeChargeCurrentMax_mA = 0L;
int32_t inputLimitedChargeCurrentSet_mA = 0L;
uint8_t inputLimitSource = INPUT_SOURCE_NONE;
BatteryCharge_State batteryChargeState = BATTERY_CHARGE_WAIT;
bool chargeCurrentFilterValid = false;
uint8_t chargeStableCounter = 0;
DCDC_State dcdcState = DCDC_STATE_OFF;
DCDC_PreChargeTest_State dcdcPreChargeTestState = DCDC_TEST_IDLE;
uint32_t dcdcPreChargeTestPhaseStart_ms = 0UL;
int32_t dcdcPreChargeTestBaseVoltage_mV = 0L;
uint8_t dcdcPreChargeTestStableCounter = 0U;
bool dcdcPreChargeTestFault = false;
bool dcdcReverseCurrentTiming = false;
uint32_t dcdcReverseCurrentStart_ms = 0UL;
uint32_t dcdcReverseBlockedUntil_ms = 0UL;
bool thermalShutdown = false;
bool batteryVoltageFault = false;
uint32_t absorptionStart_ms = 0UL;
uint32_t fullRestStart_ms = 0UL;
bool fullRestEvaluated = false;
bool forceFullChargeRequested = false;
bool lastOkPressed = false;

// ============================================================
// Display Texts in Flash
// ============================================================

const char txtBattery[] PROGMEM = "BATTERY";
const char txtMidPower[]   PROGMEM = "MID";
const char txtHighPower[]    PROGMEM = "HIGH";
const char txtLowPower[]  PROGMEM = "LOW";
const char txtCharge[]  PROGMEM = "CHARGE";
const char txtDCDC[]    PROGMEM = "DCDC";
const char txtOn[]      PROGMEM = "ON";
const char txtOff[]     PROGMEM = "OFF";

// ============================================================
// Arduino Setup
// ============================================================

void setup()
{
	DBG_BEGIN(UART_BAUDRATE);

	Wire.begin();
	Wire.setClock(I2C_CLOCK_HZ);

	IO_Init();
	LCD_Init();
	LCD_Clear();

	DBG_PRINTLN(F("BatteryCharger_24V"));

	for (uint8_t i = 0; i < INA238_COUNT; i++)
	{
		ina238[i].online = INA238_Init(&ina238[i]);
	}

	DisplayBacklight_Off();

	MidPower_Off();
	LowPower_Off();
	HighPower_Off();
	Charge_Off();
	DCDC_Disable();
	DAC_WriteCode(DCDC_SAFE_START_CODE);
	ADS1015_AlertForceOff();
}

// ============================================================
// Arduino Loop
// ============================================================

void loop()
{
	uint32_t now_ms = millis();

	SerialCommand_Task();
	ButtonBacklight_Task(now_ms);
	ButtonOK_Task(now_ms);
	DisplayBacklight_Task(now_ms);
	FanTemp_Task(now_ms);
	TemperatureProtection_Task();

	if ((uint32_t)(now_ms - lastMeasure_ms) >= MEASURE_INTERVAL_MS)
	{
		lastMeasure_ms = now_ms;

		for (uint8_t i = 0; i < INA238_COUNT; i++)
		{
			if (ina238[i].online)
			INA238_ReadValues(&ina238[i]);
		}

		if ((uint32_t)(now_ms - lastDisplay_ms) >= DISPLAY_INTERVAL_MS)
		{
			lastDisplay_ms = now_ms;
			LCD_PrintMeasurements();
		}

		BatteryVoltageProtection_Task();
		InputSource_Task();
		DCDC_Task(now_ms);
	}

	Debug_Task(now_ms);
}

// ============================================================
// UART Commands / OK Button
// ============================================================

void SerialCommand_Task()
{
	if (!Serial.available()) return;

	String command = Serial.readStringUntil('\n');
	command.trim();
	command.toUpperCase();

	if (command == "D0")
	{
		debugEnabled = false;
		Serial.println(F("Debug OFF"));
	}
	else if (command == "D1")
	{
		debugEnabled = true;
		Serial.println(F("Debug ON"));
	}
	else if (command == "P")
	Debug_PrintStatusOnce();
	else if (command == "?")
	{
		Serial.println(F("D0  Debug OFF"));
		Serial.println(F("D1  Debug ON"));
		Serial.println(F("P   Print status once"));
		Serial.println(F("R   Reboot controller"));
	}
	else if (command == "R")
	{
		Serial.println(F("Reboot"));
		Serial.flush();
		asm volatile ("jmp 0");
	}
}

void Debug_PrintStatusOnce()
{
	Debug_PrintLine(millis());
}

void ButtonOK_Task(uint32_t now_ms)
{
	(void)now_ms;
	bool okPressed = (ButtonADC_Read() == BUTTON_OK);

	if (okPressed && !lastOkPressed)
	{
		forceFullChargeRequested = true;
		DisplayBacklight_ResetTimer(millis());
	}

	lastOkPressed = okPressed;
}

// ============================================================
// UART Debug Log
// ============================================================

void Debug_Task(uint32_t now_ms)
{
	if (!debugEnabled)
	return;

	Charge_mAh_Task(now_ms);

	if ((uint32_t)(now_ms - lastLog_ms) < DEBUG_INTERVAL_MS)
	return;

	lastLog_ms = now_ms;

	if (!Debug_ShouldPrintLine())
	return;

	Debug_PrintLine(now_ms);

	if (dcdcState == DCDC_STATE_OFF)
	Debug_UpdateLastOffValues();
}

bool Debug_ShouldPrintLine()
{
	#if DEBUG_OFF_CHANGE_ONLY
	if (dcdcState != DCDC_STATE_OFF)
	return true;

	if (Debug_VoltageBucket100mV(ina238[CH_HIGH_POWER].voltage_mV) != debugLastHigh_100mV)
	return true;

	if (Debug_VoltageBucket100mV(ina238[CH_MID_POWER].voltage_mV) != debugLastMid_100mV)
	return true;

	if (Debug_VoltageBucket100mV(ina238[CH_LOW_POWER].voltage_mV) != debugLastLow_100mV)
	return true;

	if (Debug_VoltageBucket100mV(ina238[CH_BATTERY].voltage_mV) != debugLastBattery_100mV)
	return true;

	if (fetTemperature_C != debugLastFetTemperature_C)
	return true;

	if (coilTemperature_C != debugLastCoilTemperature_C)
	return true;

	if (batteryChargeState != debugLastBatteryChargeState)
	return true;

	return false;
	#else
	return true;
	#endif
}

void Debug_PrintLine(uint32_t now_ms)
{
	int32_t err_mA;
	int64_t mAh_x1000;

	err_mA = ChargeCurrent_GetSet_mA() - chargeCurrentFiltered_mA;
	mAh_x1000 = charge_mA_ms / 3600LL;

	DBG_PRINT(F("t="));
	DBG_PRINT(now_ms);
	DBG_PRINT(F("ms"));

	DBG_PRINT(F(" STATE="));
	DBG_PRINT(DCDC_StateText(dcdcState));
	DBG_PRINT(F(" BATT="));
	DBG_PRINT(BatteryCharge_StateText(batteryChargeState));
	DBG_PRINT(F(" SRC="));
	DBG_PRINT(Debug_SourceText());

	DBG_PRINT(F(" | HIGH="));
	DBG_PRINT(ina238[CH_HIGH_POWER].voltage_mV);
	DBG_PRINT(F("mV MID="));
	DBG_PRINT(ina238[CH_MID_POWER].voltage_mV);
	DBG_PRINT(F("mV LOW="));
	DBG_PRINT(ina238[CH_LOW_POWER].voltage_mV);
	DBG_PRINT(F("mV"));

	DBG_PRINT(F(" | BAT="));
	DBG_PRINT(ina238[CH_BATTERY].voltage_mV);
	DBG_PRINT(F("mV DCDC="));
	DBG_PRINT(ina238[CH_DCDC].voltage_mV);
	DBG_PRINT(F("mV"));

	DBG_PRINT(F(" | Ibat="));
	DBG_PRINT(ina238[CH_BATTERY].current_mA);
	DBG_PRINT(F("mA Ifilt="));
	DBG_PRINT(chargeCurrentFiltered_mA);
	DBG_PRINT(F("mA Iset="));
	DBG_PRINT(ChargeCurrent_GetSet_mA());
	DBG_PRINT(F("mA Imax="));
	DBG_PRINT(ChargeCurrent_GetMax_mA());
	DBG_PRINT(F("mA Iin="));
	DBG_PRINT(InputCurrent_Get_mA(inputLimitSource));
	DBG_PRINT(F("mA IinMax="));
	DBG_PRINT(InputCurrent_GetMax_mA(inputLimitSource));
	DBG_PRINT(F("mA ERR="));
	DBG_PRINT(err_mA);
	DBG_PRINT(F("mA mAh="));
	Debug_Print_mAh_x1000(mAh_x1000);

	DBG_PRINT(F(" | DAC="));
	DBG_PRINT(dacCode);

	DBG_PRINT(F(" | T="));
	DBG_PRINT(fetTemperature_C);
	DBG_PRINT(F("/"));
	DBG_PRINT(coilTemperature_C);
	DBG_PRINTLN();
}

void Debug_UpdateLastOffValues()
{
	debugLastHigh_100mV = Debug_VoltageBucket100mV(ina238[CH_HIGH_POWER].voltage_mV);
	debugLastMid_100mV = Debug_VoltageBucket100mV(ina238[CH_MID_POWER].voltage_mV);
	debugLastLow_100mV = Debug_VoltageBucket100mV(ina238[CH_LOW_POWER].voltage_mV);
	debugLastBattery_100mV = Debug_VoltageBucket100mV(ina238[CH_BATTERY].voltage_mV);
	debugLastFetTemperature_C = fetTemperature_C;
	debugLastCoilTemperature_C = coilTemperature_C;
	debugLastBatteryChargeState = batteryChargeState;
}

const char* Debug_SourceText()
{
	if (stateHighPower)
	return "HIGH";

	if (stateMidPower)
	return "MID";

	if (stateLowPower)
	return "LOW";

	if (inputSourcePending == INPUT_SOURCE_HIGH)
	return "WAIT_HIGH";

	if (inputSourcePending == INPUT_SOURCE_MID)
	return "WAIT_MID";

	if (inputSourcePending == INPUT_SOURCE_LOW)
	return "WAIT_LOW";

	return "NONE";
}

const char* BatteryCharge_StateText(BatteryCharge_State state)
{
	switch (state)
	{
		case BATTERY_CHARGE_WAIT: return "WAIT";
		case BATTERY_CHARGE_BULK: return "BULK";
		case BATTERY_CHARGE_ABS:  return "ABS";
		case BATTERY_CHARGE_FULL: return "FULL";
		case BATTERY_CHARGE_FLOAT: return "FLOAT";
	}

	return "?";
}

int32_t Debug_VoltageBucket100mV(int32_t voltage_mV)
{
	// Group voltage changes into 100 mV ranges for quiet OFF-state logging.
	return voltage_mV / 100L;
}

void Charge_mAh_Task(uint32_t now_ms)
{
	uint32_t dt_ms;
	int32_t current_mA;

	if (lastChargeAccumulator_ms == 0)
	{
		lastChargeAccumulator_ms = now_ms;
		return;
	}

	dt_ms = now_ms - lastChargeAccumulator_ms;
	lastChargeAccumulator_ms = now_ms;

	if (!stateCharge)
	return;

	if (chargeCurrentFilterValid)
	current_mA = chargeCurrentFiltered_mA;
	else
	current_mA = ina238[CH_BATTERY].current_mA;

	// Count only positive charge current into the battery.
	if (current_mA > 0)
	charge_mA_ms += (int64_t)current_mA * (int64_t)dt_ms;
}

void Debug_Print_mAh_x1000(int64_t value_x1000)
{
	int64_t whole;
	int16_t frac;

	whole = value_x1000 / 1000LL;
	frac = (int16_t)(value_x1000 % 1000LL);

	DBG_PRINT((long)whole);
	DBG_PRINT(F("."));

	if (frac < 100)
	DBG_PRINT(F("0"));

	if (frac < 10)
	DBG_PRINT(F("0"));

	DBG_PRINT(frac);
}

// ============================================================
// MCP4725 DAC Low Level
// ============================================================

bool DAC_WriteCode(uint16_t code)
{
	if (code > DAC_CODE_MAX)
	code = DAC_CODE_MAX;

	Wire.beginTransmission(MCP4725_ADDR);
	Wire.write(0x40);                              // Fast write, DAC register only.
	Wire.write((uint8_t)(code >> 4));
	Wire.write((uint8_t)((code & 0x000F) << 4));

	if (Wire.endTransmission() != 0)
	{
		DBG_PRINTLN(F("DAC write error"));
		return false;
	}

	dacCode = code;
	return true;
}

uint16_t DAC_VoltageToCode_mV(uint16_t voltage_mV)
{
	uint32_t code;

	if (voltage_mV > DAC_VREF_mV)
	voltage_mV = DAC_VREF_mV;

	code = ((uint32_t)voltage_mV * 4095UL + (DAC_VREF_mV / 2)) / DAC_VREF_mV;

	if (code > DAC_CODE_MAX)
	code = DAC_CODE_MAX;

	return (uint16_t)code;
}

uint16_t DAC_CodeToVoltage_mV(uint16_t code)
{
	if (code > DAC_CODE_MAX)
	code = DAC_CODE_MAX;

	return (uint16_t)(((uint32_t)code * DAC_VREF_mV + 2047UL) / 4095UL);
}

// ============================================================
// DAC Output Voltage Model
// ============================================================

uint16_t DCDC_CalcDacCodeForVout_mV(uint16_t vout_mV)
{
	int32_t code;

	if (vout_mV < DCDC_VOUT_MIN_mV)
	vout_mV = DCDC_VOUT_MIN_mV;

	if (vout_mV > DCDC_VOUT_MAX_mV)
	vout_mV = DCDC_VOUT_MAX_mV;

	// DCDC_mV = 16150 - 1.303 * DAC_Code
	// DAC_Code = (16150 - DCDC_mV) / 1.303
	code = ((DCDC_DAC_OFFSET_mV - (int32_t)vout_mV) * 1000L + (DCDC_DAC_SLOPE_uV_CODE / 2)) / DCDC_DAC_SLOPE_uV_CODE;

	if (code < DAC_CODE_MIN)
	code = DAC_CODE_MIN;

	if (code > DAC_CODE_MAX)
	code = DAC_CODE_MAX;

	return (uint16_t)code;
}

int32_t DCDC_CalcVoutFromDacCode_mV(uint16_t code)
{
	int32_t vout_mV;

	if (code > DAC_CODE_MAX)
	code = DAC_CODE_MAX;

	// DCDC_mV = 16150 - 1.303 * DAC_Code
	vout_mV = DCDC_DAC_OFFSET_mV - (((int32_t)code * DCDC_DAC_SLOPE_uV_CODE + 500L) / 1000L);

	return vout_mV;
}

bool DCDC_SetVout_mV(uint16_t vout_mV)
{
	uint16_t targetCode;

	targetCode = DCDC_CalcDacCodeForVout_mV(vout_mV);

	DBG_PRINT(F("DCDC target="));
	DBG_PRINT(vout_mV);
	DBG_PRINT(F("mV DAC="));
	DBG_PRINTLN(targetCode);

	return DAC_RampToCode(targetCode);
}

bool DAC_RampToCode(uint16_t targetCode)
{
	uint16_t nextCode;

	if (targetCode > DAC_CODE_MAX)
	targetCode = DAC_CODE_MAX;

	while (dacCode != targetCode)
	{
		if (dacCode < targetCode)
		{
			nextCode = dacCode + DAC_RAMP_STEP_CODE;
			if (nextCode > targetCode)
			nextCode = targetCode;
		}
		else
		{
			if (dacCode > DAC_RAMP_STEP_CODE)
			nextCode = dacCode - DAC_RAMP_STEP_CODE;
			else
			nextCode = 0;

			if (nextCode < targetCode)
			nextCode = targetCode;
		}

		if (!DAC_WriteCode(nextCode))
		return false;

		if (DAC_RAMP_STEP_DELAY_MS > 0)
		delay(DAC_RAMP_STEP_DELAY_MS);
	}

	return true;
}

// ============================================================
// IO Low Level
// ============================================================

void IO_Init()
{
	DDRD |= PORTD_OUTPUT_MASK;
	DDRD &= ~MASK_BUTTON_SET;

	// Source switches are inverted: HIGH = OFF, LOW = ON.
	PORTD |= MASK_SW_MID_POWER;
	PORTD |= MASK_SW_LOW_POWER;
	PORTD |= MASK_SW_HIGH_POWER;

	// Charge switch is not inverted: LOW = OFF, HIGH = ON.
	PORTD &= ~MASK_SW_CHARGE;
	PORTD &= ~MASK_DISPLAY_BACKLIGHT;
	PORTD |= MASK_BUTTON_SET;

	DDRC |= MASK_EN_DCDC;
	DDRC &= ~MASK_PG_DCDC;

	// DCDC inverted: HIGH = OFF, LOW = ON
	PORTC |= MASK_EN_DCDC;
	PORTC |= MASK_PG_DCDC;
}

// ============================================================
// Output Control
// ============================================================

void DisplayBacklight_On()  { PORTD |=  MASK_DISPLAY_BACKLIGHT; }
void DisplayBacklight_Off() { PORTD &= ~MASK_DISPLAY_BACKLIGHT; }

void MidPower_On()           { PORTD &= ~MASK_SW_MID_POWER; stateMidPower = true; }
void MidPower_Off()          { PORTD |=  MASK_SW_MID_POWER; stateMidPower = false; }

void LowPower_On()            { PORTD &= ~MASK_SW_LOW_POWER; stateLowPower = true; }
void LowPower_Off()           { PORTD |=  MASK_SW_LOW_POWER; stateLowPower = false; }

void HighPower_On()            { PORTD &= ~MASK_SW_HIGH_POWER; stateHighPower = true; }
void HighPower_Off()           { PORTD |=  MASK_SW_HIGH_POWER; stateHighPower = false; }

void Charge_On()            { PORTD |=  MASK_SW_CHARGE; stateCharge = true; }
void Charge_Off()           { PORTD &= ~MASK_SW_CHARGE; stateCharge = false; }

void DCDC_Enable()          { PORTC &= ~MASK_EN_DCDC; stateDCDC = true; }
void DCDC_Disable()         { PORTC |=  MASK_EN_DCDC; stateDCDC = false; }

// ============================================================
// Input Read
// ============================================================

bool DCDC_PowerGood()
{
	// PG_DCDC is inverted by hardware: LOW = power good.
	return ((PINC & MASK_PG_DCDC) == 0);
}

bool ButtonSet_Pressed()
{
	return !(PIND & MASK_BUTTON_SET);
}

// ============================================================
// Backlight Timer
// ============================================================

void DisplayBacklight_ResetTimer(uint32_t now_ms)
{
	DisplayBacklight_On();
	lightTimer_ms = now_ms + DISPLAY_LIGHT_TIME_MS;
}

void DisplayBacklight_Task(uint32_t now_ms)
{
	if ((PORTD & MASK_DISPLAY_BACKLIGHT) == 0)
	return;

	if ((int32_t)(now_ms - lightTimer_ms) >= 0)
	DisplayBacklight_Off();
}

bool DisplayBacklight_IsOn()
{
	return (PORTD & MASK_DISPLAY_BACKLIGHT);
}

void ButtonBacklight_Task(uint32_t now_ms)
{
	bool setPressed = ButtonSet_Pressed();
	uint8_t adcButton = ButtonADC_Read();

	// Every new button press switches the backlight on for the full light time.
	if ((setPressed && !lastBacklightSetPressed) || ((adcButton != BUTTON_NONE) && (lastBacklightAdcButton == BUTTON_NONE)))
	DisplayBacklight_ResetTimer(now_ms);

	lastBacklightSetPressed = setPressed;
	lastBacklightAdcButton = adcButton;
}

// ============================================================
// INA238 Low Level I2C
// ============================================================

bool INA238_Write16(uint8_t address, uint8_t reg, uint16_t value)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write((uint8_t)(value >> 8));
	Wire.write((uint8_t)(value & 0xFF));
	return (Wire.endTransmission() == 0);
}

bool INA238_Read16(uint8_t address, uint8_t reg, uint16_t* value)
{
	Wire.beginTransmission(address);
	Wire.write(reg);

	if (Wire.endTransmission(false) != 0)
	return false;

	if (Wire.requestFrom(address, (uint8_t)2) != 2)
	return false;

	*value = ((uint16_t)Wire.read() << 8) | Wire.read();
	return true;
}

uint16_t INA238_CalcCalibration(uint16_t shunt_mOhm)
{
	uint32_t cal;

	// CAL = 819200000 * CurrentLSB_A * Rshunt_Ohm
	// CurrentLSB_A = uA / 1000000
	// Rshunt_Ohm   = mOhm / 1000
	cal = 8192UL * (uint32_t)INA238_CURRENT_LSB_uA * (uint32_t)shunt_mOhm;
	cal = (cal + 5000UL) / 10000UL;

	if (cal > 65535UL)
	cal = 65535UL;

	return (uint16_t)cal;
}

// ============================================================
// INA238 Driver
// ============================================================

bool INA238_Init(INA238_Channel* ch)
{
	uint16_t dummy = 0;

	if (!INA238_Read16(ch->address, INA238_REG_DEVICE_ID, &dummy))
	return false;

	if (!INA238_Write16(ch->address, INA238_REG_CONFIG, INA238_CONFIG_VALUE))
	return false;

	if (!INA238_Write16(ch->address, INA238_REG_ADC_CONFIG, INA238_ADC_CONFIG_VALUE))
	return false;

	if (!INA238_Write16(ch->address, INA238_REG_SHUNT_CAL, INA238_CalcCalibration(ch->shunt_mOhm)))
	return false;

	return true;
}

bool INA238_ReadValues(INA238_Channel* ch)
{
	uint16_t rawBus = 0;
	uint16_t rawCurrent = 0;

	if (!INA238_Read16(ch->address, INA238_REG_VBUS, &rawBus))
	{
		ch->online = false;
		return false;
	}

	if (!INA238_Read16(ch->address, INA238_REG_CURRENT, &rawCurrent))
	{
		ch->online = false;
		return false;
	}

	// VBUS LSB = 3.125mV
	ch->voltage_mV = ((int32_t)(int16_t)rawBus * 3125L) / 1000L;

	// CURRENT LSB = INA238_CURRENT_LSB_uA.
	// The correction factor is used for board calibration only.
	ch->current_mA = ((int32_t)(int16_t)rawCurrent * INA238_CURRENT_LSB_uA) / 1000L;
	ch->current_mA = (ch->current_mA * ch->currentCorrPermille) / 1000L;

	return true;
}

// ============================================================
// ADC Button Input
// ============================================================

uint8_t ButtonADC_Read()
{
	uint16_t adc = analogRead(ADC_BUTTONS);

	if (ADC_IsNear(adc, ADC_BTN_OK_VALUE))
	return BUTTON_OK;

	if (ADC_IsNear(adc, ADC_BTN_DOWN_VALUE))
	return BUTTON_DOWN;

	if (ADC_IsNear(adc, ADC_BTN_UP_VALUE))
	return BUTTON_UP;

	return BUTTON_NONE;
}

bool ADC_IsNear(uint16_t value, uint16_t target)
{
	if (value > target)
	return ((value - target) <= BUTTON_ADC_TOLERANCE);
	else
	return ((target - value) <= BUTTON_ADC_TOLERANCE);
}


// ============================================================
// ST7565 Low Level, direct page output, no framebuffer
// ============================================================

void LCD_PinInit()
{
	DDRB |= LCD_CS_M | LCD_A0_M | LCD_RST_M | LCD_SDA_M | LCD_SCK_M;

	PORTB |= LCD_CS_M;
	PORTB &= ~LCD_SCK_M;
	PORTB &= ~LCD_SDA_M;
	PORTB |= LCD_RST_M;
}

void LCD_WriteByte(uint8_t data)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if (data & 0x80) PORTB |= LCD_SDA_M;
		else             PORTB &= ~LCD_SDA_M;

		PORTB |= LCD_SCK_M;
		PORTB &= ~LCD_SCK_M;

		data <<= 1;
	}
}

void LCD_Command(uint8_t cmd)
{
	PORTB &= ~LCD_A0_M;
	PORTB &= ~LCD_CS_M;
	LCD_WriteByte(cmd);
	PORTB |= LCD_CS_M;
}

void LCD_Data(uint8_t data)
{
	PORTB |= LCD_A0_M;
	PORTB &= ~LCD_CS_M;
	LCD_WriteByte(data);
	PORTB |= LCD_CS_M;
}

void LCD_Init()
{
	LCD_PinInit();

	PORTB &= ~LCD_RST_M;
	delay(50);
	PORTB |= LCD_RST_M;
	delay(50);

	LCD_Command(0xAE);    // Display OFF
	LCD_Command(0xA2);    // Bias 1/9
	LCD_Command(0xA0);    // ADC normal
	LCD_Command(0xC8);    // COM reverse
	LCD_Command(0x40);    // Start line 0
	LCD_Command(0x25);    // Resistor ratio
	LCD_Command(0x81);    // Electronic volume
	LCD_Command(0x28);    // Contrast
	LCD_Command(0x2F);    // Power control
	delay(50);
	LCD_Command(0xA6);    // Normal display
	LCD_Command(0xA4);    // RAM content
	LCD_Command(0xAF);    // Display ON
}

void LCD_SetPos(uint8_t x, uint8_t page)
{
	LCD_Command(0xB0 | (page & 0x07));
	LCD_Command(0x10 | (x >> 4));
	LCD_Command(0x00 | (x & 0x0F));
}

void LCD_Clear()
{
	for (uint8_t page = 0; page < LCD_PAGES; page++)
	{
		LCD_SetPos(0, page);
		for (uint8_t x = 0; x < LCD_WIDTH; x++)
		LCD_Data(0x00);
	}
}

void LCD_Update()
{
	// Not required without framebuffer.
}

void LCD_SetPixel(uint8_t x, uint8_t y)
{
	// Not used in direct text mode.
}

// ============================================================
// ST7565 Text
// ============================================================

void LCD_Char(uint8_t x, uint8_t y, char c)
{
	uint8_t pattern[5];

	LCD_GetCharPattern(c, pattern);
	LCD_SetPos(x, y >> 3);

	for (uint8_t col = 0; col < 5; col++)
	LCD_Data(pattern[col]);

	LCD_Data(0x00);
}

void LCD_Text(uint8_t x, uint8_t y, const char* txt)
{
	LCD_SetPos(x, y >> 3);

	while (*txt)
	{
		uint8_t pattern[5];
		LCD_GetCharPattern(*txt++, pattern);

		for (uint8_t col = 0; col < 5; col++)
		LCD_Data(pattern[col]);

		LCD_Data(0x00);
	}
}

void LCD_Text_P(uint8_t x, uint8_t y, const char* txt)
{
	LCD_SetPos(x, y >> 3);

	char c;
	while ((c = pgm_read_byte(txt++)))
	{
		uint8_t pattern[5];
		LCD_GetCharPattern(c, pattern);

		for (uint8_t col = 0; col < 5; col++)
		LCD_Data(pattern[col]);

		LCD_Data(0x00);
	}
}

void LCD_HLine(uint8_t x1, uint8_t x2, uint8_t y)
{
	uint8_t mask;

	mask = (uint8_t)(1 << (y & 7));
	LCD_SetPos(x1, y >> 3);

	while (x1 <= x2)
	{
		LCD_Data(mask);
		x1++;
	}
}

// ============================================================
// Display Measurements
// ============================================================

void LCD_PrintMeasurements()
{
	uint8_t sourceChannel;
	const char* sourceText_P;
	int32_t batteryCurrent_mA;
	int32_t sourceCurrent_mA;

	LCD_Clear();

	sourceChannel = LCD_GetSelectedSourceChannel();
	sourceText_P = LCD_GetSelectedSourceText_P(sourceChannel);
	batteryCurrent_mA = LCD_GetDisplayBatteryCurrent_mA();
	sourceCurrent_mA = ina238[sourceChannel].current_mA;

	// Left column: battery values.
	LCD_Text_P(LCD_CenterX(DISPLAY_LEFT_X, DISPLAY_COLUMN_W, 7), DISPLAY_LABEL_Y, txtBattery);
	LCD_HLine(DISPLAY_LEFT_X, DISPLAY_LEFT_X + DISPLAY_COLUMN_W - 4, DISPLAY_SEPARATOR_Y);
	LCD_PrintBigSignedFixed1(DISPLAY_LEFT_X, DISPLAY_COLUMN_W, DISPLAY_VOLTAGE_Y, ina238[CH_BATTERY].voltage_mV, 1000L, 'V');

	if (LCD_CurrentFlows(batteryCurrent_mA))
	LCD_PrintBigSignedFixed1(DISPLAY_LEFT_X, DISPLAY_COLUMN_W, DISPLAY_CURRENT_Y, batteryCurrent_mA, 1000L, 'A');

	// Right column: show source values only if at least one source has minimum voltage.
	if (LCD_AnySourceHasMinimumVoltage())
	{
		LCD_Text_P(LCD_CenterX(DISPLAY_RIGHT_X, DISPLAY_COLUMN_W, LCD_TextLength_P(sourceText_P)), DISPLAY_LABEL_Y, sourceText_P);
		LCD_PrintBigSignedFixed1(DISPLAY_RIGHT_X, DISPLAY_COLUMN_W, DISPLAY_VOLTAGE_Y, ina238[sourceChannel].voltage_mV, 1000L, 'V');

		if (LCD_CurrentFlows(sourceCurrent_mA))
		LCD_PrintBigSignedFixed1(DISPLAY_RIGHT_X, DISPLAY_COLUMN_W, DISPLAY_CURRENT_Y, sourceCurrent_mA, 1000L, 'A');
	}

	LCD_PrintSystemState();
	LCD_PrintHighestTemperature();
}

uint8_t LCD_GetSelectedSourceChannel()
{
	if (stateHighPower)
	return CH_HIGH_POWER;

	if (stateMidPower)
	return CH_MID_POWER;

	if (stateLowPower)
	return CH_LOW_POWER;

	// No source is switched on: show the best available source voltage.
	if (ina238[CH_HIGH_POWER].voltage_mV >= HIGH_POWER_ON_mV)
	return CH_HIGH_POWER;

	if (ina238[CH_MID_POWER].voltage_mV >= MID_POWER_ON_mV)
	return CH_MID_POWER;

	if (ina238[CH_LOW_POWER].voltage_mV >= LOW_POWER_ON_mV)
	return CH_LOW_POWER;

	return CH_HIGH_POWER;
}

const char* LCD_GetSelectedSourceText_P(uint8_t channel)
{
	if (channel == CH_MID_POWER)
	return txtMidPower;

	if (channel == CH_LOW_POWER)
	return txtLowPower;

	return txtHighPower;
}

int32_t LCD_GetDisplayBatteryCurrent_mA()
{
	// The filtered current is easier to read while charging.
	if (chargeCurrentFilterValid)
	return chargeCurrentFiltered_mA;

	return ina238[CH_BATTERY].current_mA;
}

bool LCD_CurrentFlows(int32_t current_mA)
{
	if (current_mA < 0)
	current_mA = -current_mA;

	return (current_mA >= DISPLAY_CURRENT_MIN_mA);
}

bool LCD_AnySourceHasMinimumVoltage()
{
	if (ina238[CH_HIGH_POWER].voltage_mV >= HIGH_POWER_ON_mV)
	return true;

	if (ina238[CH_MID_POWER].voltage_mV >= MID_POWER_ON_mV)
	return true;

	if (ina238[CH_LOW_POWER].voltage_mV >= LOW_POWER_ON_mV)
	return true;

	return false;
}

uint8_t LCD_TextLength_P(const char* txt)
{
	uint8_t len = 0;

	while (pgm_read_byte(txt++))
	len++;

	return len;
}

uint8_t LCD_CenterX(uint8_t x, uint8_t width, uint8_t chars)
{
	uint8_t textWidth = chars * 6;

	if (textWidth >= width)
	return x;

	return x + ((width - textWidth) / 2);
}

void LCD_PrintSystemState()
{
	// Last display line: state text only, no label.
	LCD_Text(0, DISPLAY_STATE_Y, DCDC_StateText(dcdcState));
	LCD_Text(42, DISPLAY_STATE_Y, BatteryCharge_StateText(batteryChargeState));
}

void LCD_PrintHighestTemperature()
{
	int16_t highestTemp_C;

	if (fetTemperature_C >= coilTemperature_C)
	highestTemp_C = fetTemperature_C;
	else
	highestTemp_C = coilTemperature_C;

	// Bottom right temperature, no label.
	LCD_PrintTemperatureC(110, DISPLAY_TEMP_Y, highestTemp_C);
}

void LCD_PrintBigSignedFixed1(uint8_t columnX, uint8_t columnWidth, uint8_t y, int32_t value, int32_t scale, char unit)
{
	char buf[7];
	uint8_t len = 0;
	uint16_t tenths;
	uint16_t ip;
	uint8_t fp;
	uint8_t startX;
	uint8_t numberWidth;

	if (value < 0)
	{
		buf[len++] = '-';
		value = -value;
	}

	// One decimal digit with simple rounding.
	tenths = (uint16_t)((value + (scale / 20L)) / (scale / 10L));
	ip = tenths / 10;
	fp = tenths % 10;

	if (ip >= 100)
	buf[len++] = '0' + ((ip / 100) % 10);

	if (ip >= 10)
	buf[len++] = '0' + ((ip / 10) % 10);

	buf[len++] = '0' + (ip % 10);
	buf[len++] = '.';
	buf[len++] = '0' + fp;
	buf[len] = 0;

	// Big number, small unit character.
	numberWidth = len * 12;
	if ((numberWidth + 6) >= columnWidth)
	startX = columnX;
	else
	startX = columnX + ((columnWidth - numberWidth - 6) / 2);

	LCD_BigText(startX, y, buf);
	LCD_Char(startX + numberWidth, y + 8, unit);
}


void LCD_PrintBigIntFixed(uint8_t x, uint8_t y, uint16_t val, uint8_t minDigits)
{
	char buf[5];
	uint8_t i = 0;

	do
	{
		buf[i++] = '0' + (val % 10);
		val /= 10;
	}
	while ((val > 0 || i < minDigits) && i < sizeof(buf));

	while (i > 0)
	{
		i--;
		LCD_BigChar(x, y, buf[i]);
		x += 12;
	}
}

void LCD_BigText(uint8_t x, uint8_t y, const char* txt)
{
	while (*txt)
	{
		LCD_BigChar(x, y, *txt++);
		x += 12;
	}
}

void LCD_BigChar(uint8_t x, uint8_t y, char c)
{
	uint8_t pattern[5];
	uint8_t top[5];
	uint8_t bottom[5];

	LCD_GetCharPattern(c, pattern);

	for (uint8_t col = 0; col < 5; col++)
	{
		top[col] = 0;
		bottom[col] = 0;

		for (uint8_t row = 0; row < 7; row++)
		{
			if (pattern[col] & (1 << row))
			{
				uint8_t bigRow = row * 2;

				if (bigRow < 8)
				top[col] |= (1 << bigRow);
				else
				bottom[col] |= (1 << (bigRow - 8));

				bigRow++;

				if (bigRow < 8)
				top[col] |= (1 << bigRow);
				else
				bottom[col] |= (1 << (bigRow - 8));
			}
		}
	}

	LCD_SetPos(x, y >> 3);
	for (uint8_t col = 0; col < 5; col++)
	{
		LCD_Data(top[col]);
		LCD_Data(top[col]);
	}
	LCD_Data(0x00);
	LCD_Data(0x00);

	LCD_SetPos(x, (y >> 3) + 1);
	for (uint8_t col = 0; col < 5; col++)
	{
		LCD_Data(bottom[col]);
		LCD_Data(bottom[col]);
	}
	LCD_Data(0x00);
	LCD_Data(0x00);
}

void LCD_PrintValueLine_mV(uint8_t x, uint8_t y, const char* label_P, int32_t value_mV, char unit)
{
	LCD_Text_P(x, y, label_P);
	LCD_PrintSignedFixed2(78, y, value_mV, 1000L);
	LCD_Char(114, y, unit);
}

void LCD_PrintValueLine_mA(uint8_t x, uint8_t y, const char* label_P, int32_t value_mA, char unit)
{
	LCD_Text_P(x, y, label_P);
	LCD_PrintSignedFixed2(78, y, value_mA, 1000L);
	LCD_Char(114, y, unit);
}

void LCD_PrintSignedFixed2(uint8_t x, uint8_t y, int32_t value, int32_t scale)
{
	if (value < 0)
	{
		LCD_Char(72, y, '-');
		value = -value;
	}

	uint16_t ip = (uint16_t)(value / scale);
	uint8_t fp = (uint8_t)((value % scale) / (scale / 100L));

	LCD_PrintIntFixed(x, y, ip, 2);
	LCD_Char(x + 12, y, '.');
	LCD_Char(x + 18, y, '0' + (fp / 10));
	LCD_Char(x + 24, y, '0' + (fp % 10));
}

void LCD_PrintTemperatureC(uint8_t x, uint8_t y, int16_t temp_C)
{
	if (temp_C < 0)
	{
		LCD_Char(x, y, '-');
		x += 6;
		temp_C = -temp_C;
	}

	LCD_PrintIntFixed(x, y, (uint16_t)temp_C, 2);
	LCD_Char(x + 12, y, 'C');
}

void LCD_PrintStatusLine()
{
	// Not used by the large symmetric measurement screen.
}

void LCD_PrintIntFixed(uint8_t x, uint8_t y, uint16_t val, uint8_t minDigits)
{
	char buf[5];
	uint8_t i = 0;

	do
	{
		buf[i++] = '0' + (val % 10);
		val /= 10;
	}
	while ((val > 0 || i < minDigits) && i < sizeof(buf));

	while (i > 0)
	{
		i--;
		LCD_Char(x, y, buf[i]);
		x += 6;
	}
}

// ============================================================
// Minimal 5x7 Font
// ============================================================

void LCD_GetCharPattern(char c, uint8_t* p)
{
	for (uint8_t i = 0; i < 5; i++)
	p[i] = 0x00;

	// The internal font table contains uppercase letters only.
	// Convert lowercase display text to uppercase before lookup.
	if ((c >= 'a') && (c <= 'z'))
	c = (char)(c - 32);

	switch (c)
	{
		case ' ': break;

		case '0': p[0]=0x3E; p[1]=0x51; p[2]=0x49; p[3]=0x45; p[4]=0x3E; break;
		case '1': p[0]=0x00; p[1]=0x42; p[2]=0x7F; p[3]=0x40; p[4]=0x00; break;
		case '2': p[0]=0x42; p[1]=0x61; p[2]=0x51; p[3]=0x49; p[4]=0x46; break;
		case '3': p[0]=0x21; p[1]=0x41; p[2]=0x45; p[3]=0x4B; p[4]=0x31; break;
		case '4': p[0]=0x18; p[1]=0x14; p[2]=0x12; p[3]=0x7F; p[4]=0x10; break;
		case '5': p[0]=0x27; p[1]=0x45; p[2]=0x45; p[3]=0x45; p[4]=0x39; break;
		case '6': p[0]=0x3C; p[1]=0x4A; p[2]=0x49; p[3]=0x49; p[4]=0x30; break;
		case '7': p[0]=0x01; p[1]=0x71; p[2]=0x09; p[3]=0x05; p[4]=0x03; break;
		case '8': p[0]=0x36; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x36; break;
		case '9': p[0]=0x06; p[1]=0x49; p[2]=0x49; p[3]=0x29; p[4]=0x1E; break;

		case '.': p[0]=0x00; p[1]=0x60; p[2]=0x60; p[3]=0x00; p[4]=0x00; break;
		case '-': p[0]=0x08; p[1]=0x08; p[2]=0x08; p[3]=0x08; p[4]=0x08; break;
		case '>': p[0]=0x41; p[1]=0x22; p[2]=0x14; p[3]=0x08; p[4]=0x00; break;
		case '*': p[0]=0x14; p[1]=0x08; p[2]=0x3E; p[3]=0x08; p[4]=0x14; break;

		case 'A': p[0]=0x7E; p[1]=0x11; p[2]=0x11; p[3]=0x11; p[4]=0x7E; break;
		case 'B': p[0]=0x7F; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x36; break;
		case 'C': p[0]=0x3E; p[1]=0x41; p[2]=0x41; p[3]=0x41; p[4]=0x22; break;
		case 'D': p[0]=0x7F; p[1]=0x41; p[2]=0x41; p[3]=0x22; p[4]=0x1C; break;
		case 'E': p[0]=0x7F; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x41; break;
		case 'F': p[0]=0x7F; p[1]=0x09; p[2]=0x09; p[3]=0x09; p[4]=0x01; break;
		case 'G': p[0]=0x3E; p[1]=0x41; p[2]=0x49; p[3]=0x49; p[4]=0x7A; break;
		case 'H': p[0]=0x7F; p[1]=0x08; p[2]=0x08; p[3]=0x08; p[4]=0x7F; break;
		case 'I': p[0]=0x00; p[1]=0x41; p[2]=0x7F; p[3]=0x41; p[4]=0x00; break;
		case 'K': p[0]=0x7F; p[1]=0x08; p[2]=0x14; p[3]=0x22; p[4]=0x41; break;
		case 'L': p[0]=0x7F; p[1]=0x40; p[2]=0x40; p[3]=0x40; p[4]=0x40; break;
		case 'M': p[0]=0x7F; p[1]=0x02; p[2]=0x04; p[3]=0x02; p[4]=0x7F; break;
		case 'N': p[0]=0x7F; p[1]=0x04; p[2]=0x08; p[3]=0x10; p[4]=0x7F; break;
		case 'O': p[0]=0x3E; p[1]=0x41; p[2]=0x41; p[3]=0x41; p[4]=0x3E; break;
		case 'P': p[0]=0x7F; p[1]=0x09; p[2]=0x09; p[3]=0x09; p[4]=0x06; break;
		case 'R': p[0]=0x7F; p[1]=0x09; p[2]=0x19; p[3]=0x29; p[4]=0x46; break;
		case 'S': p[0]=0x46; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x31; break;
		case 'T': p[0]=0x01; p[1]=0x01; p[2]=0x7F; p[3]=0x01; p[4]=0x01; break;
		case 'U': p[0]=0x3F; p[1]=0x40; p[2]=0x40; p[3]=0x40; p[4]=0x3F; break;
		case 'V': p[0]=0x1F; p[1]=0x20; p[2]=0x40; p[3]=0x20; p[4]=0x1F; break;
		case 'W': p[0]=0x7F; p[1]=0x20; p[2]=0x18; p[3]=0x20; p[4]=0x7F; break;
		case 'Y': p[0]=0x07; p[1]=0x08; p[2]=0x70; p[3]=0x08; p[4]=0x07; break;
	}
}

// ============================================================
// ADS1015 Low Level
// ============================================================

bool ADS1015_WriteRegister(uint8_t reg, uint16_t value)
{
	Wire.beginTransmission(ADS1015_ADDR);
	Wire.write(reg);
	Wire.write((uint8_t)(value >> 8));
	Wire.write((uint8_t)(value & 0xFF));
	return (Wire.endTransmission() == 0);
}

bool ADS1015_WriteConfig(uint16_t config)
{
	return ADS1015_WriteRegister(ADS1015_REG_CONFIG, config);
}

bool ADS1015_ReadConversion(int16_t* value)
{
	Wire.beginTransmission(ADS1015_ADDR);
	Wire.write(ADS1015_REG_CONVERSION);

	if (Wire.endTransmission(false) != 0)
	return false;

	if (Wire.requestFrom(ADS1015_ADDR, (uint8_t)2) != 2)
	return false;

	uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
	*value = ((int16_t)raw) >> 4;

	return true;
}

bool ADS1015_ReadDiff(uint16_t mux, int16_t* raw)
{
	uint16_t config = 0;

	config |= ADS1015_OS_SINGLE;
	config |= mux;
	config |= ADS1015_PGA_6V144;
	config |= ADS1015_MODE_SINGLE;
	config |= ADS1015_DR_1600SPS;
	config |= ADS1015_COMP_DISABLE;

	if (!ADS1015_WriteConfig(config))
	return false;

	delayMicroseconds(1000);

	return ADS1015_ReadConversion(raw);
}

void ADS1015_ReadTemperatures()
{
	int16_t raw1 = 0;
	int16_t raw2 = 0;

	if (ADS1015_ReadDiff(ADS1015_MUX_A0_A1, &raw1))
	fetTemperature_C = NTC_RawToTempC(raw1);

	if (ADS1015_ReadDiff(ADS1015_MUX_A2_A3, &raw2))
	coilTemperature_C = NTC_RawToTempC(raw2);
}

int16_t NTC_RawToTempC(int16_t raw)
{
	int16_t t1, t2;
	int16_t r1, r2;

	if (raw >= pgm_read_word(&ntcTable[0].raw))
	return pgm_read_word(&ntcTable[0].temp_C);

	for (uint8_t i = 0; i < NTC_TABLE_COUNT - 1; i++)
	{
		t1 = pgm_read_word(&ntcTable[i].temp_C);
		r1 = pgm_read_word(&ntcTable[i].raw);
		t2 = pgm_read_word(&ntcTable[i + 1].temp_C);
		r2 = pgm_read_word(&ntcTable[i + 1].raw);

		if ((raw <= r1) && (raw >= r2))
		return t1 + ((int32_t)(r1 - raw) * (t2 - t1)) / (r1 - r2);
	}

	return pgm_read_word(&ntcTable[NTC_TABLE_COUNT - 1].temp_C);
}

// ============================================================
// Fan Temperature Task, ALERT used as fan open-drain output
// ============================================================

void FanTemp_Task(uint32_t now_ms)
{
	if ((uint32_t)(now_ms - lastFanCheck_ms) < FAN_CHECK_INTERVAL_MS)
	return;

	lastFanCheck_ms = now_ms;
	ADS1015_ReadTemperatures();

	// The fan has two jobs: cool the internal coil and move air across the external FET heatsink.
	if (dcdcState != DCDC_STATE_OFF || stateCharge)
		fanState = true;
	else if ((fetTemperature_C <= FAN_AFTER_RUN_OFF_C) && (coilTemperature_C <= FAN_AFTER_RUN_OFF_C))
		fanState = false;
	else
		fanState = true;

	// ADS1015 ALERT is used as the open-drain fan output.
	if (fanState)
	ADS1015_AlertForceOn();
	else
	ADS1015_AlertForceOff();
}

void ADS1015_AlertForceOn()
{
	uint16_t config = 0;

	// Force comparator active. ALERT active LOW.
	ADS1015_WriteRegister(ADS1015_REG_LO_THRESH, 0x8000);
	ADS1015_WriteRegister(ADS1015_REG_HI_THRESH, 0x0000);

	config |= ADS1015_MUX_A0_A1;
	config |= ADS1015_PGA_6V144;
	config |= ADS1015_MODE_CONTINUOUS;
	config |= ADS1015_DR_1600SPS;
	config |= ADS1015_COMP_MODE_TRAD;
	config |= ADS1015_COMP_POL_LOW;
	config |= ADS1015_COMP_LAT_NON;
	config |= ADS1015_COMP_QUE_1;

	ADS1015_WriteConfig(config);
}

void ADS1015_AlertForceOff()
{
	uint16_t config = 0;

	config |= ADS1015_MUX_A0_A1;
	config |= ADS1015_PGA_6V144;
	config |= ADS1015_MODE_SINGLE;
	config |= ADS1015_DR_1600SPS;
	config |= ADS1015_COMP_DISABLE;

	ADS1015_WriteConfig(config);
}


// ============================================================
// DCDC State Machine
// ============================================================
//
// Simple charge start and hold sequence:
// 1. Select an available input source.
// 2. Set DCDC equal to the battery voltage.
//    This avoids a large voltage step when SW-Charge is switched on.
// 3. Enable DCDC.
// 4. Switch PD6 / SW-Charge ON. PD6 is HIGH active.
// 5. Slowly lower the DAC code. This raises the DCDC output voltage.
// 6. Stop the voltage ramp at battery voltage + CHARGE_MAX_OFFSET_mV.
// 7. Control the ramp by the battery INA current.
// 8. Hold the active charge current target with a small hysteresis.
// 9. If the input source is removed, switch charge and DCDC OFF immediately.
//
// Current direction:
// INA current is positive from IN+ pin 10 to IN- pin 9.
// The charge current is measured with INA_BATTERY.
// INA_DCDC current is shown only for debug.

const char* DCDC_StateText(DCDC_State state)
{
	switch (state)
	{
		case DCDC_STATE_OFF:          return "OFF";
		case DCDC_STATE_START:        return "START";
		case DCDC_STATE_CHARGE_ON:    return "CHGON";
		case DCDC_STATE_RAMP_CURRENT: return "IRAMP";
		case DCDC_STATE_READY:        return "READY";
	}

	return "?";
}

bool InputSource_Available()
{
	if (stateHighPower)
	return (ina238[CH_HIGH_POWER].online && ina238[CH_HIGH_POWER].voltage_mV >= HIGH_POWER_ON_mV);

	if (stateMidPower)
	return (ina238[CH_MID_POWER].online && ina238[CH_MID_POWER].voltage_mV >= MID_POWER_ON_mV);

	if (stateLowPower)
	return (ina238[CH_LOW_POWER].online && ina238[CH_LOW_POWER].voltage_mV >= LOW_POWER_ON_mV);

	return false;
}

void DCDC_SetState(DCDC_State newState)
{
	if (dcdcState == newState)
	return;

	DBG_PRINT(F("DCDC state "));
	DBG_PRINT(DCDC_StateText(dcdcState));
	DBG_PRINT(F(" -> "));
	DBG_PRINTLN(DCDC_StateText(newState));

	dcdcState = newState;
}

void DCDC_AllOff()
{
	// A removed source clears a latched pre-charge fault for the next connection attempt.
	dcdcPreChargeTestFault = false;
	Charge_Off();
	DCDC_Disable();
	DAC_WriteCode(DCDC_SAFE_START_CODE);
	DCDC_ResetCurrentControl();
	DCDC_PreChargeTest_Reset();
	inputLimitedChargeCurrentSet_mA = 0L;
	inputLimitSource = INPUT_SOURCE_NONE;
	DCDC_SetState(DCDC_STATE_OFF);
}

void ChargeCurrent_Task(uint32_t now_ms)
{
	uint8_t source = INPUT_SOURCE_NONE;

	if (stateHighPower)
	source = INPUT_SOURCE_HIGH;
	else if (stateMidPower)
	source = INPUT_SOURCE_MID;
	else if (stateLowPower)
	source = INPUT_SOURCE_LOW;

	if (source == INPUT_SOURCE_NONE)
	{
		activeChargeCurrentSet_mA = 0L;
		activeChargeCurrentMax_mA = 0L;
		inputLimitedChargeCurrentSet_mA = 0L;
		inputLimitSource = INPUT_SOURCE_NONE;
		return;
	}

	activeChargeCurrentMax_mA = BATTERY_MAX_CHARGE_CURRENT_mA;
	activeChargeCurrentSet_mA = InputCurrentLimit_Task(now_ms, source, BATTERY_MAX_CHARGE_CURRENT_mA);
}

int32_t InputCurrentLimit_Task(uint32_t now_ms, uint8_t source, int32_t requestedSet_mA)
{
	int32_t inputCurrent_mA;
	int32_t inputCurrentMax_mA;

	if (inputLimitSource != source)
	{
		inputLimitSource = source;
		inputLimitedChargeCurrentSet_mA = requestedSet_mA;
		lastInputCurrentRegulation_ms = now_ms;
		return inputLimitedChargeCurrentSet_mA;
	}

	if (inputLimitedChargeCurrentSet_mA > requestedSet_mA)
	inputLimitedChargeCurrentSet_mA = requestedSet_mA;

	if ((uint32_t)(now_ms - lastInputCurrentRegulation_ms) < INPUT_CURRENT_REG_INTERVAL_MS)
	return inputLimitedChargeCurrentSet_mA;

	lastInputCurrentRegulation_ms = now_ms;
	inputCurrent_mA = InputCurrent_Get_mA(source);
	inputCurrentMax_mA = InputCurrent_GetMax_mA(source);

	// Input current is too high: reduce charge current target quickly.
	if (inputCurrent_mA > (inputCurrentMax_mA + INPUT_CURRENT_HYST_mA))
	{
		if (inputLimitedChargeCurrentSet_mA > INPUT_CURRENT_FAST_DOWN_mA)
		inputLimitedChargeCurrentSet_mA -= INPUT_CURRENT_FAST_DOWN_mA;
		else
		inputLimitedChargeCurrentSet_mA = 0L;

		return inputLimitedChargeCurrentSet_mA;
	}

	// Input current is near the limit: reduce charge current target gently.
	if (inputCurrent_mA > inputCurrentMax_mA)
	{
		if (inputLimitedChargeCurrentSet_mA > INPUT_CURRENT_STEP_DOWN_mA)
		inputLimitedChargeCurrentSet_mA -= INPUT_CURRENT_STEP_DOWN_mA;
		else
		inputLimitedChargeCurrentSet_mA = 0L;

		return inputLimitedChargeCurrentSet_mA;
	}

	// Input current has enough margin again: increase charge current target slowly up to requested value.
	if (inputCurrent_mA < (inputCurrentMax_mA - INPUT_CURRENT_HYST_mA))
	{
		if (inputLimitedChargeCurrentSet_mA < (requestedSet_mA - INPUT_CURRENT_STEP_UP_mA))
		inputLimitedChargeCurrentSet_mA += INPUT_CURRENT_STEP_UP_mA;
		else
		inputLimitedChargeCurrentSet_mA = requestedSet_mA;
	}

	return inputLimitedChargeCurrentSet_mA;
}

int32_t InputCurrent_Get_mA(uint8_t source)
{
	if (source == INPUT_SOURCE_HIGH)
	return ina238[CH_HIGH_POWER].current_mA;

	if (source == INPUT_SOURCE_MID)
	return ina238[CH_MID_POWER].current_mA;

	if (source == INPUT_SOURCE_LOW)
	return ina238[CH_LOW_POWER].current_mA;

	return 0L;
}

int32_t InputCurrent_GetMax_mA(uint8_t source)
{
	if (source == INPUT_SOURCE_HIGH)
	return HIGH_POWER_INPUT_MAX_mA;

	if (source == INPUT_SOURCE_MID)
	return MID_POWER_INPUT_MAX_mA;

	if (source == INPUT_SOURCE_LOW)
	return LOW_POWER_INPUT_MAX_mA;

	return 0L;
}

int32_t ChargeCurrent_GetSet_mA()
{
	int32_t target_mV;
	int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;
	int32_t limited_mA = activeChargeCurrentSet_mA;
	int32_t powerLimited_mA = DCDC_GetPowerLimitedCurrent_mA();

	if (powerLimited_mA < limited_mA)
	limited_mA = powerLimited_mA;

	target_mV = (batteryChargeState == BATTERY_CHARGE_FLOAT) ? BATTERY_FLOAT_mV : BATTERY_ABSORPTION_mV;

	if (battery_mV >= target_mV)
	return 0L;

	if (battery_mV > (target_mV - BATTERY_VOLTAGE_TAPER_mV))
	{
		limited_mA = (limited_mA * (target_mV - battery_mV)) / BATTERY_VOLTAGE_TAPER_mV;
		if (limited_mA < 0L) limited_mA = 0L;
	}

	return limited_mA;
}

int32_t ChargeCurrent_GetMax_mA()
{
	int32_t max_mA = BATTERY_MAX_CHARGE_CURRENT_mA;
	int32_t powerLimited_mA = DCDC_GetPowerLimitedCurrent_mA();
	if (powerLimited_mA < max_mA) max_mA = powerLimited_mA;
	return max_mA;
}

void BatteryCharge_Task()
{
	uint32_t now_ms = millis();
	int32_t battery_mV;
	int32_t batteryCurrent_mA;

	if (!ina238[CH_BATTERY].online)
	return;

	battery_mV = ina238[CH_BATTERY].voltage_mV;
	batteryCurrent_mA = chargeCurrentFiltered_mA;

	if (forceFullChargeRequested)
	{
		forceFullChargeRequested = false;
		batteryChargeState = BATTERY_CHARGE_BULK;
		absorptionStart_ms = 0UL;
		fullRestStart_ms = 0UL;
		fullRestEvaluated = false;
		DBG_PRINTLN(F("STATUS: FORCE_FULL_CHARGE"));
	}

	if (batteryChargeState == BATTERY_CHARGE_WAIT)
	{
		if (battery_mV <= BATTERY_RESTART_mV)
		batteryChargeState = BATTERY_CHARGE_BULK;
		else if (ENABLE_FLOAT)
		batteryChargeState = BATTERY_CHARGE_FLOAT;
		else
		{
			batteryChargeState = BATTERY_CHARGE_FULL;
			fullRestStart_ms = now_ms;
			fullRestEvaluated = false;
		}
		return;
	}

	if (batteryChargeState == BATTERY_CHARGE_BULK)
	{
		if (battery_mV >= BATTERY_ABSORPTION_mV)
		{
			batteryChargeState = BATTERY_CHARGE_ABS;
			absorptionStart_ms = now_ms;
		}
		return;
	}

	if (batteryChargeState == BATTERY_CHARGE_ABS)
	{
		uint32_t absTime_ms = (absorptionStart_ms == 0UL) ? 0UL : (uint32_t)(now_ms - absorptionStart_ms);

		if (absTime_ms >= ABS_MAX_TIME_MS)
		{
			DBG_PRINTLN(F("WARNING: ABSORPTION_MAX_TIME - battery or external load may prevent end-current detection."));
			batteryChargeState = ENABLE_FLOAT ? BATTERY_CHARGE_FLOAT : BATTERY_CHARGE_FULL;
			fullRestStart_ms = now_ms;
			fullRestEvaluated = false;
			return;
		}

		if ((absTime_ms >= ABS_MIN_TIME_MS) && stateCharge && chargeCurrentFilterValid &&
			(battery_mV >= (BATTERY_ABSORPTION_mV - BATTERY_ABSORB_TOLERANCE_mV)) &&
			(batteryCurrent_mA >= 0L) && (batteryCurrent_mA < BATTERY_FULL_CURRENT_mA))
		{
			batteryChargeState = ENABLE_FLOAT ? BATTERY_CHARGE_FLOAT : BATTERY_CHARGE_FULL;
			fullRestStart_ms = now_ms;
			fullRestEvaluated = false;
		}
		return;
	}

	if (batteryChargeState == BATTERY_CHARGE_FULL)
	{
		if (ENABLE_FLOAT)
		{
			batteryChargeState = BATTERY_CHARGE_FLOAT;
			return;
		}

		if (fullRestStart_ms == 0UL)
		fullRestStart_ms = now_ms;

		if (!fullRestEvaluated)
		{
			if ((uint32_t)(now_ms - fullRestStart_ms) < BATTERY_REST_TIME_MS)
			return;

			fullRestEvaluated = true;

			if (battery_mV <= BATTERY_RESTART_mV)
			{
				DBG_PRINTLN(F("WARNING: BATTERY_NOT_HOLDING - battery condition or external load should be checked."));
				batteryChargeState = BATTERY_CHARGE_BULK;
				absorptionStart_ms = 0UL;
				fullRestStart_ms = 0UL;
				fullRestEvaluated = false;
			}
			return;
		}

		// After the initial 60-second rest test, a later drop is a normal restart request.
		if (battery_mV <= BATTERY_RESTART_mV)
		{
			batteryChargeState = BATTERY_CHARGE_BULK;
			absorptionStart_ms = 0UL;
			fullRestStart_ms = 0UL;
			fullRestEvaluated = false;
		}
		return;
	}

	// FLOAT remains active while a source is available. Voltage taper limits the current at BATTERY_FLOAT_mV.
}

bool BatteryCharge_IsAllowed()
{
	uint32_t now_ms = millis();

	if (thermalShutdown)
	return false;

	if (batteryVoltageFault)
	return false;

	if (dcdcPreChargeTestFault)
	return false;

	if ((int32_t)(now_ms - dcdcReverseBlockedUntil_ms) < 0)
	return false;

	return (batteryChargeState == BATTERY_CHARGE_BULK ||
		batteryChargeState == BATTERY_CHARGE_ABS ||
		batteryChargeState == BATTERY_CHARGE_FLOAT);
}

int32_t DCDC_GetAllowedPower_W()
{
	int16_t hottest_C = (fetTemperature_C > coilTemperature_C) ? fetTemperature_C : coilTemperature_C;

	if (hottest_C >= DCDC_SHUTDOWN_TEMP_C)
	return 0L;

	if (hottest_C >= 80)
	return DCDC_DERATING_80C_POWER_W;

	if (hottest_C >= 75)
	return DCDC_DERATING_75C_POWER_W - ((int32_t)(hottest_C - 75) * (DCDC_DERATING_75C_POWER_W - DCDC_DERATING_80C_POWER_W) / 5L);

	if (hottest_C >= 70)
	return DCDC_DERATING_70C_POWER_W - ((int32_t)(hottest_C - 70) * (DCDC_DERATING_70C_POWER_W - DCDC_DERATING_75C_POWER_W) / 5L);

	if (hottest_C >= DCDC_DERATING_START_C)
	return DCDC_MAX_POWER_W - ((int32_t)(hottest_C - DCDC_DERATING_START_C) * (DCDC_MAX_POWER_W - DCDC_DERATING_70C_POWER_W) / 5L);

	return DCDC_MAX_POWER_W;
}

int32_t DCDC_GetPowerLimitedCurrent_mA()
{
	int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;
	int32_t allowedPower_W = DCDC_GetAllowedPower_W();

	if (battery_mV <= 0L || allowedPower_W <= 0L)
	return 0L;

	return (int32_t)(((int64_t)allowedPower_W * 1000000LL) / battery_mV);
}

void BatteryVoltageProtection_Task()
{
	if (!ina238[CH_BATTERY].online)
	return;

	int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;
	bool outOfRange = (battery_mV < BATTERY_MIN_VOLTAGE_mV) || (battery_mV > BATTERY_MAX_VOLTAGE_mV);

	if (outOfRange && !batteryVoltageFault)
	{
		batteryVoltageFault = true;
		DBG_PRINT(F("ERROR: BATTERY_VOLTAGE_OUT_OF_RANGE - "));
		DBG_PRINT(battery_mV);
		DBG_PRINTLN(F("mV; charging stopped."));
	}
	else if (!outOfRange && batteryVoltageFault)
	{
		batteryVoltageFault = false;
		DBG_PRINTLN(F("STATUS: BATTERY_VOLTAGE_OK - charging may restart."));
	}
}

void TemperatureProtection_Task()
{
	if (!thermalShutdown)
	{
		if ((fetTemperature_C >= DCDC_SHUTDOWN_TEMP_C) || (coilTemperature_C >= DCDC_SHUTDOWN_TEMP_C))
		{
			thermalShutdown = true;
			DBG_PRINTLN(F("ERROR: DCDC_OVERTEMPERATURE - charging stopped, fan remains active."));
		}
	}
	else
	{
		if ((fetTemperature_C <= DCDC_RESTART_TEMP_C) && (coilTemperature_C <= DCDC_RESTART_TEMP_C))
		{
			thermalShutdown = false;
			DBG_PRINTLN(F("STATUS: DCDC_TEMPERATURE_OK - charging may restart."));
		}
	}
}

void DCDC_PreChargeTest_Reset()
{
	dcdcPreChargeTestState = DCDC_TEST_IDLE;
	dcdcPreChargeTestPhaseStart_ms = 0UL;
	dcdcPreChargeTestBaseVoltage_mV = 0L;
	dcdcPreChargeTestStableCounter = 0U;
	DCDC_ReverseCurrentCheck_Reset();
}

void DCDC_PreChargeTest_Fail()
{
	dcdcPreChargeTestState = DCDC_TEST_FAILED;
	dcdcPreChargeTestFault = true;
	Charge_Off();
	DCDC_Disable();
	DAC_WriteCode(DCDC_SAFE_START_CODE);
	DCDC_ResetCurrentControl();
	DCDC_SetState(DCDC_STATE_OFF);
	DBG_PRINTLN(F("ERROR: DCDC_PRECHARGE_FAILED - converter disconnected from battery."));
}

bool DCDC_PreChargeTest_AdjustToVoltage(int32_t target_mV)
{
	int32_t dcdc_mV;
	if (!ina238[CH_DCDC].online) return false;
	dcdc_mV = ina238[CH_DCDC].voltage_mV;

	if (dcdc_mV < (target_mV - DCDC_TEST_MATCH_TOL_mV))
	{
		dcdcPreChargeTestStableCounter = 0U;
		if (dacCode > DCDC_TEST_DAC_STEP_CODE) DAC_WriteCode(dacCode - DCDC_TEST_DAC_STEP_CODE);
		else DAC_WriteCode(DAC_CODE_MIN);
		return false;
	}

	if (dcdc_mV > (target_mV + DCDC_TEST_MATCH_TOL_mV))
	{
		dcdcPreChargeTestStableCounter = 0U;
		if (dacCode < (DAC_CODE_MAX - DCDC_TEST_DAC_STEP_CODE)) DAC_WriteCode(dacCode + DCDC_TEST_DAC_STEP_CODE);
		else DAC_WriteCode(DAC_CODE_MAX);
		return false;
	}

	return true;
}

bool DCDC_PreChargeTest_Task(uint32_t now_ms)
{
	int32_t battery_mV;
	int32_t dcdc_mV;
	int32_t testTarget_mV;

	if (!ina238[CH_BATTERY].online || !ina238[CH_DCDC].online)
	{
		DCDC_PreChargeTest_Fail();
		return false;
	}

	battery_mV = ina238[CH_BATTERY].voltage_mV;
	dcdc_mV = ina238[CH_DCDC].voltage_mV;

	if ((uint32_t)(now_ms - dcdcDelayStart_ms) >= CHARGE_ON_DELAY_MS && dcdc_mV < DCDC_TEST_MIN_OUTPUT_mV)
	{
		DCDC_PreChargeTest_Fail();
		return false;
	}

	if ((uint32_t)(now_ms - dcdcPreChargeTestPhaseStart_ms) > DCDC_TEST_PHASE_TIMEOUT_MS)
	{
		DCDC_PreChargeTest_Fail();
		return false;
	}

	if (dcdcPreChargeTestState == DCDC_TEST_MATCH_BATTERY)
	{
		if (!DCDC_PreChargeTest_AdjustToVoltage(battery_mV)) return false;
		if (dcdcPreChargeTestStableCounter < DCDC_TEST_STABLE_COUNT)
		{
			dcdcPreChargeTestStableCounter++;
			return false;
		}
		dcdcPreChargeTestBaseVoltage_mV = dcdc_mV;
		dcdcPreChargeTestStableCounter = 0U;
		dcdcPreChargeTestPhaseStart_ms = now_ms;
		dcdcPreChargeTestState = DCDC_TEST_RAISE_OUTPUT;
		return false;
	}

	if (dcdcPreChargeTestState == DCDC_TEST_RAISE_OUTPUT)
	{
		testTarget_mV = battery_mV + DCDC_TEST_RAISE_OFFSET_mV;
		if (testTarget_mV > DCDC_VOUT_MAX_mV) testTarget_mV = DCDC_VOUT_MAX_mV;
		if (!DCDC_PreChargeTest_AdjustToVoltage(testTarget_mV)) return false;
		if ((dcdc_mV - dcdcPreChargeTestBaseVoltage_mV) < DCDC_TEST_RISE_MIN_mV)
		{
			DCDC_PreChargeTest_Fail();
			return false;
		}
		if (dcdcPreChargeTestStableCounter < DCDC_TEST_STABLE_COUNT)
		{
			dcdcPreChargeTestStableCounter++;
			return false;
		}
		testTarget_mV = battery_mV + DCDC_CONNECT_OFFSET_mV;
		if (testTarget_mV > DCDC_VOUT_MAX_mV) testTarget_mV = DCDC_VOUT_MAX_mV;
		DAC_WriteCode(DCDC_CalcDacCodeForVout_mV((uint16_t)testTarget_mV));
		dcdcPreChargeTestPhaseStart_ms = now_ms;
		dcdcPreChargeTestState = DCDC_TEST_CONNECT_LEVEL;
		return false;
	}

	if (dcdcPreChargeTestState == DCDC_TEST_CONNECT_LEVEL)
	{
		if ((uint32_t)(now_ms - dcdcPreChargeTestPhaseStart_ms) < DCDC_CONNECT_SETTLE_MS) return false;
		dcdcPreChargeTestState = DCDC_TEST_PASSED;
		dcdcPreChargeTestFault = false;
		return true;
	}

	return (dcdcPreChargeTestState == DCDC_TEST_PASSED);
}

void DCDC_ReverseCurrentCheck_Reset()
{
	dcdcReverseCurrentTiming = false;
	dcdcReverseCurrentStart_ms = 0UL;
}

bool DCDC_ReverseCurrentCheck_Task(uint32_t now_ms)
{
	if (!ina238[CH_BATTERY].online)
	{
		DCDC_PreChargeTest_Fail();
		return false;
	}

	if (ina238[CH_BATTERY].current_mA < DCDC_REVERSE_CURRENT_LIMIT_mA)
	{
		if (!dcdcReverseCurrentTiming)
		{
			dcdcReverseCurrentTiming = true;
			dcdcReverseCurrentStart_ms = now_ms;
		}
		else if ((uint32_t)(now_ms - dcdcReverseCurrentStart_ms) >= DCDC_REVERSE_CURRENT_TIME_MS)
		{
			Charge_Off();
			DCDC_Disable();
			DAC_WriteCode(DCDC_SAFE_START_CODE);
			DCDC_ResetCurrentControl();
			DCDC_PreChargeTest_Reset();
			DCDC_SetState(DCDC_STATE_OFF);
			dcdcReverseBlockedUntil_ms = now_ms + DCDC_REVERSE_RETRY_MS;
			DBG_PRINTLN(F("STATUS: DCDC_REVERSE_CURRENT - another charger is charging at the higher voltage; local charger disconnected."));
			return false;
		}
	}
	else
	DCDC_ReverseCurrentCheck_Reset();

	return true;
}

void DCDC_Task(uint32_t now_ms)
{
	if ((uint32_t)(now_ms - lastDcdcTask_ms) < DCDC_TASK_INTERVAL_MS)
	return;

	lastDcdcTask_ms = now_ms;
	ChargeCurrent_Task(now_ms);
	BatteryCharge_Task();

	// Source removed: disconnect the battery and switch off DCDC.
	if (!InputSource_Available())
	{
		DCDC_AllOff();
		return;
	}

	// Battery full or not low enough: keep the input selected, but keep DCDC and Charge OFF.
	if (!BatteryCharge_IsAllowed())
	{
		Charge_Off();
		DCDC_Disable();
		DCDC_ResetCurrentControl();
		DCDC_SetState(DCDC_STATE_OFF);
		return;
	}

	if (dcdcState == DCDC_STATE_OFF)
	{
		Charge_Off();
		DCDC_Disable();
		DCDC_ResetCurrentControl();
		DCDC_PreChargeTest_Reset();

		// Start exactly at battery voltage, not below and not at lowest DCDC voltage.
		if (DCDC_SetStartVoltageFromBattery())
		{
			dcdcDelayStart_ms = now_ms;
			DCDC_SetState(DCDC_STATE_START);
		}

		return;
	}

	if (dcdcState == DCDC_STATE_START)
	{
		// Wait after the input source switch is ON before DCDC is enabled.
		if ((uint32_t)(now_ms - dcdcDelayStart_ms) < DCDC_ON_DELAY_MS)
		return;

		DCDC_Enable();
		lastControl_ms = now_ms;
		dcdcDelayStart_ms = now_ms;
		dcdcPreChargeTestPhaseStart_ms = now_ms;
		dcdcPreChargeTestState = DCDC_TEST_MATCH_BATTERY;
		dcdcPreChargeTestStableCounter = 0U;
		DCDC_SetState(DCDC_STATE_CHARGE_ON);
		return;
	}

	if (dcdcState == DCDC_STATE_CHARGE_ON)
	{
		Charge_Off();

		if (!DCDC_PreChargeTest_Task(now_ms))
		return;

		Charge_On();
		chargePositiveStart_ms = now_ms;
		DCDC_ReverseCurrentCheck_Reset();
		DCDC_SetState(DCDC_STATE_RAMP_CURRENT);
		return;
	}

	if (dcdcState == DCDC_STATE_RAMP_CURRENT)
	{
		if (!DCDC_ReverseCurrentCheck_Task(now_ms))
		return;

		if (DCDC_ControlOneStepToCurrent(now_ms))
		DCDC_SetState(DCDC_STATE_READY);

		return;
	}

	if (dcdcState == DCDC_STATE_READY)
	{
		Charge_On();

		// Keep the current near the target.
		DCDC_ControlOneStepToCurrent(now_ms);
	}
}

bool DCDC_ControlOneStepToCurrent(uint32_t now_ms)
{
	int32_t iBatRaw_mA;
	uint16_t minCode;

	if (!ina238[CH_BATTERY].online)
	return false;

	if ((uint32_t)(now_ms - lastControl_ms) < CHARGE_RAMP_INTERVAL_MS)
	return false;

	lastControl_ms = now_ms;
	iBatRaw_mA = ina238[CH_BATTERY].current_mA;

	// Simple integer low pass filter:
	// First valid sample loads the filter. Later samples are averaged slowly.
	if (!chargeCurrentFilterValid)
	{
		chargeCurrentFiltered_mA = iBatRaw_mA;
		chargeCurrentFilterValid = true;
	}
	else
	chargeCurrentFiltered_mA += (iBatRaw_mA - chargeCurrentFiltered_mA) / CHARGE_CURRENT_FILTER_DIV;

	// Lower DAC code means higher DCDC output voltage and more charge current.
	// Never raise DCDC more than CHARGE_MAX_OFFSET_mV above the actual battery voltage.
	minCode = DCDC_CalcDacCodeForVout_mV((uint16_t)DCDC_GetMaxTargetFromBattery_mV());

	// Absolute safety correction.
	// This parameter is user adjustable, but the software reacts immediately here.
	if ((iBatRaw_mA > ChargeCurrent_GetMax_mA()) || (chargeCurrentFiltered_mA > ChargeCurrent_GetMax_mA()))
	{
		if (dacCode < (DAC_CODE_MAX - CHARGE_FAST_STEP_CODE))
		DAC_WriteCode(dacCode + CHARGE_FAST_STEP_CODE);
		else
		DAC_WriteCode(DAC_CODE_MAX);

		chargeStableCounter = 0;
		return false;
	}

	// Fast safety correction for large overcurrent peaks.
	if (iBatRaw_mA > (ChargeCurrent_GetSet_mA() + CHARGE_OVERCURRENT_mA))
	{
		if (dacCode < (DAC_CODE_MAX - CHARGE_FAST_STEP_CODE))
		DAC_WriteCode(dacCode + CHARGE_FAST_STEP_CODE);
		else
		DAC_WriteCode(DAC_CODE_MAX);

		chargeStableCounter = 0;
		return false;
	}

	// Positive start phase after SW-Charge ON.
	// Capacitors may cause a temporary negative current reading.
	// During this time, the control only searches toward positive charge current.
	if ((uint32_t)(now_ms - chargePositiveStart_ms) < CHARGE_POSITIVE_START_MS)
	{
		if (chargeCurrentFiltered_mA < (ChargeCurrent_GetSet_mA() - CHARGE_CURRENT_HYST_mA))
		{
			if (dacCode > (minCode + CHARGE_RAMP_STEP_CODE))
			DAC_WriteCode(dacCode - CHARGE_RAMP_STEP_CODE);
			else
			DAC_WriteCode(minCode);
		}

		chargeStableCounter = 0;
		return false;
	}

	// Slow upward ramp when current is too low.
	if (chargeCurrentFiltered_mA < (ChargeCurrent_GetSet_mA() - CHARGE_CURRENT_HYST_mA))
	{
		if (dacCode > (minCode + CHARGE_RAMP_STEP_CODE))
		DAC_WriteCode(dacCode - CHARGE_RAMP_STEP_CODE);
		else
		DAC_WriteCode(minCode);

		chargeStableCounter = 0;
		return false;
	}

	// Moderate downward correction when filtered current is too high.
	if (chargeCurrentFiltered_mA > (ChargeCurrent_GetSet_mA() + CHARGE_CURRENT_HYST_mA))
	{
		if (dacCode < DAC_CODE_MAX)
		DAC_WriteCode(dacCode + CHARGE_RAMP_STEP_CODE);

		chargeStableCounter = 0;
		return false;
	}

	// Current is inside the dead band. READY is accepted only after several stable samples.
	if (chargeStableCounter < CHARGE_STABLE_COUNT)
	chargeStableCounter++;

	return (chargeStableCounter >= CHARGE_STABLE_COUNT);
}

void DCDC_ResetCurrentControl()
{
	chargeCurrentFiltered_mA = 0;
	chargeCurrentFilterValid = false;
	chargeStableCounter = 0;
	chargePositiveStart_ms = 0;
}

bool DCDC_SetStartVoltageFromBattery()
{
	int32_t target_mV;

	if (!ina238[CH_BATTERY].online)
	return false;

	target_mV = DCDC_GetStartTargetFromBattery_mV();

	if (target_mV < DCDC_VOUT_MIN_mV)
	target_mV = DCDC_VOUT_MIN_mV;

	if (target_mV > DCDC_VOUT_MAX_mV)
	target_mV = DCDC_VOUT_MAX_mV;

	return DAC_WriteCode(DCDC_CalcDacCodeForVout_mV((uint16_t)target_mV));
}

int32_t DCDC_GetStartTargetFromBattery_mV()
{
	return ina238[CH_BATTERY].voltage_mV;
}

int32_t DCDC_GetMaxTargetFromBattery_mV()
{
	int32_t target_mV;

	target_mV = ina238[CH_BATTERY].voltage_mV + CHARGE_MAX_OFFSET_mV;

	if (target_mV < DCDC_VOUT_MIN_mV)
	target_mV = DCDC_VOUT_MIN_mV;

	int32_t activeTarget_mV = (batteryChargeState == BATTERY_CHARGE_FLOAT) ? BATTERY_FLOAT_mV : BATTERY_ABSORPTION_mV;
	if (target_mV > activeTarget_mV)
	target_mV = activeTarget_mV;

	if (target_mV > DCDC_VOUT_MAX_mV)
	target_mV = DCDC_VOUT_MAX_mV;

	return target_mV;
}

int32_t DCDC_GetTargetFromBattery_mV()
{
	return DCDC_GetMaxTargetFromBattery_mV();
}

// ============================================================
// Input Source Auto Switch
// ============================================================
//
// Source switch outputs:
// LOW  = ON
// HIGH = OFF
//
// Charge switch output PD6:
// HIGH = ON
// LOW  = OFF
//
// Priority:
// 1. HIGH_POWER
// 2. MID_POWER
// 3. LOW_POWER

void InputSource_Task()
{
	bool highAvailable;
	bool midAvailable;
	bool lowAvailable;
	uint8_t wantedSource;
	uint32_t now_ms;

	highAvailable  = (ina238[0].voltage_mV >= HIGH_POWER_ON_mV);
	midAvailable = (ina238[1].voltage_mV >= MID_POWER_ON_mV);
	lowAvailable  = (ina238[4].voltage_mV >= LOW_POWER_ON_mV);

	wantedSource = INPUT_SOURCE_NONE;

	if (highAvailable)
	wantedSource = INPUT_SOURCE_HIGH;
	else if (midAvailable)
	wantedSource = INPUT_SOURCE_MID;
	else if (lowAvailable)
	wantedSource = INPUT_SOURCE_LOW;

	if (wantedSource == INPUT_SOURCE_NONE)
	{
		inputSourcePending = INPUT_SOURCE_NONE;
		HighPower_Off();
		MidPower_Off();
		LowPower_Off();
		return;
	}

	// Keep the already selected source ON.
	if ((wantedSource == INPUT_SOURCE_HIGH && stateHighPower) ||
		(wantedSource == INPUT_SOURCE_MID && stateMidPower) ||
		(wantedSource == INPUT_SOURCE_LOW  && stateLowPower))
	{
		inputSourcePending = INPUT_SOURCE_NONE;
		return;
	}

	now_ms = millis();

	// A new source was detected. Keep all source switches OFF during the delay.
	if (inputSourcePending != wantedSource)
	{
		inputSourcePending = wantedSource;
		sourceDelayStart_ms = now_ms;
		HighPower_Off();
		MidPower_Off();
		LowPower_Off();
		return;
	}

	// Wait before the selected source switch is closed.
	if ((uint32_t)(now_ms - sourceDelayStart_ms) < SOURCE_ON_DELAY_MS)
	return;

	if (wantedSource == INPUT_SOURCE_HIGH)
	{
		HighPower_On();
		MidPower_Off();
		LowPower_Off();
		return;
	}

	if (wantedSource == INPUT_SOURCE_MID)
	{
		HighPower_Off();
		MidPower_On();
		LowPower_Off();
		return;
	}

	HighPower_Off();
	MidPower_Off();
	LowPower_On();
}

