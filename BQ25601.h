#ifndef BQ25601_H
#define BQ25601_H

#include <stdbool.h>
#include <stdint.h>

#define BQ25601_I2C_ADDRESS 0x6B

#define BQ25601_PRODUCT_NUMBER 0x2

#define BQ25601_REG_NUMBER 12
#define BQ25601_ALL_REGS (-1)

enum BQ25601_SystemMinimumVoltage {
    BQ25601_SYS_MIN_2_6V = 0x0,
    BQ25601_SYS_MIN_2_8V = 0x1,
    BQ25601_SYS_MIN_3_0V = 0x2,
    BQ25601_SYS_MIN_3_2V = 0x3,
    BQ25601_SYS_MIN_3_4V = 0x4,
    BQ25601_SYS_MIN_3_5V = 0x5,
    BQ25601_SYS_MIN_3_6V = 0x6,
    BQ25601_SYS_MIN_3_7V = 0x7
};

// Minimum battery voltage for OTG mode.
enum BQ25601_MinimumBatteryVoltageForOTG {
    BQ25601_MIN_VBAT_SEL_2_8V_FALL = 0,
    BQ25601_MIN_VBAT_SEL_2_5V_FALL = 1,
};

enum BQ25601_BoostLimit {
    BQ25601_BOOST_LIM_0_5A = 0,
    BQ25601_BOOST_LIM_1_2A = 1,
};

enum BQ25601_TopOffTimer {
    BQ25601_TOPOFF_TIMER_DISABLED = 0x0,
    BQ25601_TOPOFF_TIMER_15MIN = 0x1,
    BQ25601_TOPOFF_TIMER_30MIN = 0x2,
    BQ25601_TOPOFF_TIMER_45MIN = 0x3,
};

enum BQ25601_RechargeThreshold {
    BQ25601_VRECHG_100MV = 0,
    BQ25601_VRECHG_200MV = 1,
};

enum BQ25601_WatchdogTimer {
    BQ25601_WATCHDOG_DISABLED = 0x0,
    BQ25601_WATCHDOG_40S = 0x1,
    BQ25601_WATCHDOG_80S = 0x2,
    BQ25601_WATCHDOG_160S = 0x3,
};

enum BQ25601_ChargeTimer {
    BQ25601_CHG_TIMER_5HRS = 0,
    BQ25601_CHG_TIMER_10HRS = 1,
};

enum BQ25601_ThermalRegulationThreshold {
    BQ25601_TREG_90C = 0,
    BQ25601_TREG_110C = 1,
};

enum BQ25601_JEITA_ISET {
    BQ25601_JEITA_ISET_50PCT_ICHG = 0,
    BQ25601_JEITA_ISET_20PCT_ICHG = 1,
};

enum BQ25601_VAC_OVP_Threshold {
    BQ25601_OVP_5_5V = 0x0,
    BQ25601_OVP_6_5V = 0x1, // 5-V input
    BQ25601_OVP_10_5V = 0x2, // 9-V input
    BQ25601_OVP_14V = 0x3, // 12-V input
};

enum BQ25601_BoostRegulationVoltage {
    BQ25601_BOOSTV_4_85V = 0x0,
    BQ25601_BOOSTV_5_00V = 0x1,
    BQ25601_BOOSTV_5_15V = 0x2,
    BQ25601_BOOSTV_5_30V = 0x3,
};

// 45C-60C
enum BQ25601_JEITA_VSET {
    BQ25601_JEITA_VSET_4_1V = 0,
    BQ25601_JEITA_VSET_VREG = 1,
};

enum BQ25601_VindpmTrackBatteryVoltage {
    BQ25601_VDPM_BAT_TRACK_DISABLE = 0x0,
    BQ25601_VDPM_BAT_TRACK_VBAT_PLUS_200MV = 0x1,
    BQ25601_VDPM_BAT_TRACK_VBAT_PLUS_250MV = 0x2,
    BQ25601_VDPM_BAT_TRACK_VBAT_PLUS_300MV = 0x3,
};

enum BQ25601_VBusStatus {
    BQ25601_VBUS_STAT_NO_INPUT = 0x0,
    BQ25601_VBUS_STAT_USB_HOST_SDP_500MA = 0x1,
    BQ25601_VBUS_STAT_ADAPTER_2_4A = 0x2,
    BQ25601_VBUS_STAT_OTG = 0x3,
};

enum BQ25601_ChargingStatus {
    BQ25601_CHRG_STAT_NOT_CHARGING = 0x0,
    BQ25601_CHRG_STAT_PRE_CHARGE = 0x1,
    BQ25601_CHRG_STAT_FAST_CHARGING = 0x2,
    BQ25601_CHRG_STAT_CHARGE_TERMINATION = 0x3,
};

enum BQ25601_ChargeFault {
    BQ25601_CHRG_FAULT_NORMAL = 0x0,
    BQ25601_CHRG_FAULT_INPUT_FAULT = 0x1,
    BQ25601_CHRG_FAULT_THERMAL_SHUTDOWN = 0x2,
    BQ25601_CHRG_FAULT_SAFETY_TIMER_EXPIRATION= 0x3,
};

enum BQ25601_NTC_Fault {
    BQ25601_NTC_FAULT_NORMAL = 0x0,
    BQ25601_NTC_FAULT_WARM = 0x2,
    BQ25601_NTC_FAULT_COOL = 0x3,
    BQ25601_NTC_FAULT_COLD = 0x5,
    BQ25601_NTC_FAULT_HOT = 0x6,
};

struct BQ25601_Platform {
    int (*i2cWriteReg)(uint8_t addr7bit, uint8_t regNum, const uint8_t *data, uint8_t length, uint8_t wait);
    int (*i2cReadReg)(uint8_t addr7bit, uint8_t regNum, uint8_t *data, uint8_t length, int timeout);
    void (*debugPrint)(const char *fmt, ...);
};

struct BQ25601_Data {
    uint8_t reg[BQ25601_REG_NUMBER];

    // REG00, all R/W
    bool hizEnable; // Enable HIZ Mode
    bool statPinEnable; // Enable STAT Pin function
    uint16_t inputCurrentLimit_mA; // Input Current Limit

    // REG01, all R/W
    bool pfmDisable; // Disable PFM
    bool watchdogTimerReset; // I2C Watchdog Timer Reset, true - reset
    bool otgEnable; // Enable OTG
    bool chargeEnable; // Enable Charge
    enum BQ25601_SystemMinimumVoltage systemMinimumVoltage; // System Minimum Voltage
    enum BQ25601_MinimumBatteryVoltageForOTG minimumBatteryVoltageForOTG; // Minimum battery voltage for OTG mode

    // REG02, all R/W
    enum BQ25601_BoostLimit boostLimit; // Boost current limit
    bool q1FullOn; // Use lower Q1 RDSON always (better efficiency), except boost mode
    uint16_t fastChargeCurrent_mA; // Fast Charge Current

    // REG03, all R/W
    uint16_t prechargeCurrent_mA; // Precharge Current
    uint16_t terminationCurrent_mA; // Termination Current

    // REG04, all R/W
    uint16_t chargeVoltage_mV; // Charge Voltage
    enum BQ25601_TopOffTimer topOffTimer; // The extended time following the termination condition is met
    enum BQ25601_RechargeThreshold rechargeThreshold; // Recharge Threshold

    // REG05, all R/W
    bool terminationEnable; // Enable termination
    enum BQ25601_WatchdogTimer watchdogTimer; // Watchdog Timer
    bool chargeTimerEnable; // Enable both fast charge and precharge timer
    enum BQ25601_ChargeTimer chargeTimer; // Charge Timer
    enum BQ25601_ThermalRegulationThreshold thermalRegulationThreshold; // Thermal Regulation Threshold
    enum BQ25601_JEITA_ISET jeitaIset; // JEITA ISET

    // REG06, all R/W
    enum BQ25601_VAC_OVP_Threshold vacOvpThreshold; // VAC OVP Threshold
    enum BQ25601_BoostRegulationVoltage boostRegulationVoltage; // Boost Regulation Voltage
    uint16_t absoluteVindpmThreshold_mV; // Absolute VINDPM Threshold

    // REG07, all R/W
    bool inputCurrentLimitDetection; // Force input current limit detection when VBUS is present
    bool safetyTimerSlowed2X; // Safety timer slowed by 2X during input DPM (both V and I) or JEITA cool, or thermal regulation
    bool q4TurnOff; // Turn off Q4 with tBATFET_DLY delay time
    enum BQ25601_JEITA_VSET jeitaVset; // JEITA VSET
    bool batfetDelay; // Turn off BATFET after tBATFET_DLY (typ. 10 s)
    bool batfetResetEnable; // Enable BATFET reset function
    enum BQ25601_VindpmTrackBatteryVoltage vindpmTrackBatteryVoltage; // Sets VINDPM to track BAT voltage

    // REG08, all R
    enum BQ25601_VBusStatus vBusStatus; // VBUS Status register
    enum BQ25601_ChargingStatus chargingStatus; // Charging status
    bool powerGood; // Power Good Status
    bool thermalRegulationStatus; // Thermal Regulation Status
    bool vsysMinRegulationStatus; // In VSYS_MIN regulation (BAT < VSYS_MIN)

    // REG09, all R
    bool watchdogFault; // Watchdog timer expiration
    bool boostFault; // any conditions that cannot start boost function
    enum BQ25601_ChargeFault chargeFault; // Charge fault status
    bool batteryFault; // Battery fault status
    enum BQ25601_NTC_Fault ntcFault; // JEITA

    // REG0A, all R except masks
    bool vbusAttached; // VBUS Attached
    bool vindpmStatus; // in VINDPM
    bool iindpmStatus; // in IINDPM
    bool topOffActive; // Top off timer counting
    bool acovStatus; // Device is in ACOV
    bool vindpmIntMask; // Mask VINDPM INT pulse
    bool iindpmIntMask; // Mask IINDPM INT pulse

    // REG0B, all R except reset
    bool reset; // Reset to default register value and reset safety timer
    uint8_t productNumber; // Product Number, BQ25601_PRODUCT_NUMBER
    uint8_t deviceRevision; // Device Revision
};

bool BQ25601_Read(struct BQ25601_Platform *platform, struct BQ25601_Data *data, int reg);
bool BQ25601_Write(struct BQ25601_Platform *platform, struct BQ25601_Data *data, int reg);

bool BQ25601_Reset(struct BQ25601_Platform *platform);

#endif // BQ25601_H
