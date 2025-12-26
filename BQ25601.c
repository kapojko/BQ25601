#include "BQ25601.h"

#define BQ25601_I2C_TIMEOUT 1

static const char *BQ25601_SystemMinimumVoltage_str[] = {
    "2.6V", // BQ25601_SYS_MIN_2_6V = 0x0,
    "2.8V", // BQ25601_SYS_MIN_2_8V = 0x1,
    "3.0V", // BQ25601_SYS_MIN_3_0V = 0x2,
    "3.2V", // BQ25601_SYS_MIN_3_2V = 0x3,
    "3.4V", // BQ25601_SYS_MIN_3_4V = 0x4,
    "3.5V", // BQ25601_SYS_MIN_3_5V = 0x5,
    "3.6V", // BQ25601_SYS_MIN_3_6V = 0x6,
    "3.7V"  // BQ25601_SYS_MIN_3_7V = 0x7
};

static const char *BQ25601_MinimumBatteryVoltageForOTG_str[] = {
    "2.8V Fall", // BQ25601_MIN_VBAT_SEL_2_8V_FALL = 0,
    "2.5V Fall", // BQ25601_MIN_VBAT_SEL_2_5V_FALL = 1,
};

static const char *BQ25601_BoostLimit_str[] = {
    "0.5A", // BQ25601_BOOST_LIM_0_5A = 0,
    "1.2A", // BQ25601_BOOST_LIM_1_2A = 1,
};

static const char *BQ25601_TopOffTimer_str[] = {
    "Disabled", // BQ25601_TOPOFF_TIMER_DISABLED = 0x0,
    "15min", // BQ25601_TOPOFF_TIMER_15MIN = 0x1,
    "30min", // BQ25601_TOPOFF_TIMER_30MIN = 0x2,
    "45min", // BQ25601_TOPOFF_TIMER_45MIN = 0x3,
};

static const char *BQ25601_RechargeThreshold_str[] = {
    "100mV", // BQ25601_VRECHG_100MV = 0,
    "200mV", // BQ25601_VRECHG_200MV = 1,
};

static const char *BQ25601_WatchdogTimer_str[] = {
    "Disabled", // BQ25601_WATCHDOG_DISABLED = 0x0,
    "40s", // BQ25601_WATCHDOG_40S = 0x1,
    "80s", // BQ25601_WATCHDOG_80S = 0x2,
    "160s", // BQ25601_WATCHDOG_160S = 0x3,
};

static const char *BQ25601_ChargeTimer_str[] = {
    "5hrs", // BQ25601_CHG_TIMER_5HRS = 0,
    "10hrs", // BQ25601_CHG_TIMER_10HRS = 1,
};

static const char *BQ25601_ThermalRegulationThreshold_str[] = {
    "90C", // BQ25601_TREG_90C = 0,
    "110C", // BQ25601_TREG_110C = 1,
};

static const char *BQ25601_JEITA_ISET_str[] = {
    "50% Ichg", // BQ25601_JEITA_ISET_50PCT_ICHG = 0,
    "20% Ichg", // BQ25601_JEITA_ISET_20PCT_ICHG = 1,
};

static const char *BQ25601_VAC_OVP_Threshold_str[] = {
    "5.5V", // BQ25601_OVP_5_5V = 0,
    "6.5V", // BQ25601_OVP_6_5V = 1,
    "10.5V", // BQ25601_OVP_10_5V = 2,
    "14V", // BQ25601_OVP_14V = 3,
};

static const char *BQ25601_BoostRegulationVoltage_str[] = {
    "4.85V", // BQ25601_BOOSTV_4_85V = 0,
    "5.00V", // BQ25601_BOOSTV_5_00V = 1,
    "5.15V", // BQ25601_BOOSTV_5_15V = 2,
    "5.30V", // BQ25601_BOOSTV_5_30V = 3,
};

static const char *BQ25601_JEITA_VSET_str[] = {
    "4.1V", // BQ25601_JEITA_VSET_4_1V = 0,
    "Vreg", // BQ25601_JEITA_VSET_VREG = 1,
};

static const char *BQ25601_VindpmTrackBatteryVoltage_str[] = {
    "Disable", // BQ25601_VDPM_BAT_TRACK_DISABLE = 0x0,
    "VBAT+200mV", // BQ25601_VDPM_BAT_TRACK_VBAT_PLUS_200MV = 0x1,
    "VBAT+250mV", // BQ25601_VDPM_BAT_TRACK_VBAT_PLUS_250MV = 0x2,
    "VBAT+300mV", // BQ25601_VDPM_BAT_TRACK_VBAT_PLUS_300MV = 0x3,
};

static const char *BQ25601_VBusStatus_str[] = {
    "No Input", // BQ25601_VBUS_STAT_NO_INPUT = 0x0,
    "USB Host SDP 500mA", // BQ25601_VBUS_STAT_USB_HOST_SDP_500MA = 0x1,
    "Adapter 2.4A", // BQ25601_VBUS_STAT_ADAPTER_2_4A = 0x2,
    "OTG", // BQ25601_VBUS_STAT_OTG = 0x3,
};

static const char *BQ25601_ChargingStatus_str[] = {
    "Not Charging", // BQ25601_CHRG_STAT_NOT_CHARGING = 0x0,
    "Pre-Charge", // BQ25601_CHRG_STAT_PRE_CHARGE = 0x1,
    "Fast Charging", // BQ25601_CHRG_STAT_FAST_CHARGING = 0x2,
    "Charge Termination", // BQ25601_CHRG_STAT_CHARGE_TERMINATION = 0x3,
};

static const char *BQ25601_ChargeFault_str[] = {
    "Normal", // BQ25601_CHRG_FAULT_NORMAL = 0x0,
    "Input Fault", // BQ25601_CHRG_FAULT_INPUT_FAULT = 0x1,
    "Thermal Shutdown", // BQ25601_CHRG_FAULT_THERMAL_SHUTDOWN = 0x2,
    "Safety Timer Expiration", // BQ25601_CHRG_FAULT_SAFETY_TIMER_EXPIRATION = 0x3,
};

static const char *BQ25601_NTC_Fault_str[] = {
    "Normal", // BQ25601_NTC_FAULT_NORMAL = 0x0,
    "-", // 0x1
    "Warm", // BQ25601_NTC_FAULT_WARM = 0x2,
    "Cool", // BQ25601_NTC_FAULT_COOL = 0x3,
    "-", // 0x4
    "Cold", // BQ25601_NTC_FAULT_COLD = 0x5,
    "Hot", // BQ25601_NTC_FAULT_HOT = 0x6,
    "-", // 0x7
};

bool BQ25601_Read(struct BQ25601_Platform *platform, struct BQ25601_Data *data, int reg) {
    int ret;

    // Read register data
    if (reg == BQ25601_ALL_REGS) {
        ret = platform->i2cReadReg(BQ25601_I2C_ADDRESS, 0, data->reg, BQ25601_REG_NUMBER,
                                   BQ25601_I2C_TIMEOUT);
    } else {
        ret =
            platform->i2cReadReg(BQ25601_I2C_ADDRESS, reg, &data->reg[reg], 1, BQ25601_I2C_TIMEOUT);
    }

    if (ret < 0) {
        return false;
    }

    // Decode register data

    // REG00
    if (reg == 0 || reg == BQ25601_ALL_REGS) {
        data->hizEnable = ((data->reg[0] >> 7) & 0x1) == 1;
        data->statPinEnable = ((data->reg[0] >> 5) & 0x3) == 0;
        data->inputCurrentLimit_mA = (data->reg[0] & 0x1F) * 100 + 100;
    }

    // REG01
    if (reg == 1 || reg == BQ25601_ALL_REGS) {
        data->pfmDisable = ((data->reg[1] >> 7) & 0x1) == 1;
        data->watchdogTimerReset = ((data->reg[1] >> 6) & 0x1) == 1;
        data->otgEnable = ((data->reg[1] >> 5) & 0x1) == 1;
        data->chargeEnable = ((data->reg[1] >> 4) & 0x1) == 1;
        data->systemMinimumVoltage = (enum BQ25601_SystemMinimumVoltage)((data->reg[1] >> 1) & 0x7);
        data->minimumBatteryVoltageForOTG =
            (enum BQ25601_MinimumBatteryVoltageForOTG)(data->reg[1] & 0x1);
    }

    // REG02
    if (reg == 2 || reg == BQ25601_ALL_REGS) {
        data->boostLimit = (enum BQ25601_BoostLimit)((data->reg[2] >> 7) & 0x1);
        data->q1FullOn = ((data->reg[2] >> 6) & 0x1) == 1;
        data->fastChargeCurrent_mA = (data->reg[2] & 0x3F) * 60;
    }

    // REG03
    if (reg == 3 || reg == BQ25601_ALL_REGS) {
        data->prechargeCurrent_mA = ((data->reg[3] >> 4) & 0xF) * 60 + 60;
        data->terminationCurrent_mA = (data->reg[3] & 0xF) * 60 + 60;
    }

    // REG04
    if (reg == 4 || reg == BQ25601_ALL_REGS) {
        data->chargeVoltage_mV = ((data->reg[4] >> 3) & 0x1F) * 32 + 3856;
        data->topOffTimer = (enum BQ25601_TopOffTimer)((data->reg[4] >> 1) & 0x3);
        data->rechargeThreshold = (enum BQ25601_RechargeThreshold)(data->reg[4] & 0x1);
    }

    // REG05
    if (reg == 5 || reg == BQ25601_ALL_REGS) {
        data->terminationEnable = ((data->reg[5] >> 7) & 0x1) == 1;
        data->watchdogTimer = (enum BQ25601_WatchdogTimer)((data->reg[5] >> 4) & 0x3);
        data->chargeTimerEnable = ((data->reg[5] >> 3) & 0x1) == 1;
        data->chargeTimer = (enum BQ25601_ChargeTimer)((data->reg[5] >> 2) & 0x1);
        data->thermalRegulationThreshold =
            (enum BQ25601_ThermalRegulationThreshold)((data->reg[5] >> 1) & 0x1);
        data->jeitaIset = (enum BQ25601_JEITA_ISET)(data->reg[5] & 0x1);
    }

    // REG06
    if (reg == 6 || reg == BQ25601_ALL_REGS) {
        data->vacOvpThreshold = (enum BQ25601_VAC_OVP_Threshold)((data->reg[6] >> 6) & 0x3);
        data->boostRegulationVoltage =
            (enum BQ25601_BoostRegulationVoltage)((data->reg[6] >> 4) & 0x3);
        data->absoluteVindpmThreshold_mV = (data->reg[6] & 0xF) * 100 + 3900;
    }

    // REG07
    if (reg == 7 || reg == BQ25601_ALL_REGS) {
        data->inputCurrentLimitDetection = ((data->reg[7] >> 7) & 0x1) == 1;
        data->safetyTimerSlowed2X = ((data->reg[7] >> 6) & 0x1) == 1;
        data->q4TurnOff = ((data->reg[7] >> 5) & 0x1) == 1;
        data->jeitaVset = (enum BQ25601_JEITA_VSET)((data->reg[7] >> 4) & 0x1);
        data->batfetDelay = ((data->reg[7] >> 3) & 0x1) == 1;
        data->batfetResetEnable = ((data->reg[7] >> 2) & 0x1) == 1;
        data->vindpmTrackBatteryVoltage =
            (enum BQ25601_VindpmTrackBatteryVoltage)(data->reg[7] & 0x3);
    }

    // REG08
    if (reg == 8 || reg == BQ25601_ALL_REGS) {
        data->vBusStatus = (enum BQ25601_VBusStatus)((data->reg[8] >> 5) & 0x7);
        data->chargingStatus = (enum BQ25601_ChargingStatus)((data->reg[8] >> 3) & 0x3);
        data->powerGood = ((data->reg[8] >> 2) & 0x1) == 1;
        data->thermalRegulationStatus = ((data->reg[8] >> 1) & 0x1) == 1;
        data->vsysMinRegulationStatus = (data->reg[8] & 0x1) == 1;
    }

    // REG09
    if (reg == 9 || reg == BQ25601_ALL_REGS) {
        data->watchdogFault = ((data->reg[9] >> 7) & 0x1) == 1;
        data->boostFault = ((data->reg[9] >> 6) & 0x1) == 1;
        data->chargeFault = (enum BQ25601_ChargeFault)((data->reg[9] >> 4) & 0x3);
        data->batteryFault = ((data->reg[9] >> 3) & 0x1) == 1;
        data->ntcFault = (enum BQ25601_NTC_Fault)(data->reg[9] & 0x7);
    }

    // REG0A
    if (reg == 10 || reg == BQ25601_ALL_REGS) {
        data->vbusAttached = ((data->reg[10] >> 7) & 0x1) == 1;
        data->vindpmStatus = ((data->reg[10] >> 6) & 0x1) == 1;
        data->iindpmStatus = ((data->reg[10] >> 5) & 0x1) == 1;
        data->topOffActive = ((data->reg[10] >> 3) & 0x1) == 1;
        data->acovStatus = ((data->reg[10] >> 2) & 0x1) == 1;
        data->vindpmIntMask = ((data->reg[10] >> 1) & 0x1) == 1;
        data->iindpmIntMask = (data->reg[10] & 0x1) == 1;
    }

    // REG0B
    if (reg == 11 || reg == BQ25601_ALL_REGS) {
        data->reset = ((data->reg[11] >> 7) & 0x1) == 1;
        data->productNumber = ((data->reg[11] >> 3) & 0xF);
        data->deviceRevision = (data->reg[11] & 0x3);
    }

    return true;
}

bool BQ25601_Write(struct BQ25601_Platform *platform, struct BQ25601_Data *data, int reg) {
    // Encode register data

    // REG00
    if (reg == 0 || reg == BQ25601_ALL_REGS) {
        data->reg[0] = 0;
        data->reg[0] |= ((data->hizEnable ? 1 : 0) << 7);
        data->reg[0] |= ((data->statPinEnable ? 0x0 : 0x3) << 5);

        uint8_t iindpm = (data->inputCurrentLimit_mA - 100) / 100;
        data->reg[0] |= (iindpm & 0x1F);
    }

    // REG01
    if (reg == 1 || reg == BQ25601_ALL_REGS) {
        data->reg[1] = 0;
        data->reg[1] |= ((data->pfmDisable ? 1 : 0) << 7);
        data->reg[1] |= ((data->watchdogTimerReset ? 1 : 0) << 6);
        data->reg[1] |= ((data->otgEnable ? 1 : 0) << 5);
        data->reg[1] |= ((data->chargeEnable ? 1 : 0) << 4);
        data->reg[1] |= ((data->systemMinimumVoltage & 0x7) << 1);
        data->reg[1] |= ((data->minimumBatteryVoltageForOTG & 0x1) << 0);
    }

    // REG02
    if (reg == 2 || reg == BQ25601_ALL_REGS) {
        data->reg[2] = 0;
        data->reg[2] |= ((data->boostLimit & 0x1) << 7);
        data->reg[2] |= ((data->q1FullOn ? 1 : 0) << 6);

        uint8_t ichg = (data->fastChargeCurrent_mA) / 60;
        data->reg[2] |= (ichg & 0x3F);
    }

    // REG03
    if (reg == 3 || reg == BQ25601_ALL_REGS) {
        data->reg[3] = 0;

        uint8_t iprechg = (data->prechargeCurrent_mA - 60) / 60;
        data->reg[3] |= ((iprechg & 0xF) << 4);

        uint8_t iterm = (data->terminationCurrent_mA - 60) / 60;
        data->reg[3] |= ((iterm & 0xF) << 0);
    }

    // REG04
    if (reg == 4 || reg == BQ25601_ALL_REGS) {
        data->reg[4] = 0;

        uint8_t vreg = (data->chargeVoltage_mV - 3856) / 32;
        data->reg[4] |= ((vreg & 0x1F) << 3);

        data->reg[4] |= ((data->topOffTimer & 0x3) << 1);
        data->reg[4] |= ((data->rechargeThreshold & 0x1) << 0);
    }

    // REG05
    if (reg == 5 || reg == BQ25601_ALL_REGS) {
        data->reg[5] = 0;
        data->reg[5] |= ((data->terminationEnable ? 1 : 0) << 7);
        data->reg[5] |= ((data->watchdogTimer & 0x3) << 4);
        data->reg[5] |= ((data->chargeTimerEnable ? 1 : 0) << 3);
        data->reg[5] |= ((data->chargeTimer & 0x1) << 2);
        data->reg[5] |= ((data->thermalRegulationThreshold & 0x1) << 1);
        data->reg[5] |= ((data->jeitaIset & 0x1) << 0);
    }

    // REG06
    if (reg == 6 || reg == BQ25601_ALL_REGS) {
        data->reg[6] = 0;
        data->reg[6] |= ((data->vacOvpThreshold & 0x3) << 6);
        data->reg[6] |= ((data->boostRegulationVoltage & 0x3) << 4);

        uint8_t vindpm = (data->absoluteVindpmThreshold_mV - 3900) / 100;
        data->reg[6] |= ((vindpm & 0xF) << 0);
    }

    // REG07
    if (reg == 7 || reg == BQ25601_ALL_REGS) {
        data->reg[7] = 0;
        data->reg[7] |= ((data->inputCurrentLimitDetection ? 1 : 0) << 7);
        data->reg[7] |= ((data->safetyTimerSlowed2X ? 1 : 0) << 6);
        data->reg[7] |= ((data->q4TurnOff ? 1 : 0) << 5);
        data->reg[7] |= ((data->jeitaVset & 0x1) << 4);
        data->reg[7] |= ((data->batfetDelay ? 1 : 0) << 3);
        data->reg[7] |= ((data->batfetResetEnable ? 1 : 0) << 2);
        data->reg[7] |= ((data->vindpmTrackBatteryVoltage & 0x3) << 0);
    }

    // REG08
    if (reg == 8 || reg == BQ25601_ALL_REGS) {
        // NOTE: all R
    }

    // REG09
    if (reg == 9 || reg == BQ25601_ALL_REGS) {
        // NOTE: all R
    }

    // REG0A
    if (reg == 10 || reg == BQ25601_ALL_REGS) {
        // NOTE: only interrupt masks are R/W
        data->reg[10] &= ~(0x3);
        data->reg[10] |= ((data->vindpmIntMask & 0x1) << 1);
        data->reg[10] |= ((data->iindpmIntMask & 0x1) << 0);
    }

    // REG0B
    if (reg == 11 || reg == BQ25601_ALL_REGS) {
        // NOTE: only reset is R/W
        data->reg[11] &= ~(0x1 << 7);
        data->reg[11] |= ((data->reset & 0x1) << 7);
    }

    // Write register data
    int ret;
    if (reg == BQ25601_ALL_REGS) {
        ret = platform->i2cWriteReg(BQ25601_I2C_ADDRESS, 0, data->reg, BQ25601_REG_NUMBER,
                                    BQ25601_I2C_TIMEOUT);
    } else {
        ret = platform->i2cWriteReg(BQ25601_I2C_ADDRESS, reg, &data->reg[reg], 1,
                                    BQ25601_I2C_TIMEOUT);
    }

    if (ret < 0) {
        return false;
    }

    return true;
}

bool BQ25601_Reset(struct BQ25601_Platform *platform) {
    uint8_t reg = 0x0B;
    uint8_t data = (0x1 << 7);
    int ret = platform->i2cWriteReg(BQ25601_I2C_ADDRESS, reg, &data, 1, BQ25601_I2C_TIMEOUT);
    return ret >= 0;
}

void BQ25601_DebugPrint(struct BQ25601_Platform *platform, struct BQ25601_Data *data, int reg) {
    if (reg == 0 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG00: EN_HIZ=%d EN_STAT_Pin=%d In_Cur_Limit=%dmA\r\n",
            data->hizEnable,
            data->statPinEnable,
            data->inputCurrentLimit_mA
        );
    }

    if (reg == 1 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG01: PFM_DIS=%d WD_RST=%d OTG_CONFIG=%d CHG_CONFIG=%d SYS_MIN=%s MIN_Vbat_SEL=%s\r\n",
            data->pfmDisable,
            data->watchdogTimerReset,
            data->otgEnable,
            data->chargeEnable,
            BQ25601_SystemMinimumVoltage_str[data->systemMinimumVoltage],
            BQ25601_MinimumBatteryVoltageForOTG_str[data->minimumBatteryVoltageForOTG]
        );
    }

    if (reg == 2 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG02: BOOST_LIM=%s Q1_FULLON=%d Fast_Chg_Cur=%dmA\r\n",
            BQ25601_BoostLimit_str[data->boostLimit],
            data->q1FullOn,
            data->fastChargeCurrent_mA
        );
    }

    if (reg == 3 || reg == BQ25601_ALL_REGS) {
         platform->debugPrint("REG03: Prechg_Cur=%dmA Term_Cur=%dmA\r\n",
            data->prechargeCurrent_mA,
            data->terminationCurrent_mA
        );
    }

    if (reg == 4 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG04: Chr_Volt=%dmV TOPOFF_TIMER=%s Rech_Thr=%s\r\n",
            data->chargeVoltage_mV,
            BQ25601_TopOffTimer_str[data->topOffTimer],
            BQ25601_RechargeThreshold_str[data->rechargeThreshold]
        );
    }

    if (reg == 5 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG05: EN_TERM=%d WATCHDOG=%s EN_TIMER=%d CHG_TIMER=%s TREG=%s JEITA_ISET=%s\r\n",
            data->terminationEnable,
            BQ25601_WatchdogTimer_str[data->watchdogTimer],
            data->chargeTimerEnable,
            BQ25601_ChargeTimer_str[data->chargeTimer],
            BQ25601_ThermalRegulationThreshold_str[data->thermalRegulationThreshold],
            BQ25601_JEITA_ISET_str[data->jeitaIset]
        );
    }

    if (reg == 6 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG06: VAC_OVP_Thr=%s Boost_Volt=%s Abs_VINDPM_Thr=%dmV\r\n",
            BQ25601_VAC_OVP_Threshold_str[data->vacOvpThreshold],
            BQ25601_BoostRegulationVoltage_str[data->boostRegulationVoltage],
            data->absoluteVindpmThreshold_mV
        );
    }

    if (reg == 7 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG07: IINDET_EN=%d TMR2X_EN=%d BATFET_DIS=%d JEITA_VSET=%s BATFET_DLY=%d BATFET_RST_EN=%d VDPM_BAT_TRACK=%s\r\n",
            data->inputCurrentLimitDetection,
            data->safetyTimerSlowed2X,
            data->q4TurnOff,
            BQ25601_JEITA_VSET_str[data->jeitaVset],
            data->batfetDelay,
            data->batfetResetEnable,
            BQ25601_VindpmTrackBatteryVoltage_str[data->vindpmTrackBatteryVoltage]
        );
    }

    if (reg == 8 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG08: VBUS=%s CHRG=%s PG=%d THERM=%d VSYS=%d\r\n",
            BQ25601_VBusStatus_str[data->vBusStatus],
            BQ25601_ChargingStatus_str[data->chargingStatus],
            data->powerGood,
            data->thermalRegulationStatus,
            data->vsysMinRegulationStatus
        );
    }

    if (reg == 9 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG09: WATCHDOG_FAULT=%d BOOST_FAULT=%d CHRG_FAULT=%s BAT_FAULT=%d NTC_FAULT=%s\r\n",
            data->watchdogFault,
            data->boostFault,
            BQ25601_ChargeFault_str[data->chargeFault],
            data->batteryFault,
            BQ25601_NTC_Fault_str[data->ntcFault]
        );
    }

    if (reg == 10 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG0A: VBUS_GD=%d VINDPM=%d IINDPM=%d TOPOFF_ACTIVE=%d ACOV_STAT=%d VINDPM_INT_MASK=%d IINDPM_INT_MASK=%d\r\n",
            data->vbusAttached,
            data->vindpmStatus,
            data->iindpmStatus,
            data->topOffActive,
            data->acovStatus,
            data->vindpmIntMask,
            data->iindpmIntMask
        );
    }

    if (reg == 11 || reg == BQ25601_ALL_REGS) {
        platform->debugPrint("REG0B: REG_RST=%d PN=%x DEV_REV=%x\r\n",
            data->reset,
            data->productNumber,
            data->deviceRevision
        );
    }
}
