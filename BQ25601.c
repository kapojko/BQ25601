#include "BQ25601.h"

#define BQ25601_I2C_TIMEOUT 1

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
