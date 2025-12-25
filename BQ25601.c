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
        data->pfmEnable = ((data->reg[1] >> 7) & 0x1) == 0;
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

    return true;
}

bool BQ25601_Write(struct BQ25601_Platform *platform, struct BQ25601_Data *data, int reg) {}
