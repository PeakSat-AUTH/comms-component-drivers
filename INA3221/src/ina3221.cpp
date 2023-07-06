#include "ina3221.hpp"

namespace INA3221 {

    uint16_t voltageConversion(int32_t v, uint16_t scale, uint16_t shift) {
        return (v < 0) ?
               (~(std::abs(v) / scale << shift) + 1 & 0x7FFF) | 0x8000 :
               (v / scale << shift);
    }

    void INA3221::wait(uint32_t msec) {
        HAL_Delay(msec);
    }

    etl::expected<void, Error> INA3221::i2cWrite(Register address, uint16_t value) {
        uint8_t buffer[] = {static_cast<uint8_t>(address),
                            static_cast<uint8_t>(value >> 8),
                            static_cast<uint8_t>(value & 0x00FF)};

        if (HAL_I2C_Master_Transmit(&hi2c, i2cSlaveAddress << 1, buffer, 3, 1000) != HAL_OK) {
            return {etl::unexpected(Error::I2C_FAILURE)};
        }

        return {};
    }

    etl::expected<uint16_t, Error> INA3221::i2cRead(Register address) {
        uint8_t buffer[2];
        auto regAddress = static_cast<uint8_t>(address);

        if (HAL_I2C_Master_Transmit(&hi2c, i2cSlaveAddress << 1, &regAddress, 1, 1000) != HAL_OK) {
            return etl::expected<uint16_t, Error>(etl::unexpected(Error::I2C_FAILURE));
        }

        if (HAL_I2C_Master_Receive(&hi2c, i2cSlaveAddress << 1, buffer, 2, 1000) != HAL_OK) {
            return etl::expected<uint16_t, Error>(etl::unexpected(Error::I2C_FAILURE));
        }

        uint16_t received = (static_cast<uint16_t>(buffer[0]) << 8) | static_cast<uint16_t>(buffer[1]);
        return { received };
    }

    etl::expected<uint16_t, Error> INA3221::getConfigRegister() {
        return i2cRead(Register::CONFG);
    }


    etl::expected<void, Error>
    INA3221::writeRegisterField(Register address, uint16_t value, uint16_t mask, uint16_t shift) {
        auto reg = i2cRead(address);
        if (!reg.has_value()) {
            return { etl::unexpected(reg.error()) };
        }

        uint16_t val = (reg.value() & ~mask) | (value << shift);
        return i2cWrite(address, val);
    }

    etl::expected<void, Error> INA3221::changeOperatingMode(OperatingMode operatingMode) {
        return writeRegisterField(Register::CONFG, (uint16_t) operatingMode, 0x7, 0);
    }

    etl::pair<ChannelMeasurement, ChannelMeasurement> INA3221::getMeasurement() {
        ChannelMeasurement busMeasurement = etl::exchange(busVoltage,
                                                          std::make_tuple(std::nullopt, std::nullopt, std::nullopt));
        ChannelMeasurement shuntMeasurement = etl::exchange(shuntVoltage,
                                                            std::make_tuple(std::nullopt, std::nullopt, std::nullopt));
        return etl::make_pair(busMeasurement, shuntMeasurement);
    }

    etl::expected<float, Error> INA3221::getShuntVoltage(uint8_t channel) {
        uint8_t regAddress = static_cast<uint8_t>(Register::CH1SV) + (channel - 1) * 2;

        auto regValue = i2cRead(static_cast<Register>(regAddress));

        if (!regValue.has_value()) {
            return etl::expected<float, Error>(etl::unexpected(regValue.error()));
        }

        auto scaledVolts = static_cast<int16_t>(regValue.value());
        return scaledVolts * (float) 0.005;
    }

    etl::expected<float, Error> INA3221::getBusVoltage(uint8_t channel) {
        uint8_t regAddress = static_cast<uint8_t>(Register::CH1BV) + (channel - 1) * 2;

        auto regValue = i2cRead(static_cast<Register>(regAddress));

        if (!regValue.has_value()) {
            return etl::expected<float, Error>(etl::unexpected(regValue.error()));
        }

        auto volts = static_cast<int16_t>(regValue.value());
        return volts * 1.f;
    }

    etl::expected<float, Error> INA3221::getCurrent(uint8_t channel) {
        auto mVolts = getShuntVoltage(channel); 
        if(!mVolts.has_value()) { return mVolts; } 

        float mAmpere = mVolts.value()/shuntResistor;  
        return mAmpere; 
    }

    etl::expected<uint16_t, Error> INA3221::getDieID() {
        return i2cRead(Register::DIEID);
    }

    etl::expected<uint16_t, Error> INA3221::getManID() {
        return i2cRead(Register::MFRID);
    }

    etl::expected<void, Error> INA3221::setup() {
//        uint16_t mode = (config.enableChannel1 << 14) | (config.enableChannel2 << 13) | (config.enableChannel3 << 12) |
//                        ((uint16_t) config.averagingMode << 10) | ((uint16_t) config.busVoltageTime << 6) |
//                        ((uint16_t) config.shuntVoltageTime << 3) | ((uint16_t) config.operatingMode);
        uint16_t mode = 0xF000;
//        uint16_t mode = 0x7123;

        auto err = i2cWrite(Register::CONFG, mode);
        if (!err.has_value()) { return err; }

        auto [criticalThreshold1, warningThreshold1] = config.threshold1;

        err = i2cWrite(Register::CH1CA, voltageConversion(criticalThreshold1, 40, 3));
        if (!err.has_value()) { return err; }

        err = i2cWrite(Register::CH1WA, voltageConversion(warningThreshold1, 40, 3));

        auto [criticalThreshold2, warningThreshold2] = config.threshold2;

        err = i2cWrite(Register::CH2CA, voltageConversion(criticalThreshold2, 40, 3));
        if (!err.has_value()) { return err; }

        err = i2cWrite(Register::CH2WA, voltageConversion(warningThreshold2, 40, 3));
        if (!err.has_value()) { return err; }

        auto [criticalThreshold3, warningThreshold3] = config.threshold3;

        err = i2cWrite(Register::CH3CA, voltageConversion(criticalThreshold3, 40, 3));
        if (!err.has_value()) { return err; }

        err = i2cWrite(Register::CH3WA, voltageConversion(warningThreshold3, 40, 3));
        if (!err.has_value()) { return err; }

        err = i2cWrite(Register::SHVLL, voltageConversion(config.shuntVoltageSumLimit, 40, 1));
        if (!err.has_value()) { return err; }

        err = i2cWrite(Register::PWRVU, voltageConversion(config.powerValidUpper, 8000, 1));
        if (!err.has_value()) { return err; }

        err = i2cWrite(Register::PWRVL, voltageConversion(config.powerValidLower, 8000, 1));
        if (!err.has_value()) { return err; }

        err = i2cWrite(Register::MASKE,
                       (config.summationChannelControl1 << 14) | (config.summationChannelControl2 << 13)
                       | (config.summationChannelControl3 << 12) | (config.enableWarnings << 11) |
                       (config.enableCritical << 10));

        return err;
    }

    void INA3221::handleIrq(void) {
        auto value = i2cRead(Register::MASKE).value();

        if (value & 0x0200) {
            // Send critical alert of channel 1 to FDIR
        }
        if (value & 0x0100) {
            // Send critical alert of channel 2 to FDIR
        }
        if (value & 0x0080) {
            // Send critical alert of channel 3 to FDIR
        }
        if (value & 0x0040) {
            // Send summation alert flag to FDIR
        }
        if (value & 0x0020) {
            // Send warning alert of channel 1 to FDIR
        }
        if (value & 0x0010) {
            // Send warning alert of channel 2 to FDIR
        }
        if (value & 0x0008) {
            // Send warning alert of channel 3 to FDIR
        }
        if (value & 0x0004) {
            //
        }
        if (value & 0x0002) {
            //
        }
        if (value & 0x0001) {
            uint16_t busVoltage1 = i2cRead(Register::CH1BV).value();
            uint16_t busVoltage2 = i2cRead(Register::CH2BV).value();
            uint16_t busVoltage3 = i2cRead(Register::CH3BV).value();

            uint16_t shuntVoltage1 = i2cRead(Register::CH1SV).value();
            uint16_t shuntVoltage2 = i2cRead(Register::CH2SV).value();
            uint16_t shuntVoltage3 = i2cRead(Register::CH3SV).value();


            // Check if bus voltage is monitored
            if ((uint16_t) config.operatingMode & 0x10 || (uint16_t) config.operatingMode & 0x11) {
                busVoltage = std::make_tuple(
                        config.enableChannel1 ? NULL : busVoltage1,
                        config.enableChannel2 ? NULL : busVoltage2,
                        config.enableChannel3 ? NULL : busVoltage3
                );
            } else {
                busVoltage = std::make_tuple(NULL, NULL, NULL);
            }

            // Check if shunt voltage is monitored
            if ((uint16_t) config.operatingMode & 0x01 || (uint16_t) config.operatingMode & 0x11) {
                busVoltage = std::make_tuple(
                        config.enableChannel1 ? NULL : shuntVoltage1,
                        config.enableChannel2 ? NULL : shuntVoltage2,
                        config.enableChannel3 ? NULL : shuntVoltage3
                );
            } else {
                shuntVoltage = std::make_tuple(NULL, NULL, NULL);
            }
        }
    }
}
