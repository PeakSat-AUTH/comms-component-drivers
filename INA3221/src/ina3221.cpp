#include "ina3221.hpp"

namespace INA3221 {

    uint16_t voltageConversion(int32_t voltage, uint16_t base, uint16_t shift) {
        return (voltage < 0) ?
               (~(std::abs(voltage) / base << shift) + 1 & 0x7FFF) | 0x8000 :
               (voltage / base << shift);
    }

    etl::expected<void, Error> INA3221::i2cWrite(Register address, uint16_t value) {
        etl::array<uint8_t, 3> buffer{ to_underlying(address),
                            static_cast<uint8_t>(value >> 8),
                            static_cast<uint8_t>(value & 0x00FF) };

        if (HAL_I2C_Master_Transmit(&hi2c, to_underlying(I2CSlaveAddress) << 1, buffer.data(), 3, MaxTimeoutDelay) != HAL_OK) {
            return etl::unexpected(Error::I2C_FAILURE);
        }

        return {};
    }

    etl::expected<uint16_t, Error> INA3221::i2cRead(Register address) {
        uint8_t buffer[2];
        auto regAddress = to_underlying(address);

        if (HAL_I2C_Master_Transmit(&hi2c, to_underlying(I2CSlaveAddress) << 1, &regAddress, 1, MaxTimeoutDelay) != HAL_OK) {
            return etl::unexpected(Error::I2C_FAILURE);
        }

        if (HAL_I2C_Master_Receive(&hi2c, to_underlying(I2CSlaveAddress) << 1, buffer, 2, MaxTimeoutDelay) != HAL_OK) {
            return etl::unexpected(Error::I2C_FAILURE);
        }

        uint16_t received = (static_cast<uint16_t>(buffer[0]) << 8) | static_cast<uint16_t>(buffer[1]);
        return received;
    }

    etl::expected<uint16_t, Error> INA3221::getConfigRegister() {
        return i2cRead(Register::CONFG);
    }


    etl::expected<void, Error>
    INA3221::writeRegisterField(Register registerAddress, uint16_t value, uint16_t mask, uint16_t shift) {
        auto reg = i2cRead(registerAddress);
        if (not reg.has_value()) {
            return etl::unexpected(reg.error());
        }

        uint16_t val = (reg.value() & ~mask) | (value << shift);
        return i2cWrite(registerAddress, val);
    }

    etl::expected<void, Error> INA3221::changeOperatingMode(OperatingMode operatingMode) {
        return writeRegisterField(Register::CONFG, to_underlying(operatingMode), 0x7, 0);
    }

    // etl::pair<ChannelMeasurement, ChannelMeasurement> INA3221::getMeasurement() {
    //     ChannelMeasurement busMeasurement = etl::exchange(busVoltage,
    //                                                       std::make_tuple(etl::nullopt, etl::nullopt, etl::nullopt));
    //     ChannelMeasurement shuntMeasurement = etl::exchange(shuntVoltage,
    //                                                         std::make_tuple(etl::nullopt, etl::nullopt, etl::nullopt));
    //     return etl::make_pair(busMeasurement, shuntMeasurement);
    // }

    etl::expected<float, Error> INA3221::getShuntVoltage(uint8_t channel) {
        uint8_t regAddress = to_underlying(Register::CH1SV) + (channel - 1) * 2;

        auto regValue = i2cRead(static_cast<Register>(regAddress));
        if (not regValue.has_value()) {
            return etl::unexpected(regValue.error());
        }

        auto unscaledVolts = static_cast<int16_t>(regValue.value());
        const float mVolts = unscaledVolts * ShuntVoltScale;
        return mVolts;
    }

    etl::expected<float, Error> INA3221::getBusVoltage(uint8_t channel) {
        uint8_t regAddress = to_underlying(Register::CH1BV) + (channel - 1) * 2;

        auto regValue = i2cRead(static_cast<Register>(regAddress));
        if (not regValue.has_value()) {
            return etl::unexpected(regValue.error());
        }

        auto mVolts = static_cast<int16_t>(regValue.value());
        return static_cast<float>(mVolts);
    }

    etl::expected<int32_t, Error> INA3221::getShuntVoltage(uint8_t channel) {
        auto registerAddress = static_cast<Register>(to_underlying(Register::CH1SV) + channel * 2);

        auto registerValue = i2cRead(registerAddress);
        if (not registerValue.has_value()) {
            return etl::unexpected(registerValue.error());
        }

        auto unscaledVolts = static_cast<int16_t>(registerValue.value());
        unscaledVolts >>= 3;
        const int32_t uVolts = static_cast<int32_t>(unscaledVolts) * ShuntVoltBase;

        return uVolts;
    }

    etl::expected<int32_t, Error> INA3221::getBusVoltage(uint8_t channel) {
        auto registerAddress = static_cast<Register>(to_underlying(Register::CH1BV) + channel * 2);

        auto registerValue = i2cRead(registerAddress);
        if (not registerValue.has_value()) {
            return etl::unexpected(registerValue.error());
        }

        auto unscaledVolts = static_cast<int16_t>(registerValue.value());
        unscaledVolts >>= 1;
        const int32_t uVolts = static_cast<int32_t>(unscaledVolts) * BusVoltBase;

        return uVolts;
    }

    etl::expected<uint16_t, Error> INA3221::getDieID() {
        return i2cRead(Register::DIEID);
    }

    etl::expected<uint16_t, Error> INA3221::getManID() {
        return i2cRead(Register::MFRID);
    }

    etl::expected<void, Error> INA3221::setup() {
        uint16_t mode = (config.enableChannel[0] << 14) | (config.enableChannel[1] << 13) | (config.enableChannel[2] << 12) |
                       (to_underlying(config.averagingMode) << 9) | (to_underlying(config.busVoltageTime) << 6) |
                       (to_underlying(config.shuntVoltageTime) << 3) | (to_underlying(config.operatingMode));

        auto error = i2cWrite(Register::CONFG, mode);
        if (not error.has_value()) { return error; }

        auto [criticalThreshold1, warningThreshold1] = config.threshold1;

        error = i2cWrite(Register::CH1CA, voltageConversion(criticalThreshold1, 40, 3));
        if (not error.has_value()) { return error; }

        error = i2cWrite(Register::CH1WA, voltageConversion(warningThreshold1, 40, 3));

        auto [criticalThreshold2, warningThreshold2] = config.threshold2;

        error = i2cWrite(Register::CH2CA, voltageConversion(criticalThreshold2, 40, 3));
        if (not error.has_value()) { return error; }

        error = i2cWrite(Register::CH2WA, voltageConversion(warningThreshold2, 40, 3));
        if (not error.has_value()) { return error; }

        auto [criticalThreshold3, warningThreshold3] = config.threshold3;

        error = i2cWrite(Register::CH3CA, voltageConversion(criticalThreshold3, 40, 3));
        if (not error.has_value()) { return error; }

        error = i2cWrite(Register::CH3WA, voltageConversion(warningThreshold3, 40, 3));
        if (not error.has_value()) { return error; }

        error = i2cWrite(Register::SHVLL, voltageConversion(config.shuntVoltageSumLimit, 40, 1));
        if (not error.has_value()) { return error; }

        error = i2cWrite(Register::PWRVU, voltageConversion(config.powerValidUpper, 8000, 1));
        if (not error.has_value()) { return error; }

        error = i2cWrite(Register::PWRVL, voltageConversion(config.powerValidLower, 8000, 1));
        if (not error.has_value()) { return error; }

        error = i2cWrite(Register::MASKE,
                       (config.summationChannelControl[0] << 14) | (config.summationChannelControl[1] << 13)
                       | (config.summationChannelControl[2] << 12) | (config.enableWarnings << 11) |
                       (config.enableCritical << 10));

        return error;
    }

    etl::expected<void, Error> INA3221::chipReset() {
        return i2cWrite(Register::CONFG, 0xF000);
    }

    void INA3221::handleIrq() {
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
            if (to_underlying(config.operatingMode) & 0x10 || to_underlying(config.operatingMode) & 0x11) {
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