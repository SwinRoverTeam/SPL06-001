#include "ArtronShop_SPL06-001.h"

// Register list
#define REG_PSR_B2 (0x00)
#define REG_PSR_B1 (0x01)
#define REG_PSR_B0 (0x02)
#define REG_TMP_B2 (0x03)
#define REG_TMP_B1 (0x04)
#define REG_TMP_B0 (0x05)
#define REG_PSR_CFG (0x06)
#define REG_TMP_CFG (0x07)
#define REG_MEAS_CFG (0x08)
#define REG_CFG_REG (0x09)
#define REG_INT_STS (0x0A)
#define REG_FIFO_STS (0x0B)
#define REG_RESET (0x0C)
#define REG_ID (0x0D)
// Calibration Coefficients (COEF)
#define REG_COEF_C0 (0x10)
#define REG_COEF_C0C1 (0x11)
#define REG_COEF_C1 (0x12)
#define REG_COEF_C00a (0x13)
#define REG_COEF_C00b (0x14)
#define REG_COEF_C00C10 (0x15)
#define REG_COEF_C10a (0x16)
#define REG_COEF_C10b (0x17)
#define REG_COEF_C01a (0x18)
#define REG_COEF_C01b (0x19)
#define REG_COEF_C11a (0x1A)
#define REG_COEF_C11b (0x1B)
#define REG_COEF_C20a (0x1C)
#define REG_COEF_C20b (0x1D)
#define REG_COEF_C21a (0x1E)
#define REG_COEF_C21b (0x1F)
#define REG_COEF_C30a (0x20)
#define REG_COEF_C30b (0x21)

ArtronShop_SPL06_001::ArtronShop_SPL06_001(uint8_t addr, TwoWire *wire) : _addr(addr), _wire(wire) {
    // -----
}

bool ArtronShop_SPL06_001::write_reg(uint8_t reg, uint8_t *value, size_t len) {
    this->_wire->beginTransmission(this->_addr);
    this->_wire->write(reg);
    this->_wire->write(value, len);
    return this->_wire->endTransmission() == 0;
}

bool ArtronShop_SPL06_001::write_reg(uint8_t reg, uint8_t value) {
    return this->write_reg(reg, &value, 1);
}

bool ArtronShop_SPL06_001::read_reg(uint8_t reg, uint8_t *value, size_t len) {
    this->_wire->beginTransmission(this->_addr);
    this->_wire->write(reg);
    if (this->_wire->endTransmission(false) != 0) {
        return false;
    }

    int n = this->_wire->requestFrom(this->_addr, len);
    if (n != len) {
        return false;
    }

    this->_wire->readBytes(value, len);

    return true;
}

uint8_t ArtronShop_SPL06_001::read_reg(uint8_t reg) {
    uint8_t value = 0;
    this->read_reg(reg, &value, 1);
    return value;
}

#define CHECK_OK(OP) { \
    if (!(OP)) { \
        return false; \
    } \
}

bool ArtronShop_SPL06_001::begin() {
    CHECK_OK(this->read_reg(REG_ID) == 0x10); // Check Product and Revision ID

    CHECK_OK(this->write_reg(REG_RESET, 0b1001)); // soft reset
    delay(50); // wait sensor ready and coefficients are available 
    Sesnor_Status_t status;
    CHECK_OK(this->status(&status));
    CHECK_OK(status.SENSOR_RDY && status.COEF_RDY); // Check sensor ready and coefficients are available flag
    CHECK_OK(this->write_reg(REG_MEAS_CFG, 0b111)); // Set measurement mode: Continuous pressure and temperature measurement
    CHECK_OK(this->write_reg(REG_PSR_CFG, 0b01000000)); // Pressure measurement rate: 100 - 16 measurements pr. sec., Pressure oversampling rate: 0000 - Single.
    CHECK_OK(this->write_reg(REG_TMP_CFG, 0b01000000)); // Temperature measurement rate: 100 - 16 measurements pr. sec., Temperature oversampling rate: 0000 - Single.
    
    // Read Calibration Coefficients
    uint8_t buff[18];
    CHECK_OK(this->read_reg(REG_COEF_C0, buff, sizeof(buff)));
    calibration_coefficients.c0 = ((int16_t)(buff[0]) << 4) | (buff[1] >> 4);
    calibration_coefficients.c1 = ((int16_t)(buff[1]) << 4) | buff[2];
    calibration_coefficients.c00 = (((int32_t)(buff[3]) << 16) | ((int32_t)(buff[4]) << 8) | buff[5]) >> 4;
    calibration_coefficients.c10 = ((int32_t)(buff[5] & 0x0F) << 16) | ((int32_t)(buff[6]) << 8) | buff[7];
    calibration_coefficients.c01 = ((int16_t)(buff[8]) << 8) | buff[9];
    calibration_coefficients.c11 = ((int16_t)(buff[10]) << 8) | buff[11];
    calibration_coefficients.c20 = ((int16_t)(buff[12]) << 8) | buff[13];
    calibration_coefficients.c21 = ((int16_t)(buff[14]) << 8) | buff[15];
    calibration_coefficients.c30 = ((int16_t)(buff[16]) << 8) | buff[17];

    if (bitRead(calibration_coefficients.c0, 11)) {
        calibration_coefficients.c0 |= 0xF000;
    }
    if (bitRead(calibration_coefficients.c1, 11)) {
        calibration_coefficients.c1 |= 0xF000;
    }
    if (bitRead(calibration_coefficients.c00, 19)) {
        calibration_coefficients.c00 |= 0xFFF00000;
    }
    if (bitRead(calibration_coefficients.c10, 19)) {
        calibration_coefficients.c10 |= 0xFFF00000;
    }

    delay(5); // wait first measurement

    return true;
}

bool ArtronShop_SPL06_001::status(Sesnor_Status_t *status) {
    CHECK_OK(this->read_reg(REG_MEAS_CFG, (uint8_t*) status)); // Sensor Operating Mode and Status (MEAS_CFG)

    return true;
}

#define CHECK_OK_999(OP) { \
    if (!(OP)) { \
        return -999.0f; \
    } \
}

float ArtronShop_SPL06_001::pressure() {
    Sesnor_Status_t status;
    CHECK_OK_999(this->status(&status));
    CHECK_OK_999(status.PRS_RDY);

    uint8_t buff[6];
    CHECK_OK_999(this->read_reg(REG_PSR_B2, buff, 6));
    int32_t Praw = (((int32_t) buff[0]) << 16) | (((int32_t) buff[1]) << 8) | buff[2];
    if (bitRead(Praw, 23)) {
        Praw |= 0xFF000000; // Set left bits to one for 2's complement conversion of negative number
    }
    int32_t Traw = (((int32_t) buff[3]) << 16) | (((int32_t) buff[4]) << 8) | buff[5];
    if (bitRead(Traw, 23)) {
        Traw |= 0xFF000000; // Set left bits to one for 2's complement conversion of negative number
    }

    float Traw_sc = Traw / 524288.0f; // Oversampling Rate: 1, Ref. Table 7 : Compensation Scale Factors
    float Praw_sc = Praw / 524288.0f; // Oversampling Rate: 1, Ref. Table 7 : Compensation Scale Factors

    float Pcomp = calibration_coefficients.c00 + Praw_sc * (calibration_coefficients.c10 + Praw_sc *(calibration_coefficients.c20 + Praw_sc * calibration_coefficients.c30)) + Traw_sc * calibration_coefficients.c01 + Traw_sc * Praw_sc * (calibration_coefficients.c11 + Praw_sc * calibration_coefficients.c21);

    return Pcomp;
}


float ArtronShop_SPL06_001::temperature() {
    Sesnor_Status_t status;
    CHECK_OK_999(this->status(&status));
    CHECK_OK_999(status.TMP_RDY);

    uint8_t buff[3];
    CHECK_OK_999(this->read_reg(REG_TMP_B2, buff, 3));
    int32_t Traw = (((int32_t) buff[0]) << 16) | (((int32_t) buff[1]) << 8) | buff[2];
    if (bitRead(Traw, 23)) {
        Traw |= 0xFF000000; // Set left bits to one for 2's complement conversion of negative number
    }

    float Traw_sc = Traw / 524288.0f; // Oversampling Rate: 1, Ref. Table 7 : Compensation Scale Factors
    float Tcomp = calibration_coefficients.c0 * 0.5 + calibration_coefficients.c1 * Traw_sc;

    return Tcomp;
}
