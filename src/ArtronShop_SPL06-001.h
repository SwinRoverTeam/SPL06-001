#ifndef __ARTRONSHOP_SPL06_001_H__
#define __ARTRONSHOP_SPL06_001_H__

#include <Arduino.h>
#include <Wire.h>

#define SPL06_001_ADDR0 (0x77)
#define SPL06_001_ADDR1 (0x76)

typedef struct {
    uint32_t MEAS_CTRL : 3;
    uint32_t RESERVED1 : 1;
    uint32_t PRS_RDY : 1;
    uint32_t TMP_RDY : 1;
    uint32_t SENSOR_RDY : 1;
    uint32_t COEF_RDY : 1;
} Sesnor_Status_t;

class ArtronShop_SPL06_001 {
    private:
        uint8_t _addr;
        TwoWire *_wire = NULL;
        struct {
            int16_t c0, c1, c01, c11, c20, c21, c30;
            int32_t c00, c10;
        } calibration_coefficients;

        bool write_reg(uint8_t reg, uint8_t *value, size_t len) ;
        bool write_reg(uint8_t reg, uint8_t value) ;
        bool read_reg(uint8_t reg, uint8_t *value, size_t len = 1) ;
        uint8_t read_reg(uint8_t reg) ;

        bool status(Sesnor_Status_t *status) ;

    public:
        ArtronShop_SPL06_001(uint8_t addr = SPL06_001_ADDR0, TwoWire *wire = &Wire) ;
        bool begin() ;
        float pressure() ;
        float temperature() ;

};

#endif