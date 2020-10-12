#include <AP_HAL/AP_HAL.h>
#include "AP_PM_SNGCJA5.h"
#include <stdio.h>
#include <utility>
#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define SNGCJA5_I2C_ADDRESS 0x33

#define SNGCJA5_STATUS 0x26

#define SNGCJA5_PM1_0_LL 0x00
#define SNGCJA5_PM1_0_LH 0x01
#define SNGCJA5_PM1_0_HL 0x02
#define SNGCJA5_PM1_0_HH 0x03

#define SNGCJA5_PM2_5_LL 0x04
#define SNGCJA5_PM2_5_LH 0x05
#define SNGCJA5_PM2_5_HL 0x06
#define SNGCJA5_PM2_5_HH 0x07

#define SNGCJA5_PM10_0_LL 0x08
#define SNGCJA5_PM10_0_LH 0x09
#define SNGCJA5_PM10_0_HL 0x0A
#define SNGCJA5_PM10_0_HH 0x0B

#define SNGCJA5_PC_0_5_L 0x0C
#define SNGCJA5_PC_0_5_H 0x0D

#define SNGCJA5_PC_1_0_L 0x0E
#define SNGCJA5_PC_1_0_H 0x0F

#define SNGCJA5_PC_2_5_L 0x10
#define SNGCJA5_PC_2_5_H 0x11

#define SNGCJA5_PC_5_0_L 0x14
#define SNGCJA5_PC_5_0_H 0x15

#define SNGCJA5_PC_7_5_L 0x16
#define SNGCJA5_PC_7_5_H 0x17

#define SNGCJA5_PC_10_0_L 0x18
#define SNGCJA5_PC_10_0_H 0x19

void AP_PM_SNGCJA5::init()
{
    FOREACH_I2C(i) {
        if (init(i)) {
            return;
        }
    }
    gcs().send_text(MAV_SEVERITY_INFO, "No SNGCJA5 found");
}

bool AP_PM_SNGCJA5::init(int8_t bus)
{
    dev = std::move(hal.i2c_mgr->get_device(bus, SNGCJA5_I2C_ADDRESS));
    if (!dev) {
        return false;
    }

    // read at 1Hz
    printf("Starting Particle Matter Sensor on I2C\n");

    dev->register_periodic_callback(1000000, FUNCTOR_BIND_MEMBER(&AP_PM_SNGCJA5::read_frames, void));
    return true;
}

void AP_PM_SNGCJA5::read_frames(void)
{
    WITH_SEMAPHORE(dev->get_semaphore());

    uint8_t val[1];
    if (!dev->read_registers(SNGCJA5_STATUS, val, sizeof(val))) {
        return;
    }

    uint8_t PM1_0[4];
    if (!dev->read_registers(SNGCJA5_PM1_0_LL, PM1_0, sizeof(PM1_0))) {
        return;
    }
    double fPM1_0 = (PM1_0[0] | PM1_0[1] << 8 | PM1_0[2] << 16 | PM1_0[3] << 24);

    uint8_t PM2_5[4];
    if (!dev->read_registers(SNGCJA5_PM2_5_LL, PM2_5, sizeof(PM2_5))) {
        return;
    }
    double fPM2_5 = (PM2_5[0] | PM2_5[1] << 8 | PM2_5[2] << 16 | PM2_5[3] << 24);

    uint8_t PM10_0[4];
    if (!dev->read_registers(SNGCJA5_PM10_0_LL, PM10_0, sizeof(PM10_0))) {
        return;
    }
    double fPM10_0 = (PM10_0[0] | PM10_0[1] << 8 | PM10_0[2] << 16 | PM10_0[3] << 24);

    uint8_t PC0_5[2];
    if (!dev->read_registers(SNGCJA5_PC_0_5_L, PC0_5, sizeof(PC0_5))) {
        return;
    }
    int fPC0_5 = (PC0_5[0] | PC0_5[1] << 8);

    uint8_t PC1_0[2];
    if (!dev->read_registers(SNGCJA5_PC_1_0_L, PC1_0, sizeof(PC1_0))) {
        return;
    }
    int fPC1_0 = (PC1_0[0] | PC1_0[1] << 8);

    uint8_t PC2_5[2];
    if (!dev->read_registers(SNGCJA5_PC_2_5_L, PC2_5, sizeof(PC2_5))) {
        return;
    }
    int fPC2_5 = (PC2_5[0] | PC2_5[1] << 8);

    uint8_t PC5_0[2];
    if (!dev->read_registers(SNGCJA5_PC_5_0_L, PC5_0, sizeof(PC5_0))) {
        return;
    }
    int fPC5_0 = (PC5_0[0] | PC5_0[1] << 8);

    uint8_t PC7_5[2];
    if (!dev->read_registers(SNGCJA5_PC_7_5_L, PC7_5, sizeof(PC7_5))) {
        return;
    }
    int fPC7_5 = (PC7_5[0] | PC7_5[1] << 8);

    uint8_t PC10_0[2];
    if (!dev->read_registers(SNGCJA5_PC_10_0_L, PC10_0, sizeof(PC10_0))) {
        return;
    }
    int fPC10_0 = (PC10_0[0] | PC10_0[1] << 8);

    gcs().send_text(MAV_SEVERITY_INFO,"PS Status: %u", (unsigned)val[0]);
    gcs().send_text(MAV_SEVERITY_INFO,"PS PM1.0: %lf, PM2.5: %lf, PM10.0: %lf", fPM1_0, fPM2_5, fPM10_0);
    gcs().send_text(MAV_SEVERITY_INFO,"PS PC0.5: %d, PC1.0: %d, PC2.5: %d, PC5.0: %d, PC7.5: %d, PC10.0: %d", fPC0_5, fPC1_0, fPC2_5, fPC5_0, fPC7_5, fPC10_0);
}

// periodically called from vehicle code
void AP_PM_SNGCJA5::update()
{
    read_frames();
}
