#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/mpu6050_vdev.h"

/*
 * MPU6050 6-axis IMU Virtual Device
 *
 * Key registers:
 *  0x19 SMPLRT_DIV    Sample rate divider
 *  0x1A CONFIG        DLPF config
 *  0x1B GYRO_CONFIG   Gyro full-scale range
 *  0x1C ACCEL_CONFIG  Accel full-scale range
 *  0x3B-0x40          Accel data (X/Y/Z, 16-bit big-endian)
 *  0x41-0x42          Temperature (16-bit big-endian)
 *  0x43-0x48          Gyro data (X/Y/Z, 16-bit big-endian)
 *  0x6B PWR_MGMT_1    Power management
 *  0x75 WHO_AM_I      Device ID (returns 0x68)
 *
 *  Accel: ±2g default → 16384 LSB/g
 *  Gyro:  ±250°/s default → 131 LSB/(°/s)
 *  Temp:  340 LSB/°C, offset 36.53°C  → raw = (T - 36.53) * 340
 */

static bool mpu6050_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void mpu6050_reset_v(VDevBase *base)
{
    MPU6050VDev *d = (MPU6050VDev*)base;
    d->register_address = 0x00;
    d->read_phase = 0;
    d->pwr_mgmt_1 = 0x40;   /* sleep mode by default */
    d->pwr_mgmt_2 = 0x00;
    d->smplrt_div = 0x00;
    d->config = 0x00;
    d->gyro_config = 0x00;   /* ±250°/s */
    d->accel_config = 0x00;  /* ±2g */
    d->last_tick_ns = 0;
}

static void mpu6050_tick_v(VDevBase *base, uint64_t now_ns)
{
    MPU6050VDev *d = (MPU6050VDev*)base;
    /* Only update if not in sleep mode (bit 6 of PWR_MGMT_1) */
    if (d->pwr_mgmt_1 & 0x40) return;
    if (d->last_tick_ns == 0) {
        d->last_tick_ns = now_ns;
        return;
    }
    uint64_t elapsed = now_ns - d->last_tick_ns;
    if (elapsed >= 100000ULL) { /* 100us */
        d->accel_x += (float)g_random_double_range(-d->noise_accel, d->noise_accel);
        d->accel_y += (float)g_random_double_range(-d->noise_accel, d->noise_accel);
        d->accel_z += (float)g_random_double_range(-d->noise_accel, d->noise_accel);
        d->gyro_x  += (float)g_random_double_range(-d->noise_gyro, d->noise_gyro);
        d->gyro_y  += (float)g_random_double_range(-d->noise_gyro, d->noise_gyro);
        d->gyro_z  += (float)g_random_double_range(-d->noise_gyro, d->noise_gyro);
        d->last_tick_ns = now_ns;
    }
}

static bool mpu6050_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    MPU6050VDev *d = (MPU6050VDev*)base;
    if (d->i2c_addr7 != (addr7 & 0x7F)) return false;
    return base->responding && base->present;
}

static void mpu6050_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    MPU6050VDev *d = (MPU6050VDev*)base;
    d->read_phase = 0;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static ssize_t mpu6050_i2c_write(VDevBase *base, uint8_t addr7,
                                  const uint8_t *data, size_t length)
{
    (void)addr7;
    MPU6050VDev *d = (MPU6050VDev*)base;
    if (!data || length == 0) return 0;

    if (length == 1) {
        d->register_address = data[0];
        d->read_phase = 0;
        return 1;
    }

    d->register_address = data[0];
    uint8_t val = data[1];
    switch (d->register_address) {
    case 0x19: d->smplrt_div   = val; break;
    case 0x1A: d->config       = val; break;
    case 0x1B: d->gyro_config  = val; break;
    case 0x1C: d->accel_config = val; break;
    case 0x6B: d->pwr_mgmt_1  = val; break;
    case 0x6C: d->pwr_mgmt_2  = val; break;
    }
    return (ssize_t)length;
}

static ssize_t mpu6050_i2c_read(VDevBase *base, uint8_t addr7,
                                 uint8_t *out, size_t length)
{
    (void)addr7;
    MPU6050VDev *d = (MPU6050VDev*)base;
    if (!out || length == 0) return 0;

    /* Accel sensitivity based on AFS_SEL (bits 4:3 of ACCEL_CONFIG) */
    float accel_scale;
    switch ((d->accel_config >> 3) & 0x03) {
    case 0: accel_scale = 16384.0f; break; /* ±2g */
    case 1: accel_scale = 8192.0f;  break; /* ±4g */
    case 2: accel_scale = 4096.0f;  break; /* ±8g */
    case 3: accel_scale = 2048.0f;  break; /* ±16g */
    default: accel_scale = 16384.0f;
    }

    /* Gyro sensitivity based on FS_SEL (bits 4:3 of GYRO_CONFIG) */
    float gyro_scale;
    switch ((d->gyro_config >> 3) & 0x03) {
    case 0: gyro_scale = 131.0f;   break; /* ±250°/s */
    case 1: gyro_scale = 65.5f;    break; /* ±500°/s */
    case 2: gyro_scale = 32.8f;    break; /* ±1000°/s */
    case 3: gyro_scale = 16.4f;    break; /* ±2000°/s */
    default: gyro_scale = 131.0f;
    }

    int16_t raw_ax = (int16_t)(d->accel_x * accel_scale);
    int16_t raw_ay = (int16_t)(d->accel_y * accel_scale);
    int16_t raw_az = (int16_t)(d->accel_z * accel_scale);
    int16_t raw_gx = (int16_t)(d->gyro_x * gyro_scale);
    int16_t raw_gy = (int16_t)(d->gyro_y * gyro_scale);
    int16_t raw_gz = (int16_t)(d->gyro_z * gyro_scale);
    int16_t raw_temp = (int16_t)((d->temperature - 36.53f) * 340.0f);

    for (size_t i = 0; i < length; i++) {
        uint8_t reg = d->register_address + d->read_phase;
        uint8_t val = 0x00;

        switch (reg) {
        /* Accel data: big-endian (MSB first) */
        case 0x3B: val = (uint8_t)((raw_ax >> 8) & 0xFF); break;
        case 0x3C: val = (uint8_t)(raw_ax & 0xFF); break;
        case 0x3D: val = (uint8_t)((raw_ay >> 8) & 0xFF); break;
        case 0x3E: val = (uint8_t)(raw_ay & 0xFF); break;
        case 0x3F: val = (uint8_t)((raw_az >> 8) & 0xFF); break;
        case 0x40: val = (uint8_t)(raw_az & 0xFF); break;
        /* Temperature */
        case 0x41: val = (uint8_t)((raw_temp >> 8) & 0xFF); break;
        case 0x42: val = (uint8_t)(raw_temp & 0xFF); break;
        /* Gyro data: big-endian */
        case 0x43: val = (uint8_t)((raw_gx >> 8) & 0xFF); break;
        case 0x44: val = (uint8_t)(raw_gx & 0xFF); break;
        case 0x45: val = (uint8_t)((raw_gy >> 8) & 0xFF); break;
        case 0x46: val = (uint8_t)(raw_gy & 0xFF); break;
        case 0x47: val = (uint8_t)((raw_gz >> 8) & 0xFF); break;
        case 0x48: val = (uint8_t)(raw_gz & 0xFF); break;
        /* Config registers */
        case 0x19: val = d->smplrt_div;   break;
        case 0x1A: val = d->config;       break;
        case 0x1B: val = d->gyro_config;  break;
        case 0x1C: val = d->accel_config; break;
        case 0x6B: val = d->pwr_mgmt_1;  break;
        case 0x6C: val = d->pwr_mgmt_2;  break;
        case 0x75: val = 0x68;            break; /* WHO_AM_I */
        }

        out[i] = val;
        d->read_phase++;
    }
    return (ssize_t)length;
}

static const VDevVTable mpu6050_vtable = {
    .init    = mpu6050_init_v,
    .reset   = mpu6050_reset_v,
    .destroy = NULL,
    .tick    = mpu6050_tick_v,
};

static const VDevI2COps mpu6050_i2c_ops = {
    .i2c_can_ack      = mpu6050_i2c_can_ack,
    .i2c_on_addressed = mpu6050_i2c_on_addressed,
    .i2c_write        = mpu6050_i2c_write,
    .i2c_read         = mpu6050_i2c_read,
};

MPU6050VDev* mpu6050_vdev_create(uint8_t addr7, float temp_c)
{
    MPU6050VDev *d = g_new0(MPU6050VDev, 1);
    d->base.name = "MPU6050";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.vtable = &mpu6050_vtable;
    d->i2c_addr7 = addr7 & 0x7F;
    d->pwr_mgmt_1 = 0x40;   /* sleep mode default */
    d->accel_x = 0.0f;
    d->accel_y = 0.0f;
    d->accel_z = 1.0f;       /* 1g on Z (gravity) */
    d->gyro_x = 0.0f;
    d->gyro_y = 0.0f;
    d->gyro_z = 0.0f;
    d->temperature = temp_c;
    d->noise_accel = 0.005;
    d->noise_gyro = 0.1;
    return d;
}

const VDevI2COps* mpu6050_vdev_get_i2c_ops(void)
{
    return &mpu6050_i2c_ops;
}
