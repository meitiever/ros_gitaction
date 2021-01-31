#include "BMI088.h"
#include "myiic.h"

BMI088_t BMI088;

void write8(device_type_t dev, uint8_t reg, uint8_t val);
uint8_t read8(device_type_t dev, uint8_t reg);
static uint16_t read16(device_type_t dev, uint8_t reg);
static uint16_t read16Be(device_type_t dev, uint8_t reg);
static uint32_t read24(device_type_t dev, uint8_t reg);
static void read(device_type_t dev, uint8_t reg, uint8_t* buf, uint16_t len);
void drv_BMI088_Creat(void);

void drv_BMI088(void) {
    BMI088.devAddrAcc = BMI088_ACC_ADDRESS;
    BMI088.devAddrGyro = BMI088_GYRO_ADDRESS;
	  drv_BMI088_Creat();
}

void drv_initialize(void) {
    BMI088.setAccScaleRange(RANGE_6G);
    BMI088.setAccOutputDataRate(ODR_100);
    BMI088.setAccPoweMode(ACC_ACTIVE);

    BMI088.setGyroScaleRange(RANGE_2000);
    BMI088.setGyroOutputDataRate(ODR_2000_BW_532);
    BMI088.setGyroPoweMode(GYRO_NORMAL);
}

bool drv_isConnection(void) {
    return ((BMI088.getAccID() == 0x1E) && (BMI088.getGyroID() == 0x0F));
}

void drv_resetAcc(void) {
    write8(ACC, BMI088_ACC_SOFT_RESET, 0xB6);
}

void drv_resetGyro(void) {
    write8(GYRO, BMI088_GYRO_SOFT_RESET, 0xB6);
}

uint8_t drv_getAccID(void) {
    return read8(ACC, BMI088_GYRO_CHIP_ID);
}

uint8_t drv_getGyroID(void) {
    return read8(GYRO, BMI088_GYRO_CHIP_ID);
}

void drv_setAccPoweMode(acc_power_type_t mode) {
    if (mode == ACC_ACTIVE) {
        write8(ACC, BMI088_ACC_PWR_CTRl, 0x04);
        write8(ACC, BMI088_ACC_PWR_CONF, 0x00);
    } else if (mode == ACC_SUSPEND) {
        write8(ACC, BMI088_ACC_PWR_CONF, 0x03);
        write8(ACC, BMI088_ACC_PWR_CTRl, 0x00);
    }
}

void drv_setGyroPoweMode(gyro_power_type_t mode) {
    if (mode == GYRO_NORMAL) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_NORMAL);
    } else if (mode == GYRO_SUSPEND) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_SUSPEND);
    } else if (mode == GYRO_DEEP_SUSPEND) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_DEEP_SUSPEND);
    }
}

void drv_setAccScaleRange(acc_scale_type_t range) {
    if (range == RANGE_3G) {
        BMI088.accRange = 3000;
    } else if (range == RANGE_6G) {
        BMI088.accRange = 6000;
    } else if (range == RANGE_12G) {
        BMI088.accRange = 12000;
    } else if (range == RANGE_24G) {
        BMI088.accRange = 24000;
    }

    write8(ACC, BMI088_ACC_RANGE, (uint8_t)range);
}

void drv_setAccOutputDataRate(acc_odr_type_t odr) {
    uint8_t data = 0;

    data = read8(ACC, BMI088_ACC_CONF);
    data = data & 0xf0;
    data = data | (uint8_t)odr;

    write8(ACC, BMI088_ACC_CONF, data);
}

void drv_setGyroScaleRange(gyro_scale_type_t range) {
    if (range == RANGE_2000) {
        BMI088.gyroRange = 2000;
    } else if (range == RANGE_1000) {
        BMI088.gyroRange = 1000;
    } else if (range == RANGE_500) {
        BMI088.gyroRange = 500;
    } else if (range == RANGE_250) {
        BMI088.gyroRange = 250;
    } else if (range == RANGE_125) {
        BMI088.gyroRange = 125;
    }

    write8(GYRO, BMI088_GYRO_RANGE, (uint8_t)range);
}

void drv_setGyroOutputDataRate(gyro_odr_type_t odr) {
    write8(GYRO, BMI088_GYRO_BAND_WIDTH, (uint8_t)odr);
}

void drv_getAcceleration(float* x, float* y, float* z) {
    uint8_t buf[6] = {0};
    uint16_t ax = 0, ay = 0, az = 0;
    float value = 0;

    read(ACC, BMI088_ACC_X_LSB, buf, 6);

    ax = buf[0] | (buf[1] << 8);
    ay = buf[2] | (buf[3] << 8);
    az = buf[4] | (buf[5] << 8);

    value = (int16_t)ax;
    *x = BMI088.accRange * value / 32768;

    value = (int16_t)ay;
    *y = BMI088.accRange * value / 32768;

    value = (int16_t)az;
    *z = BMI088.accRange * value / 32768;
}

float drv_getAccelerationX(void) {
    uint16_t ax = 0;
    float value = 0;

    ax = read16(ACC, BMI088_ACC_X_LSB);

    value = (int16_t)ax;
    value = BMI088.accRange * value / 32768;

    return value;
}

float drv_getAccelerationY(void) {
    uint16_t ay = 0;
    float value = 0;

    ay = read16(ACC, BMI088_ACC_Y_LSB);

    value = (int16_t)ay;
    value = BMI088.accRange * value / 32768;

    return value;
}

float drv_getAccelerationZ(void) {
    uint16_t az = 0;
    float value = 0;

    az = read16(ACC, BMI088_ACC_Z_LSB);

    value = (int16_t)az;
    value = BMI088.accRange * value / 32768;

    return value;
}

void drv_getGyroscope(float* x, float* y, float* z) {
    uint8_t buf[6] = {0};
    uint16_t gx = 0, gy = 0, gz = 0;
    float value = 0;

    read(GYRO, BMI088_GYRO_RATE_X_LSB, buf, 6);

    gx = buf[0] | (buf[1] << 8);
    gy = buf[2] | (buf[3] << 8);
    gz = buf[4] | (buf[5] << 8);

    value = (int16_t)gx;
    *x = BMI088.gyroRange * value / 32768;

    value = (int16_t)gy;
    *y = BMI088.gyroRange * value / 32768;

    value = (int16_t)gz;
    *z = BMI088.gyroRange * value / 32768;
}

float drv_getGyroscopeX(void) {
    uint16_t gx = 0;
    float value = 0;

    gx = read16(GYRO, BMI088_GYRO_RATE_X_LSB);

    value = (int16_t)gx;
    value = BMI088.gyroRange * value / 32768;

    return value;
}

float drv_getGyroscopeY(void) {
    uint16_t gy = 0;
    float value = 0;

    gy = read16(GYRO, BMI088_GYRO_RATE_Y_LSB);

    value = (int16_t)gy;
    value = BMI088.gyroRange * value / 32768;

    return value;
}

float drv_getGyroscopeZ(void) {
    uint16_t gz = 0;
    float value = 0;

    gz = read16(GYRO, BMI088_GYRO_RATE_Z_LSB);

    value = (int16_t)gz;
    value = BMI088.gyroRange * value / 32768;

    return value;
}

int16_t drv_getTemperature(void) {
    uint16_t data = 0;
    uint16_t temp1,temp2;
    data = read16Be(ACC, BMI088_ACC_TEMP_MSB);
//	  temp1 = uc_i2c_read_byte(ACC,BMI088_ACC_TEMP_MSB);
//	temp2 = uc_i2c_read_byte(ACC,BMI088_ACC_TEMP_LSB);
//	data = temp1*8 +temp2/32;
    data = data >> 5;

    if (data > 1023) {
        data = data - 2048;
    }

    return (int16_t)(data / 8 + 23);
}
 void write8(device_type_t dev, uint8_t reg, uint8_t val) {
    uint8_t addr = 0;

    if (dev) {
        addr = BMI088.devAddrGyro;
    } else {
        addr = BMI088.devAddrAcc;
    }
		uc_i2c_write_byte(addr,reg,val);
}

 uint8_t read8(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0, data = 0;

    if (dev) {
        addr = BMI088.devAddrGyro;
    } else {
        addr = BMI088.devAddrAcc;
    }
    data = uc_i2c_read_byte(addr,reg);
    return data;
}

static uint16_t read16(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0;
    uint16_t msb = 0, lsb = 0;

    if (dev) {
        addr = BMI088.devAddrGyro;
    } else {
        addr = BMI088.devAddrAcc;
    }
				u8 buf[2] ;
		uc_i2c_read_Len(addr,reg,2,buf);
		return (buf[1]<<8|buf[0]);
}

static uint16_t read16Be(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0;
    uint16_t msb = 0, lsb = 0;

    if (dev) {
        addr = BMI088.devAddrGyro;
    } else {
        addr = BMI088.devAddrAcc;
    }
		u8 buf[2] ;
		uc_i2c_read_Len(addr,reg,2,buf);
		return (buf[0]<<8|buf[1]);
}

static uint32_t read24(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0;
    uint32_t hsb = 0, msb = 0, lsb = 0;

    if (dev) {
        addr = BMI088.devAddrGyro;
    } else {
        addr = BMI088.devAddrAcc;
    }


}

static void read(device_type_t dev, uint8_t reg, uint8_t* buf, uint16_t len) {
    uint8_t addr = 0;

    if (dev) {
        addr = BMI088.devAddrGyro;
    } else {
        addr = BMI088.devAddrAcc;
    }
		//u8 addr,u8 reg,u8 len,u8 *buf);
		uc_i2c_read_Len(addr,reg,len,buf);
//    for(int i=0;i<len;i++){
//	*buf = uc_i2c_read_byte(addr,reg);
//	buf++;
//    	}
}

void drv_BMI088_Creat(void) {
				
	BMI088.isConnection	 =	drv_isConnection;
						
	BMI088.initialize	 =	drv_initialize;
						
	BMI088.setAccPoweMode	 =	drv_setAccPoweMode;
	BMI088.setGyroPoweMode 	 =	drv_setGyroPoweMode;
						
	BMI088.setAccScaleRange	 =	drv_setAccScaleRange;
	BMI088.setAccOutputDataRate	 =	drv_setAccOutputDataRate;
						
	BMI088.setGyroScaleRange	 =	drv_setGyroScaleRange;
	BMI088.setGyroOutputDataRate	 =	drv_setGyroOutputDataRate;
						
	BMI088.getAcceleration 	 =	drv_getAcceleration;
	BMI088.getAccelerationX	 =	drv_getAccelerationX;
	BMI088.getAccelerationY	 =	drv_getAccelerationY;
	BMI088.getAccelerationZ	 =	drv_getAccelerationZ;
						
	BMI088.getGyroscope	 =	drv_getGyroscope;
	BMI088.getGyroscopeX	 =	drv_getGyroscopeX;
	BMI088.getGyroscopeY	 =	drv_getGyroscopeY;
	BMI088.getGyroscopeZ	 =	drv_getGyroscopeZ;
						
	BMI088.getTemperature	 =	drv_getTemperature;
						
	BMI088.getAccID	 =	drv_getAccID;
	BMI088.getGyroID	 =	drv_getGyroID;
}
void drv_test_getAcceleration(uint16_t* ax, uint16_t* ay, uint16_t* az) {
    uint8_t buf[6] = {0};
    float value = 0;

    read(ACC, BMI088_ACC_X_LSB, buf, 6);

    *ax = buf[0] | (buf[1] << 8);
    *ay = buf[2] | (buf[3] << 8);
    *az = buf[4] | (buf[5] << 8);
}