#include "imu.h"

static Adafruit_MPU6050 mpu;
static IMU_Data current_data;
static TwoWire I2C_IMU = TwoWire(0);

bool init_IMU() {
    // Bus I2C en los pines 21 (SDA) y 22 (SCL) a 400kHz (Fast Mode)
    if (!I2C_IMU.begin(21, 22, 400000)) {
        return false;
    }
    
    // Inicializar MPU6050
    if (!mpu.begin(0x68, &I2C_IMU)) {
        return false; 
    }

    // --- CONFIGURACION DE FILTROS INTERNOS (DLPF) ---
    // Para lazos de control: 21Hz o 44Hz
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 

    // --- CONFIGURACION DE RANGOS DE MEDICION ---
    // Acelerometro: +-4G
    // Opciones: MPU6050_RANGE_2_G, _4_G, _8_G, _16_G
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);

    // Giroscopio: +-1000 grados/seg
    // Opciones: MPU6050_RANGE_250_DEG, _500_DEG, _1000_DEG, _2000_DEG
    mpu.setGyroRange(MPU6050_RANGE_1000_DEG);

    return true;
}

void update_IMU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Adafruit ya entrega m/s^2 y rad/s)
    current_data.accelX = a.acceleration.x;
    current_data.accelY = a.acceleration.y;
    current_data.accelZ = a.acceleration.z;
    
    current_data.gyroX = g.gyro.x;
    current_data.gyroY = g.gyro.y;
    current_data.gyroZ = g.gyro.z;
    
    current_data.temp   = temp.temperature;
}

IMU_Data get_IMU_Data() {
    return current_data;
}
