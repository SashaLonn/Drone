#define I2C_MASTER_SCL_IO           4      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           3    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

#define I2C_MASTER_FREQ_HZ          400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0        /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0        /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define MPU6050_SENSOR_ADDR          0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
#define MPU6050_ACCEL_XOUT_H_REG_ADDR 0x3B        /*!< X_value_H */

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7
#define MPU6050_ACCEL_YOUT_H_REG_ADDR  0x3D        /*!< Y_value_H */
#define MPU6050_ACCEL_ZOUT_H_REG_ADDR  0x3F         /*!< Z_value_H */

#define MPU6050_GYRO_XOUT_H_REG_ADDR  0x43        /*!<Gyro X_value_H */
#define MPU6050_GYRO_YOUT_H_REG_ADDR  0x45        /*!<Gyro Y_value_H */
#define MPU6050_GYRO_ZOUT_H_REG_ADDR  0x47        /*!<Gyro Z_value_H */


