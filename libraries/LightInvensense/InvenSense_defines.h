#ifndef INVENSENSE_DEFINES_H
#define INVENSENSE_DEFINES_H

#include <VarioSettings.h>

/* Time for some messy macro work. =]
 * #define MPU9150
 * is equivalent to..
 * #define MPU6050
 * #define AK8975_SECONDARY
 *
 * #define MPU9250
 * is equivalent to..
 * #define MPU6500
 * #define AK8963_SECONDARY
 */
#if defined MPU9150
#ifndef MPU6050
#define MPU6050
#endif                          /* #ifndef MPU6050 */
#if defined AK8963_SECONDARY
#error "MPU9150 and AK8963_SECONDARY cannot both be defined."
#elif !defined AK8975_SECONDARY /* #if defined AK8963_SECONDARY */
#define AK8975_SECONDARY
#endif                          /* #if defined AK8963_SECONDARY */
#elif defined MPU9250           /* #if defined MPU9150 */
#ifndef MPU6500
#define MPU6500
#endif                          /* #ifndef MPU6500 */
#if defined AK8975_SECONDARY
#error "MPU9250 and AK8975_SECONDARY cannot both be defined."
#elif !defined AK8963_SECONDARY /* #if defined AK8975_SECONDARY */
#define AK8963_SECONDARY
#endif                          /* #if defined AK8975_SECONDARY */
#endif                          /* #if defined MPU9150 */

#if defined AK8975_SECONDARY || defined AK8963_SECONDARY
#define AK89xx_SECONDARY
#else
/* #warning "No compass = less profit for Invensense. Lame." */
#endif

/* Filter configurations. */
#define  INV_FILTER_256HZ_NOLPF2 0
#define  INV_FILTER_188HZ 1
#define  INV_FILTER_98HZ 2
#define  INV_FILTER_42HZ 3
#define  INV_FILTER_20HZ 4
#define  INV_FILTER_10HZ 5
#define  INV_FILTER_5HZ 6
#define  INV_FILTER_2100HZ_NOLPF 7
#define  NUM_FILTER 8

/* Full scale ranges. */
#define INV_FSR_250DPS 0
#define INV_FSR_500DPS 1
#define INV_FSR_1000DPS 2
#define INV_FSR_2000DPS 3
#define NUM_GYRO_FSR 4

/* Full scale ranges. */
#define INV_FSR_2G 0
#define INV_FSR_4G 1
#define INV_FSR_8G 2
#define INV_FSR_16G 3
#define NUM_ACCEL_FSR 4

/* Clock sources. */
#define INV_CLK_INTERNAL 0
#define INV_CLK_PLL 1
#define NUM_CLK 2


#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)
#define BIT_ACCL_FC_B       (0x08)

#if defined AK8975_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x00)
#define AK89xx_FSR                  (9830)
#elif defined AK8963_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x10)
#define AK89xx_FSR                  (4915)
#endif

#ifdef AK89xx_SECONDARY
#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)
#endif

#if defined MPU6050
  
/* GYRO REG */
#define INV_REG_WHO_AM_I       (0x75)
#define INV_REG_RATE_DIV       (0x19)
#define INV_REG_LPF            (0x1A)
#define INV_REG_PROD_ID        (0x0C)
#define INV_REG_USER_CTRL      (0x6A)
#define INV_REG_FIFO_EN        (0x23)
#define INV_REG_GYRO_CFG       (0x1B)
#define INV_REG_ACCEL_CFG      (0x1C)
#define INV_REG_MOTION_THR     (0x1F)
#define INV_REG_MOTION_DUR     (0x20)
#define INV_REG_FIFO_COUNT_H   (0x72)
#define INV_REG_FIFO_R_W       (0x74)
#define INV_REG_RAW_GYRO       (0x43)
#define INV_REG_RAW_ACCEL      (0x3B)
#define INV_REG_TEMP           (0x41)
#define INV_REG_INT_ENABLE     (0x38)
#define INV_REG_DMP_INT_STATUS (0x39)
#define INV_REG_INT_STATUS     (0x3A)
#define INV_REG_PWR_MGMT_1     (0x6B)
#define INV_REG_PWR_MGMT_2     (0x6C)
#define INV_REG_INT_PIN_CFG    (0x37)
#define INV_REG_MEM_R_W        (0x6F)
#define INV_REG_ACCEL_OFFS     (0x06)
#define INV_REG_I2C_MST        (0x24)
#define INV_REG_BANK_SEL       (0x6D)
#define INV_REG_MEM_START_ADDR (0x6E)
#define INV_REG_PRGM_START_H   (0x70)
#ifdef AK89xx_SECONDARY
#define INV_REG_RAW_COMPASS    (0x49)
#define INV_REG_YG_OFFS_TC     (0x01)
#define INV_REG_S0_ADDR        (0x25)
#define INV_REG_S0_REG         (0x26)
#define INV_REG_S0_CTRL        (0x27)
#define INV_REG_S1_ADDR        (0x28)
#define INV_REG_S1_REG         (0x29)
#define INV_REG_S1_CTRL        (0x2A)
#define INV_REG_S4_ADDR        (0x31)
#define INV_REG_S4_REG         (0x32)
#define INV_REG_S4_DO          (0x33)
#define INV_REG_S4_CTRL        (0x34)
#define INV_REG_S4_DI          (0x35)
#define INV_REG_I2C_MST_STATUS (0x36)
#define INV_REG_S0_DO          (0x63)
#define INV_REG_S1_DO          (0x64)
#define INV_REG_I2C_DELAY_CTRL (0x67)
#endif

/* HARDWARE */ 
#define INV_HW_ADDR           (0x68)
#define INV_HW_MAX_FIFO       (1024)
#define INV_HW_NUM_REG        (118)
#define INV_HW_TEMP_SENS      (340)
#define INV_HW_TEMP_OFFSET    (-521)
#define INV_HW_BANK_SIZE      (256)
#if defined AK89xx_SECONDARY
#define INV_HW_COMPASS_FSR    (AK89xx_FSR)
#endif

/* TEST */
#define INV_TEST_GYRO_SENS      (32768/250)
#define INV_TEST_ACCEL_SENS     (32768/16)
#define INV_TEST_REG_RATE_DIV   (0)
#define INV_TEST_REG_LPF        (1)
#define INV_TEST_REG_GYRO_FSR   (0)
#define INV_TEST_REG_ACCEL_FSR  (0x18)
#define INV_TEST_WAIT_MS        (50)
#define INV_TEST_PACKET_THRESH  (5)
#define INV_TEST_MIN_DPS        (10.f)
#define INV_TEST_MAX_DPS        (105.f)
#define INV_TEST_MAX_GYRO_VAR   (0.14f)
#define INV_TEST_MIN_G          (0.3f)
#define INV_TEST_MAX_G          (0.95f)
#define INV_TEST_MAX_ACCEL_VAR  (0.14f)

#elif defined MPU6500

/* GYRO REGS */
#define INV_REG_WHO_AM_I       (0x75)
#define INV_REG_RATE_DIV       (0x19)
#define INV_REG_LPF            (0x1A)
#define INV_REG_PROD_ID        (0x0C)
#define INV_REG_USER_CTRL      (0x6A)
#define INV_REG_FIFO_EN        (0x23)
#define INV_REG_GYRO_CFG       (0x1B)
#define INV_REG_ACCEL_CFG      (0x1C)
#define INV_REG_ACCEL_CFG2     (0x1D)
#define INV_REG_LP_ACCEL_ODR   (0x1E)
#define INV_REG_MOTION_THR     (0x1F)
#define INV_REG_MOTION_DUR     (0x20)
#define INV_REG_FIFO_COUNT_H   (0x72)
#define INV_REG_FIFO_R_W       (0x74)
#define INV_REG_RAW_GYRO       (0x43)
#define INV_REG_RAW_ACCEL      (0x3B)
#define INV_REG_TEMP           (0x41)
#define INV_REG_INT_ENABLE     (0x38)
#define INV_REG_DMP_INT_STATUS (0x39)
#define INV_REG_INT_STATUS     (0x3A)
#define INV_REG_ACCEL_INTEL    (0x69)
#define INV_REG_PWR_MGMT_1     (0x6B)
#define INV_REG_PWR_MGMT_2     (0x6C)
#define INV_REG_INT_PIN_CFG    (0x37)
#define INV_REG_MEM_R_W        (0x6F)
#define INV_REG_ACCEL_OFFS     (0x77)
#define INV_REG_I2C_MST        (0x24)
#define INV_REG_BANK_SEL       (0x6D)
#define INV_REG_MEM_START_ADDR (0x6E)
#define INV_REG_PRGM_START_H   (0x70)
#ifdef AK89xx_SECONDARY
#define INV_REG_RAW_COMPASS    (0x49)
#define INV_REG_S0_ADDR        (0x25)
#define INV_REG_S0_REG         (0x26)
#define INV_REG_S0_CTRL        (0x27)
#define INV_REG_S1_ADDR        (0x28)
#define INV_REG_S1_REG         (0x29)
#define INV_REG_S1_CTRL        (0x2A)
#define INV_REG_S4_ADDR        (0x31)
#define INV_REG_S4_REG         (0x32)
#define INV_REG_S4_DO          (0x33)
#define INV_REG_S4_CTRL        (0x34)
#define INV_REG_S4_DI          (0x35)
#define INV_REG_I2C_MST_STATUS (0x36)
#define INV_REG_S0_DO          (0x63)
#define INV_REG_S1_DO          (0x64)
#define INV_REG_I2C_DELAY_CTRL (0x67)
#endif


/* HARDWARE */
#define INV_HW_ADDR           (0x68)
#define INV_HW_MAX_FIFO       (1024)
#define INV_HW_NUM_REG        (128)
#define INV_HW_TEMP_SENS      (321)
#define INV_HW_TEMP_OFFSET    (0)
#define INV_HW_BANK_SIZE      (256)
#if defined AK89xx_SECONDARY
#define INV_HW_COMPASS_FSR    (AK89xx_FSR)
#endif


/* TEST */
#define INV_TEST_GYRO_SENS      (32768/250)
#define INV_TEST_ACCEL_SENS     (32768/2)
#define INV_TEST_REG_RATE_DIV   (0)
#define INV_TEST_REG_LPF        (2)
#define INV_TEST_REG_GYRO_FSR   (0)
#define INV_TEST_REG_ACCEL_FSR  (0x0)
#define INV_TEST_WAIT_MS        (200)
#define INV_TEST_PACKET_THRESH  (200)
#define INV_TEST_MIN_DPS        (20.f)
#define INV_TEST_MAX_DPS        (60.f)
#define INV_TEST_MAX_GYRO_VAR   (.5f)
#define INV_TEST_MIN_G          (.225f)
#define INV_TEST_MAX_G          (.675f)
#define INV_TEST_MAX_ACCEL_VAR  (.5f)
#define INV_TEST_MAX_G_OFFSET   (.5f)
#define INV_TEST_SAMPLE_WAIT_MS (10)

#endif

#define MAX_PACKET_LENGTH (12)
#ifdef MPU6500
#define HWST_MAX_PACKET_LENGTH (512)
#endif

#ifdef AK89xx_SECONDARY
#define MAX_COMPASS_SAMPLE_RATE (100)
#endif

#define INV_D_EXT_GYRO_BIAS       (61 * 16)
#define INV_D_ACCEL_BIAS            (660)
#define INV_DMP_SAMPLE_RATE     (200)
#define INV_GYRO_SF             (46850825LL * 200 / INV_DMP_SAMPLE_RATE)

#endif
