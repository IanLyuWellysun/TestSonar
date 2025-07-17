/*
 * icm42607t.h
 *
 *  Created on: Sep 28, 2023
 *      Author: ian.lyu
 */

#ifndef INC_ICM42607T_H_
#define INC_ICM42607T_H_

enum
{
	ICM42607T_ADDR_B0_MCLK_RDY=0x00,
	ICM42607T_ADDR_B0_DEVICE_CONFIG=0x01,
	ICM42607T_ADDR_B0_SIGNAL_PATH_RESET=0x02,
	ICM42607T_ADDR_B0_DRIVE_CONFIG1=0x03,
	ICM42607T_ADDR_B0_DRIVE_CONFIG2=0x04,
	ICM42607T_ADDR_B0_DRIVE_CONFIG3=0x05,
	ICM42607T_ADDR_B0_INT_CONFIG=0x06,
	ICM42607T_ADDR_B0_TEMP_DATA1=0x09,
	ICM42607T_ADDR_B0_TEMP_DATA0=0x0A,
	ICM42607T_ADDR_B0_ACCEL_DATA_X1=0x0B,
	ICM42607T_ADDR_B0_ACCEL_DATA_X0=0x0C,
	ICM42607T_ADDR_B0_ACCEL_DATA_Y1=0x0D,
	ICM42607T_ADDR_B0_ACCEL_DATA_Y0=0x0E,
	ICM42607T_ADDR_B0_ACCEL_DATA_Z1=0x0F,
	ICM42607T_ADDR_B0_ACCEL_DATA_Z0=0x10,
	ICM42607T_ADDR_B0_GYRO_DATA_X1=0x11,
	ICM42607T_ADDR_B0_GYRO_DATA_X0=0x12,
	ICM42607T_ADDR_B0_GYRO_DATA_Y1=0x13,
	ICM42607T_ADDR_B0_GYRO_DATA_Y0=0x14,
	ICM42607T_ADDR_B0_GYRO_DATA_Z1=0x15,
	ICM42607T_ADDR_B0_GYRO_DATA_Z0=0x16,
	ICM42607T_ADDR_B0_TMST_FSYNCH=0x17,
	ICM42607T_ADDR_B0_TMST_FSYNCL=0x18,
	ICM42607T_ADDR_B0_APEX_DATA4=0x1D,
	ICM42607T_ADDR_B0_APEX_DATA5=0x1E,
	ICM42607T_ADDR_B0_PWR_MGMT0=0x1F,
	ICM42607T_ADDR_B0_GYRO_CONFIG0=0x20,
	ICM42607T_ADDR_B0_ACCEL_CONFIG0=0x21,
	ICM42607T_ADDR_B0_TEMP_CONFIG0=0x22,
	ICM42607T_ADDR_B0_GYRO_CONFIG1=0x23,
	ICM42607T_ADDR_B0_ACCEL_CONFIG1=0x24,
	ICM42607T_ADDR_B0_APEX_CONFIG0=0x25,
	ICM42607T_ADDR_B0_APEX_CONFIG1=0x26,
	ICM42607T_ADDR_B0_WOM_CONFIG=0x27,
	ICM42607T_ADDR_B0_FIFO_CONFIG1=0x28,
	ICM42607T_ADDR_B0_FIFO_CONFIG2=0x29,
	ICM42607T_ADDR_B0_FIFO_CONFIG3=0x2A,
	ICM42607T_ADDR_B0_INT_SOURCE0=0x2B,
	ICM42607T_ADDR_B0_INT_SOURCE1=0x2C,
	ICM42607T_ADDR_B0_INT_SOURCE3=0x2D,
	ICM42607T_ADDR_B0_INT_SOURCE4=0x2E,
	ICM42607T_ADDR_B0_FIFO_LOST_PKT0=0x2F,
	ICM42607T_ADDR_B0_FIFO_LOST_PKT1=0x30,
	ICM42607T_ADDR_B0_APEX_DATA0=0x31,
	ICM42607T_ADDR_B0_APEX_DATA1=0x32,
	ICM42607T_ADDR_B0_APEX_DATA2=0x33,
	ICM42607T_ADDR_B0_APEX_DATA3=0x34,
	ICM42607T_ADDR_B0_INTF_CONFIG0=0x35,
	ICM42607T_ADDR_B0_INTF_CONFIG1=0x36,
	ICM42607T_ADDR_B0_INT_STATUS_DRDY=0x39,
	ICM42607T_ADDR_B0_INT_STATUS=0x3A,
	ICM42607T_ADDR_B0_INT_STATUS2=0x3B,
	ICM42607T_ADDR_B0_INT_STATUS3=0x3C,
	ICM42607T_ADDR_B0_FIFO_COUNTH=0x3D,
	ICM42607T_ADDR_B0_FIFO_COUNTL=0x3E,
	ICM42607T_ADDR_B0_FIFO_DATA=0x3F,
	ICM42607T_ADDR_B0_WHO_AM_I=0x75,
	ICM42607T_ADDR_B0_BLK_SEL_W=0x79,
	ICM42607T_ADDR_B0_MADDR_W=0x7A,
	ICM42607T_ADDR_B0_M_W=0x7B,
	ICM42607T_ADDR_B0_BLK_SEL_R=0x7C,
	ICM42607T_ADDR_B0_MADDR_R=0x7D,
	ICM42607T_ADDR_B0_M_R=0x7E
}ICM42607T_ADDRESS_BANK0;

enum
{
	ICM42607T_ADDR_B1_TMST_CONFIG1=0x00,
	ICM42607T_ADDR_B1_FIFO_CONFIG5=0x01,
	ICM42607T_ADDR_B1_FIFO_CONFIG6=0x02,
	ICM42607T_ADDR_B1_FSYNC_CONFIG=0x03,
	ICM42607T_ADDR_B1_INT_CONFIG0=0x04,
	ICM42607T_ADDR_B1_INT_CONFIG1=0x05,
	ICM42607T_ADDR_B1_SENSOR_CONFIG3=0x06,
	ICM42607T_ADDR_B1_ST_CONFIG=0x13,
	ICM42607T_ADDR_B1_SELFTEST=0x14,
	ICM42607T_ADDR_B1_INTF_CONFIG6=0x23,
	ICM42607T_ADDR_B1_INTF_CONFIG10=0x25,
	ICM42607T_ADDR_B1_INTF_CONFIG7=0x28,
	ICM42607T_ADDR_B1_OTP_CONFIG=0x2B,
	ICM42607T_ADDR_B1_INT_SOURCE6=0x2F,
	ICM42607T_ADDR_B1_INT_SOURCE7=0x30,
	ICM42607T_ADDR_B1_INT_SOURCE8=0x31,
	ICM42607T_ADDR_B1_INT_SOURCE9=0x32,
	ICM42607T_ADDR_B1_INT_SOURCE10=0x33,
	ICM42607T_ADDR_B1_APEX_CONFIG2=0x44,
	ICM42607T_ADDR_B1_APEX_CONFIG3=0x45,
	ICM42607T_ADDR_B1_APEX_CONFIG4=0x46,
	ICM42607T_ADDR_B1_APEX_CONFIG5=0x47,
	ICM42607T_ADDR_B1_APEX_CONFIG9=0x48,
	ICM42607T_ADDR_B1_APEX_CONFIG10=0x49,
	ICM42607T_ADDR_B1_APEX_CONFIG11=0x4A,
	ICM42607T_ADDR_B1_ACCEL_WOM_X_THR=0x4B,
	ICM42607T_ADDR_B1_ACCEL_WOM_Y_THR=0x4C,
	ICM42607T_ADDR_B1_ACCEL_WOM_Z_THR=0x4D,
	ICM42607T_ADDR_B1_OFFSET_USER0=0x4E,
	ICM42607T_ADDR_B1_OFFSET_USER1=0x4F,
	ICM42607T_ADDR_B1_OFFSET_USER2=0x50,
	ICM42607T_ADDR_B1_OFFSET_USER3=0x51,
	ICM42607T_ADDR_B1_OFFSET_USER4=0x52,
	ICM42607T_ADDR_B1_OFFSET_USER5=0x53,
	ICM42607T_ADDR_B1_OFFSET_USER6=0x54,
	ICM42607T_ADDR_B1_OFFSET_USER7=0x55,
	ICM42607T_ADDR_B1_OFFSET_USER8=0x56,
	ICM42607T_ADDR_B1_ST_STATUS1=0x63,
	ICM42607T_ADDR_B1_ST_STATUS2=0x64,
	ICM42607T_ADDR_B1_FDR_CONFIG=0x66,
	ICM42607T_ADDR_B1_APEX_CONFIG12=0x67
}ICM42607T_ADDRESS_BANK1;

enum
{
	ICM42607T_ADDR_B2_OTP_CTRL7=0x06
}ICM42607T_ADDRESS_BANK2;

enum
{
	ICM42607T_ADDR_B3_XA_ST_DATA=0x00,
	ICM42607T_ADDR_B3_YA_ST_DATA=0x01,
	ICM42607T_ADDR_B3_ZA_ST_DATA=0x02,
	ICM42607T_ADDR_B3_XG_ST_DATA=0x03,
	ICM42607T_ADDR_B3_YG_ST_DATA=0x04,
	ICM42607T_ADDR_B3_ZG_ST_DATA=0x05,
}ICM42607T_ADDRESS_BANK3;

typedef union
{
	struct
	{
		struct
		{
			union
			{
				struct
				{
					uint8_t Reserved0:3;
					uint8_t MCLK_RDY:1;
					uint8_t Reserved1:4;
				};
				uint8_t data8[1];
			}MCLK_RDY;
			union
			{
				struct
				{
					uint8_t SPI_MODE:1;
					uint8_t Reserved0:1;
					uint8_t SPI_AP_4WIRE:1;
					uint8_t Reserved1:5;
				};
				uint8_t data8[1];
			}DEVICE_CONFIG;
			union
			{
				struct
				{
					uint8_t Reserved0:2;
					uint8_t FIFO_FLUSH:1;
					uint8_t Reserved1:1;
					uint8_t SOFT_RESET_DEVICE_CONFIG:1;
					uint8_t Reserved2:3;
				};
				uint8_t data8[1];
			}SIGNAL_PATH_RESET;
			union
			{
				struct
				{
					uint8_t I3C_SDR_SLEW_RATE:3;
					uint8_t I3C_DDR_SLEW_RATE:3;
					uint8_t Reserved0:2;
				};
				uint8_t data8[1];
			}DRIVE_CONFIG1;
			union
			{
				struct
				{
					uint8_t ALL_SLEW_RATE:3;
					uint8_t I2C_SLEW_RATE:3;
					uint8_t Reserved0:2;
				};
				uint8_t data8[1];
			}DRIVE_CONFIG2;
			union
			{
				struct
				{
					uint8_t SPI_SLEW_RATE:3;
					uint8_t Reserved0:5;
				};
				uint8_t data8[1];
			}DRIVE_CONFIG3;
			union
			{
				struct
				{
					uint8_t INT1_POLARITY:1;
					uint8_t INT1_DRIVE_CIRCUIT:1;
					uint8_t INT1_MODE:1;
					uint8_t INT2_POLARITY:1;
					uint8_t INT2_DRIVE_CIRCUIT:1;
					uint8_t INT2_MODE:1;
					uint8_t Reserved0:2;
				};
				uint8_t data8[1];
			}INT_CONFIG;
			union
			{
				uint8_t TEMP_DATA;
				uint8_t data8[1];
			}TEMP_DATA1;
			union
			{
				uint8_t TEMP_DATA;
				uint8_t data8[1];
			}TEMP_DATA0;
			union
			{
				uint8_t ACCEL_DATA_X;
				uint8_t data8[1];
			}ACCEL_DATA_X1;
			union
			{
				uint8_t ACCEL_DATA_X;
				uint8_t data8[1];
			}ACCEL_DATA_X0;
			union
			{
				uint8_t ACCEL_DATA_Y;
				uint8_t data8[1];
			}ACCEL_DATA_Y1;
			union
			{
				uint8_t ACCEL_DATA_Y;
				uint8_t data8[1];
			}ACCEL_DATA_Y0;
			union
			{
				uint8_t ACCEL_DATA_Z;
				uint8_t data8[1];
			}ACCEL_DATA_Z1;
			union
			{
				uint8_t ACCEL_DATA_Z;
				uint8_t data8[1];
			}ACCEL_DATA_Z0;
			union
			{
				uint8_t GYRO_DATA_X;
				uint8_t data8[1];
			}GYRO_DATA_X1;
			union
			{
				uint8_t GYRO_DATA_X;
				uint8_t data8[1];
			}GYRO_DATA_X0;
			union
			{
				uint8_t GYRO_DATA_Y;
				uint8_t data8[1];
			}GYRO_DATA_Y1;
			union
			{
				uint8_t GYRO_DATA_Y;
				uint8_t data8[1];
			}GYRO_DATA_Y0;
			union
			{
				uint8_t GYRO_DATA_Z;
				uint8_t data8[1];
			}GYRO_DATA_Z1;
			union
			{
				uint8_t GYRO_DATA_Z;
				uint8_t data8[1];
			}GYRO_DATA_Z0;
			union
			{
				uint8_t TMST_FSYNC_DATA;
				uint8_t data8[1];
			}TMST_FSYNCH;
			union
			{
				uint8_t TMST_FSYNC_DATA;
				uint8_t data8[1];
			}TMST_FSYNCL;
			union
			{
				uint8_t FF_DUR;
				uint8_t data8[1];
			}APEX_DATA4;
			union
			{
				uint8_t FF_DUR;
				uint8_t data8[1];
			}APEX_DATA5;
			union
			{
				struct
				{
					uint8_t ACCEL_MODE:2;
					uint8_t GYRO_MODE:2;
					uint8_t IDLE:1;
					uint8_t Reserved0:2;
					uint8_t ACCEL_LP_CLK_SEL:1;
				};
				uint8_t data8[1];
			}PWR_MGMT0;
			union
			{
				struct
				{
					uint8_t GYRO_ODR:4;
					uint8_t Reserved0:1;
					uint8_t GYRO_UI_FS_SEL:2;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}GYRO_CONFIG0;
			union
			{
				struct
				{
					uint8_t ACCEL_ODR:4;
					uint8_t Reserved0:1;
					uint8_t ACCEL_UI_FS_SEL:2;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}ACCEL_CONFIG0;
			union
			{
				struct
				{
					uint8_t Reserved0:4;
					uint8_t TEMP_FILT_BW:3;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}TEMP_CONFIG0;
			union
			{
				struct
				{
					uint8_t GYRO_UI_FILT_BW:3;
					uint8_t Reserved0:5;
				};
				uint8_t data8[1];
			}GYRO_CONFIG1;
			union
			{
				struct
				{
					uint8_t ACCEL_UI_FILT_BW:3;
					uint8_t Reserved0:1;
					uint8_t ACCEL_UI_AVG:3;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}ACCEL_CONFIG1;
			union
			{
				struct
				{
					uint8_t DMP_MEM_RESET_EN:1;
					uint8_t Reserved0:1;
					uint8_t DMP_INIT_EN:1;
					uint8_t DMP_POWER_SAVE_EN:1;
					uint8_t Reserved1:4;
				};
				uint8_t data8[1];
			}APEX_CONFIG0;
			union
			{
				struct
				{
					uint8_t DMP_ODR:2;
					uint8_t Reserved0:1;
					uint8_t PED_ENABLE:1;
					uint8_t TILT_ENABLE:1;
					uint8_t FF_ENABLE:1;
					uint8_t SMD_ENABLE:1;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}APEX_CONFIG1;
			union
			{
				struct
				{
					uint8_t WOM_EN:1;
					uint8_t WOM_MODE:1;
					uint8_t WOM_INT_MODE:1;
					uint8_t WOM_INT_DUR:2;
					uint8_t Reserved0:3;
				};
				uint8_t data8[1];
			}WOM_CONFIG;
			union
			{
				struct
				{
					uint8_t FIFO_BYPASS:1;
					uint8_t FIFO_MODE:1;
					uint8_t Reserved0:6;
				};
				uint8_t data8[1];
			}FIFO_CONFIG1;
			union
			{
				uint8_t FIFO_WM;
				uint8_t data8[1];
			}FIFO_CONFIG2;
			union
			{
				struct
				{
					uint8_t FIFO_WM:4;
					uint8_t Reserved0:4;
				};
				uint8_t data8[1];
			}FIFO_CONFIG3;
			union
			{
				struct
				{
					uint8_t AGC_RDY_INT1_EN:1;
					uint8_t FIFO_FULL_INT1_EN:1;
					uint8_t FIFO_THS_INT1_EN:1;
					uint8_t DRDY_INT1_EN:1;
					uint8_t RESET_DONE_INT1_EN:1;
					uint8_t PLL_RDY_INT1_EN:1;
					uint8_t FSYNC_INT1_EN:1;
					uint8_t ST_INT1_EN:1;
				};
				uint8_t data8[1];
			}INT_SOURCE0;
			union
			{
				struct
				{
					uint8_t WOM_X_INT1_EN:1;
					uint8_t WOM_Y_INT1_EN:1;
					uint8_t WOM_Z_INT1_EN:1;
					uint8_t SMD_INT1_EN:1;
					uint8_t Reserved0:2;
					uint8_t I3C_PROTOCOL_ERROR_INT1_EN:1;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}INT_SOURCE1;
			union
			{
				struct
				{
					uint8_t AGC_RDY_INT2_EN:1;
					uint8_t FIFO_FULL_INT2_EN:1;
					uint8_t FIFO_THS_INT2_EN:1;
					uint8_t DRDY_INT2_EN:1;
					uint8_t RESET_DONE_INT2_EN:1;
					uint8_t PLL_RDY_INT2_EN:1;
					uint8_t FSYNC_INT2_EN:1;
					uint8_t ST_INT2_EN:1;
				};
				uint8_t data8[1];
			}INT_SOURCE3;
			union
			{
				struct
				{
					uint8_t WOM_X_INT2_EN:1;
					uint8_t WOM_Y_INT2_EN:1;
					uint8_t WOM_Z_INT2_EN:1;
					uint8_t SMD_INT2_EN:1;
					uint8_t Reserved0:2;
					uint8_t I3C_PROTOCOL_ERROR_INT2_EN:1;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}INT_SOURCE4;
			union
			{
				uint8_t FIFO_LOST_PKT_CNT;
				uint8_t data8[1];
			}FIFO_LOST_PKT0;
			union
			{
				uint8_t FIFO_LOST_PKT_CNT;
				uint8_t data8[1];
			}FIFO_LOST_PKT1;
			union
			{
				uint8_t STEP_CNT;
				uint8_t data8[1];
			}APEX_DATA0;
			union
			{
				uint8_t STEP_CNT;
				uint8_t data8[1];
			}APEX_DATA1;
			union
			{
				uint8_t STEP_CADENCE;
				uint8_t data8[1];
			}APEX_DATA2;
			union
			{
				struct
				{
					uint8_t ACTIVITY_CLASS:2;
					uint8_t DMP_IDLE:1;
					uint8_t Reserved0:5;
				};
				uint8_t data8[1];
			}APEX_DATA3;
			union
			{
				struct
				{
					uint8_t Reserved0:4;
					uint8_t SENSOR_DATA_ENDIAN:1;
					uint8_t FIFO_COUNT_ENDIAN:1;
					uint8_t FIFO_COUNT_FORMAT:1;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}INTF_CONFIG0;
			union
			{
				struct
				{
					uint8_t CLKSEL:2;
					uint8_t I3C_DDR_EN:1;
					uint8_t I3C_SDR_EN:1;
					uint8_t Reserved0:4;
				};
				uint8_t data8[1];
			}INTF_CONFIG1;
			union
			{
				struct
				{
					uint8_t DATA_RDY_INT:1;
					uint8_t Reserved0:7;
				};
				uint8_t data8[1];
			}INT_STATUS_DRDY;
			union
			{
				struct
				{
					uint8_t AGC_RDY_INT:1;
					uint8_t FIFO_FULL_INT:1;
					uint8_t FIFO_THS_INT:1;
					uint8_t Reserved0:1;
					uint8_t RESET_DONE_INT:1;
					uint8_t PLL_RDY_INT:1;
					uint8_t FSYNC_INT:1;
					uint8_t ST_INT:1;
				};
				uint8_t data8[1];
			}INT_STATUS;
			union
			{
				struct
				{
					uint8_t WOM_Z_INT:1;
					uint8_t WOM_Y_INT:1;
					uint8_t WOM_X_INT:1;
					uint8_t SMD_INT:1;
					uint8_t Reserved0:4;
				};
				uint8_t data8[1];
			}INT_STATUS2;
			union
			{
				struct
				{
					uint8_t Reserved0:1;
					uint8_t LOWG_DET_INT:1;
					uint8_t FF_DET_INT:1;
					uint8_t TILT_DET_INT:1;
					uint8_t STEP_CNT_OVF_INT:1;
					uint8_t STEP_DET_INT:1;
					uint8_t Reserved1:2;
				};
				uint8_t data8[1];
			}INT_STATUS3;
			union
			{
				uint8_t FIFO_COUNT;
				uint8_t data8[1];
			}FIFO_COUNTH;
			union
			{
				uint8_t FIFO_COUNT;
				uint8_t data8[1];
			}FIFO_COUNTL;
			union
			{
				uint8_t FIFO_DATA;
				uint8_t data8[1];
			}FIFO_DATA;
			union
			{
				uint8_t WHOAMI;
				uint8_t data8[1];
			}WHO_AM_I;
			union
			{
				uint8_t BLK_SEL_W;
				uint8_t data8[1];
			}BLK_SEL_W;
			union
			{
				uint8_t MADDR_W;
				uint8_t data8[1];
			}MADDR_W;
			union
			{
				uint8_t M_W;
				uint8_t data8[1];
			}M_W;
			union
			{
				uint8_t BLK_SEL_R;
				uint8_t data8[1];
			}BLK_SEL_R;
			union
			{
				uint8_t MADDR_R;
				uint8_t data8[1];
			}MADDR_R;
			union
			{
				uint8_t M_R;
				uint8_t data8[1];
			}M_R;
		}BANK0;
		struct
		{
			union
			{
				struct
				{
					uint8_t TMST_EN:1;
					uint8_t TMST_FSYNC_EN:1;
					uint8_t TMST_DELTA_EN:1;
					uint8_t TMST_RES:1;
					uint8_t TMST_ON_SREG_EN:1;
					uint8_t Reserved0:3;
				};
				uint8_t data8[1];
			}TMST_CONFIG1;
			union
			{
				struct
				{
					uint8_t FIFO_ACCEL_EN:1;
					uint8_t FIFO_GYRO_EN:1;
					uint8_t FIFO_TMST_FSYNC_EN:1;
					uint8_t FIFO_HIRES_EN:1;
					uint8_t FIFO_RESUME_PARTIAL_RD:1;
					uint8_t FIFO_WM_GT_TH:1;
					uint8_t Reserved0:2;
				};
				uint8_t data8[1];
			}FIFO_CONFIG5;
			union
			{
				struct
				{
					uint8_t RCOSC_REQ_ON_FIFO_THS_DIS:1;
					uint8_t Reserved0:3;
					uint8_t FIFO_EMPTY_INDICATOR_DIS:1;
					uint8_t Reserved1:3;
				};
				uint8_t data8[1];
			}FIFO_CONFIG6;
			union
			{
				struct
				{
					uint8_t FSYNC_POLARITY:1;
					uint8_t FSYNC_UI_FLAG_CLEAR_SEL:1;
					uint8_t Reserved0:2;
					uint8_t FSYNC_UI_SEL:3;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}FSYNC_CONFIG;
			union
			{
				struct
				{
					uint8_t FIFO_FULL_INT_CLEAR:2;
					uint8_t FIFO_THS_INT_CLEAR:2;
					uint8_t UI_DRDY_INT_CLEAR:2;
					uint8_t Reserved0:2;
				};
				uint8_t data8[1];
			}INT_CONFIG0;
			union
			{
				struct
				{
					uint8_t Reserved0:4;
					uint8_t INT_ASYNC_RESET:1;
					uint8_t Reserved1:1;
					uint8_t INT_TPULSE_DURATION:1;
					uint8_t Reserved2:1;
				};
				uint8_t data8[1];
			}INT_CONFIG1;
			union
			{
				struct
				{
					uint8_t Reserved0:6;
					uint8_t APEX_DISABLE:1;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}SENSOR_CONFIG3;
			union
			{
				struct
				{
					uint8_t GYRO_ST_LIM:3;
					uint8_t ACCEL_ST_LIM:3;
					uint8_t ST_NUMBER_SAMPLE:1;
					uint8_t Reserved0:1;
				};
				uint8_t data8[1];
			}ST_CONFIG;
			union
			{
				struct
				{
					uint8_t Reserved0:6;
					uint8_t ACCEL_ST_EN:1;
					uint8_t GYRO_ST_EN:1;
				};
				uint8_t data8[1];
			}SELFTEST;
			union
			{
				struct
				{
					uint8_t Reserved0:2;
					uint8_t I3C_IBI_EN:1;
					uint8_t I3C_IBI_BYTE_EN:1;
					uint8_t I3C_TIMEOUT_EN:1;
					uint8_t Reserved1:3;
				};
				uint8_t data8[1];
			}INTF_CONFIG6;
			union
			{
				struct
				{
					uint8_t Reserved0:7;
					uint8_t ASYNCTIME0_DIS:1;
				};
				uint8_t data8[1];
			}INTF_CONFIG10;
			union
			{
				struct
				{
					uint8_t Reserved0:3;
					uint8_t I3C_DDR_WR_MODE:1;
					uint8_t Reserved1:4;
				};
				uint8_t data8[1];
			}INTF_CONFIG7;
			union
			{
				struct
				{
					uint8_t Reserved0:2;
					uint8_t OTP_COPY_MODE:2;
					uint8_t Reserved1:4;
				};
				uint8_t data8[1];
			}OTP_CONFIG;
			union
			{
				struct
				{
					uint8_t Reserved0:3;
					uint8_t TILT_DET_INT1_EN:1;
					uint8_t STEP_CNT_OFL_INT1_EN:1;
					uint8_t STEP_DET_INT1_EN:1;
					uint8_t LOWG_INT1_EN:1;
					uint8_t FF_INT1_EN:1;
				};
				uint8_t data8[1];
			}INT_SOURCE6;
			union
			{
				struct
				{
					uint8_t Reserved0:3;
					uint8_t TILT_DET_INT2_EN:1;
					uint8_t STEP_CNT_OFL_INT2_EN:1;
					uint8_t STEP_DET_INT2_EN:1;
					uint8_t INT2_EN:1;
					uint8_t FF_INT2_EN:1;
				};
				uint8_t data8[1];
			}INT_SOURCE7;
			union
			{
				struct
				{
					uint8_t AGC_RDY_IBI_EN:1;
					uint8_t FIFO_FULL_IBI_EN:1;
					uint8_t FIFO_THS_IBI_EN:1;
					uint8_t UI_DRDY_IBI_EN:1;
					uint8_t PLL_RDY_IBI_EN:1;
					uint8_t FSYNC_IBI_EN:1;
					uint8_t Reserved0:2;
				};
				uint8_t data8[1];
			}INT_SOURCE8;
			union
			{
				struct
				{
					uint8_t ST_DONE_IBI_EN:1;
					uint8_t WOM_X_IBI_EN:1;
					uint8_t WOM_Y_IBI_EN:1;
					uint8_t WOM_Z_IBI_EN:1;
					uint8_t SMD_IBI_EN:1;
					uint8_t LOWG_IBI_EN:1;
					uint8_t FF_IBI_EN:1;
					uint8_t I3C_PROTOCOL_ERROR_IBI_EN:1;
				};
				uint8_t data8[1];
			}INT_SOURCE9;
			union
			{
				struct
				{
					uint8_t Reserved0:3;
					uint8_t TILT_DET_IBI_EN:1;
					uint8_t STEP_CNT_OFL_IBI_EN:1;
					uint8_t STEP_DET_IBI_EN:1;
					uint8_t Reserved1:2;
				};
				uint8_t data8[1];
			}INT_SOURCE10;
			union
			{
				struct
				{
					uint8_t DMP_POWER_SAVE_TIME_SEL:4;
					uint8_t LOW_ENERGY_AMP_TH_SEL:4;
				};
				uint8_t data8[1];
			}APEX_CONFIG2;
			union
			{
				struct
				{
					uint8_t PED_STEP_CNT_TH_SEL:4;
					uint8_t PED_AMP_TH_SEL:4;
				};
				uint8_t data8[1];
			}APEX_CONFIG3;
			union
			{
				struct
				{
					uint8_t PED_HI_EN_TH_SEL:2;
					uint8_t PED_SB_TIMER_TH_SEL:3;
					uint8_t PED_STEP_DET_TH_SEL:3;
				};
				uint8_t data8[1];
			}APEX_CONFIG4;
			union
			{
				struct
				{
					uint8_t HIGHG_PEAK_TH_HYST_SEL:3;
					uint8_t LOWG_PEAK_TH_HYST_SEL:3;
					uint8_t TILT_WAIT_TIME_SEL:2;
				};
				uint8_t data8[1];
			}APEX_CONFIG5;
			union
			{
				struct
				{
					uint8_t SENSITIVITY_MODE:1;
					uint8_t SMD_SENSITIVITY_SEL:3;
					uint8_t FF_DEBOUNCE_DURATION_SEL:4;
				};
				uint8_t data8[1];
			}APEX_CONFIG9;
			union
			{
				struct
				{
					uint8_t LOWG_TIME_TH_SEL:3;
					uint8_t LOWG_PEAK_TH_SEL:5;
				};
				uint8_t data8[1];
			}APEX_CONFIG10;
			union
			{
				struct
				{
					uint8_t HIGHG_TIME_TH_SEL:3;
					uint8_t HIGHG_PEAK_TH_SEL:5;
				};
				uint8_t data8[1];
			}APEX_CONFIG11;
			union
			{
				uint8_t WOM_X_TH;
				uint8_t data8[1];
			}ACCEL_WOM_X_THR;
			union
			{
				uint8_t WOM_Y_TH;
				uint8_t data8[1];
			}ACCEL_WOM_Y_THR;
			union
			{
				uint8_t WOM_Z_TH;
				uint8_t data8[1];
			}ACCEL_WOM_Z_THR;
			union
			{
				uint8_t GYRO_X_OFFUSER;
				uint8_t data8[1];
			}OFFSET_USER0;
			union
			{
				struct
				{
					uint8_t GYRO_X_OFFUSER:4;
					uint8_t GYRO_Y_OFFUSER:4;
				};
				uint8_t data8[1];
			}OFFSET_USER1;
			union
			{
				uint8_t GYRO_Y_OFFUSER;
				uint8_t data8[1];
			}OFFSET_USER2;
			union
			{
				uint8_t GYRO_Z_OFFUSER;
				uint8_t data8[1];
			}OFFSET_USER3;
			union
			{
				struct
				{
					uint8_t GYRO_Z_OFFUSER:4;
					uint8_t ACCEL_X_OFFUSER:4;
				};
				uint8_t data8[1];
			}OFFSET_USER4;
			union
			{
				uint8_t ACCEL_X_OFFUSER;
				uint8_t data8[1];
			}OFFSET_USER5;
			union
			{
				uint8_t ACCEL_Y_OFFUSER;
				uint8_t data8[1];
			}OFFSET_USER6;
			union
			{
				struct
				{
					uint8_t ACCEL_Y_OFFUSER:4;
					uint8_t ACCEL_Z_OFFUSER:4;
				};
				uint8_t data8[1];
			}OFFSET_USER7;
			union
			{
				uint8_t ACCEL_Z_OFFUSER;
				uint8_t data8[1];
			}OFFSET_USER8;
			union
			{
				struct
				{
					uint8_t Reserved0:1;
					uint8_t AX_ST_PASS:1;
					uint8_t AY_ST_PASS:1;
					uint8_t AZ_ST_PASS:1;
					uint8_t ACCEL_ST_DONE:1;
					uint8_t ACCEL_ST_PASS:1;
					uint8_t Reserved1:2;
				};
				uint8_t data8[1];
			}ST_STATUS1;
			union
			{
				struct
				{
					uint8_t Reserved0:1;
					uint8_t GX_ST_PASS:1;
					uint8_t GY_ST_PASS:1;
					uint8_t GZ_ST_PASS:1;
					uint8_t GYRO_ST_DONE:1;
					uint8_t GYRO_ST_PASS:1;
					uint8_t ST_INCOMPLETE:1;
					uint8_t Reserved1:1;
				};
				uint8_t data8[1];
			}ST_STATUS2;
			union
			{
				struct
				{
					uint8_t FDR_SEL:4;
					uint8_t Reserved0:4;
				};
				uint8_t data8[1];
			}FDR_CONFIG;
			union
			{
				struct
				{
					uint8_t FF_MIN_DURATION_SEL:4;
					uint8_t FF_MAX_DURATION_SEL:4;
				};
				uint8_t data8[1];
			}APEX_CONFIG12;
		}BANK1;
		struct
		{
			union
			{
				struct
				{
					uint8_t Reserved0:1;
					uint8_t OTP_PWR_DOWN:1;
					uint8_t Reserved1:1;
					uint8_t OTP_RELOAD:1;
					uint8_t Reserved2:4;
				};
				uint8_t data8[1];
			}OTP_CTRL7;
		}BANK2;
		struct
		{
			union
			{
				uint8_t XA_ST_DATA;
				uint8_t data8[1];
			}XA_ST_DATA;
			union
			{
				uint8_t YA_ST_DATA;
				uint8_t data8[1];
			}YA_ST_DATA;
			union
			{
				uint8_t ZA_ST_DATA;
				uint8_t data8[1];
			}ZA_ST_DATA;
			union
			{
				uint8_t XG_ST_DATA;
				uint8_t data8[1];
			}XG_ST_DATA;
			union
			{
				uint8_t YG_ST_DATA;
				uint8_t data8[1];
			}YG_ST_DATA;
			union
			{
				uint8_t ZG_ST_DATA;
				uint8_t data8[1];
			}ZG_ST_DATA;
		}BANK3;
	};
	uint8_t data8[111];
}ICM42607T_REGISTER_MAP;

#endif /* INC_ICM42607T_H_ */
