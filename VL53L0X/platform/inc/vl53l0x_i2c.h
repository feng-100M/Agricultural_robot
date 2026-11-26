#ifndef __VL53L0_I2C_H
#define __VL53L0_I2C_H
#include "stm32h7xx.h"
#include "stm32h7xx_hal_tim.h"

 //  #define    delay_us(us)           CPU_TS_Tmr_Delay_US(us) 
void Delay_us(uint16_t us);
/* 定义读写SCL和SDA的宏，已增加代码的可移植性和可阅读性 */
	#define VL_I2C_SCL_1()  HAL_GPIO_WritePin(VL5310X_I2C_GPIO_PORT,VL5310X_I2C_SCL_PIN,GPIO_PIN_SET)		/* SCL = 1 */
	#define VL_I2C_SCL_0()  HAL_GPIO_WritePin(VL5310X_I2C_GPIO_PORT,VL5310X_I2C_SCL_PIN,GPIO_PIN_RESET)		/* SCL = 0 */
	#define VL_I2C_SDA_1()  HAL_GPIO_WritePin(VL5310X_I2C_GPIO_PORT, VL5310X_I2C_SDA_PIN,GPIO_PIN_SET)		/* SDA = 1 */
	#define VL_I2C_SDA_0()  HAL_GPIO_WritePin(VL5310X_I2C_GPIO_PORT,VL5310X_I2C_SDA_PIN,GPIO_PIN_RESET)		/* SDA = 0 */
	
	#define VL_I2C_SDA_READ()  HAL_GPIO_ReadPin(VL5310X_I2C_GPIO_PORT, VL5310X_I2C_SDA_PIN)	/* 读SDA口线状态 */


/*GPIOB模式寄存器配置*/
	#define VL_SDA_IN()           GPIOB->MODER &= ~(1 << 2*9)
	#define VL_SDA_OUT()          GPIOB->MODER |=  (1 << 2*9)
	

//状态
#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

/* 定义I2C总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */

#define VL5310X_I2C_GPIO_PORT				GPIOB			/* GPIO端口 */
#define VL5310X_I2C_GPIO_CLK()			   __HAL_RCC_GPIOB_CLK_ENABLE()		/* GPIO端口时钟 */
#define VL5310X_I2C_SCL_PIN					GPIO_PIN_8  /* 连接到SCL时钟线的GPIO */
#define VL5310X_I2C_SDA_PIN					GPIO_PIN_9		/* 连接到SDA数据线的GPIO */


///IIC操作函数
void VL53L0X_i2c_init(void);//初始化IIC的IO口

uint8_t VL53L0X_write_byte(uint8_t address,uint8_t index,uint8_t data);              //IIC写一个8位数据
uint8_t VL53L0X_write_word(uint8_t address,uint8_t index,uint16_t data);             //IIC写一个16位数据
uint8_t VL53L0X_write_dword(uint8_t address,uint8_t index,uint32_t data);            //IIC写一个32位数据
uint8_t VL53L0X_write_multi(uint8_t address, uint8_t index,uint8_t *pdata,uint16_t count);//IIC连续写

uint8_t VL53L0X_read_byte(uint8_t address,uint8_t index,uint8_t *pdata);             //IIC读一个8位数据
uint8_t VL53L0X_read_word(uint8_t address,uint8_t index,uint16_t *pdata);            //IIC读一个16位数据
uint8_t VL53L0X_read_dword(uint8_t address,uint8_t index,uint32_t *pdata);           //IIC读一个32位数据
uint8_t VL53L0X_read_multi(uint8_t address,uint8_t index,uint8_t *pdata,uint16_t count);  //IIC连续读

#endif 


