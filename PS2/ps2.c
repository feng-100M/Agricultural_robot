#include "ps2.h"
#include "vl53l0x_i2c.h"

/* 原 TIM7 每计数为 2 微秒，手柄半周期使用 16 个计数。 */
#define PS2_HALF_CLOCK_US (32U)

#define CS_H()   HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,1)    //CS拉高(别名ATT)
#define CS_L()   HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,0)    //CS拉低(别名ATT)
#define CLK_H()  HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,1)    //时钟拉高
#define CLK_L()  HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,0)   //时钟拉低
#define CMD_H()  HAL_GPIO_WritePin(CMD_GPIO_Port,CMD_Pin,1)   //命令位高
#define CMD_L()  HAL_GPIO_WritePin(CMD_GPIO_Port,CMD_Pin,0)   //命令位低

const  uint8_t PS2_cmnd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};   //请求获取数据命令                         
uint8_t PS2_data[9] = {0};  //接收的数据
JOYSTICK_TypeDef my_joystick;  //手柄键值结构体

/* 在 GPIO 初始化后拉高片选，使手柄总线空闲。此步骤不检测手柄是否连接。 */
void AX_PS2_Init(void)
{
	//关闭PS2手柄使能
	CS_H();
}


//void AX_Delayus(uint16_t us)    //定义hal库us级延迟
//{
//	uint32_t startval, tickn, delays, wait;
// 
//	startval = SysTick->VAL;
//	tickn = HAL_GetTick();
//	delays = us * 240;
//	if (delays > startval)
//	{
//		while (HAL_GetTick() == tickn)
//		{
// 
//		}
//		wait = 240000 + startval - delays;
//		while (wait < SysTick->VAL)
//		{
// 
//		}
//	}
//	else
//	{
//		wait = startval - delays;
//		while (wait < SysTick->VAL && HAL_GetTick() == tickn)
//		{
// 
//		}
//	}
//}

/* 在已选中手柄的事务内低位先行发送 cmd，同时采样并返回一个字节。
 * 位时序读取连续运行的 TIM7，调用期间须独占手柄引脚。
 */
static uint8_t PS2_ReadWriteData(uint8_t cmd)
{
	volatile uint8_t res = 0;
	volatile uint8_t ref;
	
	//写入命令，并读取一个1字节数据
	for(ref = 0x01; ref > 0x00; ref <<= 1)
	{
		////输出一位数据
		if(ref&cmd)
			CMD_H();
		else
			CMD_L();
		
		CLK_L();
		Delay_us(PS2_HALF_CLOCK_US);
		
		//读取一位数据
		if(HAL_GPIO_ReadPin(DAT_GPIO_Port,DAT_Pin))
			res |= ref; 
		CLK_H();
		Delay_us(PS2_HALF_CLOCK_US);		
	}

	//返回读出数据
    return res;	
}

/* 同步交换九个字节，把模式、按下为 1 的按键位和摇杆原始值写入 JoystickStruct。
 * 调用方提供可写结构并串行访问总线及共享缓冲。此接口不校验应答，断线时也会写入采样值。
 */
void AX_PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct)
{
	uint8_t i;
	
	//使能手柄
	CS_L();
	
	//读取PS2数据
	for(i=0; i<9; i++)
	{
		PS2_data[i] = PS2_ReadWriteData(PS2_cmnd[i]);
	}
	
	//关闭使能
	CS_H();

	//数值传递
	JoystickStruct->mode = PS2_data[1];
	JoystickStruct->btn1 = ~PS2_data[3];
	JoystickStruct->btn2 = ~PS2_data[4];
	JoystickStruct->RJoy_LR = PS2_data[5];
	JoystickStruct->RJoy_UD = PS2_data[6];
	JoystickStruct->LJoy_LR = PS2_data[7];
	JoystickStruct->LJoy_UD = PS2_data[8];
}
