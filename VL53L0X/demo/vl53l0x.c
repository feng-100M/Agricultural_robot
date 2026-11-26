#include "vl53l0x.h"
#include <stdio.h>
#include "stm32h7xx_hal_tim.h"


_VL53L0X_adjust Vl53l0x_adjust; 
_VL53L0X_adjust Vl53l0x_data;   
VL53L0X_Dev_t vl53l0x_dev;                    /*设备I2C数据参数*/
VL53L0X_Dev_t vl53l0x_dev1;   
VL53L0X_Dev_t vl53l0x_dev2;   
VL53L0X_DeviceInfo_t vl53l0x_dev_info;        /*设备ID版本信息*/
uint8_t AjustOK=0;                            /*校准标志位*/
VL53L0X_RangingMeasurementData_t vl53l0x_data;/*测距测量结构体*/
uint16_t Distance_data[4] = {0};                        /*保存测距数据*/


/*0：默认;1:高精度;2:长距离;3:高速*/
mode_data Mode_data[]=
{
  {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	  33000,
	  14,
	  10}, /*默认*/
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	  200000, 
	  14,
	  10},/*高精度*/
		
  {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	  33000,
	  18,
	  14},/*长距离*/
	
  {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	  20000,
	  14,
	  10},/*高速*/		
};

/**
  * @brief  打印错误信息
  * @param  Status详情看VL53L0X_Error参数的定义
  * @retval 无
  */
void print_pal_error(VL53L0X_Error Status)
{	
	char buf[VL53L0X_MAX_STRING_LENGTH];
	
  /*根据Status状态获取错误信息字符串*/
	VL53L0X_GetPalErrorString(Status,buf);          
	
  /*打印状态和错误信息*/
  printf("API Status: %i : %s\r\n",Status, buf);  	
}


/* 先核对默认地址上的型号，再改址并在新地址回读验证；失败时返回首次错误。
 * 改址命令成功后 dev 即记录新写地址，即使随后验证失败。
 */
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev, uint8_t newaddr)
{
    VL53L0X_Error status;
    uint16_t id;

    if (dev == NULL) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
    if (dev->I2cDevAddr == newaddr) {
        return VL53L0X_ERROR_NONE;
    }
    status = VL53L0X_WrByte(dev, 0x88, 0x00);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }
    status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &id);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }
    if (id != 0xEEAAU) {
        return VL53L0X_ERROR_NOT_SUPPORTED;
    }
    status = VL53L0X_SetDeviceAddress(dev, newaddr);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }
    dev->I2cDevAddr = newaddr;
    status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &id);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }
    return id == 0xEEAAU ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_NOT_SUPPORTED;
}

/**
  * @brief  vl53l0x复位函数
  * @param  dev:设备I2C参数结构体
  * @retval 无
  */
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
  
  /*保存设备原I2C地址*/
	addr = dev->I2cDevAddr; 
  /*失能VL53L0X*/
  LV_DISABLE(LV_XSH_PORT ,LV_XSH_PIN) ;
	delay_ms(30);
  
  /*使能VL53L0X,让传感器处于工作(I2C地址会恢复默认0X52)*/
	LV_ENABLE(LV_XSH_PORT ,LV_XSH_PIN) ; 
	delay_ms(30);	
  
	dev->I2cDevAddr=0x52;
  
  /*设置VL53L0X传感器原来上电前原I2C地址*/
	vl53l0x_Addr_set(dev,addr);          
	VL53L0X_DataInit(dev);	
}


/* 为指定位置的器件切换 XSHUT、改址并读取器件信息。其余未改址器件须保持关闭。
 * dev 由调用方持有，初始化和后续测距须串行使用总线，失败返回驱动错误。
 */
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev, uint8_t vl53l0_x_id)
{
    static const struct {
        GPIO_TypeDef *port;
        uint16_t pin;
        uint8_t address;
    } devices[] = {
        {LV_XSH_PORT, LV_XSH_PIN, 0x54U},
        {GPIOE, GPIO_PIN_3, 0x56U},
        {GPIOD, GPIO_PIN_11, 0x58U}
    };
    VL53L0X_Error status;

    if (dev == NULL || vl53l0_x_id >= sizeof(devices) / sizeof(devices[0])) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
    dev->I2cDevAddr = VL53L0X_Addr;
    dev->comms_type = 1;
    dev->comms_speed_khz = 400;
    HAL_GPIO_WritePin(devices[vl53l0_x_id].port, devices[vl53l0_x_id].pin, GPIO_PIN_RESET);
    delay_ms(30);
    HAL_GPIO_WritePin(devices[vl53l0_x_id].port, devices[vl53l0_x_id].pin, GPIO_PIN_SET);
    delay_ms(30);
    status = vl53l0x_Addr_set(dev, devices[vl53l0_x_id].address);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }
    status = VL53L0X_DataInit(dev);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }
    delay_ms(2);
    status = VL53L0X_GetDeviceInfo(dev, &vl53l0x_dev_info);
    if (status == VL53L0X_ERROR_NONE) {
        AjustOK = Vl53l0x_data.adjustok == 0xAA;
    }
    return status;
}

/* 每次初始化先关闭三路器件，确保仅有一台设备响应默认地址 0x52。
 * PE3 和 PD11 已由 MX_GPIO_Init 配置；此处补齐 PE0。任一步失败即返回，允许整组重试。
 */
VL53L0X_Error vl53l0x_init_All(void)
{
    GPIO_InitTypeDef gpio = {0};
    VL53L0X_Dev_t *devices[] = {&vl53l0x_dev, &vl53l0x_dev1, &vl53l0x_dev2};
    uint8_t i;

    LV_XSH_PORT_CLK();
    HAL_GPIO_WritePin(LV_XSH_PORT, LV_XSH_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
    gpio.Pin = LV_XSH_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LV_XSH_PORT, &gpio);
    VL53L0X_i2c_init();

    for (i = 0U; i < sizeof(devices) / sizeof(devices[0]); ++i) {
        VL53L0X_Error status = vl53l0x_init(devices[i], i);
        if (status != VL53L0X_ERROR_NONE) {
            return status;
        }
        status = vl53l0x_set_mode(devices[i], Default_Mode);
        if (status != VL53L0X_ERROR_NONE) {
            return status;
        }
    }
    return VL53L0X_ERROR_NONE;
}

/**
  * @brief  VL53L0X主测试程序
  * @param  无
  * @retval 无
  */
void vl53l0x_test(void)
{  
   /*vl53l0x初始化*/  
//	 while(vl53l0x_init(&vl53l0x_dev)) 
//	 {
//		 printf("Init Error\r\n");
//	 }
//	 printf("Init Success\r\n");

//	 while(1)
//	 { 
//      vl53l0x_general_test(&vl53l0x_dev);                 
//	 }
}

/* 为 dev 配置校准参数、测距时间预算和脉冲周期。mode 必须为 0 至 3，依次为默认、高精度、长距离和高速。
 * 同步访问设备，返回驱动状态。出错前已经生效的寄存器配置可能被保留。
 */
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,uint8_t mode)
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;

  /*复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)*/
  //vl53l0x_reset(dev);                                                                                                
  if (dev == NULL || mode >= sizeof(Mode_data) / sizeof(Mode_data[0])) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  status = VL53L0X_StaticInit(dev);
  if (status != VL53L0X_ERROR_NONE) goto error;

  /*已校准好了,写入校准值*/
  if(AjustOK!=0)                                                                                                     
  {
    /*设定Spads校准值*/
    status= VL53L0X_SetReferenceSpads(dev,Vl53l0x_data.refSpadCount,
                                      Vl53l0x_data.isApertureSpads);                  
    if(status!=VL53L0X_ERROR_NONE) goto error;	
    delay_ms(2);
    
    /*设定Ref校准值*/
    status= VL53L0X_SetRefCalibration(dev,Vl53l0x_data.VhvSettings,
                                      Vl53l0x_data.PhaseCal);                          
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    
    /*设定偏移校准值*/
    status=VL53L0X_SetOffsetCalibrationDataMicroMeter(dev,
                                      Vl53l0x_data.OffsetMicroMeter);                           
    if(status!=VL53L0X_ERROR_NONE) goto error; 
    delay_ms(2);
    
    /*设定串扰校准值*/
    status=VL53L0X_SetXTalkCompensationRateMegaCps(dev,
                              Vl53l0x_data.XTalkCompensationRateMegaCps);                  
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);		  
  }
  else
  {
    /*Ref参考校准*/
    status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);                                           
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    
    /*执行参考SPAD管理*/
    status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount,
                                                &isApertureSpads);                                
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);		 	 
  }
  
  /*使能单次测量模式*/
  status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_SINGLE_RANGING);                                             
  if(status!=VL53L0X_ERROR_NONE) goto error;
  delay_ms(2);
  
  /*使能SIGMA范围检查*/
  status = VL53L0X_SetLimitCheckEnable(dev,
                                       VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                       1);                                 
  if(status!=VL53L0X_ERROR_NONE) goto error;
  delay_ms(2);
  
   /*使能信号速率范围检查*/
  status = VL53L0X_SetLimitCheckEnable(dev,
                                       VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                       1);                          
  if(status!=VL53L0X_ERROR_NONE) goto error;
  delay_ms(2);
  
  /*设定SIGMA范围*/
  status = VL53L0X_SetLimitCheckValue(dev,
                                     VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                     Mode_data[mode].sigmaLimit);         
  if(status!=VL53L0X_ERROR_NONE) goto error;
  delay_ms(2);
  
  /*设定信号速率范围范围*/
  status = VL53L0X_SetLimitCheckValue(dev,
                                      VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                      Mode_data[mode].signalLimit);  
  if(status!=VL53L0X_ERROR_NONE) goto error;
  delay_ms(2);
  
  /*设定完整测距最长时间*/
  status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,
                                              Mode_data[mode].timingBudget);                         
  if(status!=VL53L0X_ERROR_NONE) goto error;
  delay_ms(2);
  
  /*设定VCSEL脉冲周期*/
  status = VL53L0X_SetVcselPulsePeriod(dev, 
                                      VL53L0X_VCSEL_PERIOD_PRE_RANGE, 
                                      Mode_data[mode].preRangeVcselPeriod);   
  if(status!=VL53L0X_ERROR_NONE) goto error;
  delay_ms(2);
  
  /*设定VCSEL脉冲周期范围*/
  status = VL53L0X_SetVcselPulsePeriod(dev,
                                       VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 
                                       Mode_data[mode].finalRangeVcselPeriod);

  error:/*错误信息*/
  if(status!=VL53L0X_ERROR_NONE)
  {
    print_pal_error(status);
    return status;
  }
  return status;	
}	

/* 对 dev 单次测距，将结果写入 pdata。buf 须容纳 VL53L0X_MAX_STRING_LENGTH 字节，id 取 0 至 3。
 * 测量 API 报错时直接返回其状态；成功时写状态说明并更新 Distance_data[id]（毫米）。
 * 调用方仍须检查 pdata->RangeStatus 才能采用该距离。
 */
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,
                                        VL53L0X_RangingMeasurementData_t *pdata,
                                        char *buf,uint8_t id)
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  uint8_t RangeStatus;

  /*执行单次测距并获取测距测量数据*/
  status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);   
  if(status !=VL53L0X_ERROR_NONE) return status;
  
  /*获取当前测量状态*/
  RangeStatus = pdata->RangeStatus;                              
  memset(buf,0x00,VL53L0X_MAX_STRING_LENGTH);
  /*根据测量状态读取状态字符串*/
  VL53L0X_GetRangeStatusString(RangeStatus,buf);                 
  /*保存最近一次测距测量数据*/
	
  Distance_data[id] = pdata->RangeMilliMeter;                         

  return status;
}


/**
  * @brief  启动普通测量
  * @param  dev：设备I2C参数结构体
  * @param  mode模式配置 0:默认;1:高精度;2:长距离
  * @retval 无
  */
void vl53l0x_general_start(VL53L0X_Dev_t *dev,uint8_t mode,uint8_t id)
{
  /*测试模式字符串字符缓冲区*/
  static char buf[VL53L0X_MAX_STRING_LENGTH]; 
  /*工作状态*/
	VL53L0X_Error Status=VL53L0X_ERROR_NONE;                       
	
  /*配置测量模式*/
	
//	while()                              
//	{
//		printf("Mode Set Error!!!\r\n");
//    delay_ms(500);
//		i++;
//		if(i==2)
//    {
//      return;
//    }
//	}

  /*执行一次测量	*/
	if(Status == vl53l0x_start_single_test(dev,&vl53l0x_data,buf,id))
	{
    /*打印测量距离*/
    //printf("distance: %4imm\r\n",Distance_data);
  } 
}


/**
  * @brief  工作模式信息函数
  * @param  无
  * @retval 无
  */
void Show_GenTask_Message(void)
{
//  printf("   指令   ------      功能 \r\n");
//  printf("    0     ------    默认测量模式 \r\n");
//  printf("    1     ------    高精度测量模式 \r\n"); 
//  printf("    2     ------    长距离测量模式 \r\n");
//  printf("    3     ------    高速测量模式 \r\n"); 
//  printf("***输入数字指令后，按回车键再发送*** \r\n");    
}


/**
  * @brief  vl53l0x普通测量模式测试
  * @param  dev:设备I2C参数结构体
  * @retval 无
  */
//void vl53l0x_general_test(VL53L0X_Dev_t *dev)
//{
//  uint8_t  ch;
//  
//  /*显示普通测量模式*/
//	Show_GenTask_Message(); 
//	while(1)
//	{	
//    scanf("%c",&ch);
//    printf("接收到字符：%c\r\n",ch);
//    ch = ch - 0x30;
//    if(ch<=3)
//		{
//      vl53l0x_general_start(dev,ch);
//      Show_GenTask_Message();
//    }		
//    else  
//    {
//      /*如果不是指定指令字符，打印提示信息*/
//      printf("请输入合法指令！\r\n"); 
//      Show_GenTask_Message();      
//    }
//  }
//}


void vl53l0x_general_test(VL53L0X_Dev_t *dev,uint8_t id)
{

  /*显示普通测量模式*/
	vl53l0x_general_start(dev,0,id);
}


/*********************************************END OF FILE**********************/
