#include "vl53l0x_platform.h"



/* 从 index 起发送调用方 pdata 中的 count 个字节。Dev 须有效，调用方须串行访问软件 I2C。
 * count 须小于 VL53L0X_MAX_I2C_XFER_SIZE。超限仍发起传输，底层失败时返回 VL53L0X_ERROR_CONTROL_INTERFACE。
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,uint32_t count)
{
	
	 VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	
	 int32_t status_int = 0;
	
	 uint8_t deviceAddress;
	
	 if(count >=VL53L0X_MAX_I2C_XFER_SIZE)
	 {
		 Status = VL53L0X_ERROR_INVALID_PARAMS;
	 }
	
	 deviceAddress = Dev->I2cDevAddr;
	
	 status_int = VL53L0X_write_multi(deviceAddress, index, pdata, count);
	
	 if(status_int !=0)
	   Status = VL53L0X_ERROR_CONTROL_INTERFACE;
	
	 return Status;
	
}

/* 从 index 起读取 count 个字节到调用方缓冲区，须独占总线并提供有效的 Dev 和足够空间。
 * count 须小于 VL53L0X_MAX_I2C_XFER_SIZE。超限仍发起读取，错误返回时缓冲区可能已被写入。
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,uint32_t count)
{
	
	 VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	
	 int32_t status_int;
	
	 uint8_t deviceAddress;
	
	 if(count >=VL53L0X_MAX_I2C_XFER_SIZE)
	 {
		 Status = VL53L0X_ERROR_INVALID_PARAMS;
	 } 
	 
	 deviceAddress = Dev->I2cDevAddr;
	 
	 status_int = VL53L0X_read_multi(deviceAddress, index, pdata, count);
	 
	 if(status_int!=0)
	   Status = VL53L0X_ERROR_CONTROL_INTERFACE;
	 
	 return Status;
	
}

//VL53L0X 写单字节寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:失败
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
	
	 VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	 int32_t status_int;
	 uint8_t deviceAddress;
	
	 deviceAddress = Dev->I2cDevAddr;
	
	 status_int = VL53L0X_write_byte(deviceAddress,index,data);
   
	 if(status_int!=0)
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;
	
	 return Status;
	 
}

//VL53L0X 写字（2字节）寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:失败
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
	
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	int32_t status_int;
	uint8_t deviceAddress;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_write_word(deviceAddress,index,data);

	if(status_int!=0)
	 Status = VL53L0X_ERROR_CONTROL_INTERFACE;

	return Status;
	
}

//VL53L0X 写双字（4字节）寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:失败
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
	
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	int32_t status_int;
	uint8_t deviceAddress;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_write_dword(deviceAddress, index, data);

	if (status_int != 0)
	 Status = VL53L0X_ERROR_CONTROL_INTERFACE;

	return Status;
	
}

/* 读取 Dev 的 index 寄存器，按 AndData 清位、OrData 置位后写回，调用方须独占整个读改写过程。
 * 底层报告读失败时跳过写回；读或写报告失败均返回 VL53L0X_ERROR_CONTROL_INTERFACE。
 */
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
	 
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	int32_t status_int;
	uint8_t deviceAddress;
	uint8_t data;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_read_byte(deviceAddress,index,&data);

	if(status_int!=0)
	  Status = VL53L0X_ERROR_CONTROL_INTERFACE;

	if(Status == VL53L0X_ERROR_NONE)
	{
	  data = (data & AndData) | OrData;
	  status_int = VL53L0X_write_byte(deviceAddress, index,data);
	 
	 
	  if(status_int !=0)
		 Status = VL53L0X_ERROR_CONTROL_INTERFACE;
	 
	}

	return Status;
	  
}

//VL53L0X 读单字节寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:错误
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
	
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	int32_t status_int;
	uint8_t deviceAddress;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_read_byte(deviceAddress, index, data);

	if(status_int !=0)
	Status = VL53L0X_ERROR_CONTROL_INTERFACE;

	return Status;

}

//VL53L0X 读字（2字节）寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:错误
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
	
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	int32_t status_int;
	uint8_t deviceAddress;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_read_word(deviceAddress, index, data);

	if(status_int !=0)
	  Status = VL53L0X_ERROR_CONTROL_INTERFACE;

	return Status;
	  
}

//VL53L0X 读双字（4字节）寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:错误
VL53L0X_Error  VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
	
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

    status_int = VL53L0X_read_dword(deviceAddress, index, data);

    if (status_int != 0)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

//VL53L0X 底层延时函数
//Dev:设备I2C参数结构体
//返回值: 0:成功  
//       其他:错误
#define VL53L0X_POLLINGDELAY_LOOPNB  250
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
	
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    volatile uint32_t i;

    for(i=0;i<VL53L0X_POLLINGDELAY_LOOPNB;i++){
        //Do nothing
        
    }

    return status;
}

