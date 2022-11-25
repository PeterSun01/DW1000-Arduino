/*
 * 本例给了一个与DWM1000建立连接并获取基础数据的范例
 * 
 * 版本：V1.1
 * 完成日期：2020年5月1日
 */

#include <SPI.h>
#include <DW1000.h>

//定义硬件接口
const uint8_t PIN_RST = 21; // DW1000复位端口
const uint8_t PIN_IRQ = 35; // DW1000中断端口
const uint8_t PIN_SS  = 15; // DW1000-SPI片选端口

void setup() 
{
  //设置DEBUG调试端口波特率
  Serial.begin(115200);
  //初始化DWM1000硬件接口，设置SPI和中断
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DWM1000 initialized ..."));
  //初始化参数配置
  DW1000.newConfiguration();
  DW1000.setDeviceAddress(0x0001);//设置设备短地址2字节
  DW1000.setNetworkId(0x1234);//设置网络地址2字节
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));

  delay(1000);
}

void loop() 
{
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  Serial.println();

  delay(1000);
}
