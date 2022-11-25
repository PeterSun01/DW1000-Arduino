/*
 * 本例演示两个DWM1000相互通信的范例，配合BasicReceiver.ino使用
 * 
 * 版本：V1.0
 * 完成日期：2020年5月1日
 *  
 */
#include <SPI.h>
#include <DW1000.h>

//定义硬件接口
const uint8_t PIN_RST = 21; // DW1000复位端口
const uint8_t PIN_IRQ = 34; // DW1000中断端口
const uint8_t PIN_SS  = 15; // DW1000-SPI片选端口

volatile boolean sentAck = false;
int16_t sentNum = 0; 

void setup() 
{
  //设置DEBUG调试端口波特率
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("### DW1000-arduino-sender-test ###"));
  //初始化DWM1000硬件接口，设置SPI和中断
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  //初始化参数配置
  DW1000.newConfiguration();
  DW1000.setDefaults_noFrameFilter();//创建默认配置，不使用帧过滤
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  //输出DWM1000的基础信息
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  //设置数据发送callback函数
  DW1000.attachSentHandler(handleSent);
}

//发送数据成功
void handleSent() 
{
  sentAck = true;
}

//发送数据
void transmitter() 
{
  sentNum++;
  Serial.print("Transmitting packet ... #"); Serial.println(sentNum);
  DW1000.newTransmit();
  //DW1000.setDefaults();
  String msg = "Hello DW1000 #"; msg += sentNum;
  DW1000.setData(msg);
  DW1000.startTransmit();
}

void loop() 
{
  if (sentAck) 
  {
    sentAck = false;//清空标志位
  }
  transmitter();
  delay(200);

}
