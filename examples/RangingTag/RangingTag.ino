/*
 * 本例演示两个DWM1000进行TWR测距的范例，配合RangingAnchor.ino使用
 * 
 * 版本：V1.0
 * 完成日期：2020年5月15日
 *  
 */
#include <SPI.h>
#include <DW1000.h>

//定义硬件接口
const uint8_t PIN_RST = 21; 
const uint8_t PIN_IRQ = 34; 
const uint8_t PIN_SS  = 15; 

//定义数据数组
#define MAX_LEN_DATA 25
byte data[MAX_LEN_DATA];

//定义中断标志位
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
volatile boolean receivetimeoutAck = false;

//定义TWR时间戳
DW1000Time timePollSent;
DW1000Time timeRespReceived;
DW1000Time timeFinalSent;

uint32_t lastActivity;//记录活动时间
const uint32_t RangingPeriod = 500;//测距周期
const uint16_t replyDelayTimeUS = 2500;//回发数据延时
static byte seq_number = 0;//测距消息流水号
volatile byte expectedMsgId = FC_RESP;//发送数据后需要接收的数据功能码
const uint16_t target_anchor_addr=0x0001;//目标基站短地址

/*
* 函数名称：setup() 
* 功能：Arduino入口函数，初始化设置
*/
void setup() 
{
    Serial.begin(115200);
    delay(1000);
    //初始化硬件接口
    Serial.println(F("### DW1000-arduino-ranging-tag ###"));
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    Serial.println("DW1000 initialized ...");
    //初始化参数
    DW1000.newConfiguration();//创建新的配置信息
    DW1000.setDefaults();//设置默认配置//CHANNEL5，110KBPS, 64MHZ, PREAMBLE1024 ,PAC8
    DW1000.setDeviceAddress(0x0002);//设置设备短地址
    DW1000.setNetworkId(0x1234);//设置网络ID
    DW1000.commitConfiguration();//提交完成设置
    DW1000.setReceiveFrameWaitTimeoutPeriod(5000);//设置接收超时时间
    Serial.println(F("Committed configuration ..."));


    //输出芯片版本信息
    char msg[128];
    DW1000.getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000.getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000.getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000.getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    
    //设置中断callback
    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);
    DW1000.attachReceiveTimeoutHandler(handleReceiveTimeout);
    
    transmitPoll(target_anchor_addr);//发送POLL消息
    noteActivity();//记录当前时间（喂狗）
}

/*
* 函数名称：noteActivity() 
* 功能：记录当前活动时间戳
*/
void noteActivity()
{
    lastActivity = millis();
}

/*
* 函数名称：resetInactive() 
* 功能：重新开始一次测距周期，发送POLL消息
*/
void resetInactive() 
{
    transmitPoll(target_anchor_addr);
    noteActivity();
}

/*
* 函数名称：handleReceiveTimeout()
* 功能：接收数据超时
*/
void handleReceiveTimeout() 
{
    receivetimeoutAck = true;
}

/*
* 函数名称：handleSent()
* 功能：发送数据成功
*/
void handleSent() 
{
    sentAck = true;
}

/*
* 函数名称：handleReceived()
* 功能：接收数据成功
*/
void handleReceived() 
{
    receivedAck = true;
}

/*
* 函数名称：transmitPoll()
* 功能：打包和发送POLL消息 
*/
void transmitPoll(uint16_t address) 
{
    byte target_anchor[2];
    DW1000.writeValueToBytes(target_anchor, address,2);//转换基站地址

    data[0] = DATA;
    data[1] = SHORT_SRC_AND_DEST;
    data[2] = seq_number++;
    DW1000.getNetworkId(&data[3]);      //网络ID
    memcpy(&data[5], target_anchor, 2); //目标地址
    DW1000.getDeviceAddress(&data[7]);  //源地址
    data[9] = FC_POLL;//功能码

    DW1000.newTransmit();  //设置发送模式
    DW1000.setData(data, POLL_LEN);//设置发送的消息
    DW1000.startTransmit();//开始发送数据

    expectedMsgId = FC_RESP;  //期待收到消息功能码为RESP
}

/*
* 函数名称：transmitFinal()
* 功能：打包和发送FINAL消息
*/
void transmitFinal(uint16_t address) 
{
    byte target_anchor[2];
    DW1000.writeValueToBytes(target_anchor, address,2);
    
    data[0] = DATA;
    data[1] = SHORT_SRC_AND_DEST;
    data[2] = seq_number++;
    DW1000.getNetworkId(&data[3]);
    memcpy(&data[5], target_anchor, 2);
    DW1000.getDeviceAddress(&data[7]);   
    data[9] = FC_FINAL;

    DW1000.newTransmit();
    
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    timeFinalSent = DW1000.setDelay(deltaTime);

    timePollSent.getTimestamp(data + 10);
    timeRespReceived.getTimestamp(data + 15);
    timeFinalSent.getTimestamp(data + 20);
    
    DW1000.setData(data, FINAL_LEN);  
    DW1000.startTransmit();

    expectedMsgId = FC_REPORT;
}

/*
* 函数名称：receiver() 
* 功能：开启接收机，准备接收数据
*/
void receiver() 
{
    DW1000.newReceive();
    DW1000.startReceive();
}

/*
* 函数名称：loop() 
* 功能：主功能函数
*/
void loop() 
{
    if(receivetimeoutAck)//接收数据超时
    {
        receivetimeoutAck = false;
        if(expectedMsgId == FC_RESP)
        {
            Serial.println("recv RESP time out");
        }
        else if(expectedMsgId == FC_REPORT)
        {
            Serial.println("recv REPORT time out");
        }

    }
    //超过测距周期（RangingPeriod）未发送和接收成功，则重新启动测距周期，重新发起发送POLL消息
    //作用类似看门狗
    if (!sentAck && !receivedAck) 
    {
        if (millis() - lastActivity > RangingPeriod) 
        {
            resetInactive();
        }
        return;
    }
    //发送数据成功
    if (sentAck) 
    {
        sentAck = false;//清空标志位
        byte msgId = data[9];//读取功能码
        if (msgId == FC_POLL) //发送的数据是POLL数据
        {
            DW1000.getTransmitTimestamp(timePollSent);//记录POLL发送时间戳
            noteActivity();//记录当前时间（喂狗）
            receiver();//打开接收机，等待接收RESP消息
        } 
        else if (msgId == FC_FINAL)  //发送的数据是FINAL数据
        {
            DW1000.getTransmitTimestamp(timeFinalSent);//记录FINAL发送时间戳
            receiver();//打开接收机，等待接收REPORT消息
            noteActivity();//记录当前时间（喂狗）
        }
    }
    //接收数据成功
    if (receivedAck)   
    {
        receivedAck = false;//清空标志位
        uint8_t data_len=DW1000.getDataLength();//取得数据长度
        DW1000.getData(data, data_len);//取得接收数据
        byte msgId = data[9];//获取数据功能码
        if (msgId != expectedMsgId) //功能码非预期则重新开启测距周期
        {
            resetInactive();
            return;
        }
        if (msgId == FC_RESP) //收到RESP消息
        {
            //puts("recv resp");
            DW1000.getReceiveTimestamp(timeRespReceived); //记录RESP接收时间戳

            transmitFinal(target_anchor_addr);//发送FINAL消息
            noteActivity();//记录当前时间（喂狗）
        } 
        else if (msgId == FC_REPORT) //收到REPORT消息
        {
            uint16_t distance;
            distance=(uint16_t)data[10]<<8|data[11];//取得REPORT消息中的距离数据
            Serial.print("Range: "); Serial.print((float)(distance/1000.0)); Serial.println(" m");
            DW1000.idle();//测距周期结束，进入空闲模式
            noteActivity();//记录当前时间（喂狗）
            DW1000Time timeTest;
            DW1000.getSystemTimestamp(timeTest);
            Serial.print("timePollSent=");Serial.println(timePollSent);
            Serial.print("time now=");Serial.println(timeTest);
            Serial.print("time----=");Serial.println(timeTest-timePollSent);
            Serial.print("time----=");Serial.println((timeTest-timePollSent).getAsMicroSeconds());
        } 
    }
}
