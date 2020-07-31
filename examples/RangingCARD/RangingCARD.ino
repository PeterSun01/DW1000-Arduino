/*
 * 本例演示两个DWM1000进行TWR测距的范例，配合RangingTag.ino使用
 * 
 * 版本：V1.0
 * 完成日期：2020年5月15日
 *  
 */
#include <SPI.h>
#include <DW1000.h>
#include <WiFi.h>

//定义硬件接口
const uint8_t PIN_RST = 21; 
const uint8_t PIN_IRQ = 34; 
const uint8_t PIN_SS  = 15; 

//定义数据数组
#define MAX_LEN_DATA 25
byte data[MAX_LEN_DATA];

//定义测距周期
#define TIME_RANGING_PERIOD    10 //测距周期

#define STA_ANCHOR    0x01
#define STA_TAG       0x02

//定义中断标志位
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
volatile boolean receivetimeoutAck = false;

//定义TWR时间戳
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timeRespSent;
DW1000Time timeRespReceived;
DW1000Time timeFinalSent;
DW1000Time timeFinalReceived;
DW1000Time timeComputedRange;

volatile uint32_t lastActivity;//记录活动时间
const uint32_t RangingPeriod = TIME_RANGING_PERIOD;//测距周期
const uint16_t replyDelayTimeUS = 1200;//回发数据延时
static byte seq_number = 0;//测距消息流水号
volatile byte expectedMsgId = FC_POLL;//发送数据后需要接收的数据功能码
const uint16_t target_anchor_addr=0x0001;//目标基站短地址

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t status_now=STA_TAG;


void IRAM_ATTR onTimer()
{
  xSemaphoreGiveFromISR(timerSemaphore, NULL);

}

/*
* 函数名称：setup() 
* 功能：Arduino入口函数，初始化设置
*/
void setup() 
{
    Serial.begin(115200);
    delay(1000);
    //初始化硬件接口
    Serial.println(F("### DW1000-arduino-ranging-anchor ###"));
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    Serial.println(F("DW1000 initialized ..."));
    //初始化参数
    DW1000.newConfiguration();//创建新的配置信息
    DW1000.setDefaults();//设置默认配置//CHANNEL5，6.8M, 64MHZ, PREAMBLE1024 ,PAC8
    DW1000.setDeviceAddress(0x0002);//设置设备短地址
    DW1000.setNetworkId(0x1234);//设置网络ID
    DW1000.commitConfiguration();//提交完成设置
    DW1000.setReceiveFrameWaitTimeoutPeriod(6500);
    Serial.println(F("Committed configuration ..."));

    uint8_t MAC_array[6];
    WiFi.macAddress(MAC_array);
    byte DEV_EUI[8]={0xDE,0XCA,0,0,0,0,0,0};
    memcpy(&DEV_EUI[2],&MAC_array[0],6);
    DW1000.setEUI(DEV_EUI);
    // Serial.print("mac Address: ");
    // Serial.println(WiFi.macAddress()); //打印mac地址

    //输出芯片版本信息等
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

    //创建定时器
    timerSemaphore = xSemaphoreCreateBinary();
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    // Set alarm to call onTimer function every second (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(timer, 1000000, true);
    // Start an alarm
    timerAlarmEnable(timer);

    // //打开接收机，准备接收标签发送来的数据
    // receiver();
    // noteActivity();//记录当前时间（喂狗）
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
* 功能：重新开始一次测距周期，打开接收机，等待接收POLL消息
*/
void resetInactive_anc() 
{
    expectedMsgId = FC_POLL;
    receiver();
    noteActivity();
}

/*
* 函数名称：resetInactive() 
* 功能：重新开始一次测距周期，发送POLL消息
*/
void resetInactive_tag() 
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
* 函数名称：transmitResp(uint16_t address) 
* 功能：打包和发送RESP消息
*/
void transmitResp(uint16_t address) 
{
    byte target_tag[2];
    DW1000.writeValueToBytes(target_tag, address,2);

    data[0] = DATA;
    data[1] = SHORT_SRC_AND_DEST;
    data[2] = seq_number++;
    DW1000.getNetworkId(&data[3]);
    memcpy(&data[5], target_tag, 2);
    DW1000.getDeviceAddress(&data[7]);    
    data[9] = FC_RESP;

    DW1000.newTransmit();
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    DW1000.setDelay(deltaTime);
    DW1000.setData(data, RESP_LEN);
    DW1000.startTransmit();
    
    expectedMsgId = FC_FINAL;//下一个接收消息预期为FINAL消息
}

/*
* 函数名称：transmitRangeReport(uint16_t address,float distance) 
* 功能：打包和发送REPORT消息
*/
void transmitRangeReport(uint16_t address,float distance) 
{
    byte target_tag[2];
    DW1000.writeValueToBytes(target_tag, address,2);

    data[0] = DATA;
    data[1] = SHORT_SRC_AND_DEST;
    data[2] = seq_number++;
    DW1000.getNetworkId(&data[3]);
    memcpy(&data[5], target_tag, 2);
    DW1000.getDeviceAddress(&data[7]);   
    data[9] = FC_REPORT;

    uint16_t distance_int=distance*1000;
    data[10] = (byte)(distance_int>>8);
    data[11] = (byte)distance_int;

    DW1000.newTransmit();
    DW1000.setData(data, REPORT_LEN);
    DW1000.startTransmit();

    expectedMsgId = FC_POLL;//下一个接收消息预期为POLL消息
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
    DW1000.getNetworkId(&data[3]);   //网络ID
    memcpy(&data[5], target_anchor, 2);//目标地址
    DW1000.getDeviceAddress(&data[7]); //源地址
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
    DW1000.idle();
    DW1000.newReceive();
    DW1000.startReceive();
}
/*
* 函数名称：receiver() 
* 功能：开启接收机，准备接收数据
*/
void receiver2() 
{
    DW1000.newReceive();
    DW1000.startReceive();
}

/*
* 函数名称：computeTOF() 
* 功能：计算飞行时间TOF
*/
void computeTOF() 
{
    DW1000Time round1 = (timeRespReceived - timePollSent).wrap();
    DW1000Time reply1 = (timeRespSent - timePollReceived).wrap();
    DW1000Time round2 = (timeFinalReceived - timeRespSent).wrap();
    DW1000Time reply2 = (timeFinalSent - timeRespReceived).wrap();
    DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);

    timeComputedRange.setTimestamp(tof);
}

void loop()
{
    if(xSemaphoreTake(timerSemaphore, 0) == pdTRUE)
    {
        if(status_now==STA_ANCHOR)
        {
            puts("tag");
            status_now=STA_TAG;
            timerAlarmWrite(timer, 100000, true);
        }
        else if(status_now==STA_TAG)
        {
            puts("anchor");
            status_now=STA_ANCHOR;
            timerAlarmWrite(timer, 900000, true);
        }
    }
    
    if(status_now==STA_ANCHOR)
    {
        loop_anchor();
    }
    else if(status_now==STA_TAG)
    {
        loop_tag();
    }

}

/*
* 函数名称：loop_tag() 
* 功能：标签主功能函数
*/
void loop_tag() 
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
            resetInactive_tag();
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
            receiver2();//打开接收机，等待接收RESP消息
        } 
        else if (msgId == FC_FINAL)  //发送的数据是FINAL数据
        {
            DW1000.getTransmitTimestamp(timeFinalSent);//记录FINAL发送时间戳
            receiver2();//打开接收机，等待接收REPORT消息
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
            resetInactive_tag();
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
            Serial.print("tag Range: "); Serial.print((float)(distance/1000.0)); Serial.println(" m");
            resetInactive_tag();
            noteActivity();//记录当前时间（喂狗）
            // DW1000Time timeTest;
            // DW1000.getSystemTimestamp(timeTest);
            // Serial.print("timePollSent=");Serial.println(timePollSent);
            // Serial.print("time now=");Serial.println(timeTest);
            // Serial.print("time----=");Serial.println(timeTest-timePollSent);
            // Serial.print("time----=");Serial.println((timeTest-timePollSent).getAsMicroSeconds());
        } 
    }
}




/*
* 函数名称：loop_anchor()
* 功能：基站主功能函数
*/
void loop_anchor() 
{
    if(receivetimeoutAck)//接收数据超时
    {
        receivetimeoutAck = false;
        Serial.println("recv time out");
    }
    
    //超过测距周期（RangingPeriod）未发送和接收成功，则重新启动测距周期，打开接收器，准备接收POLL消息
    //作用类似看门狗
    if (!sentAck && !receivedAck) 
    {
        if (millis() - lastActivity > RangingPeriod) 
        {
            resetInactive_anc();

        }
        return;
    }
    //发送数据成功
    if (sentAck) 
    {
        sentAck = false;//清空标志位
        byte msgId = data[9];//读取功能码
        if (msgId == FC_RESP) //发送的数据是RESP数据则记录RESP发送时间戳
        {
            DW1000.getTransmitTimestamp(timeRespSent);//记录RESP发送时间戳
            receiver2();//打开接收机，准备接收FINAL消息
            noteActivity();//记录当前时间（喂狗）
        }
        else if (msgId == FC_REPORT) //发送的数据是RESP数据则记录RESP发送时间戳
        {
            receiver2();//一个测距周期完毕，进入接收模式
            noteActivity();//记录当前时间（喂狗）
        }
        
    }
    //接收数据成功
    if (receivedAck) 
    {
        receivedAck = false;//清空标志位
        uint8_t data_len=DW1000.getDataLength();//取得数据长度
        DW1000.getData(data, data_len);//取得数据
        byte msgId = data[9];//获取数据功能码
        if (msgId != expectedMsgId) //功能码非预期则重新开启测距周期
        {
            resetInactive_anc();
            return;
        }
        if (msgId == FC_POLL) //收到POLL消息
        {
            DW1000.getReceiveTimestamp(timePollReceived);//记录POLL接收时间戳
            //puts("recv poll");
            uint16_t tag_address=((uint16_t)data[8])<<8|data[7];//记录标签短地址
            transmitResp(tag_address);//发送RESP消息
            noteActivity();//记录当前时间（喂狗）
        }
        else if (msgId == FC_FINAL) //收到FINAL消息
        {
            DW1000.getReceiveTimestamp(timeFinalReceived);//记录FINAL接收时间戳
            //puts("recv final");
            timePollSent.setTimestamp(data + 10);    //接收FINLA数据中的POLL发送时间戳
            timeRespReceived.setTimestamp(data + 15);//接收FINLA数据中的RESP接收时间戳
            timeFinalSent.setTimestamp(data + 20);   //接收FINLA数据中的FINAL发送时间戳
            computeTOF(); //用6个时间戳计算飞行时间TOF
            double distance = timeComputedRange.getAsMeters();//计算距离
            //Serial.print("Range1: "); Serial.print(distance); Serial.println(" m");
            distance=DW1000.correctRange(distance);//校准距离值
            uint16_t tag_address=((uint16_t)data[8])<<8|data[7];//记录标签短地址
            transmitRangeReport(tag_address,distance);//发送REPORT消息
            Serial.print("Range: "); Serial.print(distance); Serial.print(" m");
            Serial.print("\t RX power: "); Serial.print(DW1000.getReceivePower()); Serial.println(" dBm");
            noteActivity();//记录当前时间（喂狗）
        }
    }
}

