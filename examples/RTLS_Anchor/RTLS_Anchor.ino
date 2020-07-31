/*
 * 本例演示通过DWM1000进行多基站TWR测距的范例
 * 通过拨码开关可调节自身基站ID为0-3
 * 本例为基站代码，需配合RTLS_T0.ino代码使用
 * 
 * 版本：V1.0
 * 完成日期：2020年5月30日
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


//定义测距周期
#define TIME_RANGING_PERIOD    (200) //测距周期

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

uint32_t lastActivity;//记录活动时间
const uint32_t RangingPeriod = TIME_RANGING_PERIOD;//测距周期
const uint16_t replyDelayTimeUS = 2500;//回发数据延时
static byte seq_number = 0;//测距消息流水号
volatile byte expectedMsgId = FC_POLL;//发送数据后需要接收的数据功能码

/*
* 函数名称：setup() 
* 功能：Arduino入口函数，初始化设置
*/
void setup() 
{
    Serial.begin(115200);
    uint16_t Dev_Addr=DW1000.ReadSwitch(ROLE_ANCHOR); //读取拨码开关
    if(Dev_Addr==ERR_ADDR)
    {
        Serial.println(F("Switch setting error,please check"));
        while(1);
    }
    delay(1000);
    //初始化硬件接口
    Serial.println(F("### DW1000-arduino-ranging-anchor ###"));
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    Serial.println(F("DW1000 initialized ..."));
    //初始化参数
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(Dev_Addr);
    DW1000.setNetworkId(0x1234);
    DW1000.commitConfiguration();
    DW1000.setReceiveFrameWaitTimeoutPeriod(60000);
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
    //打开接收机，准备接收标签发送来的数据
    //DW1000.idle();
    receiver();
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
* 功能：重新开始一次测距周期，打开接收机，等待接收POLL消息
*/
void resetInactive() 
{
    expectedMsgId = FC_POLL;
    //DW1000.idle();
    receiver();
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

    expectedMsgId = FC_RANGEDATA;//下一个接收消息预期为RANGEDATA消息
}

/*
* 函数名称：receiver() 
* 功能：开启接收机，准备接收数据
*/
void receiver() 
{
    //DW1000.idle();
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

/*
* 函数名称：loop() 
* 功能：主功能函数
*/
void loop() 
{
    if(receivetimeoutAck)//接收数据超时
    {
        receivetimeoutAck = false;
        if(expectedMsgId == FC_POLL)//接收POLL数据超时
        {
            Serial.println("recv POLL time out");
        }
        else
        {
            Serial.print("recv time out 0x");
            Serial.println(expectedMsgId,HEX);
        }
    }
    //超过测距周期（RangingPeriod）未发送和接收成功，则重新启动测距周期，打开接收器，准备接收POLL消息
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
        if (msgId == FC_RESP) //发送的数据是RESP数据则记录RESP发送时间戳
        {
            DW1000.getTransmitTimestamp(timeRespSent);//记录RESP发送时间戳
            receiver();//打开接收机，准备接收FINAL消息
            noteActivity();//记录当前时间（喂狗）
        }
        else if (msgId == FC_REPORT) //发送的数据是REPORT
        {
            receiver();//打开接收机，准备接收RANGEDATA消息
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
            resetInactive(); 
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
        else if (msgId == FC_RANGEDATA) //接收的数据是RANGEDATA
        {
            static uint16_t out_data_count=0;
            char out_data[100];
            byte device_addr[2];
            DW1000.getDeviceAddress(&device_addr[0]);   
            //数据打包串口发送
            sprintf(out_data,"mc %02x %08x %08x %08x %08x %04x %02x %08x a%d:%d",\
            data[18],(uint16_t)(data[10])<<8|data[11],(uint16_t)(data[12])<<8|data[13],(uint16_t)(data[14])<<8|data[15],(uint16_t)(data[16])<<8|data[17],\
            out_data_count++,seq_number,0,data[7],device_addr[0]);
            
            Serial.println(out_data);

            DW1000.idle();//一个测距周期完毕，接入空闲模式
            noteActivity();//记录当前时间（喂狗）
        }


    }
}
