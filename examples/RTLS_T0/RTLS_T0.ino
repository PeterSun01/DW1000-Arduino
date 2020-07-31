/*
 * 本例演示通过DWM1000进行多基站TWR测距的范例
 * 本例为T0代码，需配合RTLS_Anchor.ino代码使用
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


//定义中断标志位
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
volatile boolean receivetimeoutAck = false;

//定义TWR时间戳
DW1000Time timePollSent;
DW1000Time timeRespReceived;
DW1000Time timeFinalSent;

uint32_t lastActivity;//记录活动时间
const uint32_t RangingPeriod = 200;//测距周期200ms 4个基站测距20ms 总周期约220ms一次，4HZ
const uint16_t replyDelayTimeUS = 2500;//回发数据延时
static byte seq_number = 0;//测距消息流水号
volatile byte expectedMsgId = FC_RESP;//发送数据后需要接收的数据功能码
uint16_t target_anchor_addr=A0_ADDR;//目标基站短地址

//标识上位机通信协议中距离值的有效情况，如mask=0000 0001则代表A0基站测距数据有效
//如mask=0000 0011则代表A0/A1基站测距数据有效
uint8_t range_mask=0x00;
uint16_t range_A0=0,range_A1=0,range_A2=0,range_A3=0;

/*
* 函数名称：setup() 
* 功能：Arduino入口函数，初始化设置
*/
void setup() 
{
    Serial.begin(115200);
    uint16_t Dev_Addr=DW1000.ReadSwitch(ROLE_TAG);
    if(Dev_Addr==ERR_ADDR)
    {
        Serial.println(F("Switch setting error,please check"));
        while(1);
    }
    delay(1000);
    //初始化硬件接口
    Serial.println(F("### DW1000-arduino-ranging-tag ###"));
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    Serial.println("DW1000 initialized ...");
    //初始化参数
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(Dev_Addr);
    DW1000.setNetworkId(0x1234);
    DW1000.commitConfiguration();
    DW1000.setReceiveFrameWaitTimeoutPeriod(6000);
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
    
    target_anchor_addr=A0_ADDR;
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
* 函数名称：next_range() 
* 功能：进行下一个基站的测距
*/
void next_range()
{
    if(target_anchor_addr==A0_ADDR)
    {
        target_anchor_addr=A1_ADDR;
        transmitPoll(target_anchor_addr);//发送POLL消息
    }
    else if(target_anchor_addr==A1_ADDR)
    {
        target_anchor_addr=A2_ADDR;
        transmitPoll(target_anchor_addr);//发送POLL消息
    }
    else if(target_anchor_addr==A2_ADDR)
    {
        target_anchor_addr=A3_ADDR;
        transmitPoll(target_anchor_addr);//发送POLL消息
    }
    else if(target_anchor_addr==A3_ADDR)//一次测距周期结束，进行数据打包
    {
        static uint16_t out_data_count=0;
        char out_data[100];
        sprintf(out_data,"mc %02x %08x %08x %08x %08x %04x %02x %08x t%d:0",range_mask,range_A0,range_A1,range_A2,range_A3,\
        out_data_count++,seq_number,0,(uint8_t)T0_ADDR);
        
        Serial.println(out_data);
        transmitRangeData(0XFFFF);//广播发送汇总测距结果
        noteActivity();//记录当前时间（喂狗）
    }
}

/*
* 函数名称：resetInactive() 
* 功能：重新开始一次测距周期，发送POLL消息
*/
void resetInactive() 
{
    range_mask=0x0;
    range_A0=0;range_A1=0;range_A2=0;range_A3=0;
    expectedMsgId = FC_RESP;
    target_anchor_addr = A0_ADDR;
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
    DW1000.writeValueToBytes(target_anchor, address,2);

    data[0] = DATA;
    data[1] = SHORT_SRC_AND_DEST;
    data[2] = seq_number++;
    DW1000.getNetworkId(&data[3]);
    memcpy(&data[5], target_anchor, 2);
    DW1000.getDeviceAddress(&data[7]); 
    data[9] = FC_POLL;

    DW1000.newTransmit();
    DW1000.setData(data, POLL_LEN);
    DW1000.startTransmit();

    expectedMsgId = FC_RESP;
}


/*
* 函数名称：transmitRangeData()
* 功能：打包和发送所有测距消息
*/
void transmitRangeData(uint16_t address) 
{
    byte target_anchor[2];
    DW1000.writeValueToBytes(target_anchor, address,2);

    data[0] = DATA;
    data[1] = SHORT_SRC_AND_DEST;
    data[2] = seq_number++;
    DW1000.getNetworkId(&data[3]);
    memcpy(&data[5], target_anchor, 2);
    DW1000.getDeviceAddress(&data[7]); 
    data[9] = FC_RANGEDATA;

    data[10] = (byte)(range_A0>>8);
    data[11] = (byte)range_A0;
    data[12] = (byte)(range_A1>>8);
    data[13] = (byte)range_A1;
    data[14] = (byte)(range_A2>>8);
    data[15] = (byte)range_A2;
    data[16] = (byte)(range_A3>>8);
    data[17] = (byte)range_A3;

    data[18] = (byte)range_mask;

    DW1000.newTransmit();
    DW1000.setData(data, RANGEDATA_LEN);
    DW1000.startTransmit();
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
            Serial.print("recv ANCHOR RESP time out,Addr=0x");
            Serial.println(target_anchor_addr,HEX);
        }
        else if(expectedMsgId == FC_REPORT)
        {
            Serial.print("recv ANCHOR REPORT time out,Addr=0x");
            Serial.println(target_anchor_addr,HEX);
        }
        next_range();//进行下一个基站测距
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
            receiver();//打开接收机，等待接收RESP消息
        } 
        else if (msgId == FC_FINAL)  //发送的数据是FINAL数据
        {
            DW1000.getTransmitTimestamp(timeFinalSent);//记录FINAL发送时间戳
            receiver();//打开接收机，等待接收REPORT消息
            noteActivity();//记录当前时间（喂狗）
        }
        else if (msgId == FC_RANGEDATA)  //发送的数据是RANGEDATA数据
        {
            DW1000.idle();//测距周期结束，进入空闲模式
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
            DW1000.getReceiveTimestamp(timeRespReceived); //记录RESP接收时间戳
            transmitFinal(target_anchor_addr);//发送FINAL消息
            noteActivity();//记录当前时间（喂狗）
        } 
        else if (msgId == FC_REPORT) //收到REPORT消息
        {
            uint16_t distance;
            distance=(uint16_t)data[10]<<8|data[11];//取得REPORT消息中的距离数据
            if(target_anchor_addr==A0_ADDR)  
            {
                range_A0=distance;
                range_mask=range_mask|0x01;     //记录range_mask，表示哪几个测距数据有效
            }
            else if(target_anchor_addr==A1_ADDR)
            {
                range_A1=distance;
                range_mask=range_mask|0x02;
            }
            else if(target_anchor_addr==A2_ADDR)
            {
                range_A2=distance;
                range_mask=range_mask|0x04;
            }
            else if(target_anchor_addr==A3_ADDR)
            {
                range_A3=distance;
                range_mask=range_mask|0x08;
            }

            Serial.print("Range From A"); 
            Serial.print((uint8_t)target_anchor_addr,HEX); 
            Serial.print("="); 
            Serial.print((float)(distance/1000.0)); Serial.println(" m");
            noteActivity();//记录当前时间（喂狗）
            next_range();//进行下一个基站的测距
        } 
    }
}
