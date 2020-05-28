#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include <iconv.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#define FRAME_LEN   640
#define BUFFER_SIZE 4096


//awakenflag 默认应为0，保持睡眠，测试或实时监听打开为1
int awakenFlag   = 1 ;
int resultFlag   = 0 ;

//输出结果result
static void show_result(char *string, char is_over)
{
    resultFlag=1;   
    printf("\rResult: [ %s ]", string);
    if(is_over)
        putchar('\n');
}

//内存分配
static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
    if (result) {
        size_t left = g_buffersize - 1 - strlen(g_result);
        size_t size = strlen(result);
        if (left < size) {
            g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
            if (g_result)
                g_buffersize += BUFFER_SIZE;
            else {
                printf("mem alloc failed\n");
                return;
            }
        }
        strncat(g_result, result, size);
        show_result(g_result, is_last);
    }
}

void on_speech_begin()
{
    if (g_result)
    {
        free(g_result);
    }
    g_result = (char*)malloc(BUFFER_SIZE);
    g_buffersize = BUFFER_SIZE;
    memset(g_result, 0, g_buffersize);

    printf("Start Listening...\n");
}

void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)
        printf("\nSpeaking done \n");
    else
        printf("\nRecognizer error %d\n", reason);
}

//麦克风侦测，在线语言识别
static void mic_asr(const char* session_begin_params)
{
    int errcode;
    int speech_time = 0;

    struct speech_rec iat;

    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end
    };

    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode) {
        printf("speech recognizer init failed\n");
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode) {
        printf("start listen failed %d\n", errcode);
    }
    /* demo speech_time seconds recording */
    while(speech_time++ < 5)
        sleep(1);
    errcode = sr_stop_listening(&iat);
    if (errcode) {
        printf("stop listening failed %d\n", errcode);
    }

    sr_uninit(&iat);
}

//语音唤醒
void awakenBC(const std_msgs::String::ConstPtr& msg)
{
    printf("waking up\r\n");
    // printf("%s\n",msg.data );
    sleep(10);
    awakenFlag=1;
    //printf("%d\n%d\n", awakenFlag,resultFlag);
}


//主函数
int main(int argc, char* argv[])
{
    // 初始化ROS
    ros::init(argc, argv, "voice_asr_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);

    // 声明Publisher和Subscriber
    // 订阅唤醒语音识别的信号
    ros::Subscriber awaken = node.subscribe("awaken", 1000, awakenBC);   
    // 订阅唤醒语音识别的信号    
    ros::Publisher voice_txt = node.advertise<std_msgs::String>("voice_txt", 1);  

    ROS_INFO("Sleeping...");
    int count=1;
    while(ros::ok())
    {
        // 语音识别唤醒
        if (awakenFlag){
            ROS_INFO("awaken...");
            int ret = MSP_SUCCESS;
            const char* login_params = "appid = 5e1d6a08, work_dir = .";

            const char* session_begin_params =
                "sub = iat, domain = iat, language = zh_cn, "
                "accent = mandarin, sample_rate = 16000, "
                "result_type = plain, result_encoding = utf8";

            ret = MSPLogin(NULL, NULL, login_params);
            if(MSP_SUCCESS != ret){
                MSPLogout();
                printf("MSPLogin failed , Error code %d.\n",ret);
            }
            printf("Speak in 5 seconds\n");

            mic_asr(session_begin_params);

            printf("5 sec passed\n");

            //awakenflag 识别完成应关闭为0，实时监听打开为1
            awakenFlag=1;
            MSPLogout();
        }

        // 语音识别完成
        if(resultFlag){
            resultFlag=0;
            std_msgs::String msg;
            msg.data = g_result;
            voice_txt.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

exit:
    MSPLogout();

    return 0;
}