#include "taskManager.h"
#include "config.h"
#include "time_utils.h"
#include "cmsis_os.h" // For osDelay

// 定义连接逻辑的常量
static const timestamp_t DISCONNECTION_TIMEOUT_TICKS = TimeStamp_FromMilliseconds(250); // 100ms 未收到包则掉线
static const timestamp_t RECOVERY_WINDOW_TICKS = TimeStamp_FromMilliseconds(500);     // 500ms 恢复窗口期
static const timestamp_t MAX_INTER_PACKET_DELAY_TICKS = TimeStamp_FromMilliseconds(250); // 恢复期间包之间最大间隔100ms
static const uint8_t MIN_STREAK_FOR_RECOVERY = 2; // 至少连续5个包满足条件才能恢复

ground_station_rx_t ground_station_rx_data; // NRF接收数据结构体
ground_station_status_t ground_station_status_data; // NRF状态数据结构体

// NRF通信回调
void ground_station_rx_callback(uint8_t channel, uint8_t* data, uint8_t len)
{
    if(len != sizeof(ground_station_rx_t)) return;
    ground_station_rx_data = *(ground_station_rx_t*)data;

    //return; // 直接返回，避免多次调用

    timestamp_t current_ts = TimeUtils_GetGlobalTick();
    ground_station_status_data.last_rx_timestamp = current_ts;

    if (!ground_station_status_data.is_connected)
    {
        if (ground_station_status_data.recovery_packet_streak == 0) // 新的恢复尝试开始
        {
            ground_station_status_data.recovery_packet_streak = 1;
            ground_station_status_data.recovery_first_packet_ts = current_ts;
            ground_station_status_data.recovery_last_packet_ts = current_ts;
        }
        else // 继续当前的恢复尝试
        {
            timestamp_t inter_packet_delay = current_ts - ground_station_status_data.recovery_last_packet_ts;
            if (inter_packet_delay <= MAX_INTER_PACKET_DELAY_TICKS)
            {
                ground_station_status_data.recovery_packet_streak++;
                ground_station_status_data.recovery_last_packet_ts = current_ts;
            }
            else // 包间隔过长，中断了当前恢复序列，用当前包开始新的序列
            {
                ground_station_status_data.recovery_packet_streak = 1;
                ground_station_status_data.recovery_first_packet_ts = current_ts;
                ground_station_status_data.recovery_last_packet_ts = current_ts;
            }
        }
    }
}

void taskGroundStation_Init(void* argument)
{
    nrf.init();
    ground_station_status_data.is_connected = 0; // 初始状态为未连接
    ground_station_status_data.last_rx_timestamp = 0;
    ground_station_status_data.recovery_first_packet_ts = 0;
    ground_station_status_data.recovery_last_packet_ts = 0;
    ground_station_status_data.recovery_packet_streak = 0;
}

void taskGroundStation(void *argument)
{
    static uint8_t dummy[4] = {0,0,0,0};
    // 初始化NRF和状态
    //taskGroundStation_Init(argument);
    
    // 进入循环
    while (1)
    {
        // osDelay(1000);
        // continue;
        timestamp_t current_ts = TimeUtils_GetGlobalTick();

        if (ground_station_status_data.is_connected)
        {
            if ((current_ts - ground_station_status_data.last_rx_timestamp) > DISCONNECTION_TIMEOUT_TICKS)
            {
                ground_station_status_data.is_connected = 0;
                ground_station_status_data.recovery_packet_streak = 0; // 重置恢复状态
                nrf.transmit(dummy, sizeof(dummy)); // 发送空包
            }
        }
        else // 未连接
        {
            if (ground_station_status_data.recovery_packet_streak >= MIN_STREAK_FOR_RECOVERY)
            {
                // 检查整个恢复序列是否在恢复窗口期内完成
                if ((ground_station_status_data.recovery_last_packet_ts - ground_station_status_data.recovery_first_packet_ts) <= RECOVERY_WINDOW_TICKS)
                {
                    ground_station_status_data.is_connected = 1;
                    ground_station_status_data.recovery_packet_streak = 0; // 清除恢复状态
                }
                else
                {
                    // 虽然包数量够了，但整个序列时间太长，此恢复序列无效
                    ground_station_status_data.recovery_packet_streak = 0;
                    nrf.transmit(dummy, sizeof(dummy)); // 发送空包
                }
            }
            else if (ground_station_status_data.recovery_packet_streak > 0) // 有部分恢复序列，但数量不足
            {
                // 检查此部分恢复序列是否已超出总恢复窗口期
                if ((current_ts - ground_station_status_data.recovery_first_packet_ts) > RECOVERY_WINDOW_TICKS)
                {
                    ground_station_status_data.recovery_packet_streak = 0; // 部分恢复序列超时，作废
                    nrf.transmit(dummy, sizeof(dummy)); // 发送空包
                }
            }
            else
            {   
                nrf.transmit(dummy, sizeof(dummy)); // 发送空包
            }
        }
        
        osDelay(10); // 每10ms检查一次状态
    }
}