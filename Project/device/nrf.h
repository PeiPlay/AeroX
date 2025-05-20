#pragma once
#include "main.h"
#include "spi.h"
#include <cstring> // Include for memcpy and memset



typedef struct 
{
    SPI_HandleTypeDef* hspi;
    struct 
    {
        GPIO_TypeDef* port;
        uint16_t pin;
    } ce, nss, irq;
    struct 
    {
        //* Nrf可以监听6个地址，P0-P5，但是P0需要进行发送，所以只监听P1-P5
        //* 接收地址由最低字节地址p1-5和它们的高字节地址high_addr组成
        //* 也就是说，P1-5通道的监听地址除了最低字节地址可变外，高字节地址是固定的
        uint8_t p1;
        uint8_t p2;
        uint8_t p3;
        uint8_t p4;
        uint8_t p5;
        uint8_t high_addr[4];
    } address_receive;              // 接收地址
    uint8_t address_transmit[5];    // 发送地址
    uint8_t rf_channel;             // 射频通道
    struct              
    {
        uint8_t buf[32];
        uint8_t len;
    } rx_data[6];                   // 各个通道的接收数据

    enum Nrf_TxState_t
    {
        Nrf_Transmit_Idle,
        Nrf_Transmit_Ongoing,
        Nrf_Transmit_Success,
        Nrf_Transmit_Failed,
    } tx_state;           //NRF发送状态
    enum Nrf_Mode_t
    {
        Nrf_Mode_Transmit,
        Nrf_Mode_Receive
    } mode;                 //NRF模式

    struct Nrf_SignalQuality_t
    {
        uint64_t check_buf[2];          //记录最近的64*2 = 128次的信号发送是否成功
        uint16_t check_index;           //当前检查的位置
        float quality;                  //信号质量
    } signal_quality;       //信号质量

    void (*nrf_rx_callback)(uint8_t channel, uint8_t* data, uint8_t len); //接收回调函数
} Nrf_t;


void Nrf_Init(Nrf_t* nrf);
void Nrf_EXTI_Callback(Nrf_t* nrf,uint16_t gpio_pin);
void Nrf_Transmit(Nrf_t* nrf, uint8_t* data, uint8_t len);
void Nrf_SetTransmitAddress(Nrf_t* nrf, uint8_t* address);

// C++ Wrapper Class
#ifdef __cplusplus
class Nrf {
private:
    Nrf_t nrf_handle; // Embed the C struct
    bool initialized;

    // Private constructor to prevent direct instantiation without parameters
    Nrf() : initialized(false) {}

public:
    // Constructor to initialize NRF with necessary parameters
    Nrf(SPI_HandleTypeDef* hspi,
        GPIO_TypeDef* ce_port, uint16_t ce_pin,
        GPIO_TypeDef* nss_port, uint16_t nss_pin,
        GPIO_TypeDef* irq_port, uint16_t irq_pin,
        uint8_t rf_channel,
        uint8_t rx_addr_p1, uint8_t rx_addr_p2, uint8_t rx_addr_p3, uint8_t rx_addr_p4, uint8_t rx_addr_p5,
        uint8_t* rx_high_addr, // Pointer to 4-byte array
        uint8_t* tx_addr)       // Pointer to 5-byte array
        : initialized(false)
    {
        nrf_handle.hspi = hspi;
        nrf_handle.ce.port = ce_port;
        nrf_handle.ce.pin = ce_pin;
        nrf_handle.nss.port = nss_port;
        nrf_handle.nss.pin = nss_pin;
        nrf_handle.irq.port = irq_port;
        nrf_handle.irq.pin = irq_pin;
        nrf_handle.rf_channel = rf_channel;
        nrf_handle.address_receive.p1 = rx_addr_p1;
        nrf_handle.address_receive.p2 = rx_addr_p2;
        nrf_handle.address_receive.p3 = rx_addr_p3;
        nrf_handle.address_receive.p4 = rx_addr_p4;
        nrf_handle.address_receive.p5 = rx_addr_p5;
        memcpy(nrf_handle.address_receive.high_addr, rx_high_addr, 4);
        memcpy(nrf_handle.address_transmit, tx_addr, 5);
        nrf_handle.nrf_rx_callback = nullptr; // Initialize callback to null
        memset(nrf_handle.signal_quality.check_buf, 0xFF, sizeof(nrf_handle.signal_quality.check_buf)); // Initialize quality to 1.0
        nrf_handle.signal_quality.check_index = 0;
        nrf_handle.signal_quality.quality = 1.0f;
        nrf_handle.tx_state = Nrf_t::Nrf_Transmit_Idle; // Use scope resolution
        nrf_handle.mode = Nrf_t::Nrf_Mode_Receive; // Use scope resolution
    }

    Nrf(const Nrf_t& nrf) : nrf_handle(nrf), initialized(false) {
        // Copy constructor
    }

    // Initialize the NRF module
    void init() {
        Nrf_Init(&nrf_handle);
        initialized = true;
    }

    // Transmit data
    void transmit(uint8_t* data, uint8_t len) {
        if (!initialized) return; // Or handle error
        Nrf_Transmit(&nrf_handle, data, len);
    }

    // Set the transmit address
    void setTransmitAddress(uint8_t* address) {
        if (!initialized) return;
        Nrf_SetTransmitAddress(&nrf_handle, address);
    }

    // Get the current signal quality (0.0 to 1.0)
    float getSignalQuality() const {
        return nrf_handle.signal_quality.quality;
    }

    // Get the current transmit state
    Nrf_t::Nrf_TxState_t getTransmitState() const {
        return nrf_handle.tx_state;
    }

    // Set the receive callback function
    void setRxCallback(void (*callback)(uint8_t channel, uint8_t* data, uint8_t len)) {
        nrf_handle.nrf_rx_callback = callback;
    }

    // Handle EXTI interrupt (intended to be called from the global EXTI handler)
    void handleEXTI(uint16_t gpio_pin) {
        if (!initialized) return;
        // Check if the interrupt pin matches this instance's IRQ pin
        if (gpio_pin == nrf_handle.irq.pin) {
            Nrf_EXTI_Callback(&nrf_handle, gpio_pin);
        }
    }

    // Provide access to the underlying C struct if needed for external C functions
    Nrf_t* getHandle() {
        return &nrf_handle;
    }
};
#endif // __cplusplus
