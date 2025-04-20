#include "test.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"

void test_usb_cdc_sendmsg(const char *msg, uint32_t len) {
    CDC_Transmit_FS((uint8_t *)msg, len);
}

void test_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    char buffer[256]; // Adjust size as needed
    vsnprintf(buffer, sizeof(buffer), format, args);
    test_usb_cdc_sendmsg(buffer, strlen(buffer));
    va_end(args);
    osDelay(2); // Yield to allow other tasks to run
}



void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	test_cpp();
    osDelay(5);
    test_printf("connected:%d\r\n", HAL_GetTick());
    osDelay(245);
  }
  /* USER CODE END StartDefaultTask */
}