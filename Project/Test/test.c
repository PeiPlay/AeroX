#include "test.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"
#include "main.h"

void test_usb_cdc_sendmsg(const char *msg, uint32_t len)
{
    CDC_Transmit_HS((uint8_t *)msg, len);
}

void test_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    char buffer[256]; // Adjust size as needed
  vsnprintf(buffer, sizeof(buffer), format, args);
  test_usb_cdc_sendmsg(buffer, strlen(buffer));
  va_end(args);
  osDelay(1); // Yield to allow other tasks to run
}


void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  test_cpp_task(); // This will now run the allocation tests

  // The loop below might not be reached if test_cpp enters an infinite loop
  for (;;)
  {
    osDelay(100); 
  }
  /* USER CODE END StartDefaultTask */
}


void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN StartDebugTask */

  test_cpp_debug(); // This will now run the allocation tests
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDebugTask */
}
