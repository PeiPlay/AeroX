#include "vofa.h"
// #include "vofa_private.h" // No longer needed
#include "test.h"         // For test_usb_cdc_sendmsg (used in vofa_print_c)
#include <string.h>       // For memcpy (used in vofa_print_c)
#include <cmsis_os.h>     // For osDelay (used in vofa_print_c)
#include <stdarg.h>       // For va_list stuff (used in vofa_print_c)

// Define the maximum number of floats that can be sent in one frame
// #define MAX_VOFA_FLOATS 32 // Definition moved to vofa.h

// 使用 extern "C" 确保这些变量以 C 风格命名导出
#ifdef __cplusplus
extern "C" {
#endif

// Define the frame tail for Vofa+ justfloat protocol (Definition)
// Declaration is now extern in vofa.h for C++ template
const unsigned char vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

// Static buffer to avoid dynamic allocation (Definition)
// Declaration is now extern in vofa.h for C++ template
unsigned char vofa_buffer[MAX_VOFA_FLOATS * sizeof(float) + sizeof(vofa_tail)];

#ifdef __cplusplus
}
#endif

// Implementation of the C-style function (renamed)
void vofa_print_c(int count, ...) {
    // Check if the requested count exceeds the buffer capacity (using macro from vofa.h)
    if (count <= 0 || count > MAX_VOFA_FLOATS) {
        // Handle error: requested size is invalid or too large
        // test_printf("Error: vofa_print_c count %d exceeds max %d\n", count, MAX_VOFA_FLOATS);
        return;
    }

    // Calculate data sizes based on the actual count
    size_t float_data_size = count * sizeof(float);
    size_t total_size = float_data_size + sizeof(vofa_tail);

    // Use the static buffer directly (definition is in this file)
    unsigned char *buffer = vofa_buffer;

    va_list args;
    va_start(args, count);

    // Copy float arguments into the buffer
    float *float_ptr = (float *)buffer;
    for (int i = 0; i < count; ++i) {
        // va_arg promotes float to double, so we need to cast it back
        *float_ptr++ = (float)va_arg(args, double);
    }

    va_end(args);

    // Copy the tail bytes to the end of the float data in the buffer
    memcpy(buffer + float_data_size, vofa_tail, sizeof(vofa_tail));

    // Send the entire frame via USB CDC
    test_usb_cdc_sendmsg((const char *)buffer, total_size);

    // No need to free the buffer as it's static

    // Small delay to prevent overwhelming the USB CDC buffer or the host
    osDelay(1);
}
