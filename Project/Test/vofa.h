// #ifndef __VOFA_H__
// #define __VOFA_H__

// #include <stdint.h>
// #include <stdarg.h> // For va_list

// // Define the maximum number of floats that can be sent in one frame
// // Needs to be visible to both C++ template and C implementation
// #define MAX_VOFA_FLOATS 32

// #ifdef __cplusplus
// #include <type_traits> // For std::is_convertible, static_assert, etc.
// #include <string.h>    // For memcpy (used in template)
// #include <cmsis_os.h>  // For osDelay (used in template)
// #include "test.h"      // For test_usb_cdc_sendmsg (used in template)

// // Declare buffer and tail defined in vofa.cpp for use by the inline template function
// extern "C" {
//     extern const unsigned char vofa_tail[4];
//     extern unsigned char vofa_buffer[MAX_VOFA_FLOATS * sizeof(float) + sizeof(vofa_tail)];
//     extern void test_usb_cdc_sendmsg(const char *msg, uint32_t len);
// }

// // --- Template Implementation Start ---

// // Helper to check if all types are convertible to float
// template<typename... Args>
// struct all_convertible_to_float : std::true_type {};

// template<typename T, typename... Args>
// struct all_convertible_to_float<T, Args...> : std::integral_constant<bool,
//     std::is_convertible<T, float>::value && all_convertible_to_float<Args...>::value> {};

// // Variadic template function definition (inline in header)
// template<typename... Args>
// inline void vofa_print(Args... args) {
//     // Compile-time check: ensure all arguments are convertible to float
//     static_assert(all_convertible_to_float<Args...>::value, "All arguments to vofa_print must be convertible to float.");

//     constexpr int count = sizeof...(Args);

//     // Runtime check for buffer overflow
//     if (count <= 0 || count > MAX_VOFA_FLOATS) {
//          // test_printf("Error: vofa_print count %d exceeds max %d\n", count, MAX_VOFA_FLOATS);
//         return;
//     }

//     // Calculate data sizes
//     constexpr size_t float_data_size = count * sizeof(float);
//     constexpr size_t total_size = float_data_size + sizeof(vofa_tail);

//     // Use the static buffer (declared extern above)
//     unsigned char *buffer = vofa_buffer;
//     float *float_ptr = (float *)buffer;

//     // Using initializer list trick (works in C++11)
//     int dummy[] = { 0, ((void)(*float_ptr++ = static_cast<float>(args)), 0)... };
//     (void)dummy; // Avoid unused variable warning

//     // Copy the tail bytes
//     memcpy(buffer + float_data_size, vofa_tail, sizeof(vofa_tail));

//     // Send the entire frame
//     test_usb_cdc_sendmsg((const char *)buffer, total_size);

//     // Small delay
//     osDelay(1);
// }

// // --- Template Implementation End ---


// extern "C" {
// #endif

// /**
//  * @brief (C Interface) Sends a variable number of float values to Vofa+ using the justfloat protocol.
//  * @param count The number of float values being sent (must be <= MAX_VOFA_FLOATS).
//  * @param ... The float values to send.
//  */
// void vofa_print_c(int count, ...); // C-style function

// #ifdef __cplusplus
// } // extern "C"

// // No need to include vofa_impl.h anymore

// #else // If not C++

// // Define vofa_print as a macro alias to the C version when compiling as C
// #define vofa_print vofa_print_c

// #endif // __cplusplus

// #endif // __VOFA_H__
