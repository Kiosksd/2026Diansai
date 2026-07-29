#ifndef _line_sensor_ir8_uart_h_
#define _line_sensor_ir8_uart_h_

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

// MSPM0G3519 motherboard P8 serial connector.
// Sensor TX -> MCU RX/B16; sensor RX -> MCU TX/B15.
#define LINE_SENSOR_IR8_UART_INDEX           ( UART_7 )
#define LINE_SENSOR_IR8_UART_BAUDRATE        ( 115200 )
#define LINE_SENSOR_IR8_UART_TX_PIN          ( UART7_TX_B15 )
#define LINE_SENSOR_IR8_UART_RX_PIN          ( UART7_RX_B16 )
#define LINE_SENSOR_IR8_DIGITAL_COMMAND      ( "$0,0,1#" )
#define LINE_SENSOR_IR8_CHANNEL_COUNT        ( 8 )

typedef struct
{
    uint8  raw;
    uint32 frame_count;
    uint32 rx_byte_count;
    uint32 parse_error_count;
} line_sensor_ir8_uart_snapshot_struct;

void line_sensor_ir8_uart_init          (void);
void line_sensor_ir8_uart_request_data  (void);
void line_sensor_ir8_uart_snapshot_get  (line_sensor_ir8_uart_snapshot_struct *snapshot);
void line_sensor_ir8_uart_raw_to_bin    (uint8 raw, uint8 bin[LINE_SENSOR_IR8_CHANNEL_COUNT]);

#endif
