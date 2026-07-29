#include "zf_common_headfile.h"
#include "line_sensor_ir8_uart.h"

// Digital frame defined by the sensor vendor:
// $D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
#define LINE_SENSOR_IR8_FRAME_LENGTH         ( 43 )
#define LINE_SENSOR_IR8_FRAME_BUFFER_SIZE    ( 64 )

static volatile uint8  line_sensor_ir8_raw = 0xFF;
static volatile uint32 line_sensor_ir8_frame_count = 0;
static volatile uint32 line_sensor_ir8_rx_byte_count = 0;
static volatile uint32 line_sensor_ir8_parse_error_count = 0;
static uint8 line_sensor_ir8_frame_buffer[LINE_SENSOR_IR8_FRAME_BUFFER_SIZE];
static uint8 line_sensor_ir8_frame_length = 0;
static bool line_sensor_ir8_receiving = false;

static void line_sensor_ir8_parse_byte (uint8 data)
{
    uint8 index;
    uint8 value_position;
    uint8 raw_value = 0;
    bool valid = true;

    line_sensor_ir8_rx_byte_count ++;

    if('$' == data)
    {
        line_sensor_ir8_receiving = true;
        line_sensor_ir8_frame_length = 1;
        line_sensor_ir8_frame_buffer[0] = data;
        return;
    }

    if(!line_sensor_ir8_receiving)
    {
        return;
    }

    if(line_sensor_ir8_frame_length >= LINE_SENSOR_IR8_FRAME_BUFFER_SIZE)
    {
        line_sensor_ir8_receiving = false;
        line_sensor_ir8_frame_length = 0;
        line_sensor_ir8_parse_error_count ++;
        return;
    }

    line_sensor_ir8_frame_buffer[line_sensor_ir8_frame_length ++] = data;
    if('#' != data)
    {
        return;
    }

    line_sensor_ir8_receiving = false;
    if((LINE_SENSOR_IR8_FRAME_LENGTH != line_sensor_ir8_frame_length)
        || ('D' != line_sensor_ir8_frame_buffer[1])
        || (',' != line_sensor_ir8_frame_buffer[2]))
    {
        valid = false;
    }

    if(valid)
    {
        for(index = 0; index < LINE_SENSOR_IR8_CHANNEL_COUNT; index ++)
        {
            value_position = 6 + index * 5;
            if(('x' != line_sensor_ir8_frame_buffer[value_position - 3])
                || ((uint8)('1' + index) != line_sensor_ir8_frame_buffer[value_position - 2])
                || (':' != line_sensor_ir8_frame_buffer[value_position - 1])
                || (('0' != line_sensor_ir8_frame_buffer[value_position])
                    && ('1' != line_sensor_ir8_frame_buffer[value_position])))
            {
                valid = false;
                break;
            }

            if(index < (LINE_SENSOR_IR8_CHANNEL_COUNT - 1))
            {
                if(',' != line_sensor_ir8_frame_buffer[value_position + 1])
                {
                    valid = false;
                    break;
                }
            }
            else if('#' != line_sensor_ir8_frame_buffer[value_position + 1])
            {
                valid = false;
                break;
            }

            raw_value = (uint8)((raw_value << 1)
                | (line_sensor_ir8_frame_buffer[value_position] - '0'));
        }
    }

    if(valid)
    {
        line_sensor_ir8_raw = raw_value;
        line_sensor_ir8_frame_count ++;
    }
    else
    {
        line_sensor_ir8_parse_error_count ++;
    }
    line_sensor_ir8_frame_length = 0;
}

static void line_sensor_ir8_receive_callback (uint32 event, void *ptr)
{
    uint8 data;

    (void)ptr;
    if(UART_INTERRUPT_STATE_RX != event)
    {
        return;
    }

    while(uart_query_byte(LINE_SENSOR_IR8_UART_INDEX, &data))
    {
        line_sensor_ir8_parse_byte(data);
    }
}

void line_sensor_ir8_uart_init (void)
{
    line_sensor_ir8_raw = 0xFF;
    line_sensor_ir8_frame_count = 0;
    line_sensor_ir8_rx_byte_count = 0;
    line_sensor_ir8_parse_error_count = 0;
    line_sensor_ir8_frame_length = 0;
    line_sensor_ir8_receiving = false;

    uart_init(
        LINE_SENSOR_IR8_UART_INDEX,
        LINE_SENSOR_IR8_UART_BAUDRATE,
        LINE_SENSOR_IR8_UART_TX_PIN,
        LINE_SENSOR_IR8_UART_RX_PIN);
    uart_set_callback(
        LINE_SENSOR_IR8_UART_INDEX,
        line_sensor_ir8_receive_callback,
        NULL);
    uart_set_interrupt_config(
        LINE_SENSOR_IR8_UART_INDEX,
        UART_INTERRUPT_CONFIG_RX_ENABLE);
}

void line_sensor_ir8_uart_request_data (void)
{
    uart_write_string(
        LINE_SENSOR_IR8_UART_INDEX,
        LINE_SENSOR_IR8_DIGITAL_COMMAND);
}

void line_sensor_ir8_uart_snapshot_get (
    line_sensor_ir8_uart_snapshot_struct *snapshot)
{
    uint32 primask;

    if(NULL == snapshot)
    {
        return;
    }

    primask = interrupt_global_disable();
    snapshot->raw = line_sensor_ir8_raw;
    snapshot->frame_count = line_sensor_ir8_frame_count;
    snapshot->rx_byte_count = line_sensor_ir8_rx_byte_count;
    snapshot->parse_error_count = line_sensor_ir8_parse_error_count;
    interrupt_global_enable(primask);
}

void line_sensor_ir8_uart_raw_to_bin (
    uint8 raw,
    uint8 bin[LINE_SENSOR_IR8_CHANNEL_COUNT])
{
    uint8 index;

    if(NULL == bin)
    {
        return;
    }

    for(index = 0; index < LINE_SENSOR_IR8_CHANNEL_COUNT; index ++)
    {
        bin[index] = (raw >> (7 - index)) & 0x01;
    }
}
