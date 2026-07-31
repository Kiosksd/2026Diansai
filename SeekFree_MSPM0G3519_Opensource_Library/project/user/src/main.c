/*********************************************************************************************************************
* MSPM0G3519 Opensource Library 即（MSPM0G3519 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2026 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3519 开源库的一部分
* 
* MSPM0G3519 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          mian
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.38
* 适用平台          MSPM0G3519
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2026-06-1        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "line_sensor_ir8_uart.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


// **************************** 代码区域 ****************************

// 新电机重新标定期间优先启用安全无线调试模式。
// 此模式不会初始化循迹、编码器闭环或陀螺仪；电机仅响应无线单通道短脉冲命令，其余时间 PWM=0。
#define NEW_MOTOR_SAFE_WIRELESS_TEST    ( 1 )

#if NEW_MOTOR_SAFE_WIRELESS_TEST

#define SAFE_TEST_MOTOR_CH1_DIR         ( A1 )
#define SAFE_TEST_MOTOR_CH1_PWM         ( PWM_TIM_A0_CH0_A0 )
#define SAFE_TEST_MOTOR_CH2_DIR         ( B13 )
#define SAFE_TEST_MOTOR_CH2_PWM         ( PWM_TIM_A0_CH2_B12 )
#define SAFE_TEST_MOTOR_PWM_FREQUENCY   ( 17000 )
#define SAFE_TEST_ENCODER_G8_TIMER       ( TIM_G8 )
#define SAFE_TEST_ENCODER_G8_A           ( TIMG8_ENCODER1_CH1_A26 )
#define SAFE_TEST_ENCODER_G8_B           ( TIMG8_ENCODER1_CH2_A27 )
#define SAFE_TEST_ENCODER_G9_TIMER       ( TIM_G9 )
#define SAFE_TEST_ENCODER_G9_A           ( TIMG9_ENCODER1_CH1_B7 )
#define SAFE_TEST_ENCODER_G9_B           ( TIMG9_ENCODER1_CH2_B9 )
#define SAFE_TEST_ENCODER_PERIOD_MS      ( 100 )
#define SAFE_TEST_MOTOR_PULSE_MS         ( 400 )
#define SAFE_TEST_MOTOR_PWM_DEFAULT      ( 1000 )
#define SAFE_TEST_MOTOR_PWM_MINIMUM      ( 600 )
#define SAFE_TEST_MOTOR_PWM_MAXIMUM      ( 2000 )
#define SAFE_TEST_MOTOR_PWM_STEP         ( 200 )

#define SAFE_TEST_STAGE_OPEN_LOOP        ( 1 )
#define SAFE_TEST_STAGE_SINGLE_WHEEL_PI  ( 2 )
#define SAFE_TEST_STAGE_LOW_SPEED_TRACK  ( 3 )
#define SAFE_TEST_STAGE_IR8_I2C          ( 4 )
#define SAFE_TEST_STAGE_IR8_UART         ( 5 )
#define SAFE_TEST_STAGE_IR8_UART_TRACK   ( 6 )
#define SAFE_TEST_STAGE                  ( SAFE_TEST_STAGE_IR8_UART_TRACK )

static void safe_test_motors_stop (void)
{
    pwm_set_duty(SAFE_TEST_MOTOR_CH1_PWM, 0);
    pwm_set_duty(SAFE_TEST_MOTOR_CH2_PWM, 0);
}

#if (SAFE_TEST_STAGE == SAFE_TEST_STAGE_OPEN_LOOP)

static bool safe_test_motor_pulse_start (uint8 command, uint16 pwm_duty)
{
    safe_test_motors_stop();

    switch(command)
    {
        case '1':
        {
            gpio_set_level(SAFE_TEST_MOTOR_CH1_DIR, GPIO_LOW);
            pwm_set_duty(SAFE_TEST_MOTOR_CH1_PWM, pwm_duty);
        }break;

        case '2':
        {
            gpio_set_level(SAFE_TEST_MOTOR_CH1_DIR, GPIO_HIGH);
            pwm_set_duty(SAFE_TEST_MOTOR_CH1_PWM, pwm_duty);
        }break;

        case '3':
        {
            gpio_set_level(SAFE_TEST_MOTOR_CH2_DIR, GPIO_LOW);
            pwm_set_duty(SAFE_TEST_MOTOR_CH2_PWM, pwm_duty);
        }break;

        case '4':
        {
            gpio_set_level(SAFE_TEST_MOTOR_CH2_DIR, GPIO_HIGH);
            pwm_set_duty(SAFE_TEST_MOTOR_CH2_PWM, pwm_duty);
        }break;

        default:
        {
            return false;
        }
    }

    return true;
}

int main (void)
{
    uint8 receive_buffer[WIRELESS_UART_BUFFER_SIZE];
    char send_buffer[96];
    uint32 receive_length;
    uint32 receive_index;
    uint16 encoder_elapsed_ms = 0;
    uint16 motor_pulse_elapsed_ms = 0;
    uint16 motor_test_pwm = SAFE_TEST_MOTOR_PWM_DEFAULT;
    int16 encoder_g8_count;
    int16 encoder_g9_count;
    int32 motor_test_g8_total = 0;
    int32 motor_test_g9_total = 0;
    uint8 command;
    uint8 motor_test_command = 0;
    bool motor_pulse_active = false;

    clock_init(SYSTEM_CLOCK_80M);

    // 复位释放后尽早建立确定的 DIR 电平和 0 占空比。
    gpio_init(SAFE_TEST_MOTOR_CH1_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(SAFE_TEST_MOTOR_CH2_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(SAFE_TEST_MOTOR_CH1_PWM, SAFE_TEST_MOTOR_PWM_FREQUENCY, 0);
    pwm_init(SAFE_TEST_MOTOR_CH2_PWM, SAFE_TEST_MOTOR_PWM_FREQUENCY, 0);
    safe_test_motors_stop();

    encoder_quad_init(
        SAFE_TEST_ENCODER_G8_TIMER,
        SAFE_TEST_ENCODER_G8_A,
        SAFE_TEST_ENCODER_G8_B);
    encoder_quad_init(
        SAFE_TEST_ENCODER_G9_TIMER,
        SAFE_TEST_ENCODER_G9_A,
        SAFE_TEST_ENCODER_G9_B);
    encoder_clear_count(SAFE_TEST_ENCODER_G8_TIMER);
    encoder_clear_count(SAFE_TEST_ENCODER_G9_TIMER);

    debug_init();
    system_delay_ms(300);

    if(wireless_uart_init())
    {
        printf("WIRELESS INIT FAILED. Motors remain stopped.\r\n");
        while(true)
        {
            safe_test_motors_stop();
            system_delay_ms(10);
        }
    }

    interrupt_global_enable(0);

    printf("\r\nNEW MOTOR SAFE WIRELESS TEST.\r\n");
    printf("CH1(A0/A1) PWM=0, CH2(B12/B13) PWM=0.\r\n");
    printf("Encoder increments are reported every %d ms. Motors remain stopped.\r\n",
        SAFE_TEST_ENCODER_PERIOD_MS);

    wireless_uart_send_string("\r\nMANUAL ENCODER MAPPING TEST READY\r\n");
    wireless_uart_send_string("MOTORS: CH1 PWM=0, CH2 PWM=0\r\n");
    wireless_uart_send_string("ENCODERS: G8=A26/A27, G9=B7/B9, PERIOD=100ms\r\n");
    wireless_uart_send_string("1=CH1 LOW, 2=CH1 HIGH, 3=CH2 LOW, 4=CH2 HIGH\r\n");
    wireless_uart_send_string("0=STOP, +/-=PWM STEP, ?=STATUS; DEFAULT PWM=1000\r\n");

    while(true)
    {
        if(!motor_pulse_active)
        {
            safe_test_motors_stop();
        }

        receive_length = wireless_uart_read_buffer(
            receive_buffer,
            WIRELESS_UART_BUFFER_SIZE);

        if(receive_length > 0)
        {
            printf("WIRELESS RX length=%lu\r\n", (unsigned long)receive_length);

            for(receive_index = 0; receive_index < receive_length; receive_index ++)
            {
                command = receive_buffer[receive_index];

                switch(command)
                {
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    {
                        safe_test_motors_stop();
                        encoder_clear_count(SAFE_TEST_ENCODER_G8_TIMER);
                        encoder_clear_count(SAFE_TEST_ENCODER_G9_TIMER);
                        motor_test_g8_total = 0;
                        motor_test_g9_total = 0;
                        motor_pulse_elapsed_ms = 0;
                        motor_test_command = command;
                        motor_pulse_active = safe_test_motor_pulse_start(
                            command,
                            motor_test_pwm);

                        sprintf(
                            send_buffer,
                            "START CMD=%c PWM=%u PULSE=%ums\r\n",
                            command,
                            motor_test_pwm,
                            SAFE_TEST_MOTOR_PULSE_MS);
                        wireless_uart_send_string(send_buffer);
                    }break;

                    case '0':
                    {
                        safe_test_motors_stop();
                        motor_pulse_active = false;
                        motor_test_command = 0;
                        wireless_uart_send_string("STOP: BOTH MOTOR PWM=0\r\n");
                    }break;

                    case '+':
                    {
                        if(motor_pulse_active)
                        {
                            wireless_uart_send_string("WAIT: MOTOR PULSE ACTIVE\r\n");
                            break;
                        }
                        if(motor_test_pwm < SAFE_TEST_MOTOR_PWM_MAXIMUM)
                        {
                            motor_test_pwm += SAFE_TEST_MOTOR_PWM_STEP;
                            if(motor_test_pwm > SAFE_TEST_MOTOR_PWM_MAXIMUM)
                            {
                                motor_test_pwm = SAFE_TEST_MOTOR_PWM_MAXIMUM;
                            }
                        }
                        sprintf(send_buffer, "PWM SET=%u\r\n", motor_test_pwm);
                        wireless_uart_send_string(send_buffer);
                    }break;

                    case '-':
                    {
                        if(motor_pulse_active)
                        {
                            wireless_uart_send_string("WAIT: MOTOR PULSE ACTIVE\r\n");
                            break;
                        }
                        if(motor_test_pwm > SAFE_TEST_MOTOR_PWM_MINIMUM)
                        {
                            motor_test_pwm -= SAFE_TEST_MOTOR_PWM_STEP;
                            if(motor_test_pwm < SAFE_TEST_MOTOR_PWM_MINIMUM)
                            {
                                motor_test_pwm = SAFE_TEST_MOTOR_PWM_MINIMUM;
                            }
                        }
                        sprintf(send_buffer, "PWM SET=%u\r\n", motor_test_pwm);
                        wireless_uart_send_string(send_buffer);
                    }break;

                    case '?':
                    {
                        sprintf(
                            send_buffer,
                            "STATUS: PWM=%u ACTIVE=%u CMD=%c\r\n",
                            motor_test_pwm,
                            motor_pulse_active ? 1 : 0,
                            motor_pulse_active ? motor_test_command : '-');
                        wireless_uart_send_string(send_buffer);
                    }break;

                    case 'H':
                    case 'h':
                    {
                        wireless_uart_send_string(
                            "1=CH1 LOW,2=CH1 HIGH,3=CH2 LOW,4=CH2 HIGH,0=STOP,+/-=PWM\r\n");
                    }break;

                    default:
                    {
                        // Ignore CR, LF and unknown bytes.
                    }break;
                }
            }
        }

        encoder_elapsed_ms += 10;
        if(encoder_elapsed_ms >= SAFE_TEST_ENCODER_PERIOD_MS)
        {
            encoder_elapsed_ms = 0;
            encoder_g8_count = encoder_get_count(SAFE_TEST_ENCODER_G8_TIMER);
            encoder_g9_count = encoder_get_count(SAFE_TEST_ENCODER_G9_TIMER);
            encoder_clear_count(SAFE_TEST_ENCODER_G8_TIMER);
            encoder_clear_count(SAFE_TEST_ENCODER_G9_TIMER);

            if(motor_pulse_active)
            {
                motor_test_g8_total += encoder_g8_count;
                motor_test_g9_total += encoder_g9_count;
            }

            sprintf(
                send_buffer,
                "ENC G8=%d G9=%d\r\n",
                encoder_g8_count,
                encoder_g9_count);
            wireless_uart_send_string(send_buffer);
        }

        if(motor_pulse_active)
        {
            motor_pulse_elapsed_ms += 10;
            if(motor_pulse_elapsed_ms >= SAFE_TEST_MOTOR_PULSE_MS)
            {
                safe_test_motors_stop();

                encoder_g8_count = encoder_get_count(SAFE_TEST_ENCODER_G8_TIMER);
                encoder_g9_count = encoder_get_count(SAFE_TEST_ENCODER_G9_TIMER);
                encoder_clear_count(SAFE_TEST_ENCODER_G8_TIMER);
                encoder_clear_count(SAFE_TEST_ENCODER_G9_TIMER);
                motor_test_g8_total += encoder_g8_count;
                motor_test_g9_total += encoder_g9_count;
                motor_pulse_active = false;

                sprintf(
                    send_buffer,
                    "RESULT CMD=%c PWM=%u G8_TOTAL=%ld G9_TOTAL=%ld\r\n",
                    motor_test_command,
                    motor_test_pwm,
                    (long)motor_test_g8_total,
                    (long)motor_test_g9_total);
                wireless_uart_send_string(send_buffer);
                motor_test_command = 0;
            }
        }

        system_delay_ms(10);
    }
}

#elif (SAFE_TEST_STAGE == SAFE_TEST_STAGE_SINGLE_WHEEL_PI)

#define SAFE_PI_CONTROL_PIT              ( PIT_TIM_G0 )
#define SAFE_PI_CONTROL_PERIOD_MS        ( 10 )
#define SAFE_PI_PRINT_PERIOD_MS          ( 100 )
#define SAFE_PI_TEST_DURATION_MS         ( 3000 )
#define SAFE_PI_TEST_DURATION_TICKS      ( SAFE_PI_TEST_DURATION_MS / SAFE_PI_CONTROL_PERIOD_MS )
#define SAFE_PI_TARGET_COUNT             ( 8 )
#define SAFE_PI_FEEDFORWARD_STATIC       ( 250 )
#define SAFE_PI_FEEDFORWARD_PER_COUNT    ( 105 )
#define SAFE_PI_KP                       ( 35 )
#define SAFE_PI_KI                       ( 1 )
#define SAFE_PI_INTEGRAL_LIMIT           ( 500 )
#define SAFE_PI_PWM_LIMIT                ( 2000 )
#define SAFE_PI_GS08RA_THRESHOLD         ( 30 )

typedef enum
{
    SAFE_PI_IDLE = 0,
    SAFE_PI_LEFT,
    SAFE_PI_RIGHT,
    SAFE_PI_BOTH,
} safe_pi_mode_enum;

static volatile safe_pi_mode_enum safe_pi_mode = SAFE_PI_IDLE;
static volatile safe_pi_mode_enum safe_pi_finished_mode = SAFE_PI_IDLE;
static volatile int16 safe_pi_left_count = 0;
static volatile int16 safe_pi_right_count = 0;
static volatile int32 safe_pi_left_pwm = 0;
static volatile int32 safe_pi_right_pwm = 0;
static volatile int32 safe_pi_left_integral = 0;
static volatile int32 safe_pi_right_integral = 0;
static volatile int32 safe_pi_left_total = 0;
static volatile int32 safe_pi_right_total = 0;
static volatile int32 safe_pi_finished_left_total = 0;
static volatile int32 safe_pi_finished_right_total = 0;
static volatile uint16 safe_pi_test_tick = 0;
static volatile bool safe_pi_finished = false;

static int32 safe_pi_limit (int32 value, int32 minimum, int32 maximum)
{
    if(value < minimum)
    {
        value = minimum;
    }
    else if(value > maximum)
    {
        value = maximum;
    }
    return value;
}

static bool safe_gray_error_calculate (int16 *error)
{
    int16 left = -1;
    int16 right = -1;
    uint8 index;

    for(index = 0; index < GS08A_CHANNEL_NUM; index ++)
    {
        if(0 == gs08ra_bin_val[index])
        {
            left = index;
            break;
        }
    }

    for(index = GS08A_CHANNEL_NUM; index > 0; index --)
    {
        if(0 == gs08ra_bin_val[index - 1])
        {
            right = index - 1;
            break;
        }
    }

    if((left < 0) || (right < 0))
    {
        return false;
    }

    *error = left + right - (GS08A_CHANNEL_NUM - 1);
    return true;
}

static int32 safe_pi_calculate (int16 measured_count, volatile int32 *integral)
{
    int32 error;
    int32 output;

    error = SAFE_PI_TARGET_COUNT - measured_count;
    *integral = safe_pi_limit(
        *integral + error,
        -SAFE_PI_INTEGRAL_LIMIT,
        SAFE_PI_INTEGRAL_LIMIT);

    output = SAFE_PI_FEEDFORWARD_STATIC
           + SAFE_PI_FEEDFORWARD_PER_COUNT * SAFE_PI_TARGET_COUNT
           + SAFE_PI_KP * error
           + SAFE_PI_KI * (*integral);

    return safe_pi_limit(output, 0, SAFE_PI_PWM_LIMIT);
}

static void safe_pi_control_callback (uint32 event, void *ptr)
{
    safe_pi_mode_enum current_mode;
    int16 raw_g8_count;
    int16 raw_g9_count;

    (void)event;
    (void)ptr;

    raw_g8_count = encoder_get_count(SAFE_TEST_ENCODER_G8_TIMER);
    raw_g9_count = encoder_get_count(SAFE_TEST_ENCODER_G9_TIMER);
    encoder_clear_count(SAFE_TEST_ENCODER_G8_TIMER);
    encoder_clear_count(SAFE_TEST_ENCODER_G9_TIMER);

    safe_pi_left_count = (int16)(-raw_g8_count);
    safe_pi_right_count = raw_g9_count;
    current_mode = safe_pi_mode;

    if(SAFE_PI_IDLE == current_mode)
    {
        safe_pi_left_pwm = 0;
        safe_pi_right_pwm = 0;
        safe_test_motors_stop();
        return;
    }

    if((SAFE_PI_LEFT == current_mode) || (SAFE_PI_BOTH == current_mode))
    {
        safe_pi_left_pwm = safe_pi_calculate(
            safe_pi_left_count,
            &safe_pi_left_integral);
        safe_pi_left_total += safe_pi_left_count;
    }
    else
    {
        safe_pi_left_pwm = 0;
    }

    if((SAFE_PI_RIGHT == current_mode) || (SAFE_PI_BOTH == current_mode))
    {
        safe_pi_right_pwm = safe_pi_calculate(
            safe_pi_right_count,
            &safe_pi_right_integral);
        safe_pi_right_total += safe_pi_right_count;
    }
    else
    {
        safe_pi_right_pwm = 0;
    }

    pwm_set_duty(SAFE_TEST_MOTOR_CH1_PWM, (uint16)safe_pi_left_pwm);
    pwm_set_duty(SAFE_TEST_MOTOR_CH2_PWM, (uint16)safe_pi_right_pwm);

    safe_pi_test_tick ++;
    if(safe_pi_test_tick >= SAFE_PI_TEST_DURATION_TICKS)
    {
        safe_pi_finished_mode = current_mode;
        safe_pi_finished_left_total = safe_pi_left_total;
        safe_pi_finished_right_total = safe_pi_right_total;
        safe_pi_mode = SAFE_PI_IDLE;
        safe_pi_left_pwm = 0;
        safe_pi_right_pwm = 0;
        safe_pi_finished = true;
        safe_test_motors_stop();
    }
}

static bool safe_pi_test_start (safe_pi_mode_enum new_mode)
{
    if(SAFE_PI_IDLE != safe_pi_mode)
    {
        return false;
    }

    safe_test_motors_stop();
    encoder_clear_count(SAFE_TEST_ENCODER_G8_TIMER);
    encoder_clear_count(SAFE_TEST_ENCODER_G9_TIMER);
    safe_pi_left_count = 0;
    safe_pi_right_count = 0;
    safe_pi_left_pwm = 0;
    safe_pi_right_pwm = 0;
    safe_pi_left_integral = 0;
    safe_pi_right_integral = 0;
    safe_pi_left_total = 0;
    safe_pi_right_total = 0;
    safe_pi_test_tick = 0;
    safe_pi_finished = false;
    safe_pi_finished_mode = SAFE_PI_IDLE;
    safe_pi_mode = new_mode;
    return true;
}

int main (void)
{
    uint8 receive_buffer[WIRELESS_UART_BUFFER_SIZE];
    char send_buffer[192];
    uint32 receive_length;
    uint32 receive_index;
    uint16 print_elapsed_ms = 0;
    uint8 command;
    int16 gray_error;
    bool gray_detected;
    safe_pi_mode_enum finished_mode;
    int32 finished_left_total;
    int32 finished_right_total;

    clock_init(SYSTEM_CLOCK_80M);

    gpio_init(SAFE_TEST_MOTOR_CH1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(SAFE_TEST_MOTOR_CH2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(SAFE_TEST_MOTOR_CH1_PWM, SAFE_TEST_MOTOR_PWM_FREQUENCY, 0);
    pwm_init(SAFE_TEST_MOTOR_CH2_PWM, SAFE_TEST_MOTOR_PWM_FREQUENCY, 0);
    safe_test_motors_stop();

    encoder_quad_init(
        SAFE_TEST_ENCODER_G8_TIMER,
        SAFE_TEST_ENCODER_G8_A,
        SAFE_TEST_ENCODER_G8_B);
    encoder_quad_init(
        SAFE_TEST_ENCODER_G9_TIMER,
        SAFE_TEST_ENCODER_G9_A,
        SAFE_TEST_ENCODER_G9_B);
    encoder_clear_count(SAFE_TEST_ENCODER_G8_TIMER);
    encoder_clear_count(SAFE_TEST_ENCODER_G9_TIMER);

    debug_init();
    system_delay_ms(300);

    gs08ra_init();
    gs08ra_set_threshold(SAFE_PI_GS08RA_THRESHOLD);

    if(wireless_uart_init())
    {
        printf("WIRELESS INIT FAILED. Motors remain stopped.\r\n");
        while(true)
        {
            safe_test_motors_stop();
            system_delay_ms(10);
        }
    }

    pit_ms_init(
        SAFE_PI_CONTROL_PIT,
        SAFE_PI_CONTROL_PERIOD_MS,
        safe_pi_control_callback,
        NULL);
    interrupt_global_enable(0);

    printf("\r\nNEW MOTOR SINGLE-WHEEL PI TEST.\r\n");
    printf("Target=%d count/%dms, duration=%dms, PWM limit=%d.\r\n",
        SAFE_PI_TARGET_COUNT,
        SAFE_PI_CONTROL_PERIOD_MS,
        SAFE_PI_TEST_DURATION_MS,
        SAFE_PI_PWM_LIMIT);

    wireless_uart_send_string("\r\nSINGLE-WHEEL SPEED PI TEST READY\r\n");
    wireless_uart_send_string("L=LEFT, R=RIGHT, B=BOTH FORWARD PI, 0=STOP\r\n");
    wireless_uart_send_string("G=GS08RA SNAPSHOT (MOTORS MUST BE IDLE)\r\n");
    wireless_uart_send_string("TARGET=8 count/10ms, DURATION=3000ms, PWM_LIMIT=2000\r\n");
    wireless_uart_send_string("LIFT WHEELS AND WAIT FOR RESULT BEFORE NEXT COMMAND\r\n");

    while(true)
    {
        receive_length = wireless_uart_read_buffer(
            receive_buffer,
            WIRELESS_UART_BUFFER_SIZE);

        for(receive_index = 0; receive_index < receive_length; receive_index ++)
        {
            command = receive_buffer[receive_index];

            switch(command)
            {
                case 'L':
                case 'l':
                {
                    if(safe_pi_test_start(SAFE_PI_LEFT))
                    {
                        print_elapsed_ms = 0;
                        wireless_uart_send_string("START PI LEFT/CH1\r\n");
                    }
                    else
                    {
                        wireless_uart_send_string("BUSY: SEND 0 OR WAIT\r\n");
                    }
                }break;

                case 'R':
                case 'r':
                {
                    if(safe_pi_test_start(SAFE_PI_RIGHT))
                    {
                        print_elapsed_ms = 0;
                        wireless_uart_send_string("START PI RIGHT/CH2\r\n");
                    }
                    else
                    {
                        wireless_uart_send_string("BUSY: SEND 0 OR WAIT\r\n");
                    }
                }break;

                case 'B':
                case 'b':
                {
                    if(safe_pi_test_start(SAFE_PI_BOTH))
                    {
                        print_elapsed_ms = 0;
                        wireless_uart_send_string("START PI BOTH: LEFT/CH1 + RIGHT/CH2\r\n");
                    }
                    else
                    {
                        wireless_uart_send_string("BUSY: SEND 0 OR WAIT\r\n");
                    }
                }break;

                case '0':
                {
                    safe_pi_mode = SAFE_PI_IDLE;
                    safe_pi_left_pwm = 0;
                    safe_pi_right_pwm = 0;
                    safe_pi_finished = false;
                    safe_test_motors_stop();
                    wireless_uart_send_string("STOP: BOTH MOTOR PWM=0\r\n");
                }break;

                case '?':
                {
                    sprintf(
                        send_buffer,
                        "STATUS MODE=%d TARGET=%d L[c=%d p=%ld] R[c=%d p=%ld]\r\n",
                        safe_pi_mode,
                        SAFE_PI_TARGET_COUNT,
                        safe_pi_left_count,
                        (long)safe_pi_left_pwm,
                        safe_pi_right_count,
                        (long)safe_pi_right_pwm);
                    wireless_uart_send_string(send_buffer);
                }break;

                case 'G':
                case 'g':
                {
                    if(SAFE_PI_IDLE != safe_pi_mode)
                    {
                        wireless_uart_send_string("BUSY: SEND 0 OR WAIT BEFORE GRAY TEST\r\n");
                        break;
                    }

                    safe_test_motors_stop();
                    gs08ra_scan_read();
                    gray_detected = safe_gray_error_calculate(&gray_error);
                    if(!gray_detected)
                    {
                        gray_error = 99;
                    }

                    sprintf(
                        send_buffer,
                        "GRAY BIN=%u%u%u%u%u%u%u%u det=%u err=%d "
                        "RAW=%u,%u,%u,%u,%u,%u,%u,%u "
                        "NORM=%u,%u,%u,%u,%u,%u,%u,%u TH=%u\r\n",
                        gs08ra_bin_val[0],
                        gs08ra_bin_val[1],
                        gs08ra_bin_val[2],
                        gs08ra_bin_val[3],
                        gs08ra_bin_val[4],
                        gs08ra_bin_val[5],
                        gs08ra_bin_val[6],
                        gs08ra_bin_val[7],
                        gray_detected,
                        gray_error,
                        gs08ra_raw_val[0],
                        gs08ra_raw_val[1],
                        gs08ra_raw_val[2],
                        gs08ra_raw_val[3],
                        gs08ra_raw_val[4],
                        gs08ra_raw_val[5],
                        gs08ra_raw_val[6],
                        gs08ra_raw_val[7],
                        gs08ra_deal_val[0],
                        gs08ra_deal_val[1],
                        gs08ra_deal_val[2],
                        gs08ra_deal_val[3],
                        gs08ra_deal_val[4],
                        gs08ra_deal_val[5],
                        gs08ra_deal_val[6],
                        gs08ra_deal_val[7],
                        gs08ra_threshold);
                    wireless_uart_send_string(send_buffer);
                }break;

                default:
                {
                    // Ignore CR, LF and unknown bytes.
                }break;
            }
        }

        if(SAFE_PI_IDLE != safe_pi_mode)
        {
            print_elapsed_ms += 10;
            if(print_elapsed_ms >= SAFE_PI_PRINT_PERIOD_MS)
            {
                print_elapsed_ms = 0;
                if(SAFE_PI_BOTH == safe_pi_mode)
                {
                    sprintf(
                        send_buffer,
                        "PI B t=%ums target=%d L[c=%d p=%ld] R[c=%d p=%ld]\r\n",
                        (unsigned int)(safe_pi_test_tick * SAFE_PI_CONTROL_PERIOD_MS),
                        SAFE_PI_TARGET_COUNT,
                        safe_pi_left_count,
                        (long)safe_pi_left_pwm,
                        safe_pi_right_count,
                        (long)safe_pi_right_pwm);
                }
                else
                {
                    sprintf(
                        send_buffer,
                        "PI %c t=%ums target=%d count=%d pwm=%ld\r\n",
                        (SAFE_PI_LEFT == safe_pi_mode) ? 'L' : 'R',
                        (unsigned int)(safe_pi_test_tick * SAFE_PI_CONTROL_PERIOD_MS),
                        SAFE_PI_TARGET_COUNT,
                        (SAFE_PI_LEFT == safe_pi_mode)
                            ? safe_pi_left_count
                            : safe_pi_right_count,
                        (long)((SAFE_PI_LEFT == safe_pi_mode)
                            ? safe_pi_left_pwm
                            : safe_pi_right_pwm));
                }
                wireless_uart_send_string(send_buffer);
            }
        }

        if(safe_pi_finished)
        {
            finished_mode = safe_pi_finished_mode;
            finished_left_total = safe_pi_finished_left_total;
            finished_right_total = safe_pi_finished_right_total;
            safe_pi_finished = false;
            if(SAFE_PI_BOTH == finished_mode)
            {
                sprintf(
                    send_buffer,
                    "RESULT PI BOTH TARGET=%d L[total=%ld avg_x100=%ld] R[total=%ld avg_x100=%ld]\r\n",
                    SAFE_PI_TARGET_COUNT,
                    (long)finished_left_total,
                    (long)((finished_left_total * 100) / SAFE_PI_TEST_DURATION_TICKS),
                    (long)finished_right_total,
                    (long)((finished_right_total * 100) / SAFE_PI_TEST_DURATION_TICKS));
            }
            else
            {
                finished_left_total = (SAFE_PI_LEFT == finished_mode)
                    ? finished_left_total
                    : finished_right_total;
                sprintf(
                    send_buffer,
                    "RESULT PI %s TARGET=%d TOTAL=%ld AVG_X100=%ld\r\n",
                    (SAFE_PI_LEFT == finished_mode) ? "LEFT" : "RIGHT",
                    SAFE_PI_TARGET_COUNT,
                    (long)finished_left_total,
                    (long)((finished_left_total * 100) / SAFE_PI_TEST_DURATION_TICKS));
            }
            wireless_uart_send_string(send_buffer);
            wireless_uart_send_string("MOTOR STOPPED; SEND NEXT COMMAND\r\n");
        }

        system_delay_ms(10);
    }
}

#elif (SAFE_TEST_STAGE == SAFE_TEST_STAGE_IR8_UART)

// Eight-way infrared sensor UART test.
// Sensor TX -> P8 RX/B16; sensor RX -> P8 TX/B15; 115200 baud, 8N1.
// The sensor returns: $D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
// X1 is stored in bit7 and X8 in bit0; black=0, white=1.
#define IR8_UART_INDEX                  ( UART_7 )
#define IR8_UART_BAUDRATE               ( 115200 )
#define IR8_UART_TX_PIN                 ( UART7_TX_B15 )
#define IR8_UART_RX_PIN                 ( UART7_RX_B16 )
#define IR8_UART_FRAME_LENGTH           ( 43 )
#define IR8_UART_FRAME_BUFFER_SIZE      ( 64 )
#define IR8_UART_LOOP_PERIOD_MS         ( 10 )
#define IR8_UART_PRINT_PERIOD_MS        ( 200 )
#define IR8_UART_RESEND_PERIOD_MS       ( 1000 )
#define IR8_UART_DIGITAL_COMMAND        ( "$0,0,1#" )

static volatile uint8  ir8_uart_raw_value = 0xFF;
static volatile uint32 ir8_uart_frame_count = 0;
static volatile uint32 ir8_uart_rx_byte_count = 0;
static volatile uint32 ir8_uart_parse_error_count = 0;
static uint8 ir8_uart_frame_buffer[IR8_UART_FRAME_BUFFER_SIZE];
static uint8 ir8_uart_frame_length = 0;
static bool ir8_uart_receiving = false;

static void ir8_uart_parse_byte (uint8 data)
{
    uint8 index;
    uint8 value_position;
    uint8 raw_value = 0;
    bool valid = true;

    ir8_uart_rx_byte_count ++;

    if('$' == data)
    {
        ir8_uart_receiving = true;
        ir8_uart_frame_length = 1;
        ir8_uart_frame_buffer[0] = data;
        return;
    }

    if(!ir8_uart_receiving)
    {
        return;
    }

    if(ir8_uart_frame_length >= IR8_UART_FRAME_BUFFER_SIZE)
    {
        ir8_uart_receiving = false;
        ir8_uart_frame_length = 0;
        ir8_uart_parse_error_count ++;
        return;
    }

    ir8_uart_frame_buffer[ir8_uart_frame_length ++] = data;
    if('#' != data)
    {
        return;
    }

    ir8_uart_receiving = false;
    if((IR8_UART_FRAME_LENGTH != ir8_uart_frame_length)
        || ('D' != ir8_uart_frame_buffer[1])
        || (',' != ir8_uart_frame_buffer[2]))
    {
        valid = false;
    }

    if(valid)
    {
        for(index = 0; index < 8; index ++)
        {
            value_position = 6 + index * 5;
            if(('x' != ir8_uart_frame_buffer[value_position - 3])
                || ((uint8)('1' + index) != ir8_uart_frame_buffer[value_position - 2])
                || (':' != ir8_uart_frame_buffer[value_position - 1])
                || (('0' != ir8_uart_frame_buffer[value_position])
                    && ('1' != ir8_uart_frame_buffer[value_position])))
            {
                valid = false;
                break;
            }

            if(index < 7)
            {
                if(',' != ir8_uart_frame_buffer[value_position + 1])
                {
                    valid = false;
                    break;
                }
            }
            else if('#' != ir8_uart_frame_buffer[value_position + 1])
            {
                valid = false;
                break;
            }

            raw_value = (uint8)((raw_value << 1)
                | (ir8_uart_frame_buffer[value_position] - '0'));
        }
    }

    if(valid)
    {
        ir8_uart_raw_value = raw_value;
        ir8_uart_frame_count ++;
    }
    else
    {
        ir8_uart_parse_error_count ++;
    }
    ir8_uart_frame_length = 0;
}

static void ir8_uart_receive_callback (uint32 event, void *ptr)
{
    uint8 data;

    (void)ptr;
    if(UART_INTERRUPT_STATE_RX != event)
    {
        return;
    }

    while(uart_query_byte(IR8_UART_INDEX, &data))
    {
        ir8_uart_parse_byte(data);
    }
}

static void ir8_uart_test_send (bool wireless_ready, const char *message)
{
    printf("%s", message);
    if(wireless_ready)
    {
        wireless_uart_send_string(message);
    }
}

int main (void)
{
    char send_buffer[192];
    uint8 raw_value;
    uint8 black_count;
    uint8 index;
    uint16 print_elapsed_ms = 0;
    uint16 no_frame_elapsed_ms = 0;
    uint32 frame_count;
    uint32 last_frame_count = 0;
    uint32 rx_byte_count;
    uint32 parse_error_count;
    uint32 primask;
    bool wireless_ready;

    clock_init(SYSTEM_CLOCK_80M);

    // Establish a known safe motor state immediately after reset release.
    gpio_init(SAFE_TEST_MOTOR_CH1_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(SAFE_TEST_MOTOR_CH2_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(SAFE_TEST_MOTOR_CH1_PWM, SAFE_TEST_MOTOR_PWM_FREQUENCY, 0);
    pwm_init(SAFE_TEST_MOTOR_CH2_PWM, SAFE_TEST_MOTOR_PWM_FREQUENCY, 0);
    safe_test_motors_stop();

    debug_init();
    uart_init(
        IR8_UART_INDEX,
        IR8_UART_BAUDRATE,
        IR8_UART_TX_PIN,
        IR8_UART_RX_PIN);
    uart_set_callback(IR8_UART_INDEX, ir8_uart_receive_callback, NULL);
    uart_set_interrupt_config(IR8_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);

    system_delay_ms(300);
    wireless_ready = (0 == wireless_uart_init());
    interrupt_global_enable(0);

    ir8_uart_test_send(wireless_ready, "\r\nIR8 UART DIGITAL TEST READY\r\n");
    ir8_uart_test_send(wireless_ready, "MOTORS: CH1 PWM=0, CH2 PWM=0\r\n");
    ir8_uart_test_send(wireless_ready, "P8: SENSOR TX->B16/RX, SENSOR RX->B15/TX, 115200 8N1\r\n");
    ir8_uart_test_send(wireless_ready, "ORDER: X1 X2 X3 X4 X5 X6 X7 X8; BLACK=0 WHITE=1\r\n");
    if(!wireless_ready)
    {
        printf("WARNING: WIRELESS INIT FAILED; USE DEBUG UART OUTPUT.\r\n");
    }

    // The module sends no data until this mode command is received.
    system_delay_ms(700);
    uart_write_string(IR8_UART_INDEX, IR8_UART_DIGITAL_COMMAND);

    while(true)
    {
        safe_test_motors_stop();

        primask = interrupt_global_disable();
        frame_count = ir8_uart_frame_count;
        raw_value = ir8_uart_raw_value;
        rx_byte_count = ir8_uart_rx_byte_count;
        parse_error_count = ir8_uart_parse_error_count;
        interrupt_global_enable(primask);

        if(frame_count != last_frame_count)
        {
            last_frame_count = frame_count;
            no_frame_elapsed_ms = 0;
        }
        else if(no_frame_elapsed_ms < IR8_UART_RESEND_PERIOD_MS)
        {
            no_frame_elapsed_ms += IR8_UART_LOOP_PERIOD_MS;
        }

        if(no_frame_elapsed_ms >= IR8_UART_RESEND_PERIOD_MS)
        {
            uart_write_string(IR8_UART_INDEX, IR8_UART_DIGITAL_COMMAND);
            no_frame_elapsed_ms = 0;
        }

        print_elapsed_ms += IR8_UART_LOOP_PERIOD_MS;
        if(print_elapsed_ms >= IR8_UART_PRINT_PERIOD_MS)
        {
            print_elapsed_ms = 0;
            if(frame_count > 0)
            {
                black_count = 0;
                for(index = 0; index < 8; index ++)
                {
                    if(0 == ((raw_value >> (7 - index)) & 0x01))
                    {
                        black_count ++;
                    }
                }

                sprintf(
                    send_buffer,
                    "IR8 UART OK RAW=0x%02X BIN=%u%u%u%u%u%u%u%u BLACK=%u FRAME=%lu RX=%lu ERR=%lu\r\n",
                    raw_value,
                    (raw_value >> 7) & 0x01,
                    (raw_value >> 6) & 0x01,
                    (raw_value >> 5) & 0x01,
                    (raw_value >> 4) & 0x01,
                    (raw_value >> 3) & 0x01,
                    (raw_value >> 2) & 0x01,
                    (raw_value >> 1) & 0x01,
                    raw_value & 0x01,
                    black_count,
                    (unsigned long)frame_count,
                    (unsigned long)rx_byte_count,
                    (unsigned long)parse_error_count);
            }
            else
            {
                sprintf(
                    send_buffer,
                    "IR8 UART WAIT: NO VALID FRAME RX=%lu ERR=%lu; COMMAND AUTO-RETRY\r\n",
                    (unsigned long)rx_byte_count,
                    (unsigned long)parse_error_count);
            }
            ir8_uart_test_send(wireless_ready, send_buffer);
        }

        system_delay_ms(IR8_UART_LOOP_PERIOD_MS);
    }
}

#elif (SAFE_TEST_STAGE == SAFE_TEST_STAGE_IR8_I2C)

// Eight-way infrared sensor standalone test.
// Hardware: P6 SCL=B8, SDA=B26; sensor 7-bit address=0x12.
// Register 0x30 mapping: bit7=X1 ... bit0=X8; black=0, white=1.
// Motors are initialized at zero duty and never enabled in this stage.
#define IR8_I2C_ADDRESS                 ( 0x12 )
#define IR8_DIGITAL_REGISTER            ( 0x30 )
#define IR8_SOFT_IIC_DELAY              ( 100 )
#define IR8_SCL_PIN                     ( B8 )
#define IR8_SDA_PIN                     ( B26 )
#define IR8_PRINT_PERIOD_MS             ( 200 )

typedef enum
{
    IR8_READ_OK = 0,
    IR8_READ_NO_ACK_ADDRESS_WRITE,
    IR8_READ_NO_ACK_REGISTER,
    IR8_READ_NO_ACK_ADDRESS_READ,
} ir8_read_status_enum;

static soft_iic_info_struct ir8_iic;

static ir8_read_status_enum ir8_digital_read (uint8 *raw_value)
{
    soft_iic_start(&ir8_iic);
    if(!soft_iic_send_data(&ir8_iic, IR8_I2C_ADDRESS << 1))
    {
        soft_iic_stop(&ir8_iic);
        return IR8_READ_NO_ACK_ADDRESS_WRITE;
    }
    if(!soft_iic_send_data(&ir8_iic, IR8_DIGITAL_REGISTER))
    {
        soft_iic_stop(&ir8_iic);
        return IR8_READ_NO_ACK_REGISTER;
    }
    soft_iic_stop(&ir8_iic);

    soft_iic_start(&ir8_iic);
    if(!soft_iic_send_data(&ir8_iic, (IR8_I2C_ADDRESS << 1) | 0x01))
    {
        soft_iic_stop(&ir8_iic);
        return IR8_READ_NO_ACK_ADDRESS_READ;
    }
    *raw_value = soft_iic_read_data(&ir8_iic, 1);
    soft_iic_stop(&ir8_iic);
    return IR8_READ_OK;
}

static void ir8_test_send (bool wireless_ready, const char *message)
{
    printf("%s", message);
    if(wireless_ready)
    {
        wireless_uart_send_string(message);
    }
}

int main (void)
{
    char send_buffer[160];
    uint8 raw_value = 0xFF;
    uint8 black_count;
    uint8 index;
    uint16 failed_reads = 0;
    bool wireless_ready;
    ir8_read_status_enum read_status;

    clock_init(SYSTEM_CLOCK_80M);

    // Establish a known safe motor state immediately after reset release.
    gpio_init(SAFE_TEST_MOTOR_CH1_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(SAFE_TEST_MOTOR_CH2_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(SAFE_TEST_MOTOR_CH1_PWM, SAFE_TEST_MOTOR_PWM_FREQUENCY, 0);
    pwm_init(SAFE_TEST_MOTOR_CH2_PWM, SAFE_TEST_MOTOR_PWM_FREQUENCY, 0);
    safe_test_motors_stop();

    debug_init();
    system_delay_ms(300);

    soft_iic_init(
        &ir8_iic,
        IR8_I2C_ADDRESS,
        IR8_SOFT_IIC_DELAY,
        IR8_SCL_PIN,
        IR8_SDA_PIN);
    wireless_ready = (0 == wireless_uart_init());
    interrupt_global_enable(0);

    ir8_test_send(wireless_ready, "\r\nIR8 I2C DIGITAL TEST READY\r\n");
    ir8_test_send(wireless_ready, "MOTORS: CH1 PWM=0, CH2 PWM=0\r\n");
    ir8_test_send(wireless_ready, "P6: SCL=B8 SDA=B26, ADDR=0x12 REG=0x30\r\n");
    ir8_test_send(wireless_ready, "ORDER: X1 X2 X3 X4 X5 X6 X7 X8; BLACK=0 WHITE=1\r\n");
    if(!wireless_ready)
    {
        printf("WARNING: WIRELESS INIT FAILED; USE DEBUG UART OUTPUT.\r\n");
    }

    while(true)
    {
        safe_test_motors_stop();
        read_status = ir8_digital_read(&raw_value);

        if(IR8_READ_OK == read_status)
        {
            failed_reads = 0;
            black_count = 0;
            for(index = 0; index < 8; index ++)
            {
                if(0 == ((raw_value >> (7 - index)) & 0x01))
                {
                    black_count ++;
                }
            }

            sprintf(
                send_buffer,
                "IR8 OK RAW=0x%02X BIN=%u%u%u%u%u%u%u%u BLACK=%u\r\n",
                raw_value,
                (raw_value >> 7) & 0x01,
                (raw_value >> 6) & 0x01,
                (raw_value >> 5) & 0x01,
                (raw_value >> 4) & 0x01,
                (raw_value >> 3) & 0x01,
                (raw_value >> 2) & 0x01,
                (raw_value >> 1) & 0x01,
                raw_value & 0x01,
                black_count);
        }
        else
        {
            if(failed_reads < 65535)
            {
                failed_reads ++;
            }
            sprintf(
                send_buffer,
                "IR8 I2C_ERROR STEP=%u CONSECUTIVE=%u (NOT SENSOR DATA)\r\n",
                read_status,
                failed_reads);
        }

        ir8_test_send(wireless_ready, send_buffer);
        system_delay_ms(IR8_PRINT_PERIOD_MS);
    }
}

#elif ((SAFE_TEST_STAGE == SAFE_TEST_STAGE_LOW_SPEED_TRACK) \
    || (SAFE_TEST_STAGE == SAFE_TEST_STAGE_IR8_UART_TRACK))

// One shared controller for both the archived GS08RA sensor and the new IR8
// UART sensor. Sensor-specific transport is isolated behind safe_track_bin[].
#if (SAFE_TEST_STAGE == SAFE_TEST_STAGE_IR8_UART_TRACK)
#define SAFE_TRACK_USE_IR8_UART          ( 1 )
#else
#define SAFE_TRACK_USE_IR8_UART          ( 0 )
#endif

#define SAFE_TRACK_CONTROL_PIT           ( PIT_TIM_G0 )
#define SAFE_TRACK_CONTROL_PERIOD_MS     ( 10 )
#define SAFE_TRACK_PRINT_PERIOD_MS       ( 100 )
#define SAFE_TRACK_DISPLAY_PERIOD_MS     ( 100 )
#define SAFE_TRACK_VOFA_COMMAND_SIZE     ( 32 )
#define SAFE_TRACK_GS08RA_THRESHOLD      ( 30 )
#define SAFE_TRACK_SENSOR_CHANNELS       ( 8 )
#define SAFE_TRACK_SENSOR_TIMEOUT_MS     ( 100 )
#define SAFE_TRACK_SENSOR_RETRY_MS       ( 1000 )

#if SAFE_TRACK_USE_IR8_UART
#define SAFE_TRACK_TARGET_MAX            ( 40 )
#else
#define SAFE_TRACK_TARGET_MAX            ( 35 )
#endif

#define SAFE_TRACK_BASE_TARGET           ( 23 )
#define SAFE_TRACK_SLOW_BASE_TARGET      ( 15 )
#define SAFE_TRACK_PID_KP                ( 3 )
#define SAFE_TRACK_PID_KI                ( 0 )
#define SAFE_TRACK_PID_KD                ( 2 )
#define SAFE_TRACK_PID_I_DIV             ( 100 )
#define SAFE_TRACK_PID_I_LIMIT           ( 500 )
#define SAFE_TRACK_STEER_LIMIT            ( 10 )
#define SAFE_TRACK_FEEDFORWARD_STATIC    ( 250 )
#define SAFE_TRACK_FEEDFORWARD_PER_COUNT ( 105 )
#define SAFE_TRACK_SPEED_KP              ( 20 )
#define SAFE_TRACK_SPEED_KI              ( 1 )
#define SAFE_TRACK_INTEGRAL_LIMIT        ( 500 )
#define SAFE_TRACK_PWM_LIMIT             ( 4500 )
#define SAFE_TRACK_TARGET_RISE_STEP      ( 1 )
#define SAFE_TRACK_TARGET_FALL_STEP      ( 2 )
#define SAFE_TRACK_LAP_TARGET_MS         ( 20000 )
#define SAFE_TRACK_LAP_MINIMUM_MS        ( 12000 )
#define SAFE_TRACK_FINISH_BLACK_MIN      ( 4 )
#define SAFE_TRACK_FINISH_CONFIRM_TICKS  ( 1 )

typedef enum
{
    SAFE_TRACK_MODE_NORMAL = 0,
    SAFE_TRACK_MODE_SLOW,
} safe_track_mode_enum;

static volatile int16 safe_track_left_count = 0;
static volatile int16 safe_track_right_count = 0;
static volatile int16 safe_track_left_target = 0;
static volatile int16 safe_track_right_target = 0;
static volatile int32 safe_track_left_pwm = 0;
static volatile int32 safe_track_right_pwm = 0;
static volatile int32 safe_track_left_integral = 0;
static volatile int32 safe_track_right_integral = 0;
static volatile bool safe_track_running = false;
static volatile uint32 safe_track_elapsed_ms = 0;

static int16 safe_track_error = 0;
static int16 safe_track_last_error = 0;
static int16 safe_track_correction = 0;
static int32 safe_track_error_integral = 0;
static uint16 safe_track_lost_ticks = 0;
static uint16 safe_track_finish_ticks = 0;
static uint8 safe_track_black_count = 0;
static uint8 safe_track_black_peak = 0;
static bool safe_track_line_detected = false;
static safe_track_mode_enum safe_track_mode = SAFE_TRACK_MODE_NORMAL;
static uint8 safe_track_bin[SAFE_TRACK_SENSOR_CHANNELS] =
    {1, 1, 1, 1, 1, 1, 1, 1};

#if SAFE_TRACK_USE_IR8_UART
static volatile uint16 safe_track_sensor_age_ms = SAFE_TRACK_SENSOR_TIMEOUT_MS;
static uint8 safe_track_sensor_raw = 0xFF;
static uint32 safe_track_sensor_frame_count = 0;
static uint32 safe_track_sensor_rx_byte_count = 0;
static uint32 safe_track_sensor_parse_error_count = 0;
#endif

// Runtime parameters can be changed from VOFA+ without rebuilding. The macros
// above remain the power-on defaults; parameter changes are not saved to flash.
static volatile int16 safe_track_param_base = SAFE_TRACK_BASE_TARGET;
static volatile int16 safe_track_active_base = SAFE_TRACK_BASE_TARGET;
static volatile int16 safe_track_param_kp = SAFE_TRACK_PID_KP;
static volatile int16 safe_track_param_ki = SAFE_TRACK_PID_KI;
static volatile int16 safe_track_param_kd = SAFE_TRACK_PID_KD;

static char safe_track_vofa_command[SAFE_TRACK_VOFA_COMMAND_SIZE];
static uint8 safe_track_vofa_command_length = 0;
static bool safe_track_vofa_command_receiving = false;

static void safe_track_integrals_reset (void)
{
    safe_track_left_integral = 0;
    safe_track_right_integral = 0;
    safe_track_error_integral = 0;
}

static void safe_track_vofa_parameters_send (void)
{
    char vofa_buffer[80];

    // params channels: BASE,KP,KI,KD
    sprintf(
        vofa_buffer,
        "params:%d,%d,%d,%d\n",
        safe_track_param_base,
        safe_track_param_kp,
        safe_track_param_ki,
        safe_track_param_kd);
    wireless_uart_send_string(vofa_buffer);
}

static void safe_track_vofa_telemetry_send (void)
{
    char vofa_buffer[160];
    uint16 sensor_age_ms = 0;

#if SAFE_TRACK_USE_IR8_UART
    sensor_age_ms = safe_track_sensor_age_ms;
#endif
    // car channels: time_ms,run,error,turn,Ltarget,Lcount,Lpwm,
    //               Rtarget,Rcount,Rpwm,black_count,sensor_age_ms
    sprintf(
        vofa_buffer,
        "car:%lu,%u,%d,%d,%d,%d,%ld,%d,%d,%ld,%u,%u\n",
        (unsigned long)safe_track_elapsed_ms,
        safe_track_running,
        safe_track_error,
        safe_track_correction,
        safe_track_left_target,
        safe_track_left_count,
        (long)safe_track_left_pwm,
        safe_track_right_target,
        safe_track_right_count,
        (long)safe_track_right_pwm,
        safe_track_black_count,
        sensor_age_ms);
    wireless_uart_send_string(vofa_buffer);
}

static bool safe_track_vofa_uint_parse (const char *text, uint32 *value)
{
    uint32 parsed_value = 0;

    if(('\0' == text[0]) || (NULL == value))
    {
        return false;
    }

    while('\0' != *text)
    {
        if((*text < '0') || (*text > '9'))
        {
            return false;
        }
        if(parsed_value > 100000UL)
        {
            return false;
        }
        parsed_value = parsed_value * 10UL + (uint32)(*text - '0');
        text ++;
    }

    *value = parsed_value;
    return true;
}

static bool safe_track_vofa_parameter_set (const char *name, uint32 value)
{
    bool track_pid_changed = false;

    if((0 == strcmp(name, "BASE")) && (value <= SAFE_TRACK_TARGET_MAX))
    {
        safe_track_param_base = (int16)value;
        if((!safe_track_running) || (SAFE_TRACK_MODE_NORMAL == safe_track_mode))
        {
            safe_track_active_base = (int16)value;
        }
    }
    else if((0 == strcmp(name, "KP")) && (value <= 20))
    {
        safe_track_param_kp = (int16)value;
        track_pid_changed = true;
    }
    else if((0 == strcmp(name, "KI")) && (value <= 20))
    {
        safe_track_param_ki = (int16)value;
        track_pid_changed = true;
    }
    else if((0 == strcmp(name, "KD")) && (value <= 20))
    {
        safe_track_param_kd = (int16)value;
        track_pid_changed = true;
    }
    else
    {
        return false;
    }

    if(track_pid_changed)
    {
        safe_track_error_integral = 0;
    }
    return true;
}

static void safe_track_vofa_command_execute (void)
{
    char response_buffer[80];
    char *separator;
    char *value_text;
    uint32 value;
    uint8 index;

    safe_track_vofa_command[safe_track_vofa_command_length] = '\0';
    for(index = 0; index < safe_track_vofa_command_length; index ++)
    {
        if((safe_track_vofa_command[index] >= 'a')
            && (safe_track_vofa_command[index] <= 'z'))
        {
            safe_track_vofa_command[index] -= ('a' - 'A');
        }
    }

    if(0 == strcmp(safe_track_vofa_command, "GET"))
    {
        safe_track_vofa_parameters_send();
        return;
    }

    separator = strchr(safe_track_vofa_command, '=');
    if(NULL == separator)
    {
        wireless_uart_send_string("ERR:USE @NAME=VALUE# OR @GET#\r\n");
        return;
    }

    *separator = '\0';
    value_text = separator + 1;
    if(!safe_track_vofa_uint_parse(value_text, &value)
        || !safe_track_vofa_parameter_set(safe_track_vofa_command, value))
    {
        wireless_uart_send_string("ERR:UNKNOWN PARAMETER OR VALUE OUT OF RANGE\r\n");
        return;
    }

    sprintf(
        response_buffer,
        "ACK:%s=%lu\r\n",
        safe_track_vofa_command,
        (unsigned long)value);
    wireless_uart_send_string(response_buffer);
    safe_track_vofa_parameters_send();
}

static bool safe_track_vofa_command_byte_process (uint8 data)
{
    if('@' == data)
    {
        safe_track_vofa_command_length = 0;
        safe_track_vofa_command_receiving = true;
        return true;
    }

    if(!safe_track_vofa_command_receiving)
    {
        return false;
    }

    if(('\r' == data) || ('\n' == data) || ('#' == data))
    {
        if(0 != safe_track_vofa_command_length)
        {
            safe_track_vofa_command_execute();
        }
        safe_track_vofa_command_length = 0;
        safe_track_vofa_command_receiving = false;
        return true;
    }

    if(safe_track_vofa_command_length < (SAFE_TRACK_VOFA_COMMAND_SIZE - 1))
    {
        safe_track_vofa_command[safe_track_vofa_command_length] = (char)data;
        safe_track_vofa_command_length ++;
    }
    else
    {
        safe_track_vofa_command_length = 0;
        safe_track_vofa_command_receiving = false;
        wireless_uart_send_string("ERR:COMMAND TOO LONG\r\n");
    }
    return true;
}

static void safe_track_display_update (void)
{
    char time_text[24];
    uint32 elapsed_ms = safe_track_elapsed_ms;

    ips200_set_color(
        safe_track_running ? RGB565_GREEN : RGB565_RED,
        RGB565_BLACK);
    ips200_show_string(
        72,
        48,
        safe_track_running
            ? ((SAFE_TRACK_MODE_SLOW == safe_track_mode) ? "SLOW    " : "NORMAL  ")
            : "STOPPED ");

    sprintf(
        time_text,
        "%lu.%02lu s      ",
        (unsigned long)(elapsed_ms / 1000UL),
        (unsigned long)((elapsed_ms % 1000UL) / 10UL));
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(72, 80, time_text);
}

static void safe_track_display_init (void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);
    gpio_init(IPS200_BLk_PIN_SPI, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    ips200_clear();

    ips200_show_string(16, 16, "LINE FOLLOW CAR");
    ips200_show_string(16, 48, "STATE:");
    ips200_show_string(16, 80, "TIME:");
    ips200_show_string(16, 112, "S1:NORMAL S2:SLOW");
    safe_track_display_update();
}

static int32 safe_track_limit (int32 value, int32 minimum, int32 maximum)
{
    if(value < minimum)
    {
        value = minimum;
    }
    else if(value > maximum)
    {
        value = maximum;
    }
    return value;
}

static int16 safe_track_target_slew (int16 current, int16 desired)
{
    if(desired > current)
    {
        current += SAFE_TRACK_TARGET_RISE_STEP;
        if(current > desired)
        {
            current = desired;
        }
    }
    else if(desired < current)
    {
        current -= SAFE_TRACK_TARGET_FALL_STEP;
        if(current < desired)
        {
            current = desired;
        }
    }
    return current;
}

static bool safe_track_sensor_update (void)
{
#if SAFE_TRACK_USE_IR8_UART
    line_sensor_ir8_uart_snapshot_struct snapshot;

    line_sensor_ir8_uart_snapshot_get(&snapshot);
    safe_track_sensor_rx_byte_count = snapshot.rx_byte_count;
    safe_track_sensor_parse_error_count = snapshot.parse_error_count;

    if(0 == snapshot.frame_count)
    {
        return false;
    }

    if(snapshot.frame_count != safe_track_sensor_frame_count)
    {
        safe_track_sensor_frame_count = snapshot.frame_count;
        safe_track_sensor_raw = snapshot.raw;
        line_sensor_ir8_uart_raw_to_bin(snapshot.raw, safe_track_bin);
        safe_track_sensor_age_ms = 0;
        return true;
    }
    return false;
#else
    uint8 index;

    gs08ra_scan_read();
    for(index = 0; index < SAFE_TRACK_SENSOR_CHANNELS; index ++)
    {
        safe_track_bin[index] = gs08ra_bin_val[index];
    }
    return true;
#endif
}

static uint8 safe_track_black_count_get (void)
{
    uint8 index;
    uint8 black_count = 0;

    for(index = 0; index < SAFE_TRACK_SENSOR_CHANNELS; index ++)
    {
        if(0 == safe_track_bin[index])
        {
            black_count ++;
        }
    }
    return black_count;
}

static bool safe_track_line_error_calculate (int16 *error)
{
    int16 left = -1;
    int16 right = -1;
    uint8 index;

    for(index = 0; index < SAFE_TRACK_SENSOR_CHANNELS; index ++)
    {
        if(0 == safe_track_bin[index])
        {
            left = index;
            break;
        }
    }

    for(index = SAFE_TRACK_SENSOR_CHANNELS; index > 0; index --)
    {
        if(0 == safe_track_bin[index - 1])
        {
            right = index - 1;
            break;
        }
    }

    if((left < 0) || (right < 0))
    {
        return false;
    }

    *error = left + right - (SAFE_TRACK_SENSOR_CHANNELS - 1);
    return true;
}

static int32 safe_track_speed_pi_calculate (
    int16 target,
    int16 measured,
    volatile int32 *integral)
{
    int32 error;
    int32 output;

    if(target <= 0)
    {
        *integral = 0;
        return 0;
    }

    error = target - measured;
    *integral = safe_track_limit(
        *integral + error,
        -SAFE_TRACK_INTEGRAL_LIMIT,
        SAFE_TRACK_INTEGRAL_LIMIT);

    output = SAFE_TRACK_FEEDFORWARD_STATIC
           + SAFE_TRACK_FEEDFORWARD_PER_COUNT * target
           + SAFE_TRACK_SPEED_KP * error
           + SAFE_TRACK_SPEED_KI * (*integral);

    return safe_track_limit(output, 0, SAFE_TRACK_PWM_LIMIT);
}

static void safe_track_control_callback (uint32 event, void *ptr)
{
    int16 raw_g8_count;
    int16 raw_g9_count;

    (void)event;
    (void)ptr;

    raw_g8_count = encoder_get_count(SAFE_TEST_ENCODER_G8_TIMER);
    raw_g9_count = encoder_get_count(SAFE_TEST_ENCODER_G9_TIMER);
    encoder_clear_count(SAFE_TEST_ENCODER_G8_TIMER);
    encoder_clear_count(SAFE_TEST_ENCODER_G9_TIMER);

    // New motor mapping: physical LEFT=CH1/TIMG8 (forward raw negative),
    // physical RIGHT=CH2/TIMG9 (forward raw positive).
    safe_track_left_count = (int16)(-raw_g8_count);
    safe_track_right_count = raw_g9_count;

#if SAFE_TRACK_USE_IR8_UART
    // Track UART freshness even while stopped, so START can never accept a
    // stale frame left over from an earlier healthy sensor connection.
    if(safe_track_sensor_age_ms
        <= (uint16)(65535 - SAFE_TRACK_CONTROL_PERIOD_MS))
    {
        safe_track_sensor_age_ms += SAFE_TRACK_CONTROL_PERIOD_MS;
    }
#endif

    if(!safe_track_running)
    {
        safe_track_left_pwm = 0;
        safe_track_right_pwm = 0;
        safe_test_motors_stop();
        return;
    }

    // Measure lap time from the hardware PIT period. The foreground loop also
    // scans gray sensors and sends logs, so counting one nominal 10 ms period
    // per foreground iteration makes the reported lap time run too slowly.
    safe_track_elapsed_ms += SAFE_TRACK_CONTROL_PERIOD_MS;

    safe_track_left_pwm = safe_track_speed_pi_calculate(
        safe_track_left_target,
        safe_track_left_count,
        &safe_track_left_integral);
    safe_track_right_pwm = safe_track_speed_pi_calculate(
        safe_track_right_target,
        safe_track_right_count,
        &safe_track_right_integral);

    pwm_set_duty(SAFE_TEST_MOTOR_CH1_PWM, (uint16)safe_track_left_pwm);
    pwm_set_duty(SAFE_TEST_MOTOR_CH2_PWM, (uint16)safe_track_right_pwm);
}

static void safe_track_stop (const char *reason)
{
    safe_track_running = false;
    safe_track_left_target = 0;
    safe_track_right_target = 0;
    safe_track_left_pwm = 0;
    safe_track_right_pwm = 0;
    safe_track_integrals_reset();
    safe_test_motors_stop();
    safe_track_display_update();
    safe_track_vofa_telemetry_send();

    wireless_uart_send_string("STOP: ");
    wireless_uart_send_string(reason);
    wireless_uart_send_string("\r\n");
}

static void safe_track_targets_update (void)
{
    int16 derivative;
    int16 base_target;
    int16 desired_left_target;
    int16 desired_right_target;

    safe_track_line_detected = safe_track_line_error_calculate(&safe_track_error);

    if(safe_track_line_detected)
    {
        safe_track_lost_ticks = 0;
        derivative = safe_track_error - safe_track_last_error;
        safe_track_error_integral = safe_track_limit(
            safe_track_error_integral + safe_track_error,
            -SAFE_TRACK_PID_I_LIMIT,
            SAFE_TRACK_PID_I_LIMIT);
        safe_track_correction = (int16)safe_track_limit(
            safe_track_param_kp * safe_track_error
                + (safe_track_param_ki * safe_track_error_integral)
                    / SAFE_TRACK_PID_I_DIV
                + safe_track_param_kd * derivative,
            -SAFE_TRACK_STEER_LIMIT,
            SAFE_TRACK_STEER_LIMIT);
        safe_track_last_error = safe_track_error;
        base_target = safe_track_active_base;
    }
    else
    {
        if(safe_track_lost_ticks < 65535)
        {
            safe_track_lost_ticks ++;
        }
        safe_track_error_integral = 0;
        safe_track_correction = 0;
        base_target = safe_track_active_base;
    }

    desired_left_target = (int16)safe_track_limit(
        base_target + safe_track_correction,
        0,
        SAFE_TRACK_TARGET_MAX);
    desired_right_target = (int16)safe_track_limit(
        base_target - safe_track_correction,
        0,
        SAFE_TRACK_TARGET_MAX);

    safe_track_left_target = safe_track_target_slew(
        safe_track_left_target,
        desired_left_target);
    safe_track_right_target = safe_track_target_slew(
        safe_track_right_target,
        desired_right_target);
}

static bool safe_track_start (safe_track_mode_enum mode)
{
    safe_test_motors_stop();
    safe_track_sensor_update();
#if SAFE_TRACK_USE_IR8_UART
    if((0 == safe_track_sensor_frame_count)
        || (safe_track_sensor_age_ms >= SAFE_TRACK_SENSOR_TIMEOUT_MS))
    {
        wireless_uart_send_string("START REFUSED: IR8 UART DATA TIMEOUT\r\n");
        return false;
    }
#endif
    safe_track_line_detected = safe_track_line_error_calculate(&safe_track_error);
    if(!safe_track_line_detected)
    {
        wireless_uart_send_string("START REFUSED: NO BLACK LINE\r\n");
        return false;
    }

    encoder_clear_count(SAFE_TEST_ENCODER_G8_TIMER);
    encoder_clear_count(SAFE_TEST_ENCODER_G9_TIMER);
    safe_track_left_count = 0;
    safe_track_right_count = 0;
    safe_track_left_pwm = 0;
    safe_track_right_pwm = 0;
    safe_track_integrals_reset();
    safe_track_mode = mode;
    safe_track_active_base = (SAFE_TRACK_MODE_SLOW == mode)
        ? SAFE_TRACK_SLOW_BASE_TARGET
        : safe_track_param_base;
    safe_track_last_error = safe_track_error;
    safe_track_correction = (int16)safe_track_limit(
        safe_track_param_kp * safe_track_error,
        -SAFE_TRACK_STEER_LIMIT,
        SAFE_TRACK_STEER_LIMIT);
    safe_track_lost_ticks = 0;
    safe_track_elapsed_ms = 0;
    safe_track_finish_ticks = 0;
    safe_track_black_count = 0;
    safe_track_black_peak = 0;
#if SAFE_TRACK_USE_IR8_UART
    safe_track_sensor_age_ms = 0;
#endif
    safe_track_left_target = 0;
    safe_track_right_target = 0;
    safe_track_running = true;
    safe_track_display_update();
    safe_track_vofa_telemetry_send();
    return true;
}

static void safe_track_sensor_snapshot_send (char *send_buffer)
{
    int16 sensor_error;
    bool sensor_detected;

    safe_test_motors_stop();
    safe_track_sensor_update();
    sensor_detected = safe_track_line_error_calculate(&sensor_error);
    if(!sensor_detected)
    {
        sensor_error = 99;
    }

#if SAFE_TRACK_USE_IR8_UART
    sprintf(
        send_buffer,
        "IR8 BIN=%u%u%u%u%u%u%u%u RAW=0x%02X det=%u err=%d "
        "age=%ums frame=%lu rx=%lu parse_err=%lu\r\n",
        safe_track_bin[0],
        safe_track_bin[1],
        safe_track_bin[2],
        safe_track_bin[3],
        safe_track_bin[4],
        safe_track_bin[5],
        safe_track_bin[6],
        safe_track_bin[7],
        safe_track_sensor_raw,
        sensor_detected,
        sensor_error,
        safe_track_sensor_age_ms,
        (unsigned long)safe_track_sensor_frame_count,
        (unsigned long)safe_track_sensor_rx_byte_count,
        (unsigned long)safe_track_sensor_parse_error_count);
#else
    sprintf(
        send_buffer,
        "GRAY BIN=%u%u%u%u%u%u%u%u det=%u err=%d "
        "RAW=%u,%u,%u,%u,%u,%u,%u,%u "
        "NORM=%u,%u,%u,%u,%u,%u,%u,%u TH=%u\r\n",
        safe_track_bin[0],
        safe_track_bin[1],
        safe_track_bin[2],
        safe_track_bin[3],
        safe_track_bin[4],
        safe_track_bin[5],
        safe_track_bin[6],
        safe_track_bin[7],
        sensor_detected,
        sensor_error,
        gs08ra_raw_val[0],
        gs08ra_raw_val[1],
        gs08ra_raw_val[2],
        gs08ra_raw_val[3],
        gs08ra_raw_val[4],
        gs08ra_raw_val[5],
        gs08ra_raw_val[6],
        gs08ra_raw_val[7],
        gs08ra_deal_val[0],
        gs08ra_deal_val[1],
        gs08ra_deal_val[2],
        gs08ra_deal_val[3],
        gs08ra_deal_val[4],
        gs08ra_deal_val[5],
        gs08ra_deal_val[6],
        gs08ra_deal_val[7],
        gs08ra_threshold);
#endif
    wireless_uart_send_string(send_buffer);
}

int main (void)
{
    uint8 receive_buffer[WIRELESS_UART_BUFFER_SIZE];
    char send_buffer[192];
    uint32 receive_length;
    uint32 receive_index;
    uint32 finished_lap_ms;
    uint32 recommended_scale_x1000;
    uint16 print_elapsed_ms = 0;
    uint16 display_elapsed_ms = 0;
    uint8 command;
    key_state_enum key1_state;
    key_state_enum key2_state;
#if SAFE_TRACK_USE_IR8_UART
    uint16 sensor_silence_ms = 0;
    bool sensor_new_frame;
#endif

    clock_init(SYSTEM_CLOCK_80M);

    gpio_init(SAFE_TEST_MOTOR_CH1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(SAFE_TEST_MOTOR_CH2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(SAFE_TEST_MOTOR_CH1_PWM, SAFE_TEST_MOTOR_PWM_FREQUENCY, 0);
    pwm_init(SAFE_TEST_MOTOR_CH2_PWM, SAFE_TEST_MOTOR_PWM_FREQUENCY, 0);
    safe_test_motors_stop();

    encoder_quad_init(
        SAFE_TEST_ENCODER_G8_TIMER,
        SAFE_TEST_ENCODER_G8_A,
        SAFE_TEST_ENCODER_G8_B);
    encoder_quad_init(
        SAFE_TEST_ENCODER_G9_TIMER,
        SAFE_TEST_ENCODER_G9_A,
        SAFE_TEST_ENCODER_G9_B);
    encoder_clear_count(SAFE_TEST_ENCODER_G8_TIMER);
    encoder_clear_count(SAFE_TEST_ENCODER_G9_TIMER);

    debug_init();
    system_delay_ms(300);
    key_init(SAFE_TRACK_CONTROL_PERIOD_MS);
    safe_track_display_init();

#if SAFE_TRACK_USE_IR8_UART
    line_sensor_ir8_uart_init();
#else
    gs08ra_init();
    gs08ra_set_threshold(SAFE_TRACK_GS08RA_THRESHOLD);
#endif

    if(wireless_uart_init())
    {
        printf("WIRELESS INIT FAILED. Motors remain stopped.\r\n");
        while(true)
        {
            safe_test_motors_stop();
            system_delay_ms(10);
        }
    }

    pit_ms_init(
        SAFE_TRACK_CONTROL_PIT,
        SAFE_TRACK_CONTROL_PERIOD_MS,
        safe_track_control_callback,
        NULL);
    interrupt_global_enable(0);

#if SAFE_TRACK_USE_IR8_UART
    system_delay_ms(700);
    line_sensor_ir8_uart_request_data();
#endif

#if SAFE_TRACK_USE_IR8_UART
    wireless_uart_send_string("\r\nIR8 UART HALF-SPEED LINE TRACK READY\r\n");
    wireless_uart_send_string("SENSOR: X1=PHYSICAL LEFT, X8=RIGHT, BLACK=0, WHITE=1\r\n");
    wireless_uart_send_string("S1(A30)=NORMAL, S2(A31)=SLOW; EITHER KEY STOPS WHILE RUNNING\r\n");
    wireless_uart_send_string("S/1=NORMAL, L/2=SLOW, P/0=STOP, G=IR8 SNAPSHOT, ?=STATUS\r\n");
#else
    wireless_uart_send_string("\r\nNEW LARGE-CAR LOW-SPEED LINE TRACK READY\r\n");
    wireless_uart_send_string("S1(A30)=NORMAL, S2(A31)=SLOW; EITHER KEY STOPS WHILE RUNNING\r\n");
    wireless_uart_send_string("S/1=NORMAL, L/2=SLOW, P/0=STOP, G=GRAY SNAPSHOT, ?=STATUS\r\n");
#endif
    sprintf(
        send_buffer,
        "TRACK: NORMAL=%d SLOW=%d count/10ms, PID KP=%d KI=%d KD=%d\r\n",
        safe_track_param_base,
        SAFE_TRACK_SLOW_BASE_TARGET,
        safe_track_param_kp,
        safe_track_param_ki,
        safe_track_param_kd);
    wireless_uart_send_string(send_buffer);
    sprintf(
        send_buffer,
        "STEER_LIMIT=%d, PWM_LIMIT=%d, NO FIXED TIME LIMIT\r\n",
        SAFE_TRACK_STEER_LIMIT,
        SAFE_TRACK_PWM_LIMIT);
    wireless_uart_send_string(send_buffer);
    sprintf(
        send_buffer,
        "TARGET LAP=%dms; AFTER %dms BLACK_CHANNELS>=%d FOR %dms -> STOP\r\n",
        SAFE_TRACK_LAP_TARGET_MS,
        SAFE_TRACK_LAP_MINIMUM_MS,
        SAFE_TRACK_FINISH_BLACK_MIN,
        SAFE_TRACK_FINISH_CONFIRM_TICKS * SAFE_TRACK_CONTROL_PERIOD_MS);
    wireless_uart_send_string(send_buffer);
#if SAFE_TRACK_USE_IR8_UART
    wireless_uart_send_string("IR8 UART DATA TIMEOUT 100ms -> FORCED STOP\r\n");
#endif
    wireless_uart_send_string(
        "LOST LINE -> KEEP STRAIGHT AT BASE SPEED; NO LINE-LOSS STOP\r\n");
    wireless_uart_send_string(
        "VOFA+ FIREWATER: car=12 channels, params=4 channels, 115200 baud\r\n");
    wireless_uart_send_string(
        "VOFA TUNE NORMAL: @BASE=23# @KP=3# @KI=0# @KD=1# @GET#\r\n");
    safe_track_vofa_parameters_send();

    while(true)
    {
        key_scanner();
        key1_state = key_get_state(KEY_1);
        key2_state = key_get_state(KEY_2);
        if((KEY_SHORT_PRESS == key1_state) || (KEY_SHORT_PRESS == key2_state))
        {
            if(KEY_SHORT_PRESS == key1_state)
            {
                key_clear_state(KEY_1);
            }
            if(KEY_SHORT_PRESS == key2_state)
            {
                key_clear_state(KEY_2);
            }
            display_elapsed_ms = 0;

            if(safe_track_running)
            {
                safe_track_stop(
                    (KEY_SHORT_PRESS == key1_state) ? "S1/A30 PRESSED" : "S2/A31 PRESSED");
            }
            else if(KEY_SHORT_PRESS == key1_state)
            {
                safe_track_start(SAFE_TRACK_MODE_NORMAL);
            }
            else
            {
                safe_track_start(SAFE_TRACK_MODE_SLOW);
            }
        }

#if SAFE_TRACK_USE_IR8_UART
        sensor_new_frame = safe_track_sensor_update();
        if(sensor_new_frame)
        {
            sensor_silence_ms = 0;
        }
        else if(sensor_silence_ms < SAFE_TRACK_SENSOR_RETRY_MS)
        {
            sensor_silence_ms += SAFE_TRACK_CONTROL_PERIOD_MS;
        }

        if(sensor_silence_ms >= SAFE_TRACK_SENSOR_RETRY_MS)
        {
            line_sensor_ir8_uart_request_data();
            sensor_silence_ms = 0;
        }
#else
        safe_track_sensor_update();
#endif

        receive_length = wireless_uart_read_buffer(
            receive_buffer,
            WIRELESS_UART_BUFFER_SIZE);

        for(receive_index = 0; receive_index < receive_length; receive_index ++)
        {
            command = receive_buffer[receive_index];
            if(safe_track_vofa_command_byte_process(command))
            {
                continue;
            }

            switch(command)
            {
                case 'S':
                case 's':
                case '1':
                {
                    if(safe_track_running)
                    {
                        wireless_uart_send_string("ALREADY RUNNING: SEND 0 TO STOP\r\n");
                    }
                    else if(safe_track_start(SAFE_TRACK_MODE_NORMAL))
                    {
                        sprintf(
                            send_buffer,
                            "START BIN=%u%u%u%u%u%u%u%u err=%d Ltarget=%d Rtarget=%d\r\n",
                            safe_track_bin[0],
                            safe_track_bin[1],
                            safe_track_bin[2],
                            safe_track_bin[3],
                            safe_track_bin[4],
                            safe_track_bin[5],
                            safe_track_bin[6],
                            safe_track_bin[7],
                            safe_track_error,
                            safe_track_left_target,
                            safe_track_right_target);
                        wireless_uart_send_string(send_buffer);
                    }
                }break;

                case 'L':
                case 'l':
                case '2':
                {
                    if(safe_track_running)
                    {
                        wireless_uart_send_string("ALREADY RUNNING: SEND 0 TO STOP\r\n");
                    }
                    else if(safe_track_start(SAFE_TRACK_MODE_SLOW))
                    {
                        sprintf(
                            send_buffer,
                            "START SLOW base=%d err=%d Ltarget=%d Rtarget=%d\r\n",
                            safe_track_active_base,
                            safe_track_error,
                            safe_track_left_target,
                            safe_track_right_target);
                        wireless_uart_send_string(send_buffer);
                    }
                }break;

                case 'P':
                case 'p':
                case '0':
                {
                    safe_track_stop("WIRELESS COMMAND");
                }break;

                case 'G':
                case 'g':
                {
                    if(safe_track_running)
                    {
                        wireless_uart_send_string("BUSY: STOP BEFORE GRAY SNAPSHOT\r\n");
                    }
                    else
                    {
                        safe_track_sensor_snapshot_send(send_buffer);
                    }
                }break;

                case '?':
                {
                    sprintf(
                        send_buffer,
                        "STATUS run=%u mode=%s base=%d t=%lums det=%u err=%d lost=%u "
                        "L[t=%d c=%d p=%ld] R[t=%d c=%d p=%ld]\r\n",
                        safe_track_running,
                        (SAFE_TRACK_MODE_SLOW == safe_track_mode) ? "SLOW" : "NORMAL",
                        safe_track_active_base,
                        (unsigned long)safe_track_elapsed_ms,
                        safe_track_line_detected,
                        safe_track_error,
                        safe_track_lost_ticks,
                        safe_track_left_target,
                        safe_track_left_count,
                        (long)safe_track_left_pwm,
                        safe_track_right_target,
                        safe_track_right_count,
                        (long)safe_track_right_pwm);
                    wireless_uart_send_string(send_buffer);
                }break;

                case 'H':
                case 'h':
                {
#if SAFE_TRACK_USE_IR8_UART
                    wireless_uart_send_string(
                        "S/1=NORMAL,L/2=SLOW,P/0=STOP,G=IR8,?=STATUS,@GET#; LOST KEEPS STRAIGHT\r\n");
#else
                    wireless_uart_send_string(
                        "S/1=NORMAL,L/2=SLOW,P/0=STOP,G=GRAY,?=STATUS,@GET#; LOST KEEPS STRAIGHT\r\n");
#endif
                }break;

                default:
                {
                    // Ignore CR, LF and unknown bytes.
                }break;
            }
        }

        if(safe_track_running)
        {
            safe_track_targets_update();
            print_elapsed_ms += SAFE_TRACK_CONTROL_PERIOD_MS;
            display_elapsed_ms += SAFE_TRACK_CONTROL_PERIOD_MS;
            if(display_elapsed_ms >= SAFE_TRACK_DISPLAY_PERIOD_MS)
            {
                display_elapsed_ms = 0;
                safe_track_display_update();
            }
            safe_track_black_count = safe_track_black_count_get();
            if(safe_track_black_count > safe_track_black_peak)
            {
                safe_track_black_peak = safe_track_black_count;
            }

            if((safe_track_elapsed_ms >= SAFE_TRACK_LAP_MINIMUM_MS)
                && (safe_track_black_count >= SAFE_TRACK_FINISH_BLACK_MIN))
            {
                safe_track_finish_ticks ++;
            }
            else
            {
                safe_track_finish_ticks = 0;
            }

#if SAFE_TRACK_USE_IR8_UART
            if(safe_track_sensor_age_ms >= SAFE_TRACK_SENSOR_TIMEOUT_MS)
            {
                safe_track_stop("IR8 UART DATA TIMEOUT 100ms");
            }
            else
#endif
            if(safe_track_finish_ticks >= SAFE_TRACK_FINISH_CONFIRM_TICKS)
            {
                finished_lap_ms = safe_track_elapsed_ms;
                recommended_scale_x1000 =
                    (finished_lap_ms * 1000UL) / SAFE_TRACK_LAP_TARGET_MS;
                safe_track_stop("LAP FINISH LINE");
                sprintf(
                    send_buffer,
                    "LAP time=%lums target=%ums black=%u peak=%u scale_x1000=%lu "
                    "NEXT BASE=%lu\r\n",
                    (unsigned long)finished_lap_ms,
                    SAFE_TRACK_LAP_TARGET_MS,
                    safe_track_black_count,
                    safe_track_black_peak,
                    (unsigned long)recommended_scale_x1000,
                    (unsigned long)((safe_track_active_base
                        * recommended_scale_x1000 + 500UL) / 1000UL));
                wireless_uart_send_string(send_buffer);
            }
            else if(print_elapsed_ms >= SAFE_TRACK_PRINT_PERIOD_MS)
            {
                print_elapsed_ms = 0;
                safe_track_vofa_telemetry_send();
            }
        }
        else
        {
            safe_test_motors_stop();
        }

        system_delay_ms(SAFE_TRACK_CONTROL_PERIOD_MS);
    }
}

#else
#error "Unsupported SAFE_TEST_STAGE"
#endif // SAFE_TEST_STAGE

#else

// 当前先执行电机通道与编码器通道映射测试。
// 映射确认后将此宏改为 0，即可重新启用下方保留的速度 PI 程序。
#define MOTOR_ENCODER_MAPPING_TEST      ( 0 )

#if MOTOR_ENCODER_MAPPING_TEST

#define MOTOR_CH1_DIR                   ( A1 )
#define MOTOR_CH1_PWM                   ( PWM_TIM_A0_CH0_A0 )
#define MOTOR_CH2_DIR                   ( B13 )
#define MOTOR_CH2_PWM                   ( PWM_TIM_A0_CH2_B12 )

#define ENCODER_G8_TIMER                ( TIM_G8 )
#define ENCODER_G8_A                    ( TIMG8_ENCODER1_CH1_A26 )
#define ENCODER_G8_B                    ( TIMG8_ENCODER1_CH2_A27 )
#define ENCODER_G9_TIMER                ( TIM_G9 )
#define ENCODER_G9_A                    ( TIMG9_ENCODER1_CH1_B7 )
#define ENCODER_G9_B                    ( TIMG9_ENCODER1_CH2_B9 )

#define MAPPING_TEST_PWM                ( 2000 )
#define MAPPING_START_DELAY_MS          ( 2000 )
#define MAPPING_SAMPLE_PERIOD_MS        ( 100 )
#define MAPPING_SAMPLE_COUNT            ( 10 )
#define MAPPING_CHANNEL_GAP_MS          ( 1000 )

static void mapping_motor_stop (void)
{
    pwm_set_duty(MOTOR_CH1_PWM, 0);
    pwm_set_duty(MOTOR_CH2_PWM, 0);
}

static void mapping_encoder_clear (void)
{
    encoder_clear_count(ENCODER_G8_TIMER);
    encoder_clear_count(ENCODER_G9_TIMER);
}

static void run_motor_channel_mapping_test (
    const char *channel_name,
    gpio_pin_enum dir_pin,
    pwm_channel_enum pwm_pin)
{
    uint32 sample_index;
    int16 g8_count;
    int16 g9_count;
    int32 g8_total = 0;
    int32 g9_total = 0;

    mapping_motor_stop();
    mapping_encoder_clear();
    system_delay_ms(300);

    printf("\r\nSTART %s: PWM=%d, DIR=HIGH.\r\n", channel_name, MAPPING_TEST_PWM);
    gpio_set_level(dir_pin, GPIO_HIGH);
    pwm_set_duty(pwm_pin, MAPPING_TEST_PWM);

    for(sample_index = 0; sample_index < MAPPING_SAMPLE_COUNT; sample_index ++)
    {
        system_delay_ms(MAPPING_SAMPLE_PERIOD_MS);

        g8_count = encoder_get_count(ENCODER_G8_TIMER);
        g9_count = encoder_get_count(ENCODER_G9_TIMER);
        mapping_encoder_clear();

        g8_total += g8_count;
        g9_total += g9_count;

        printf("%s sample=%u G8=%d G9=%d\r\n",
            channel_name,
            (unsigned int)(sample_index + 1),
            g8_count,
            g9_count);
    }

    mapping_motor_stop();
    printf("RESULT %s: G8_TOTAL=%ld G9_TOTAL=%ld\r\n",
        channel_name,
        (long)g8_total,
        (long)g9_total);

    system_delay_ms(MAPPING_CHANNEL_GAP_MS);
}

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);
    debug_init();
    system_delay_ms(300);

    gpio_init(MOTOR_CH1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR_CH2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR_CH1_PWM, 17000, 0);
    pwm_init(MOTOR_CH2_PWM, 17000, 0);

    encoder_quad_init(ENCODER_G8_TIMER, ENCODER_G8_A, ENCODER_G8_B);
    encoder_quad_init(ENCODER_G9_TIMER, ENCODER_G9_A, ENCODER_G9_B);
    interrupt_global_enable(0);

    printf("\r\nMotor/encoder channel mapping test.\r\n");
    printf("Lift wheels. CH1(A0/A1) runs first, then CH2(B12/B13).\r\n");
    printf("Each channel runs %d samples x %d ms, then stops.\r\n",
        MAPPING_SAMPLE_COUNT,
        MAPPING_SAMPLE_PERIOD_MS);

    system_delay_ms(MAPPING_START_DELAY_MS);

    run_motor_channel_mapping_test("CH1_A0_A1", MOTOR_CH1_DIR, MOTOR_CH1_PWM);
    run_motor_channel_mapping_test("CH2_B12_B13", MOTOR_CH2_DIR, MOTOR_CH2_PWM);

    mapping_motor_stop();
    printf("\r\nMapping test finished. Motors stopped.\r\n");
    printf("Send the two RESULT lines for channel pairing and direction setup.\r\n");

    while(true)
    {
        system_delay_ms(1000);
    }
}

#else

// 低速闭环循迹测试：GS08RA 计算黑线偏差，PD 外环产生物理左右轮目标速度，
// 两路编码器 PI 内环分别跟随目标速度。
// 安全提示：启动时无黑线则拒绝起步；丢线 150ms 或运行 3 秒后自动停止。

// -------------------------------- 主板接口 --------------------------------
#define MOTOR_LEFT_DIR                  ( A1 )
#define MOTOR_LEFT_PWM                  ( PWM_TIM_A0_CH0_A0 )
#define MOTOR_LEFT_FORWARD_LEVEL        ( GPIO_HIGH )
#define MOTOR_RIGHT_DIR                 ( B13 )
#define MOTOR_RIGHT_PWM                 ( PWM_TIM_A0_CH2_B12 )
#define MOTOR_RIGHT_FORWARD_LEVEL       ( GPIO_HIGH )

// 2026-07-25 更换电机后重新实测：CH1(A0/A1) 驱动物理左轮/TIMG8；
// CH2(B12/B13) 驱动物理右轮/TIMG9，左右轮在 DIR=HIGH 时均驱动车辆前进。
// 左轮在车辆前进时原始计数为负，右轮原始计数为正，因此仅对左轮反馈取反。
#define ENCODER_LEFT_TIMER              ( TIM_G8 )
#define ENCODER_LEFT_A                  ( TIMG8_ENCODER1_CH1_A26 )
#define ENCODER_LEFT_B                  ( TIMG8_ENCODER1_CH2_A27 )
#define ENCODER_LEFT_SIGN               ( -1 )
#define ENCODER_RIGHT_TIMER             ( TIM_G9 )
#define ENCODER_RIGHT_A                 ( TIMG9_ENCODER1_CH1_B7 )
#define ENCODER_RIGHT_B                 ( TIMG9_ENCODER1_CH2_B9 )
#define ENCODER_RIGHT_SIGN              ( 1 )

// TIMA0 已被电机 PWM 使用，TIMG8/TIMG9 已被编码器使用，因此控制周期使用 TIMG0。
#define CONTROL_PIT                     ( PIT_TIM_G0 )

// -------------------------------- 控制参数 --------------------------------
#define MOTOR_PWM_FREQUENCY_HZ          ( 17000 )
#define CONTROL_PERIOD_MS               ( 10 )
#define PRINT_PERIOD_MS                 ( 100 )

// Wireless UART V2.4 uses UART1: MCU TX B6, MCU RX B5, RTS B2, 115200 baud.
// Commands are single bytes so the receiver never waits for a line ending.
#define WIRELESS_COMMAND_START           ( 'S' )
#define WIRELESS_COMMAND_STOP            ( 'P' )

// Gyroscope UART: B15 is UART7 TX and B16 is UART7 RX.
// The sensor sends: 0A 03 04 angle_H angle_L dps_H dps_L CRC16_L CRC16_H.
#define GYRO_UART_INDEX                  ( UART_7 )
#define GYRO_UART_BAUDRATE               ( 115200 )
#define GYRO_UART_TX_PIN                 ( UART7_TX_B15 )
#define GYRO_UART_RX_PIN                 ( UART7_RX_B16 )
#define GYRO_FRAME_LENGTH                ( 9 )
#define GYRO_PRINT_PERIOD_MS             ( 100 )
#define GYRO_ANGLE_SCALE_X100            ( 977 )

// 单位：每 10ms 的编码器计数。直道采用原速度的三倍，进入弯道后自动降速。
#define TRACK_STRAIGHT_TARGET_COUNT     ( 24 )
#define TRACK_CURVE_TARGET_COUNT        ( 16 )
#define TRACK_LOST_TARGET_COUNT         ( 5 )
#define TRACK_TARGET_MAX                ( 28 )
#define TRACK_STEER_LIMIT               ( 12 )
#define TRACK_STRAIGHT_ERROR_LIMIT      ( 1 )
#define TRACK_STRAIGHT_CORRECTION_LIMIT ( 2 )

// GS08RA 实测：通道 0 在车体左侧，通道 7 在车体右侧；白底黑线时黑线为 0。
// 偏差范围 -7..+7：负数表示黑线在左，正数表示黑线在右。
#define GS08RA_BINARY_THRESHOLD         ( 30 )
#define TRACK_PD_KP_NUM                 ( 2 )
#define TRACK_PD_KD_NUM                 ( 1 )
#define TRACK_PD_GAIN_DIV               ( 2 )
#define TRACK_LOST_STOP_TICKS           ( 150 / CONTROL_PERIOD_MS )

// 直角弯处理方案：
// 方案一：两轮均向前，慢速弧线转约 45 度，避免原地转向直接脱线。
// 方案二：先较快直行约 10 cm，再执行低速原地差速转向。
#define SHARP_TURN_SCHEME_ARC_45        ( 1 )
#define SHARP_TURN_SCHEME_ADVANCE_10CM  ( 2 )
#ifndef SHARP_TURN_SCHEME
#define SHARP_TURN_SCHEME               ( SHARP_TURN_SCHEME_ADVANCE_10CM )
#endif

#define SHARP_TURN_ERROR_THRESHOLD      ( 5 )
#define SHARP_TURN_LOST_THRESHOLD       ( 3 )
#define SHARP_TURN_CONFIRM_TICKS        ( 2 )
#define SHARP_TURN_REACQUIRE_TICKS      ( 3 )
#define SHARP_TURN_NO_GYRO_MIN_TICKS    ( 300 / CONTROL_PERIOD_MS )
#define SHARP_TURN_TIMEOUT_TICKS        ( 6000 / CONTROL_PERIOD_MS )

// 所有转弯速度均明显低于原来的 12 count/10ms，降低云台晃动。
#define SHARP_TURN_ARC_INNER_COUNT      ( 2 )
#define SHARP_TURN_ARC_OUTER_COUNT      ( 6 )
#define SHARP_TURN_ARC_FINE_INNER_COUNT ( 2 )
#define SHARP_TURN_ARC_FINE_OUTER_COUNT ( 4 )
#define SHARP_TURN_PIVOT_COUNT          ( 5 )
#define SHARP_TURN_PIVOT_FINE_COUNT     ( 4 )
#define SHARP_TURN_EXIT_FORWARD_COUNT   ( 3 )
#define SHARP_TURN_ADVANCE_COUNT        ( 7 )

// 方案二按左右轮平均编码器累计值控制前进距离。
// 750 由原 8 cm/600 count 等比例换算为约 10 cm；实车测量后只需微调此值。
#define SHARP_TURN_ADVANCE_10CM_ENCODER_COUNT ( 750 )

// 陀螺仪角度单位为 0.01 度。
#if (SHARP_TURN_SCHEME == SHARP_TURN_SCHEME_ARC_45)
#define SHARP_TURN_GYRO_SLOW_ANGLE      ( 3000 )
#define SHARP_TURN_GYRO_REACQUIRE_ANGLE ( 3500 )
#define SHARP_TURN_GYRO_TARGET_ANGLE    ( 4500 )
#elif (SHARP_TURN_SCHEME == SHARP_TURN_SCHEME_ADVANCE_10CM)
#define SHARP_TURN_GYRO_SLOW_ANGLE      ( 6500 )
#define SHARP_TURN_GYRO_REACQUIRE_ANGLE ( 7500 )
#define SHARP_TURN_GYRO_TARGET_ANGLE    ( 8800 )
#else
#error "Unsupported SHARP_TURN_SCHEME"
#endif

// PWM_DUTY_MAX 为 10000。默认最大限制 3000，即 30%。
#define PWM_OUTPUT_LIMIT                ( 3000 )
#define SPEED_PWM_STATIC                ( 300 )
#define SPEED_PWM_PER_COUNT             ( 70 )

// 整数 PI：Kp 的单位为 PWM/计数；Ki 每个 10ms 控制周期累加一次。
// 首次测试以稳定和安全为主，后续根据串口曲线再调整。
#define SPEED_PI_KP                     ( 30 )
#define SPEED_PI_KI                     ( 1 )
#define SPEED_PI_INTEGRAL_LIMIT         ( 800 )

#if (PWM_OUTPUT_LIMIT > PWM_DUTY_MAX)
#error "PWM_OUTPUT_LIMIT must not exceed PWM_DUTY_MAX"
#endif

typedef struct
{
    int32 integral;
} speed_pi_struct;

typedef enum
{
    TRACK_STATE_NORMAL = 0,
    TRACK_STATE_TURN_LEFT,
    TRACK_STATE_TURN_RIGHT,
} track_state_enum;

typedef enum
{
    SHARP_TURN_PHASE_TURN = 0,
    SHARP_TURN_PHASE_ADVANCE,
    SHARP_TURN_PHASE_EXIT,
} sharp_turn_phase_enum;

static speed_pi_struct left_speed_pi  = { 0 };
static speed_pi_struct right_speed_pi = { 0 };

static volatile int16  left_speed_count  = 0;
static volatile int16  right_speed_count = 0;
static volatile int16  left_target_count = 0;
static volatile int16  right_target_count = 0;
static volatile int32  left_pwm_output   = 0;
static volatile int32  right_pwm_output  = 0;
static volatile uint32 control_tick      = 0;
static volatile bool   test_running      = false;
static bool            wireless_ready    = false;

// Latest valid gyroscope angle. The ISR owns writes; the foreground reads a
// consistent snapshot through gyro_data_get(). Angle = angle_raw / 9.77 deg.
static volatile int16  gyro_angle_raw    = 0;
static volatile bool   gyro_new_data     = false;
static volatile bool   gyro_data_valid   = false;
static uint8           gyro_frame[GYRO_FRAME_LENGTH];
static uint8           gyro_parser_state = 0;
static uint8           gyro_frame_index  = 0;

static int16 line_error       = 0;
static int16 last_line_error  = 0;
static int16 track_correction = 0;
static uint16 line_lost_ticks = 0;
static bool line_detected     = false;
static bool track_safety_stop = false;
static track_state_enum track_state = TRACK_STATE_NORMAL;
static int16 sharp_candidate_direction = 0;
static uint16 sharp_candidate_ticks = 0;
static uint16 sharp_turn_ticks = 0;
static uint16 sharp_turn_phase_ticks = 0;
static uint16 sharp_reacquire_ticks = 0;
static bool sharp_turn_timeout_stop = false;
static bool sharp_turn_gyro_active = false;
static bool sharp_turn_gyro_target_reached = false;
static int32 sharp_turn_start_angle_centidegree = 0;
static int32 sharp_turn_angle_centidegree = 0;
static volatile sharp_turn_phase_enum sharp_turn_phase = SHARP_TURN_PHASE_TURN;
static volatile uint32 sharp_turn_advance_encoder_count = 0;

static int32 limit_int32 (int32 value, int32 minimum, int32 maximum)
{
    if(value < minimum)
    {
        value = minimum;
    }
    else if(value > maximum)
    {
        value = maximum;
    }
    return value;
}

static uint16 gyro_crc16_calculate (const uint8 *buffer, uint8 length)
{
    uint16 crc = 0xFFFF;
    uint8 index;
    uint8 bit;

    for(index = 0; index < length; index ++)
    {
        crc ^= buffer[index];
        for(bit = 0; bit < 8; bit ++)
        {
            if(crc & 1)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static void gyro_frame_parse_byte (uint8 data)
{
    uint16 crc_calculated;
    uint16 crc_received;

    switch(gyro_parser_state)
    {
        case 0:
        {
            if(0x0A == data)
            {
                gyro_frame_index = 0;
                gyro_frame[gyro_frame_index ++] = data;
                gyro_parser_state = 1;
            }
        }break;

        case 1:
        {
            if(0x03 == data)
            {
                gyro_frame[gyro_frame_index ++] = data;
                gyro_parser_state = 2;
            }
            else
            {
                gyro_parser_state = 0;
            }
        }break;

        case 2:
        {
            if(0x04 == data)
            {
                gyro_frame[gyro_frame_index ++] = data;
                gyro_parser_state = 3;
            }
            else
            {
                gyro_parser_state = 0;
            }
        }break;

        default:
        {
            gyro_frame[gyro_frame_index ++] = data;
            if(GYRO_FRAME_LENGTH <= gyro_frame_index)
            {
                crc_calculated = gyro_crc16_calculate(gyro_frame, 7);
                crc_received = (uint16)gyro_frame[7]
                             | ((uint16)gyro_frame[8] << 8);

                if(crc_calculated == crc_received)
                {
                    gyro_angle_raw = (int16)(((uint16)gyro_frame[3] << 8)
                                            | gyro_frame[4]);
                    gyro_data_valid = true;
                    gyro_new_data = true;
                }
                gyro_parser_state = 0;
            }
        }break;
    }
}

static void gyro_uart_callback (uint32 event, void *ptr)
{
    uint8 data;

    (void)ptr;
    if(UART_INTERRUPT_STATE_RX != event)
    {
        return;
    }

    while(uart_query_byte(GYRO_UART_INDEX, &data))
    {
        gyro_frame_parse_byte(data);
    }
}

static void gyro_uart_init (void)
{
    uart_init(
        GYRO_UART_INDEX,
        GYRO_UART_BAUDRATE,
        GYRO_UART_TX_PIN,
        GYRO_UART_RX_PIN);
    uart_set_callback(GYRO_UART_INDEX, gyro_uart_callback, NULL);
    uart_set_interrupt_config(GYRO_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

static bool gyro_data_get (int16 *angle_raw)
{
    uint32 primask;
    bool updated;

    primask = interrupt_global_disable();
    *angle_raw = gyro_angle_raw;
    updated = gyro_new_data;
    gyro_new_data = false;
    interrupt_global_enable(primask);

    return updated;
}

static bool gyro_angle_snapshot_get (int16 *angle_raw)
{
    uint32 primask;
    bool valid;

    primask = interrupt_global_disable();
    *angle_raw = gyro_angle_raw;
    valid = gyro_data_valid;
    interrupt_global_enable(primask);

    return valid;
}

static int32 gyro_angle_centidegree_get (int16 angle_raw)
{
    int32 angle_centidegree;

    // angle_deg = angle_raw / 9.77 = angle_raw * 100 / 977.
    angle_centidegree = ((int32)angle_raw * 10000) / GYRO_ANGLE_SCALE_X100;

    while(angle_centidegree >= 18000)
    {
        angle_centidegree -= 36000;
    }
    while(angle_centidegree < -18000)
    {
        angle_centidegree += 36000;
    }

    return angle_centidegree;
}

static int32 gyro_angle_delta_centidegree_get (int32 current, int32 reference)
{
    int32 delta = current - reference;

    while(delta >= 18000)
    {
        delta -= 36000;
    }
    while(delta < -18000)
    {
        delta += 36000;
    }

    return delta;
}

static void sharp_turn_gyro_reference_reset (void)
{
    int16 gyro_angle;

    sharp_turn_gyro_active = gyro_angle_snapshot_get(&gyro_angle);
    sharp_turn_gyro_target_reached = false;
    sharp_turn_angle_centidegree = 0;
    sharp_turn_start_angle_centidegree = sharp_turn_gyro_active
        ? gyro_angle_centidegree_get(gyro_angle)
        : 0;
}

static int32 speed_feedforward_calculate (int16 target)
{
    int32 magnitude;

    if(0 == target)
    {
        return 0;
    }

    magnitude = (target > 0) ? target : -target;
    magnitude = SPEED_PWM_STATIC + SPEED_PWM_PER_COUNT * magnitude;

    // 返回无符号幅值；speed_pi_calculate 根据 target 正负添加输出方向。
    return magnitude;
}

// 取最左、最右黑色通道，按官方公式计算二倍偏差：left + right - 7。
static bool line_error_calculate (int16 *error)
{
    int16 left = -1;
    int16 right = -1;
    uint8 index;

    for(index = 0; index < GS08A_CHANNEL_NUM; index ++)
    {
        if(0 == gs08ra_bin_val[index])
        {
            left = index;
            break;
        }
    }

    for(index = GS08A_CHANNEL_NUM; index > 0; index --)
    {
        if(0 == gs08ra_bin_val[index - 1])
        {
            right = index - 1;
            break;
        }
    }

    if((left < 0) || (right < 0))
    {
        return false;
    }

    *error = left + right - (GS08A_CHANNEL_NUM - 1);
    return true;
}

static bool sensors_are_all_black (void)
{
    uint8 index;

    for(index = 0; index < GS08A_CHANNEL_NUM; index ++)
    {
        if(0 != gs08ra_bin_val[index])
        {
            return false;
        }
    }

    return true;
}

static void track_targets_set (int16 base_target, int16 correction)
{
    // 黑线在右侧时 correction>0：物理左轮加速、物理右轮减速，使车辆右转。
    left_target_count = (int16)limit_int32(
        base_target + correction,
        0,
        TRACK_TARGET_MAX);
    right_target_count = (int16)limit_int32(
        base_target - correction,
        0,
        TRACK_TARGET_MAX);
}

static bool center_line_is_detected (void)
{
    return ((0 == gs08ra_bin_val[3]) || (0 == gs08ra_bin_val[4]));
}

static void sharp_turn_targets_apply (void)
{
#if (SHARP_TURN_SCHEME == SHARP_TURN_SCHEME_ARC_45)
    int16 inner_speed;
    int16 outer_speed;
#else
    int16 turn_speed;
#endif

    if(SHARP_TURN_PHASE_ADVANCE == sharp_turn_phase)
    {
        left_target_count = SHARP_TURN_ADVANCE_COUNT;
        right_target_count = SHARP_TURN_ADVANCE_COUNT;
        track_correction = 0;
        return;
    }

    if(sharp_turn_gyro_target_reached)
    {
        // 达到目标角度后低速向前，等待中心探头稳定捕获新赛道。
        left_target_count = SHARP_TURN_EXIT_FORWARD_COUNT;
        right_target_count = SHARP_TURN_EXIT_FORWARD_COUNT;
        track_correction = 0;
        return;
    }

#if (SHARP_TURN_SCHEME == SHARP_TURN_SCHEME_ARC_45)
    inner_speed = SHARP_TURN_ARC_INNER_COUNT;
    outer_speed = SHARP_TURN_ARC_OUTER_COUNT;
    if(sharp_turn_gyro_active
    && (sharp_turn_angle_centidegree >= SHARP_TURN_GYRO_SLOW_ANGLE))
    {
        inner_speed = SHARP_TURN_ARC_FINE_INNER_COUNT;
        outer_speed = SHARP_TURN_ARC_FINE_OUTER_COUNT;
    }

    if(TRACK_STATE_TURN_LEFT == track_state)
    {
        // 左转时两轮均向前，左内轮慢、右外轮快，形成平滑前进弧线。
        left_target_count = inner_speed;
        right_target_count = outer_speed;
        track_correction = inner_speed - outer_speed;
    }
    else
    {
        // 右转时两轮均向前，右内轮慢、左外轮快，形成平滑前进弧线。
        left_target_count = outer_speed;
        right_target_count = inner_speed;
        track_correction = outer_speed - inner_speed;
    }
#else
    turn_speed = SHARP_TURN_PIVOT_COUNT;
    if(sharp_turn_gyro_active
    && (sharp_turn_angle_centidegree >= SHARP_TURN_GYRO_SLOW_ANGLE))
    {
        turn_speed = SHARP_TURN_PIVOT_FINE_COUNT;
    }

    if(TRACK_STATE_TURN_LEFT == track_state)
    {
        left_target_count = -turn_speed;
        right_target_count = turn_speed;
        track_correction = -turn_speed;
    }
    else
    {
        left_target_count = turn_speed;
        right_target_count = -turn_speed;
        track_correction = turn_speed;
    }
#endif
}

static void sharp_turn_enter (track_state_enum new_state)
{
    track_state = new_state;
    sharp_candidate_direction = 0;
    sharp_candidate_ticks = 0;
    sharp_turn_ticks = 0;
    sharp_turn_phase_ticks = 0;
    sharp_reacquire_ticks = 0;
    line_lost_ticks = 0;
    left_speed_pi.integral = 0;
    right_speed_pi.integral = 0;
    sharp_turn_advance_encoder_count = 0;
#if (SHARP_TURN_SCHEME == SHARP_TURN_SCHEME_ADVANCE_10CM)
    sharp_turn_phase = SHARP_TURN_PHASE_ADVANCE;
#else
    sharp_turn_phase = SHARP_TURN_PHASE_TURN;
#endif
    sharp_turn_gyro_reference_reset();
    sharp_turn_targets_apply();

    printf("RIGHT_ANGLE: enter %s turn.\r\n",
        (TRACK_STATE_TURN_LEFT == track_state) ? "LEFT" : "RIGHT");
#if (SHARP_TURN_SCHEME == SHARP_TURN_SCHEME_ARC_45)
    printf("RIGHT_ANGLE: scheme 1, slow forward arc to about 45 deg.\r\n");
#else
    printf("RIGHT_ANGLE: scheme 2, advance about 10 cm before slow pivot.\r\n");
#endif
    if(sharp_turn_gyro_active)
    {
        printf("RIGHT_ANGLE: gyro start=%ld centideg.\r\n",
            (long)sharp_turn_start_angle_centidegree);
    }
    else
    {
        printf("RIGHT_ANGLE: gyro unavailable, keep line-sensor fallback.\r\n");
    }
}

static bool sharp_turn_candidate_update (int16 error)
{
    int16 direction = 0;

    if(error <= -SHARP_TURN_ERROR_THRESHOLD)
    {
        direction = -1;
    }
    else if(error >= SHARP_TURN_ERROR_THRESHOLD)
    {
        direction = 1;
    }

    if(0 == direction)
    {
        sharp_candidate_direction = 0;
        sharp_candidate_ticks = 0;
        return false;
    }

    if(direction == sharp_candidate_direction)
    {
        if(sharp_candidate_ticks < 0xFFFF)
        {
            sharp_candidate_ticks ++;
        }
    }
    else
    {
        sharp_candidate_direction = direction;
        sharp_candidate_ticks = 1;
    }

    if(sharp_candidate_ticks >= SHARP_TURN_CONFIRM_TICKS)
    {
        sharp_turn_enter((direction < 0) ? TRACK_STATE_TURN_LEFT : TRACK_STATE_TURN_RIGHT);
        return true;
    }

    return false;
}

static void track_update (void)
{
    int16 new_error;
    int16 derivative;
    int16 gyro_angle;
    int16 base_target;
    int32 correction;
    int32 gyro_current_angle;
    int32 gyro_delta_angle;
    bool gyro_reacquire_ready;

    line_detected = line_error_calculate(&new_error);

    if(TRACK_STATE_NORMAL != track_state)
    {
        if(sharp_turn_ticks < 0xFFFF)
        {
            sharp_turn_ticks ++;
        }
        if(sharp_turn_phase_ticks < 0xFFFF)
        {
            sharp_turn_phase_ticks ++;
        }

        if((SHARP_TURN_PHASE_ADVANCE == sharp_turn_phase)
        && (sharp_turn_advance_encoder_count >= SHARP_TURN_ADVANCE_10CM_ENCODER_COUNT))
        {
            sharp_turn_phase = SHARP_TURN_PHASE_TURN;
            sharp_turn_phase_ticks = 0;
            left_speed_pi.integral = 0;
            right_speed_pi.integral = 0;
            sharp_turn_gyro_reference_reset();
            printf("RIGHT_ANGLE: advance finished at %lu encoder counts, start slow pivot.\r\n",
                (unsigned long)sharp_turn_advance_encoder_count);
        }

        if((SHARP_TURN_PHASE_ADVANCE != sharp_turn_phase)
        && sharp_turn_gyro_active
        && gyro_angle_snapshot_get(&gyro_angle))
        {
            gyro_current_angle = gyro_angle_centidegree_get(gyro_angle);
            gyro_delta_angle = gyro_angle_delta_centidegree_get(
                gyro_current_angle,
                sharp_turn_start_angle_centidegree);
            sharp_turn_angle_centidegree = (gyro_delta_angle < 0)
                ? -gyro_delta_angle
                : gyro_delta_angle;

            if(sharp_turn_angle_centidegree >= SHARP_TURN_GYRO_TARGET_ANGLE)
            {
                sharp_turn_gyro_target_reached = true;
                sharp_turn_phase = SHARP_TURN_PHASE_EXIT;
            }
        }

        if(line_detected)
        {
            line_error = new_error;
            last_line_error = new_error;
        }
        else
        {
            line_error = last_line_error;
        }

        sharp_turn_targets_apply();
        gyro_reacquire_ready = (SHARP_TURN_PHASE_ADVANCE != sharp_turn_phase)
            && ((sharp_turn_gyro_active
              && (sharp_turn_angle_centidegree >= SHARP_TURN_GYRO_REACQUIRE_ANGLE))
             || (!sharp_turn_gyro_active
              && (sharp_turn_phase_ticks >= SHARP_TURN_NO_GYRO_MIN_TICKS)));

        if(gyro_reacquire_ready
        && line_detected
        && center_line_is_detected()
        && !sensors_are_all_black())
        {
            if(sharp_reacquire_ticks < 0xFFFF)
            {
                sharp_reacquire_ticks ++;
            }

            if(sharp_reacquire_ticks >= SHARP_TURN_REACQUIRE_TICKS)
            {
                track_state = TRACK_STATE_NORMAL;
                sharp_turn_ticks = 0;
                sharp_turn_phase_ticks = 0;
                sharp_reacquire_ticks = 0;
                line_lost_ticks = 0;
                left_speed_pi.integral = 0;
                right_speed_pi.integral = 0;
                track_correction = 0;
                track_targets_set(TRACK_STRAIGHT_TARGET_COUNT, 0);
                printf("RIGHT_ANGLE: center line reacquired at %ld centideg, resume normal tracking.\r\n",
                    (long)sharp_turn_angle_centidegree);
                sharp_turn_gyro_active = false;
                sharp_turn_gyro_target_reached = false;
                sharp_turn_start_angle_centidegree = 0;
                sharp_turn_angle_centidegree = 0;
                sharp_turn_phase = SHARP_TURN_PHASE_TURN;
                sharp_turn_advance_encoder_count = 0;
                return;
            }
        }
        else
        {
            sharp_reacquire_ticks = 0;
        }

        if(sharp_turn_ticks >= SHARP_TURN_TIMEOUT_TICKS)
        {
            left_target_count = 0;
            right_target_count = 0;
            track_correction = 0;
            sharp_turn_timeout_stop = true;
            track_safety_stop = true;
        }
        return;
    }

    if(line_detected)
    {
        line_lost_ticks = 0;

        // 弯角可能短暂呈全黑；若进入全黑前已偏到边缘，优先按最后方向转弯。
        if(sensors_are_all_black()
        && ((last_line_error <= -SHARP_TURN_ERROR_THRESHOLD)
         || (last_line_error >= SHARP_TURN_ERROR_THRESHOLD)))
        {
            sharp_turn_enter((last_line_error < 0) ? TRACK_STATE_TURN_LEFT : TRACK_STATE_TURN_RIGHT);
            return;
        }

        derivative = new_error - last_line_error;
        line_error = new_error;

        if(sharp_turn_candidate_update(line_error))
        {
            return;
        }

        correction = TRACK_PD_KP_NUM * line_error
                   + TRACK_PD_KD_NUM * derivative;
        correction /= TRACK_PD_GAIN_DIV;
        track_correction = (int16)limit_int32(
            correction,
            -TRACK_STEER_LIMIT,
            TRACK_STEER_LIMIT);

        if((line_error >= -TRACK_STRAIGHT_ERROR_LIMIT)
        && (line_error <= TRACK_STRAIGHT_ERROR_LIMIT)
        && (track_correction >= -TRACK_STRAIGHT_CORRECTION_LIMIT)
        && (track_correction <= TRACK_STRAIGHT_CORRECTION_LIMIT))
        {
            base_target = TRACK_STRAIGHT_TARGET_COUNT;
        }
        else
        {
            // 偏差或变化率较大时视为弯道，降低平均轮速以保持车体和云台稳定。
            base_target = TRACK_CURVE_TARGET_COUNT;
        }

        last_line_error = line_error;
        track_targets_set(base_target, track_correction);
    }
    else
    {
        // 黑线从边缘消失通常意味着已经到达直角拐点，直接进入对应方向转向。
        if((last_line_error <= -SHARP_TURN_LOST_THRESHOLD)
        || (last_line_error >= SHARP_TURN_LOST_THRESHOLD))
        {
            sharp_turn_enter((last_line_error < 0) ? TRACK_STATE_TURN_LEFT : TRACK_STATE_TURN_RIGHT);
            return;
        }

        sharp_candidate_direction = 0;
        sharp_candidate_ticks = 0;

        if(line_lost_ticks < 0xFFFF)
        {
            line_lost_ticks ++;
        }

        line_error = last_line_error;

        if(line_lost_ticks >= TRACK_LOST_STOP_TICKS)
        {
            left_target_count = 0;
            right_target_count = 0;
            track_correction = 0;
            track_safety_stop = true;
        }
        else
        {
            // 短暂丢线时降速，并按最后一次偏差方向寻找黑线。
            track_correction = (int16)limit_int32(
                last_line_error,
                -TRACK_STEER_LIMIT,
                TRACK_STEER_LIMIT);
            track_targets_set(TRACK_LOST_TARGET_COUNT, track_correction);
        }
    }
}

static void motor_set_output (
    gpio_pin_enum dir_pin,
    pwm_channel_enum pwm_pin,
    gpio_level_enum forward_level,
    int32 output)
{
    uint32 duty;

    output = limit_int32(output, -PWM_OUTPUT_LIMIT, PWM_OUTPUT_LIMIT);

    if(output >= 0)
    {
        gpio_set_level(dir_pin, forward_level);
        duty = (uint32)output;
    }
    else
    {
        gpio_set_level(dir_pin, (GPIO_HIGH == forward_level) ? GPIO_LOW : GPIO_HIGH);
        duty = (uint32)(-output);
    }

    pwm_set_duty(pwm_pin, duty);
}

static void motor_stop (void)
{
    pwm_set_duty(MOTOR_LEFT_PWM, 0);
    pwm_set_duty(MOTOR_RIGHT_PWM, 0);
}

static int32 speed_pi_calculate (
    speed_pi_struct *pi,
    int16 target,
    int16 measured,
    int32 feedforward)
{
    int32 error;
    int32 output;

    if(0 == target)
    {
        pi->integral = 0;
        return 0;
    }

    error = (int32)target - (int32)measured;
    pi->integral = limit_int32(
        pi->integral + error,
        -SPEED_PI_INTEGRAL_LIMIT,
        SPEED_PI_INTEGRAL_LIMIT);

    if(target > 0)
    {
        output = feedforward
               + SPEED_PI_KP * error
               + SPEED_PI_KI * pi->integral;

        // 正向测试时不允许 PI 因超调突然反转电机。
        output = limit_int32(output, 0, PWM_OUTPUT_LIMIT);
    }
    else
    {
        output = -feedforward
               + SPEED_PI_KP * error
               + SPEED_PI_KI * pi->integral;

        // 反向测试时同样禁止控制器跨方向输出。
        output = limit_int32(output, -PWM_OUTPUT_LIMIT, 0);
    }

    return output;
}

static void speed_control_callback (uint32 event, void *ptr)
{
    int16 current_left_target;
    int16 current_right_target;
    int32 left_distance_count;
    int32 right_distance_count;

    (void)event;
    (void)ptr;

    left_speed_count  = (int16)(ENCODER_LEFT_SIGN * encoder_get_count(ENCODER_LEFT_TIMER));
    right_speed_count = (int16)(ENCODER_RIGHT_SIGN * encoder_get_count(ENCODER_RIGHT_TIMER));
    encoder_clear_count(ENCODER_LEFT_TIMER);
    encoder_clear_count(ENCODER_RIGHT_TIMER);

    if(!test_running)
    {
        left_pwm_output  = 0;
        right_pwm_output = 0;
        motor_stop();
        return;
    }

    if(SHARP_TURN_PHASE_ADVANCE == sharp_turn_phase)
    {
        left_distance_count = left_speed_count;
        right_distance_count = right_speed_count;
        if(left_distance_count < 0)
        {
            left_distance_count = -left_distance_count;
        }
        if(right_distance_count < 0)
        {
            right_distance_count = -right_distance_count;
        }
        sharp_turn_advance_encoder_count +=
            (uint32)((left_distance_count + right_distance_count) / 2);
    }

    current_left_target = left_target_count;
    current_right_target = right_target_count;

    left_pwm_output = speed_pi_calculate(
        &left_speed_pi,
        current_left_target,
        left_speed_count,
        speed_feedforward_calculate(current_left_target));

    right_pwm_output = speed_pi_calculate(
        &right_speed_pi,
        current_right_target,
        right_speed_count,
        speed_feedforward_calculate(current_right_target));

    motor_set_output(
        MOTOR_LEFT_DIR,
        MOTOR_LEFT_PWM,
        MOTOR_LEFT_FORWARD_LEVEL,
        left_pwm_output);
    motor_set_output(
        MOTOR_RIGHT_DIR,
        MOTOR_RIGHT_PWM,
        MOTOR_RIGHT_FORWARD_LEVEL,
        right_pwm_output);

    control_tick ++;
}

static void car_stop (const char *reason)
{
    test_running = false;
    left_target_count = 0;
    right_target_count = 0;
    left_pwm_output = 0;
    right_pwm_output = 0;
    left_speed_pi.integral = 0;
    right_speed_pi.integral = 0;
    track_state = TRACK_STATE_NORMAL;
    sharp_candidate_direction = 0;
    sharp_candidate_ticks = 0;
    sharp_turn_ticks = 0;
    sharp_turn_phase_ticks = 0;
    sharp_reacquire_ticks = 0;
    sharp_turn_gyro_active = false;
    sharp_turn_gyro_target_reached = false;
    sharp_turn_start_angle_centidegree = 0;
    sharp_turn_angle_centidegree = 0;
    sharp_turn_phase = SHARP_TURN_PHASE_TURN;
    sharp_turn_advance_encoder_count = 0;
    motor_stop();

    printf("STOP: %s. Motors stopped. Press KEY1 or send S to start again.\r\n", reason);

    if(wireless_ready)
    {
        wireless_uart_send_string("STOP: ");
        wireless_uart_send_string(reason);
        wireless_uart_send_string("\r\n");
    }
}

static bool car_start (void)
{
    gs08ra_scan_read();

    line_error = 0;
    last_line_error = 0;
    line_lost_ticks = 0;
    track_safety_stop = false;
    track_state = TRACK_STATE_NORMAL;
    sharp_candidate_direction = 0;
    sharp_candidate_ticks = 0;
    sharp_turn_ticks = 0;
    sharp_turn_phase_ticks = 0;
    sharp_reacquire_ticks = 0;
    sharp_turn_gyro_active = false;
    sharp_turn_gyro_target_reached = false;
    sharp_turn_start_angle_centidegree = 0;
    sharp_turn_angle_centidegree = 0;
    sharp_turn_phase = SHARP_TURN_PHASE_TURN;
    sharp_turn_advance_encoder_count = 0;
    sharp_turn_timeout_stop = false;
    track_update();

    if(!line_detected)
    {
        left_target_count = 0;
        right_target_count = 0;
        motor_stop();
        printf("START ABORTED: no black line detected. Reposition the car, then press KEY1 or send S.\r\n");
        if(wireless_ready)
        {
            wireless_uart_send_string("START ABORTED: NO LINE\r\n");
        }
        return false;
    }

    encoder_clear_count(ENCODER_LEFT_TIMER);
    encoder_clear_count(ENCODER_RIGHT_TIMER);
    left_speed_pi.integral = 0;
    right_speed_pi.integral = 0;
    control_tick = 0;
    test_running = true;

    printf("START BIN=%u%u%u%u%u%u%u%u error=%d Ltarget=%d Rtarget=%d\r\n",
        gs08ra_bin_val[0],
        gs08ra_bin_val[1],
        gs08ra_bin_val[2],
        gs08ra_bin_val[3],
        gs08ra_bin_val[4],
        gs08ra_bin_val[5],
        gs08ra_bin_val[6],
        gs08ra_bin_val[7],
        line_error,
        left_target_count,
        right_target_count);

    if(wireless_ready)
    {
        wireless_uart_send_string("STARTED\r\n");
    }

    return true;
}

static void wireless_status_send (void)
{
    if(!wireless_ready)
    {
        return;
    }

    wireless_uart_send_string(test_running
        ? "STATUS: RUNNING\r\n"
        : "STATUS: IDLE\r\n");
}

static void wireless_command_poll (void)
{
    static uint8 receive_buffer[WIRELESS_UART_BUFFER_SIZE];
    uint32 data_length;
    uint32 index;
    uint8 command;

    if(!wireless_ready)
    {
        return;
    }

    data_length = wireless_uart_read_buffer(
        receive_buffer,
        WIRELESS_UART_BUFFER_SIZE);

    for(index = 0; index < data_length; index ++)
    {
        command = receive_buffer[index];

        switch(command)
        {
            case WIRELESS_COMMAND_START:
            case 's':
            case '1':
            {
                if(test_running)
                {
                    wireless_status_send();
                }
                else
                {
                    car_start();
                }
            }break;

            case WIRELESS_COMMAND_STOP:
            case 'p':
            case '0':
            {
                if(test_running)
                {
                    car_stop("wireless stop command");
                }
                else
                {
                    wireless_status_send();
                }
            }break;

            case '?':
            {
                wireless_status_send();
            }break;

            case 'H':
            case 'h':
            {
                wireless_uart_send_string(
                    "COMMANDS: S/1=START, P/0=STOP, ?=STATUS\r\n");
            }break;

            default:
            {
                // Ignore CR/LF and unknown bytes. Commands are intentionally single-byte.
            }break;
        }
    }
}

int main (void)
{
    uint16 print_elapsed_ms = 0;
    uint16 gyro_print_elapsed_ms = 0;
    key_state_enum key_state;
    int16 gyro_angle;
    int32 gyro_angle_centidegree;
    uint32 gyro_angle_abs_centidegree;

    clock_init(SYSTEM_CLOCK_80M);                                               // 时钟配置及系统初始化<务必保留>

    // 复位释放后尽早把 PWM 置 0；按住 RESET 时仍需硬件下拉保证停机。
    gpio_init(MOTOR_LEFT_DIR, GPO, MOTOR_LEFT_FORWARD_LEVEL, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_DIR, GPO, MOTOR_RIGHT_FORWARD_LEVEL, GPO_PUSH_PULL);
    pwm_init(MOTOR_LEFT_PWM, MOTOR_PWM_FREQUENCY_HZ, 0);
    pwm_init(MOTOR_RIGHT_PWM, MOTOR_PWM_FREQUENCY_HZ, 0);

    debug_init();                                                               // 调试串口信息初始化
    system_delay_ms(300);                                                       // 等待主板外设电源稳定

    encoder_quad_init(ENCODER_LEFT_TIMER, ENCODER_LEFT_A, ENCODER_LEFT_B);
    encoder_quad_init(ENCODER_RIGHT_TIMER, ENCODER_RIGHT_A, ENCODER_RIGHT_B);
    gs08ra_init();
    gs08ra_set_threshold(GS08RA_BINARY_THRESHOLD);
    key_init(CONTROL_PERIOD_MS);
	gpio_init(A14, GPO, GPIO_LOW, GPO_PUSH_PULL);

    if(0 == wireless_uart_init())
    {
        wireless_ready = true;
    }
    else
    {
        printf("WARNING: wireless UART init failed; KEY1 control remains available.\r\n");
    }

    gyro_uart_init();

    pit_ms_init(CONTROL_PIT, CONTROL_PERIOD_MS, speed_control_callback, NULL);
    interrupt_global_enable(0);

    printf("\r\nGS08RA key-controlled line-follow.\r\n");
    printf("Physical mapping: LEFT=CH1/TIMG8, RIGHT=CH2/TIMG9.\r\n");
    printf("KEY1(A30): press once to start, press again to stop.\r\n");
    printf("Wireless UART1(B5/B6): S/1=start, P/0=stop, ?=status.\r\n");
    printf("Gyro UART7: B16=RX, B15=TX, 115200. Sensor TX -> B16.\r\n");
    printf("Lost line %d ms -> forced stop.\r\n",
        TRACK_LOST_STOP_TICKS * CONTROL_PERIOD_MS);
#if (SHARP_TURN_SCHEME == SHARP_TURN_SCHEME_ARC_45)
    printf("Right-angle scheme 1: forward arc inner=%d, outer=%d count/10ms.\r\n",
        SHARP_TURN_ARC_INNER_COUNT,
        SHARP_TURN_ARC_OUTER_COUNT);
#else
    printf("Right-angle scheme 2: advance target=%lu counts, speed=%d, pivot=%d.\r\n",
        (unsigned long)SHARP_TURN_ADVANCE_10CM_ENCODER_COUNT,
        SHARP_TURN_ADVANCE_COUNT,
        SHARP_TURN_PIVOT_COUNT);
#endif
    printf("Right-angle: edge confirm %d ms, timeout=%d ms.\r\n",
        SHARP_TURN_CONFIRM_TICKS * CONTROL_PERIOD_MS,
        SHARP_TURN_TIMEOUT_TICKS * CONTROL_PERIOD_MS);
    printf("Right-angle gyro: slow=%ld, reacquire=%ld, target=%ld centideg.\r\n",
        (long)SHARP_TURN_GYRO_SLOW_ANGLE,
        (long)SHARP_TURN_GYRO_REACQUIRE_ANGLE,
        (long)SHARP_TURN_GYRO_TARGET_ANGLE);
    printf("Tracking speed: straight=%d, curve=%d count/%dms, steering limit=%d, PWM limit=%d.\r\n",
        TRACK_STRAIGHT_TARGET_COUNT,
        TRACK_CURVE_TARGET_COUNT,
        CONTROL_PERIOD_MS,
        TRACK_STEER_LIMIT,
        PWM_OUTPUT_LIMIT);
    printf("IDLE: place the car on the black line, then press KEY1 or send S.\r\n");

    if(wireless_ready)
    {
        wireless_uart_send_string("\r\nLINE CAR READY\r\n");
        wireless_uart_send_string("COMMANDS: S/1=START, P/0=STOP, ?=STATUS\r\n");
        wireless_status_send();
    }

    while(true)
    {
        system_delay_ms(CONTROL_PERIOD_MS);
        wireless_command_poll();
        key_scanner();
        key_state = key_get_state(KEY_1);

        gyro_print_elapsed_ms += CONTROL_PERIOD_MS;
        if(gyro_print_elapsed_ms >= GYRO_PRINT_PERIOD_MS)
        {
            gyro_print_elapsed_ms = 0;
            if(gyro_data_get(&gyro_angle))
            {
                gyro_angle_centidegree = gyro_angle_centidegree_get(gyro_angle);
                gyro_angle_abs_centidegree = (uint32)((gyro_angle_centidegree < 0)
                    ? -gyro_angle_centidegree
                    : gyro_angle_centidegree);
                printf("GYRO angle=%c%lu.%02lu deg\r\n",
                    (gyro_angle_centidegree < 0) ? '-' : '+',
                    (unsigned long)(gyro_angle_abs_centidegree / 100),
                    (unsigned long)(gyro_angle_abs_centidegree % 100));
            }
        }

        if(KEY_SHORT_PRESS == key_state)
        {
            key_clear_state(KEY_1);

            if(test_running)
            {
                car_stop("KEY1 pressed");
            }
            else
            {
                car_start();
            }

            print_elapsed_ms = 0;
        }

        if(!test_running)
        {
            continue;
        }

        gs08ra_scan_read();
        track_update();

        if(track_safety_stop)
        {
            car_stop(sharp_turn_timeout_stop
                ? "right-angle turn timeout"
                : "line lost safety");
            continue;
        }

        print_elapsed_ms += CONTROL_PERIOD_MS;
        if(print_elapsed_ms >= PRINT_PERIOD_MS)
        {
            print_elapsed_ms = 0;
            printf("t=%ums state=%d BIN=%u%u%u%u%u%u%u%u det=%u err=%d turn=%d "
                   "lost=%u sharp=%u L[t=%d c=%d p=%d] R[t=%d c=%d p=%d]\r\n",
                (unsigned int)(control_tick * CONTROL_PERIOD_MS),
                track_state,
                gs08ra_bin_val[0],
                gs08ra_bin_val[1],
                gs08ra_bin_val[2],
                gs08ra_bin_val[3],
                gs08ra_bin_val[4],
                gs08ra_bin_val[5],
                gs08ra_bin_val[6],
                gs08ra_bin_val[7],
                line_detected ? 1 : 0,
                line_error,
                track_correction,
                line_lost_ticks,
                sharp_turn_ticks,
                left_target_count,
                left_speed_count,
                (int)left_pwm_output,
                right_target_count,
                right_speed_count,
                (int)right_pwm_output);
        }
    }
}

#endif // MOTOR_ENCODER_MAPPING_TEST

#endif // NEW_MOTOR_SAFE_WIRELESS_TEST

// **************************** 代码区域 ****************************
