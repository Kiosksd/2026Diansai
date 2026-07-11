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
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


// **************************** 代码区域 ****************************

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

// 本程序用于验证：
// 1. 两路 AB 相编码器在固定 10ms 周期内的速度计数
// 2. 两路 DRV8701E 电机能否分别跟随相同的目标速度
// 3. 左右轮 PI 输出是否稳定
//
// 安全提示：第一次运行请架空车轮。上电后等待 2 秒，电机运行 5 秒后自动停止。

// -------------------------------- 主板接口 --------------------------------
#define MOTOR_LEFT_DIR                  ( A1 )
#define MOTOR_LEFT_PWM                  ( PWM_TIM_A0_CH0_A0 )
#define MOTOR_LEFT_FORWARD_LEVEL        ( GPIO_HIGH )
#define MOTOR_RIGHT_DIR                 ( B13 )
#define MOTOR_RIGHT_PWM                 ( PWM_TIM_A0_CH2_B12 )
#define MOTOR_RIGHT_FORWARD_LEVEL       ( GPIO_HIGH )

// 2026-07-11 实车映射测试：A0/A1 驱动 TIMG9 编码器；B12/B13 驱动 TIMG8 编码器。
// 物理方向复测：左右轮在 DIR=HIGH 时均驱动车辆前进。
// 左轮在车辆前进时原始计数为正；右轮原始计数为负，因此仅对右轮反馈取反。
#define ENCODER_LEFT_TIMER              ( TIM_G9 )
#define ENCODER_LEFT_A                  ( TIMG9_ENCODER1_CH1_B7 )
#define ENCODER_LEFT_B                  ( TIMG9_ENCODER1_CH2_B9 )
#define ENCODER_LEFT_SIGN               ( 1 )
#define ENCODER_RIGHT_TIMER             ( TIM_G8 )
#define ENCODER_RIGHT_A                 ( TIMG8_ENCODER1_CH1_A26 )
#define ENCODER_RIGHT_B                 ( TIMG8_ENCODER1_CH2_A27 )
#define ENCODER_RIGHT_SIGN              ( -1 )

// TIMA0 已被电机 PWM 使用，TIMG8/TIMG9 已被编码器使用，因此控制周期使用 TIMG0。
#define CONTROL_PIT                     ( PIT_TIM_G0 )

// -------------------------------- 测试参数 --------------------------------
#define MOTOR_PWM_FREQUENCY_HZ          ( 17000 )
#define CONTROL_PERIOD_MS               ( 10 )
#define PRINT_PERIOD_MS                 ( 100 )
#define START_DELAY_MS                  ( 2000 )
#define TEST_DURATION_MS                ( 5000 )

// 单位：每 10ms 的编码器计数。速度内环已在目标 20 时完成实车验证。
#define LEFT_TARGET_COUNT               ( 20 )
#define RIGHT_TARGET_COUNT              ( 20 )

// PWM_DUTY_MAX 为 10000。默认最大限制 3000，即 30%。
#define PWM_OUTPUT_LIMIT                ( 3000 )
#define LEFT_PWM_FEEDFORWARD            ( 1500 )
#define RIGHT_PWM_FEEDFORWARD           ( 1500 )

// 整数 PI：Kp 的单位为 PWM/计数；Ki 每个 10ms 控制周期累加一次。
// 首次测试以稳定和安全为主，后续根据串口曲线再调整。
#define SPEED_PI_KP                     ( 30 )
#define SPEED_PI_KI                     ( 1 )
#define SPEED_PI_INTEGRAL_LIMIT         ( 800 )

#define TEST_CONTROL_TICKS              ( TEST_DURATION_MS / CONTROL_PERIOD_MS )

#if ((TEST_DURATION_MS % CONTROL_PERIOD_MS) != 0)
#error "TEST_DURATION_MS must be divisible by CONTROL_PERIOD_MS"
#endif

#if (PWM_OUTPUT_LIMIT > PWM_DUTY_MAX)
#error "PWM_OUTPUT_LIMIT must not exceed PWM_DUTY_MAX"
#endif

typedef struct
{
    int32 integral;
} speed_pi_struct;

static speed_pi_struct left_speed_pi  = { 0 };
static speed_pi_struct right_speed_pi = { 0 };

static volatile int16  left_speed_count  = 0;
static volatile int16  right_speed_count = 0;
static volatile int32  left_pwm_output   = 0;
static volatile int32  right_pwm_output  = 0;
static volatile uint32 control_tick      = 0;
static volatile bool   test_running      = false;
static volatile bool   test_finished     = false;

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

    if(control_tick >= TEST_CONTROL_TICKS)
    {
        test_running  = false;
        test_finished = true;
        left_pwm_output  = 0;
        right_pwm_output = 0;
        motor_stop();
        return;
    }

    left_pwm_output = speed_pi_calculate(
        &left_speed_pi,
        LEFT_TARGET_COUNT,
        left_speed_count,
        LEFT_PWM_FEEDFORWARD);

    right_pwm_output = speed_pi_calculate(
        &right_speed_pi,
        RIGHT_TARGET_COUNT,
        right_speed_count,
        RIGHT_PWM_FEEDFORWARD);

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

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);                                               // 时钟配置及系统初始化<务必保留>
    debug_init();                                                               // 调试串口信息初始化

    system_delay_ms(300);                                                       // 等待主板外设电源稳定

    gpio_init(MOTOR_LEFT_DIR, GPO, MOTOR_LEFT_FORWARD_LEVEL, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_DIR, GPO, MOTOR_RIGHT_FORWARD_LEVEL, GPO_PUSH_PULL);
    pwm_init(MOTOR_LEFT_PWM, MOTOR_PWM_FREQUENCY_HZ, 0);
    pwm_init(MOTOR_RIGHT_PWM, MOTOR_PWM_FREQUENCY_HZ, 0);

    encoder_quad_init(ENCODER_LEFT_TIMER, ENCODER_LEFT_A, ENCODER_LEFT_B);
    encoder_quad_init(ENCODER_RIGHT_TIMER, ENCODER_RIGHT_A, ENCODER_RIGHT_B);

    pit_ms_init(CONTROL_PIT, CONTROL_PERIOD_MS, speed_control_callback, NULL);
    interrupt_global_enable(0);

    printf("\r\nDRV8701E dual motor speed PI test.\r\n");
    printf("Lift wheels before first run. Start after %d ms, stop after %d ms.\r\n",
        START_DELAY_MS,
        TEST_DURATION_MS);
    printf("Target: left=%d, right=%d count/%dms; PWM limit=%d.\r\n",
        LEFT_TARGET_COUNT,
        RIGHT_TARGET_COUNT,
        CONTROL_PERIOD_MS,
        PWM_OUTPUT_LIMIT);

    system_delay_ms(START_DELAY_MS);

    encoder_clear_count(ENCODER_LEFT_TIMER);
    encoder_clear_count(ENCODER_RIGHT_TIMER);
    left_speed_pi.integral  = 0;
    right_speed_pi.integral = 0;
    control_tick  = 0;
    test_finished = false;
    test_running  = true;

    while(!test_finished)
    {
        system_delay_ms(PRINT_PERIOD_MS);

        printf("t=%ums L[target=%d count=%d pwm=%d] R[target=%d count=%d pwm=%d]\r\n",
            (unsigned int)(control_tick * CONTROL_PERIOD_MS),
            LEFT_TARGET_COUNT,
            left_speed_count,
            (int)left_pwm_output,
            RIGHT_TARGET_COUNT,
            right_speed_count,
            (int)right_pwm_output);
    }

    motor_stop();
    printf("Test finished. Motors stopped. Press reset to run again.\r\n");

    while(true)
    {
        system_delay_ms(1000);
    }
}

#endif

// **************************** 代码区域 ****************************
