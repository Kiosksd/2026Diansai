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

// 低速闭环循迹测试：GS08RA 计算黑线偏差，PD 外环产生物理左右轮目标速度，
// 两路编码器 PI 内环分别跟随目标速度。
// 安全提示：启动时无黑线则拒绝起步；丢线 150ms 或运行 3 秒后自动停止。

// -------------------------------- 主板接口 --------------------------------
#define MOTOR_LEFT_DIR                  ( B13 )
#define MOTOR_LEFT_PWM                  ( PWM_TIM_A0_CH2_B12 )
#define MOTOR_LEFT_FORWARD_LEVEL        ( GPIO_HIGH )
#define MOTOR_RIGHT_DIR                 ( A1 )
#define MOTOR_RIGHT_PWM                 ( PWM_TIM_A0_CH0_A0 )
#define MOTOR_RIGHT_FORWARD_LEVEL       ( GPIO_HIGH )

// 2026-07-11 实车映射测试：A0/A1 驱动 TIMG9 编码器；B12/B13 驱动 TIMG8 编码器。
// 物理轮位复测：CH1(A0/A1) 是右轮，CH2(B12/B13) 是左轮。
// 物理方向复测：左右轮在 DIR=HIGH 时均驱动车辆前进。
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
#define START_DELAY_MS                  ( 2000 )
#define TEST_DURATION_MS                ( 10000 )

// 单位：每 10ms 的编码器计数。首次循迹进一步降速，且不允许车轮反转。
#define TRACK_BASE_TARGET_COUNT         ( 8 )
#define TRACK_LOST_TARGET_COUNT         ( 5 )
#define TRACK_TARGET_MAX                ( 14 )
#define TRACK_STEER_LIMIT               ( 6 )

// GS08RA 实测：通道 0 在车体左侧，通道 7 在车体右侧；白底黑线时黑线为 0。
// 偏差范围 -7..+7：负数表示黑线在左，正数表示黑线在右。
#define GS08RA_BINARY_THRESHOLD         ( 30 )
#define TRACK_PD_KP_NUM                 ( 2 )
#define TRACK_PD_KD_NUM                 ( 1 )
#define TRACK_PD_GAIN_DIV               ( 2 )
#define TRACK_LOST_STOP_TICKS           ( 150 / CONTROL_PERIOD_MS )

// PWM_DUTY_MAX 为 10000。默认最大限制 3000，即 30%。
#define PWM_OUTPUT_LIMIT                ( 3000 )
#define SPEED_PWM_STATIC                ( 300 )
#define SPEED_PWM_PER_COUNT             ( 70 )

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
static volatile int16  left_target_count = 0;
static volatile int16  right_target_count = 0;
static volatile int32  left_pwm_output   = 0;
static volatile int32  right_pwm_output  = 0;
static volatile uint32 control_tick      = 0;
static volatile bool   test_running      = false;
static volatile bool   test_finished     = false;

static int16 line_error       = 0;
static int16 last_line_error  = 0;
static int16 track_correction = 0;
static uint16 line_lost_ticks = 0;
static bool line_detected     = false;
static bool track_safety_stop = false;

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

static int32 speed_feedforward_calculate (int16 target)
{
    int32 magnitude;

    if(0 == target)
    {
        return 0;
    }

    magnitude = (target > 0) ? target : -target;
    magnitude = SPEED_PWM_STATIC + SPEED_PWM_PER_COUNT * magnitude;

    return (target > 0) ? magnitude : -magnitude;
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

static void track_update (void)
{
    int16 new_error;
    int16 derivative;
    int32 correction;

    line_detected = line_error_calculate(&new_error);

    if(line_detected)
    {
        line_lost_ticks = 0;
        derivative = new_error - last_line_error;
        line_error = new_error;

        correction = TRACK_PD_KP_NUM * line_error
                   + TRACK_PD_KD_NUM * derivative;
        correction /= TRACK_PD_GAIN_DIV;
        track_correction = (int16)limit_int32(
            correction,
            -TRACK_STEER_LIMIT,
            TRACK_STEER_LIMIT);

        last_line_error = line_error;
        track_targets_set(TRACK_BASE_TARGET_COUNT, track_correction);
    }
    else
    {
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

int main (void)
{
    uint16 print_elapsed_ms = 0;

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

    pit_ms_init(CONTROL_PIT, CONTROL_PERIOD_MS, speed_control_callback, NULL);
    interrupt_global_enable(0);

    printf("\r\nGS08RA corrected low-speed line-follow test.\r\n");
    printf("Physical mapping: LEFT=CH2/TIMG8, RIGHT=CH1/TIMG9.\r\n");
    printf("Place the black line under the sensor center before reset.\r\n");
    printf("Start after %d ms, stop after %d ms; lost line %d ms -> safety stop.\r\n",
        START_DELAY_MS,
        TEST_DURATION_MS,
        TRACK_LOST_STOP_TICKS * CONTROL_PERIOD_MS);
    printf("Base=%d count/%dms, steering limit=%d, PWM limit=%d.\r\n",
        TRACK_BASE_TARGET_COUNT,
        CONTROL_PERIOD_MS,
        TRACK_STEER_LIMIT,
        PWM_OUTPUT_LIMIT);

    system_delay_ms(START_DELAY_MS);

    gs08ra_scan_read();
    line_lost_ticks = 0;
    track_safety_stop = false;
    track_update();

    printf("START BIN=%u%u%u%u%u%u%u%u detected=%u error=%d "
           "Ltarget=%d Rtarget=%d\r\n",
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
        left_target_count,
        right_target_count);

    if(!line_detected)
    {
        motor_stop();
        printf("START ABORTED: no black line detected. Reposition the car and reset.\r\n");

        while(true)
        {
            system_delay_ms(1000);
        }
    }

    encoder_clear_count(ENCODER_LEFT_TIMER);
    encoder_clear_count(ENCODER_RIGHT_TIMER);
    left_speed_pi.integral  = 0;
    right_speed_pi.integral = 0;
    control_tick  = 0;
    test_finished = false;
    test_running  = true;

    while(!test_finished)
    {
        system_delay_ms(CONTROL_PERIOD_MS);

        if(test_finished)
        {
            break;
        }

        gs08ra_scan_read();
        track_update();

        if(track_safety_stop)
        {
            test_running = false;
            test_finished = true;
            left_pwm_output = 0;
            right_pwm_output = 0;
            motor_stop();
        }

        print_elapsed_ms += CONTROL_PERIOD_MS;
        if(print_elapsed_ms >= PRINT_PERIOD_MS)
        {
            print_elapsed_ms = 0;
            printf("t=%ums BIN=%u%u%u%u%u%u%u%u det=%u err=%d turn=%d lost=%u "
                   "L[t=%d c=%d p=%d] R[t=%d c=%d p=%d]\r\n",
                (unsigned int)(control_tick * CONTROL_PERIOD_MS),
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
                left_target_count,
                left_speed_count,
                (int)left_pwm_output,
                right_target_count,
                right_speed_count,
                (int)right_pwm_output);
        }
    }

    motor_stop();
    if(track_safety_stop)
    {
        printf("Safety stop: black line lost for %d ms.\r\n",
            TRACK_LOST_STOP_TICKS * CONTROL_PERIOD_MS);
    }
    else
    {
        printf("Line-follow test finished after %d ms.\r\n", TEST_DURATION_MS);
    }
    printf("Motors stopped. Press reset to run again.\r\n");

    while(true)
    {
        system_delay_ms(1000);
    }
}

#endif

// **************************** 代码区域 ****************************
