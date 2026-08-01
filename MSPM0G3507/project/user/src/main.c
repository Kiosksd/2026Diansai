/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3507 开源库的一部分
* 
* MSPM0G3507 开源库 是免费软件
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
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 店铺链接          https://seekfree.taobao.com/
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <stdarg.h>
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 用户代码区域 ****************************

// D36A 第一路：B12=STEP、B13=DIR、B8=EN（高电平使能）。
#define STEPPER_STEP_PIN                     (B12)
#define STEPPER_DIR_PIN                      (B13)
#define STEPPER_EN_PIN                       (B8)
#define STEPPER_STEP_PWM                     (PWM_TIM_A0_CH1_B12)
#define STEPPER_PWM_CHANNEL_INDEX            (1U)
#define STEPPER_PWM_CLOCK_HZ                 (1000000U)

// 电机重新安装后的闭环测试参数：32 细分，目标限制在水平零点附近 +/-20 度。
// 位置误差经 P 控制器转换为 STEP 脉冲频率；本阶段故意限制速度和加速度。
#define STEPPER_TARGET_LIMIT_X100            (2000)
#define STEPPER_TEST_TRIP_LIMIT_X100         (3000)
#define STEPPER_POSITION_DEADBAND_X100       (15)
#define STEPPER_KP_PPS_PER_DEG               (400U)
#define STEPPER_MIN_FREQUENCY_PPS            (100U)
#define STEPPER_MAX_FREQUENCY_PPS            (1800U)
#define STEPPER_SLEW_PPS_PER_MS              (50)

// 电机重新安装后实测机械极限约为 115.22~213.66 度；保留至少 5 度余量。
#define STEPPER_ABSOLUTE_MIN_X100            (12100U)
#define STEPPER_ABSOLUTE_MAX_X100            (20800U)
#define STEPPER_ABSOLUTE_LIMIT_CONFIRM_SAMPLES (5U)  // require 5 consecutive valid samples outside soft range
#define STEPPER_FEEDBACK_TIMEOUT_LOOPS       (50U)

// 每 50ms 检查一次误差是否反而增大，自动拦截 DIR 极性接反造成的失控。
#define STEPPER_DIRECTION_CHECK_LOOPS        (50U)
#define STEPPER_DIRECTION_ERROR_X100         (30U)

// MS42CG 的 PWM 接到 B10；B10 的复用功能 3 是 TIMG8_CCP0。
#define MS42_PWM_PIN                         (B10)
#define MS42_PWM_TIMER_LOAD                  (0xFFFFU)

// MT6816 PWM 帧：16 个固定高电平时钟 + 4095 个角度时钟 + 8 个固定低电平时钟。
#define MS42_PWM_FRAME_CLOCKS                (4119U)
#define MS42_PWM_HEADER_HIGH_CLOCKS          (16U)
#define MS42_PWM_ANGLE_COUNTS                (4096U)
#define MS42_PWM_EDGE_TOLERANCE_CLOCKS       (2U)
#define MS42_PWM_FAST_PERIOD_MIN_TICKS       (16000U) // 971Hz mode, nominal period about 20590 ticks
#define MS42_PWM_FAST_PERIOD_MAX_TICKS       (25000U)
#define MS42_PWM_SLOW_PERIOD_MIN_TICKS       (33000U) // 486Hz mode, nominal period about 41180 ticks
#define MS42_PWM_SLOW_PERIOD_MAX_TICKS       (50000U)
#define MS42_MAX_SAMPLE_DELTA_X100           (300)   // reject impossible >3.00deg change in one PWM frame
#define MS42_RESYNC_REQUIRED_SAMPLES         (3U)    // accept a new baseline only after 3 consistent frames
#define MS42_RESYNC_MAX_JUMP_X100            (2500U) // never auto-resync a jump larger than 25.00deg

#define WIRELESS_DEBUG_BUFFER_SIZE           (256U)
#define WIRELESS_TX_QUEUE_SIZE               (1024U)
#define WIRELESS_TX_ACTIVE_CHUNK_SIZE        (2U)    // <=0.18ms at 115200bps while motor is active
#define WIRELESS_TX_IDLE_CHUNK_SIZE          (8U)
#define WIRELESS_COMMAND_BUFFER_SIZE         (24U)
#define WIRELESS_COMMAND_IDLE_LOOPS          (20U)
#define WIRELESS_STATUS_PERIOD_LOOPS         (100U)
#define WIRELESS_ACTIVE_STATUS_PERIOD_LOOPS  (500U)
#define WIRELESS_DIAGNOSTIC_PERIOD_LOOPS     (40U)   // RAM sample at about 25Hz
#define DIAGNOSTIC_LOG_CAPACITY              (896U)  // about 36 seconds at 25Hz; preserves MCU RAM margin

// MaixCAM2 独立使用 UART2：主板串口插座 B15/T=TX、B16/R=RX，115200 8N1。
#define CAMERA_UART_INDEX                    (UART_2)
#define CAMERA_UART_BAUDRATE                 (115200U)
#define CAMERA_UART_TX_PIN                   (UART2_TX_B15)
#define CAMERA_UART_RX_PIN                   (UART2_RX_B16)
#define CAMERA_FRAME_HEAD_0                  (0xAAU)
#define CAMERA_FRAME_HEAD_1                  (0x55U)
#define CAMERA_FRAME_TYPE_BALL               (0x01U)
#define CAMERA_FRAME_TAIL                    (0x0DU)
#define CAMERA_FRAME_LENGTH                  (13U)
#define CAMERA_RX_RING_SIZE                  (128U)
#define CAMERA_RX_RING_MASK                  (CAMERA_RX_RING_SIZE - 1U)
#define CAMERA_LINK_TIMEOUT_LOOPS            (300U)
#define CAMERA_NOMINAL_FRAME_PERIOD_MS       (20U)

// 串级双环 PID：位置外环输出目标球速，速度内环输出摆杆目标角。
// 所有量均使用 x100 定点数：位置 cm、速度 cm/s、角度 deg。
// 正球速表示向右；机构实测正摆角使球向左，因此速度环输出到摆角时需要反号。
#define GENERIC_POSITION_PID_KP_X100         (605)   // 8926ff0 profile: 3.50 (cm/s)/cm
#define GENERIC_POSITION_PID_KI_X100         (8)     // 8926ff0 profile: 0.08 (cm/s)/(cm*s)
#define GENERIC_POSITION_PID_KD_RIGHT_X100   (170)// 8926ff0 profile: 1.00 in both directions
#define GENERIC_POSITION_PID_KD_LEFT_X100    (172)
#define GENERIC_POSITION_PID_I_LIMIT_X100    (200)   // target-speed integral term +/-2.00cm/s
#define GENERIC_POSITION_PID_SPEED_LIMIT_X100 (2000) // target-ball-speed limit +/-20.00cm/s
#define GENERIC_POSITION_BRAKE_RIGHT_X100    (1600)  // 8926ff0 braking planner: 15.00cm/s^2
#define GENERIC_POSITION_BRAKE_LEFT_X100     (1600)

// KEY1 competition parameters are deliberately duplicated instead of aliased
// to the generic profile.  Future tuning of another task must not change S1.
#define S1_POSITION_PID_KP_X100              (350)
#define S1_POSITION_PID_KI_X100              (8)
#define S1_POSITION_PID_KD_RIGHT_X100        (30)
#define S1_POSITION_PID_KD_LEFT_X100         (55)
#define S1_POSITION_PID_I_LIMIT_X100         (200)
#define S1_POSITION_PID_SPEED_LIMIT_X100     (2000)
#define S1_POSITION_BRAKE_RIGHT_X100         (900)
#define S1_POSITION_BRAKE_LEFT_X100          (1500)

// Preserve the exact pre-run/abort behavior that the current KEY1 code used
// before its dedicated final-run profile was selected.
#define S1_PREP_POSITION_PID_KD_RIGHT_X100   (30)
#define S1_PREP_POSITION_PID_KD_LEFT_X100    (40)
#define S1_PREP_POSITION_BRAKE_RIGHT_X100    (900)
#define S1_PREP_POSITION_BRAKE_LEFT_X100     (1500)

// S2/KEY2 (A31) center-hold profile.  Keep it independent so later center
// tuning cannot change the fixed S1 competition trajectory.
#define S2_POSITION_PID_KP_X100              (360)
#define S2_POSITION_PID_KI_X100              (8)
#define S2_POSITION_PID_KD_RIGHT_X100        (100)
#define S2_POSITION_PID_KD_LEFT_X100         (100)
#define S2_POSITION_PID_I_LIMIT_X100         (200)
#define S2_POSITION_PID_SPEED_LIMIT_X100     (2000)
#define S2_POSITION_BRAKE_RIGHT_X100         (900)
#define S2_POSITION_BRAKE_LEFT_X100          (1500)

#define POSITION_SPEED_ACCEL_X100_PER_S      (25000) // 目标球速加速斜率 200.00cm/s^2
#define POSITION_SETTLE_BAND_X100            (5)     // 位置稳定带 +/-0.08cm
#define POSITION_INTEGRAL_FREEZE_X100        (20)    // 目标附近不积累位置积分 +/-0.20cm
#define POSITION_LANDING_ARM_DISTANCE_X100   (200)   // use landing capture only after travel of at least 2.00cm
#define POSITION_LANDING_BAND_X100           (95)    // measured first-stop window: strictly inside +/-1.00cm
#define POSITION_LANDING_SPEED_X100          (50)    // capture after braking below 0.50cm/s
#define POSITION_LANDING_RETREAT_X100        (5)     // or capture after retreating 0.05cm from the best landing
#define POSITION_LANDING_RETREAT_SPEED_X100  (200)   // retreat capture is forbidden above 2.00cm/s
#define POSITION_LANDING_MAX_ERROR_X100      (99)    // never latch a new landing at or beyond 1.00cm error
#define POSITION_LANDING_RELEASE_RIGHT_X100  (120)   // X+: retain proven 1.20cm HOLD release
#define POSITION_LANDING_RELEASE_LEFT_X100   (160)   // X-: keep recovery active through measured rebound
#define POSITION_HOLD_KP_X100                (200)   // HOLD requests 2.00cm/s per cm of residual error
#define POSITION_HOLD_SPEED_LIMIT_X100       (150)   // quiet HOLD correction limited to +/-1.50cm/s
#define POSITION_HOLD_ANGLE_LIMIT_X100       (400)   // HOLD never commands more than +/-4.00deg
#define POSITION_HOLD_TRANSIENT_LIMIT_X100   (800)   // X- catch/trim stays inside the full PID +/-8deg limit
#define POSITION_HOLD_CATCH_RIGHT_ANGLE_X100 (460)   // X+: measured breakaway angle, used only during catch
#define POSITION_HOLD_CATCH_LEFT_ANGLE_X100  (650)   // X-: arrest the measured rebound immediately
#define POSITION_HOLD_CATCH_RIGHT_FRAMES     (25U)   // X+: up to about 500ms on the 50Hz vision stream
#define POSITION_HOLD_CATCH_LEFT_FRAMES      (25U)   // X-: up to about 500ms, same bounded landing window
#define POSITION_HOLD_CATCH_RIGHT_CANCEL_X100 (40)   // X+: withdraw catch at target-0.40cm for overshoot margin
#define POSITION_HOLD_CATCH_LEFT_CANCEL_X100 (40)    // X-: withdraw near target+0.40cm
#define POSITION_HOLD_RIGHT_TRIM_START_X100  (45)    // X+: keep 0.05cm margin above target-0.50cm
#define POSITION_HOLD_RIGHT_TRIM_STOP_X100   (40)    // X+: no trim once target-0.40cm is reached
#define POSITION_HOLD_RIGHT_TRIM_BASE_X100   (460)   // first measured rightward breakaway attempt
#define POSITION_HOLD_RIGHT_TRIM_STEP_X100   (20)    // add 0.20deg after each no-motion timeout
#define POSITION_HOLD_RIGHT_TRIM_MAX_X100    (540)   // bounded adaptive trim ceiling
#define POSITION_HOLD_RIGHT_TRIM_DISTANCE_X100 (5)   // withdraw each trim after 0.05cm rightward motion
#define POSITION_HOLD_RIGHT_TRIM_SPEED_X100  (30)    // or after rightward speed reaches 0.30cm/s
#define POSITION_HOLD_RIGHT_TRIM_FRAMES      (12U)   // hard limit about 240ms per trim
#define POSITION_HOLD_RIGHT_TRIM_COOLDOWN_FRAMES (5U) // allow braking/vision to settle for about 100ms
#define POSITION_HOLD_RIGHT_TRIM_MAX_ATTEMPTS (16U)  // enough 0.05cm steps for a measured 0.45cm recovery
#define POSITION_HOLD_RIGHT_SUPPORT_I_X100   (250)   // seed the proven near-3.5deg support after a trim moves
#define POSITION_HOLD_RIGHT_CEILING_ENTER_X100 (40)  // start reverse guard at target+0.40cm
#define POSITION_HOLD_RIGHT_CEILING_EXIT_X100  (20)  // release guard after returning inside target+0.20cm
#define POSITION_HOLD_RIGHT_CEILING_ANGLE_X100 (460) // transient reverse guard, still below full PID limit
#define POSITION_HOLD_LEFT_TRIM_START_X100   (45)    // X-: keep 0.05cm margin above the 1cm acceptance edge
#define POSITION_HOLD_LEFT_TRIM_STOP_X100    (40)    // X-: no trim once inside target+0.40cm
#define POSITION_HOLD_LEFT_TRIM_BASE_X100    (650)   // continue from the stronger X- catch level
#define POSITION_HOLD_LEFT_TRIM_STEP_X100    (25)    // add 0.25deg after each no-motion timeout
#define POSITION_HOLD_LEFT_TRIM_MAX_X100     (800)   // full PID limit, with 0.05cm immediate release
#define POSITION_HOLD_LEFT_SUPPORT_I_X100    (-250)  // symmetric speed-integral support after leftward motion
#define POSITION_HOLD_LEFT_FLOOR_ENTER_X100  (40)    // start reverse guard at target-0.40cm
#define POSITION_HOLD_LEFT_FLOOR_EXIT_X100   (20)    // release after returning inside target-0.20cm
#define POSITION_HOLD_LEFT_FLOOR_ANGLE_X100  (-460)  // rightward recovery after a left-side overshoot
#define POSITION_HOLD_LEFT_FLOOR_RELEASE_DISTANCE_X100 (5) // withdraw after 0.05cm rightward motion

#define AUTO_RUN_RIGHT_TARGET_X100           (500)   // O -> +5.00cm
#define AUTO_RUN_LEFT_TARGET_X100            (-500)  // generic A command keeps the normal point target
#define S1_AUTO_RUN_LEFT_TARGET_X100         (-520)  // fixed S1 bias at the center of its accepted range
#define AUTO_RUN_LEFT_NOMINAL_X100           (-500)  // reported competition endpoint remains -5.00cm
// The AUTO_FINAL group below belongs only to the S1 competition profile.
// Other tasks use the generic cascade state and their own PID/target settings.
#define AUTO_FINAL_MODE_ENTER_X100           (-440)  // take ownership before legacy HOLD can capture at -4.5cm
#define AUTO_FINAL_RANGE_NEAR_X100           (-450)  // accepted final range: -6.00cm <= x <= -4.50cm
#define AUTO_FINAL_RANGE_FAR_X100            (-600)
#define AUTO_FINAL_GUARD_ENTER_X100          (-585)  // brake before the -6.00cm hard edge
#define AUTO_FINAL_GUARD_REARM_X100          (-570)
#define AUTO_FINAL_SUPPORT_ANGLE_X100        (280)   // measured left-end static balance bias
#define AUTO_FINAL_DAMPING_KP_X100           (80)    // 0.80deg per cm/s, velocity damping only
#define AUTO_FINAL_DAMPING_MIN_X100           (-250) // bounded braking while travelling left
#define AUTO_FINAL_DAMPING_MAX_X100           (400)  // bounded rightward rebound catch inside the range
#define AUTO_FINAL_GUARD_ANGLE_X100          (-300)  // bounded rightward recovery near the far edge
#define AUTO_FINAL_GUARD_RELEASE_X100        (5)     // withdraw after 0.05cm rightward displacement
#define AUTO_FINAL_ANGLE_SLEW_X100           (300)   // faster sign reversal while catching the final range
#define AUTO_FINAL_RECOVERY_BASE_X100        (650)   // outside the range, restart leftward motion decisively
#define AUTO_FINAL_RECOVERY_STEP_X100        (25)
#define AUTO_FINAL_RECOVERY_MAX_X100         (800)
#define AUTO_FINAL_RECOVERY_RELEASE_X100     (5)     // withdraw after 0.05cm leftward displacement
#define AUTO_FINAL_RECOVERY_SPEED_X100       (30)
#define AUTO_FINAL_RECOVERY_ACTIVE_FRAMES    (12U)   // each launch is limited to about 240ms
#define AUTO_FINAL_RECOVERY_COOLDOWN_FRAMES  (4U)
#define AUTO_RUN_START_POSITION_X100         (50)    // require start within +/-0.50cm of O
#define AUTO_RUN_START_SPEED_X100            (50)    // require start below 0.50cm/s
#define AUTO_RUN_ENDPOINT_ERROR_X100         (100)   // both endpoints must be within +/-1.00cm
#define AUTO_RUN_FINAL_SPEED_X100            (50)    // generic A point target: below 0.50cm/s
#define AUTO_RUN_FINAL_STABLE_FRAMES         (5U)    // generic A point target: about 100ms
#define S1_AUTO_RUN_FINAL_SPEED_X100         (30)    // S1 range must be quiet below 0.30cm/s
#define S1_AUTO_RUN_FINAL_STABLE_FRAMES      (8U)    // verify the S1 range for about 160ms
#define AUTO_RUN_TIME_LIMIT_MS               (5000U)

// S1/KEY1 (A30) runs the fixed competition task. S2/KEY2 (A31) uses the same
// safe zero/enable/vision preparation and then holds the ball at O. Holding
// either key for one second is an emergency stop.
#define COMPETITION_KEY_SCAN_PERIOD_MS        (10U)
#define COMPETITION_LEVEL_BAND_X100           (15)    // beam settled within +/-0.15deg
#define COMPETITION_PREP_STABLE_FRAMES        (5U)    // about 100ms at the camera frame rate
#define COMPETITION_HANDS_OFF_DELAY_MS        (400U)   // short release/move-away margin after S1

#define GENERIC_SPEED_PID_KP_X100            (110)    // 8926ff0 speed-loop profile
#define GENERIC_SPEED_PID_KI_X100            (10)
#define GENERIC_SPEED_PID_KD_X100            (5)
#define GENERIC_SPEED_PID_I_LIMIT_X100       (250)
#define GENERIC_SPEED_PID_ACCEL_LIMIT_X100   (5000)
#define GENERIC_SPEED_PID_POS_ANGLE_X100     (1000)
#define GENERIC_SPEED_PID_NEG_ANGLE_X100     (1000)

#define S1_SPEED_PID_KP_X100                 (80)    // fixed KEY1 competition profile
#define S1_SPEED_PID_KI_X100                 (10)
#define S1_SPEED_PID_KD_X100                 (3)
#define S1_SPEED_PID_I_LIMIT_X100            (250)
#define S1_SPEED_PID_ACCEL_LIMIT_X100        (5000)
#define S1_SPEED_PID_POS_ANGLE_X100          (1000)
#define S1_SPEED_PID_NEG_ANGLE_X100          (1000)

#define S2_SPEED_PID_KP_X100                 (100)
#define S2_SPEED_PID_KI_X100                 (10)
#define S2_SPEED_PID_KD_X100                 (5)
#define S2_SPEED_PID_I_LIMIT_X100            (250)
#define S2_SPEED_PID_ACCEL_LIMIT_X100        (5000)
#define S2_SPEED_PID_POS_ANGLE_X100          (1000)
#define S2_SPEED_PID_NEG_ANGLE_X100          (1000)
#define SPEED_SETTLE_BAND_X100               (15)    // 速度稳定带 +/-0.15cm/s

#define PID_INTEGRAL_SCALE                   (100000)

// 加速时平缓改变摆角；需要制动时允许更快地反向倾斜。
#define VISION_ACCEL_ANGLE_SLEW_X100         (120)   // 1.20deg per camera frame
#define VISION_BRAKE_ANGLE_SLEW_X100         (200)   // 2.00deg per camera frame
#define VISION_POSITION_LIMIT_X100           (1200)  // calibrated useful range +/-12.00cm
#define VISION_VELOCITY_LIMIT_X100           (5000)  // reject implausible values above 50.00cm/s
#define VISION_TARGET_POSITION_LIMIT_X100    (1000)  // target range +/-10.00cm
#define VISION_START_ERROR_LIMIT_X100        (2000)  // allow full -10cm -> +10cm travel
#define VISION_START_ANGLE_LIMIT_X100        (300)   // V accepted only near level (+/-3.00deg)
#define VISION_BALL_LOST_FRAME_LIMIT         (3U)    // about 60ms on the 50Hz vision stream
#define VISION_REACQUIRE_VALID_FRAMES        (2U)    // about 40ms stable reacquisition
#define VISION_LINK_TIMEOUT_LOOPS            (120U)  // dedicated-link loss: return level in about 120ms

// Manual breakaway-pulse calibration. These commands are available only in
// manual/level mode and automatically return the angle target to zero.
#define MANUAL_PULSE_POSITIVE_X100           (460)   // P+: +4.60deg
#define MANUAL_PULSE_NEGATIVE_X100           (-440)  // P-: -4.40deg
#define MANUAL_PULSE_MAX_LOOPS               (500U)  // absolute maximum about 500ms
#define MANUAL_PULSE_HOLD_LOOPS              (120U)  // hold launch angle for about 120ms
#define MANUAL_PULSE_POSITIVE_HOLD_X100      (440)   // enter hold phase at +4.40deg
#define MANUAL_PULSE_NEGATIVE_HOLD_X100      (-420)  // enter hold phase at -4.20deg
#define MANUAL_PULSE_POSITIVE_OVERRUN_X100   (490)   // independent +4.90deg safety stop
#define MANUAL_PULSE_NEGATIVE_OVERRUN_X100   (-470)  // independent -4.70deg safety stop
#define MANUAL_PULSE_START_ANGLE_X100        (100)   // require mechanism within +/-1.00deg
#define MANUAL_PULSE_POSITION_GATE_X100      (30)    // start only outside +/-0.30cm
#define MANUAL_PULSE_START_SPEED_X100        (30)    // require |velocity| <= 0.30cm/s
#define MANUAL_PULSE_STOP_DISTANCE_X100      (15)    // stop after 0.15cm detected motion
#define MANUAL_PULSE_STOP_SPEED_X100         (50)    // or |velocity| >= 0.50cm/s

// Automatic static-friction compensation for the vision speed loop.  The
// launch angles include 0.20deg margin for the angle execution deadband.
// Compensation is removed as soon as directional displacement or velocity is
// detected, so normal PID braking remains in control near the target.
#define SPEED_STICTION_POSITION_ERROR_X100   (60)    // correct only while error is above 0.60cm
#define SPEED_STICTION_VELOCITY_X100         (30)    // detect a stuck ball below 0.30cm/s
#define SPEED_STICTION_TARGET_SPEED_X100     (50)    // ignore tiny velocity requests below 0.50cm/s
#define SPEED_STICTION_RELEASE_SPEED_X100    (30)    // release after 0.30cm/s directional motion
#define SPEED_STICTION_RELEASE_DISTANCE_X100 (5)     // or after 0.05cm directional displacement
#define SPEED_STICTION_DETECT_FRAMES         (3U)    // about 60ms on the 50Hz vision stream
#define SPEED_STICTION_MAX_ACTIVE_FRAMES     (12U)   // limit one launch attempt to about 240ms
#define SPEED_STICTION_LEFT_BASE_X100        (480)   // first launch target for ball motion left
#define SPEED_STICTION_RIGHT_BASE_X100       (-460)  // first launch target for ball motion right
#define SPEED_STICTION_ANGLE_STEP_X100       (25)    // add 0.25deg after a no-motion timeout
#define SPEED_STICTION_LEFT_MAX_X100         (650)   // adaptive launch ceiling +6.50deg
#define SPEED_STICTION_RIGHT_MAX_X100        (-650)  // adaptive launch floor -6.50deg



typedef enum
{
    STEPPER_FAULT_NONE = 0,
    STEPPER_FAULT_ENCODER_LOST,
    STEPPER_FAULT_ABSOLUTE_LIMIT,
    STEPPER_FAULT_TEST_RANGE,
    STEPPER_FAULT_DIRECTION,
} stepper_fault_enum;

typedef enum
{
    MANUAL_PULSE_PHASE_IDLE = 0,
    MANUAL_PULSE_PHASE_RAMP,
    MANUAL_PULSE_PHASE_HOLD,
} manual_pulse_phase_enum;

typedef enum
{
    AUTO_RUN_IDLE = 0,
    AUTO_RUN_TO_RIGHT,
    AUTO_RUN_TO_LEFT,
    AUTO_RUN_COMPLETE,
} auto_run_phase_enum;

typedef enum
{
    COMPETITION_IDLE = 0,
    COMPETITION_WAIT_LEVEL,
    COMPETITION_WAIT_VISION,
    COMPETITION_HANDS_OFF,
    COMPETITION_RUNNING,
    COMPETITION_COMPLETE,
} competition_phase_enum;

typedef enum
{
    CENTER_HOLD_IDLE = 0,
    CENTER_HOLD_WAIT_LEVEL,
    CENTER_HOLD_WAIT_VISION,
    CENTER_HOLD_ACTIVE,
} center_hold_phase_enum;

// Top-level ownership of the shared camera, cascade controller and stepper.
// KEY1 always owns APP_CONTROL_KEY1_FIXED; the original 8926ff0 controller is
// retained as APP_CONTROL_GENERIC_8926 for serial commands and future tasks.
typedef enum
{
    APP_CONTROL_IDLE = 0,
    APP_CONTROL_GENERIC_8926,
    APP_CONTROL_KEY1_FIXED,
    APP_CONTROL_KEY2_CENTER,
} app_control_mode_enum;

typedef enum
{
    CASCADE_PROFILE_GENERIC_8926 = 0,
    CASCADE_PROFILE_KEY1_FIXED,
    CASCADE_PROFILE_KEY2_CENTER,
} cascade_profile_enum;

typedef struct
{
    int32 position_kp_x100;
    int32 position_ki_x100;
    int32 position_kd_right_x100;
    int32 position_kd_left_x100;
    int32 position_i_limit_x100;
    int32 position_speed_limit_x100;
    int32 position_brake_right_x100;
    int32 position_brake_left_x100;
    int32 speed_kp_x100;
    int32 speed_ki_x100;
    int32 speed_kd_x100;
    int32 speed_i_limit_x100;
    int32 speed_accel_limit_x100;
    int32 speed_positive_angle_limit_x100;
    int32 speed_negative_angle_limit_x100;
} cascade_pid_profile_struct;

static const cascade_pid_profile_struct s_pid_profile_generic_8926 =
{
    GENERIC_POSITION_PID_KP_X100,
    GENERIC_POSITION_PID_KI_X100,
    GENERIC_POSITION_PID_KD_RIGHT_X100,
    GENERIC_POSITION_PID_KD_LEFT_X100,
    GENERIC_POSITION_PID_I_LIMIT_X100,
    GENERIC_POSITION_PID_SPEED_LIMIT_X100,
    GENERIC_POSITION_BRAKE_RIGHT_X100,
    GENERIC_POSITION_BRAKE_LEFT_X100,
    GENERIC_SPEED_PID_KP_X100,
    GENERIC_SPEED_PID_KI_X100,
    GENERIC_SPEED_PID_KD_X100,
    GENERIC_SPEED_PID_I_LIMIT_X100,
    GENERIC_SPEED_PID_ACCEL_LIMIT_X100,
    GENERIC_SPEED_PID_POS_ANGLE_X100,
    GENERIC_SPEED_PID_NEG_ANGLE_X100,
};

static const cascade_pid_profile_struct s_pid_profile_key1_fixed =
{
    S1_POSITION_PID_KP_X100,
    S1_POSITION_PID_KI_X100,
    S1_POSITION_PID_KD_RIGHT_X100,
    S1_POSITION_PID_KD_LEFT_X100,
    S1_POSITION_PID_I_LIMIT_X100,
    S1_POSITION_PID_SPEED_LIMIT_X100,
    S1_POSITION_BRAKE_RIGHT_X100,
    S1_POSITION_BRAKE_LEFT_X100,
    S1_SPEED_PID_KP_X100,
    S1_SPEED_PID_KI_X100,
    S1_SPEED_PID_KD_X100,
    S1_SPEED_PID_I_LIMIT_X100,
    S1_SPEED_PID_ACCEL_LIMIT_X100,
    S1_SPEED_PID_POS_ANGLE_X100,
    S1_SPEED_PID_NEG_ANGLE_X100,
};

static const cascade_pid_profile_struct s_pid_profile_key1_prep =
{
    S1_POSITION_PID_KP_X100,
    S1_POSITION_PID_KI_X100,
    S1_PREP_POSITION_PID_KD_RIGHT_X100,
    S1_PREP_POSITION_PID_KD_LEFT_X100,
    S1_POSITION_PID_I_LIMIT_X100,
    S1_POSITION_PID_SPEED_LIMIT_X100,
    S1_PREP_POSITION_BRAKE_RIGHT_X100,
    S1_PREP_POSITION_BRAKE_LEFT_X100,
    S1_SPEED_PID_KP_X100,
    S1_SPEED_PID_KI_X100,
    S1_SPEED_PID_KD_X100,
    S1_SPEED_PID_I_LIMIT_X100,
    S1_SPEED_PID_ACCEL_LIMIT_X100,
    S1_SPEED_PID_POS_ANGLE_X100,
    S1_SPEED_PID_NEG_ANGLE_X100,
};

static const cascade_pid_profile_struct s_pid_profile_key2_center =
{
    S2_POSITION_PID_KP_X100,
    S2_POSITION_PID_KI_X100,
    S2_POSITION_PID_KD_RIGHT_X100,
    S2_POSITION_PID_KD_LEFT_X100,
    S2_POSITION_PID_I_LIMIT_X100,
    S2_POSITION_PID_SPEED_LIMIT_X100,
    S2_POSITION_BRAKE_RIGHT_X100,
    S2_POSITION_BRAKE_LEFT_X100,
    S2_SPEED_PID_KP_X100,
    S2_SPEED_PID_KI_X100,
    S2_SPEED_PID_KD_X100,
    S2_SPEED_PID_I_LIMIT_X100,
    S2_SPEED_PID_ACCEL_LIMIT_X100,
    S2_SPEED_PID_POS_ANGLE_X100,
    S2_SPEED_PID_NEG_ANGLE_X100,
};

typedef struct
{
    uint16 capture_ms;
    int16 position_x100;
    int16 velocity_x100;
    int16 target_position_x100;
    int16 position_error_x100;
    int16 target_velocity_x100;
    int16 angle_command_x100;
    int16 relative_angle_x100;
    int16 step_frequency_pps;
    uint8 sequence;
    uint8 measurement_valid;
    uint8 stiction_active;
    uint8 stiction_blocked;
    uint8 landing_hold;
    uint8 position_limited;
    uint8 speed_limited;
    uint8 fault;
} diagnostic_sample_struct;

volatile uint32 g_ms42_pwm_high_ticks = 0;
volatile uint32 g_ms42_pwm_period_ticks = 0;
volatile uint8  g_ms42_pwm_sample_ready = 0;
volatile uint8  g_ms42_pwm_synced = 0;
volatile uint8  g_ms42_pwm_signal_ok = 0;

static uint32 s_ms42_pwm_previous_rise = 0;
static uint32 s_ms42_pwm_last_fall = 0;
static uint8  s_ms42_pwm_fall_valid = 0;
static uint8  s_ms42_pwm_edge_seen = 0;

static uint32 s_encoder_single_x100 = 0;
static uint32 s_encoder_last_x100 = 0;
static uint32 s_encoder_high_ticks = 0;
static uint32 s_encoder_period_ticks = 0;
static int32  s_encoder_total_x100 = 0;
static int32  s_encoder_wrap_count = 0;
static uint32 s_encoder_feedback_age = STEPPER_FEEDBACK_TIMEOUT_LOOPS + 1U;
static uint8  s_encoder_multiturn_initialized = 0;
static uint8  s_encoder_feedback_valid = 0;
static uint8  s_encoder_absolute_violation_count = 0;
static uint32 s_encoder_rejected_sample_count = 0;
static uint32 s_encoder_resync_event_count = 0;
static uint32 s_encoder_resync_candidate_x100 = 0;
static uint8  s_encoder_resync_candidate_valid = 0U;
static uint8  s_encoder_resync_candidate_count = 0U;

static int32  s_stepper_target_x100 = 0;
static int32  s_stepper_frequency_pps = 0;
static int8   s_stepper_applied_direction = 0;
static uint8  s_stepper_enabled = 0;
static uint8  s_stepper_zero_set = 0;
static uint8  s_stepper_direction_inverted = 0;
static stepper_fault_enum s_stepper_fault = STEPPER_FAULT_NONE;

static uint8  s_direction_check_active = 0;
static uint8  s_direction_check_reference_valid = 0;
static uint32 s_direction_check_counter = 0;
static uint32 s_direction_check_reference_error = 0;

static char   s_wireless_command_buffer[WIRELESS_COMMAND_BUFFER_SIZE];
static uint32 s_wireless_command_length = 0;
static uint32 s_wireless_command_idle = 0;
static uint8  s_wireless_tx_queue[WIRELESS_TX_QUEUE_SIZE];
static uint16 s_wireless_tx_head = 0U;
static uint16 s_wireless_tx_tail = 0U;
static uint32 s_wireless_tx_dropped_message_count = 0U;
static uint8  s_diagnostic_log_enabled = 0;
static uint8  s_diagnostic_sequence_seen = 0;
static uint8  s_diagnostic_last_sequence = 0;
static uint8  s_diagnostic_log_full = 0;
static uint16 s_diagnostic_sample_count = 0;
static diagnostic_sample_struct s_diagnostic_samples[DIAGNOSTIC_LOG_CAPACITY];

static volatile uint8  s_camera_rx_ring[CAMERA_RX_RING_SIZE];
static volatile uint8  s_camera_rx_head = 0;
static volatile uint8  s_camera_rx_tail = 0;
static volatile uint32 s_camera_rx_overflow_count = 0;

static uint8  s_camera_frame_buffer[CAMERA_FRAME_LENGTH];
static uint8  s_camera_frame_index = 0;
static uint8  s_camera_sequence = 0;
static uint8  s_camera_sequence_seen = 0;
static uint8  s_camera_measurement_valid = 0;
static int32  s_camera_position_x100 = 0;
static int32  s_camera_velocity_x100 = 0;
static uint16 s_camera_capture_ms_low16 = 0;
static uint32 s_camera_link_age = CAMERA_LINK_TIMEOUT_LOOPS + 1U;
static uint32 s_camera_valid_frame_count = 0;
static uint32 s_camera_bad_frame_count = 0;
static uint32 s_camera_dropped_frame_count = 0;

static uint8  s_vision_control_active = 0;
static uint8  s_vision_ball_lost = 1;
static uint8  s_vision_last_sequence = 0;
static uint8  s_vision_sequence_seen = 0;
static uint8  s_vision_lost_frame_count = 0;
static uint8  s_vision_reacquire_count = 0;
static int32  s_ball_target_position_x100 = 0;
static int32  s_position_error_x100 = 0;
static int32  s_position_p_term_x100 = 0;
static int32  s_position_i_accumulator = 0;
static int32  s_position_i_term_x100 = 0;
static int32  s_position_d_term_x100 = 0;
static int32  s_ball_target_velocity_x100 = 0;
static uint8  s_position_output_limited = 0;
static int32  s_position_previous_error_x100 = 0;
static uint8  s_position_error_seen = 0;
static uint8  s_position_target_crossed = 0;
static uint8  s_landing_capture_armed = 0;
static uint8  s_landing_hold_active = 0;
static int8   s_landing_move_direction = 0;
static uint8  s_landing_catch_frames_remaining = 0;
static uint32 s_landing_catch_angle_x100 = 0;
static uint32 s_landing_catch_cancel_x100 = 0;
static uint32 s_landing_best_error_x100 = 0;
static uint8  s_landing_trim_active = 0;
static uint8  s_landing_trim_frames = 0;
static uint8  s_landing_trim_cooldown_frames = 0;
static uint8  s_landing_trim_attempts = 0;
static uint8  s_landing_trim_no_motion_retries = 0;
static int32  s_landing_trim_angle_x100 = 0;
static int32  s_landing_trim_start_position_x100 = 0;
static uint8  s_landing_ceiling_guard_active = 0;
static int32  s_landing_guard_start_position_x100 = 0;
static uint8  s_landing_guard_motion_released = 0U;
static auto_run_phase_enum s_auto_run_phase = AUTO_RUN_IDLE;
static uint16 s_auto_run_start_ms = 0U;
static uint16 s_auto_run_right_ms = 0U;
static int32  s_auto_run_right_position_x100 = 0;
static uint8  s_auto_run_timeout_reported = 0U;
static uint8  s_auto_run_final_stable_frames = 0U;
static app_control_mode_enum s_app_control_mode = APP_CONTROL_IDLE;
static cascade_profile_enum s_cascade_profile = CASCADE_PROFILE_GENERIC_8926;
static uint8  s_auto_final_range_active = 0U;
static uint8  s_auto_final_range_seen = 0U;
static uint8  s_auto_final_far_guard_active = 0U;
static uint8  s_auto_final_far_guard_released = 0U;
static int32  s_auto_final_far_guard_start_position_x100 = 0;
static uint8  s_auto_final_recovery_active = 0U;
static uint8  s_auto_final_recovery_frames = 0U;
static uint8  s_auto_final_recovery_cooldown_frames = 0U;
static uint8  s_auto_final_recovery_retry = 0U;
static int32  s_auto_final_recovery_start_position_x100 = 0;
static int32  s_auto_final_recovery_angle_x100 = 0;
static competition_phase_enum s_competition_phase = COMPETITION_IDLE;
static uint8  s_competition_key_long_handled = 0U;
static uint8  s_competition_sequence_seen = 0U;
static uint8  s_competition_last_sequence = 0U;
static uint8  s_competition_stable_frames = 0U;
static uint16 s_competition_hands_off_start_ms = 0U;
static center_hold_phase_enum s_center_hold_phase = CENTER_HOLD_IDLE;
static uint8  s_center_hold_key_long_handled = 0U;
static uint8  s_center_hold_sequence_seen = 0U;
static uint8  s_center_hold_last_sequence = 0U;
static uint8  s_center_hold_stable_frames = 0U;

static int32  s_speed_error_x100 = 0;
static int32  s_speed_p_term_x100 = 0;
static int32  s_speed_i_accumulator = 0;
static int32  s_speed_i_term_x100 = 0;
static int32  s_speed_d_term_x100 = 0;
static int32  s_speed_filtered_accel_x100 = 0;
static int32  s_speed_last_velocity_x100 = 0;
static int32  s_vision_angle_command_x100 = 0;
static uint16 s_cascade_last_capture_ms = 0;
static uint8  s_cascade_time_seen = 0;
static uint8  s_speed_output_saturated = 0;
static uint8  s_speed_stiction_detect_count = 0;
static uint8  s_speed_stiction_active = 0;
static uint8  s_speed_stiction_active_frames = 0;
static int8   s_speed_stiction_direction = 0;
static int32  s_speed_stiction_start_position_x100 = 0;
static int32  s_speed_stiction_launch_angle_x100 = 0;
static int32  s_speed_stiction_last_launch_angle_x100 = 0;
static uint8  s_speed_stiction_retry_level = 0;
static uint8  s_speed_stiction_blocked = 0;
static int8   s_speed_stiction_last_direction = 0;
static uint32 s_speed_stiction_event_count = 0;
static uint8  s_manual_pulse_active = 0;
static uint32 s_manual_pulse_counter = 0;
static int8   s_manual_pulse_direction = 0;
static uint32 s_manual_pulse_limit = 0;
static int32  s_manual_pulse_start_position_x100 = 0;
static uint8  s_manual_pulse_last_sequence = 0;
static manual_pulse_phase_enum s_manual_pulse_phase = MANUAL_PULSE_PHASE_IDLE;
static uint32 s_manual_pulse_ramp_loops = 0;
static uint32 s_manual_pulse_hold_counter = 0;


static uint8 cascade_profile_is_key1_fixed (void)
{
    return (CASCADE_PROFILE_KEY1_FIXED == s_cascade_profile) ? 1U : 0U;
}

static const cascade_pid_profile_struct *cascade_pid_profile_get (void)
{
    if(cascade_profile_is_key1_fixed())
    {
        return &s_pid_profile_key1_fixed;
    }
    if((CASCADE_PROFILE_KEY2_CENTER == s_cascade_profile) ||
       (APP_CONTROL_KEY2_CENTER == s_app_control_mode))
    {
        return &s_pid_profile_key2_center;
    }
    if(APP_CONTROL_KEY1_FIXED == s_app_control_mode)
    {
        return &s_pid_profile_key1_prep;
    }
    return &s_pid_profile_generic_8926;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将格式化调试信息统一通过无线串口模块发送
// 备注信息     无线模块使用 UART1：B6=MCU_TX，B7=MCU_RX，B2=RTS
//-------------------------------------------------------------------------------------------------------------------
static uint16 wireless_tx_queue_free (void)
{
    if(s_wireless_tx_head >= s_wireless_tx_tail)
    {
        return (uint16)(WIRELESS_TX_QUEUE_SIZE -
            (s_wireless_tx_head - s_wireless_tx_tail) - 1U);
    }

    return (uint16)(s_wireless_tx_tail - s_wireless_tx_head - 1U);
}

static void wireless_tx_enqueue (const uint8 *data, uint16 length)
{
    uint16 index;

    if((NULL == data) || (0U == length))
    {
        return;
    }
    if(length > wireless_tx_queue_free())
    {
        s_wireless_tx_dropped_message_count ++;
        return;
    }

    for(index = 0U; index < length; index ++)
    {
        s_wireless_tx_queue[s_wireless_tx_head] = data[index];
        s_wireless_tx_head ++;
        if(s_wireless_tx_head >= WIRELESS_TX_QUEUE_SIZE)
        {
            s_wireless_tx_head = 0U;
        }
    }
}

static void wireless_tx_flush_one_chunk (void)
{
    uint16 available;
    uint16 contiguous;
    uint16 chunk_limit;

    if((s_wireless_tx_tail == s_wireless_tx_head) ||
       gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return;
    }

    if(s_wireless_tx_head > s_wireless_tx_tail)
    {
        available = (uint16)(s_wireless_tx_head - s_wireless_tx_tail);
    }
    else
    {
        available = (uint16)(WIRELESS_TX_QUEUE_SIZE - s_wireless_tx_tail);
    }
    contiguous = available;
    chunk_limit = s_stepper_enabled ? WIRELESS_TX_ACTIVE_CHUNK_SIZE :
        WIRELESS_TX_IDLE_CHUNK_SIZE;
    if(contiguous > chunk_limit)
    {
        contiguous = chunk_limit;
    }

    uart_write_buffer(WIRELESS_UART_INDEX,
        &s_wireless_tx_queue[s_wireless_tx_tail], contiguous);
    s_wireless_tx_tail = (uint16)(s_wireless_tx_tail + contiguous);
    if(s_wireless_tx_tail >= WIRELESS_TX_QUEUE_SIZE)
    {
        s_wireless_tx_tail = 0U;
    }
}

// Only call while the motor is disabled.  This preserves complete boot/DLOG
// output without putting the 100ms RTS wait in the live control path.
static void wireless_tx_flush_blocking (void)
{
    uint16 contiguous;
    uint32 remaining;
    uint16 sent;

    while(s_wireless_tx_tail != s_wireless_tx_head)
    {
        if(s_wireless_tx_head > s_wireless_tx_tail)
        {
            contiguous = (uint16)(s_wireless_tx_head - s_wireless_tx_tail);
        }
        else
        {
            contiguous = (uint16)(WIRELESS_TX_QUEUE_SIZE - s_wireless_tx_tail);
        }

        remaining = wireless_uart_send_buffer(
            &s_wireless_tx_queue[s_wireless_tx_tail], contiguous);
        sent = (uint16)(contiguous - remaining);
        s_wireless_tx_tail = (uint16)(s_wireless_tx_tail + sent);
        if(s_wireless_tx_tail >= WIRELESS_TX_QUEUE_SIZE)
        {
            s_wireless_tx_tail = 0U;
        }
        if(0U == sent)
        {
            break;
        }
    }
}

static void wireless_debug_printf (const char *format, ...)
{
    static char output_buffer[WIRELESS_DEBUG_BUFFER_SIZE];
    va_list args;
    int output_length;
    uint32 send_length;

    va_start(args, format);
    output_length = vsnprintf(output_buffer, WIRELESS_DEBUG_BUFFER_SIZE, format, args);
    va_end(args);

    if(output_length <= 0)
    {
        return;
    }

    send_length = (uint32)output_length;
    if(send_length >= WIRELESS_DEBUG_BUFFER_SIZE)
    {
        send_length = WIRELESS_DEBUG_BUFFER_SIZE - 1U;
    }
    wireless_tx_enqueue((const uint8 *)output_buffer, (uint16)send_length);
}

static uint32 int32_abs_to_uint32 (int32 value)
{
    return (value < 0) ? (uint32)(-value) : (uint32)value;
}

static int32 int32_clamp (int32 value, int32 minimum, int32 maximum)
{
    if(value < minimum)
    {
        return minimum;
    }
    if(value > maximum)
    {
        return maximum;
    }
    return value;
}

// Integer square root used by v_limit=sqrt(2*a*distance).  The inputs use
// x100 units, so the returned speed is also in x100 cm/s.
static uint32 integer_sqrt_u64 (uint64 value)
{
    uint64 result = 0U;
    uint64 bit = ((uint64)1U << 62U);

    while(bit > value)
    {
        bit >>= 2U;
    }
    while(0U != bit)
    {
        if(value >= (result + bit))
        {
            value -= result + bit;
            result = (result >> 1U) + bit;
        }
        else
        {
            result >>= 1U;
        }
        bit >>= 2U;
    }
    return (uint32)result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     CRC-8，多项式 0x07，初值 0x00
// 备注信息     与 MaixCAM2 发送端完全一致，用于丢弃串口干扰产生的错误帧
//-------------------------------------------------------------------------------------------------------------------
static uint8 camera_crc8_poly07 (const uint8 *data, uint8 length)
{
    uint8 crc = 0U;
    uint8 index;
    uint8 bit;

    for(index = 0U; index < length; index ++)
    {
        crc ^= data[index];
        for(bit = 0U; bit < 8U; bit ++)
        {
            crc = (0U != (crc & 0x80U)) ?
                (uint8)((crc << 1U) ^ 0x07U) : (uint8)(crc << 1U);
        }
    }
    return crc;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART2 接收中断：只把字节压入环形缓冲区，解析留在主循环完成
//-------------------------------------------------------------------------------------------------------------------
static void camera_uart_rx_callback (uint32 state, void *ptr)
{
    uint8 data;
    uint8 next_head;

    (void)ptr;
    if(UART_INTERRUPT_STATE_RX != state)
    {
        return;
    }

    while(uart_query_byte(CAMERA_UART_INDEX, &data))
    {
        next_head = (uint8)((s_camera_rx_head + 1U) & CAMERA_RX_RING_MASK);
        if(next_head != s_camera_rx_tail)
        {
            s_camera_rx_ring[s_camera_rx_head] = data;
            s_camera_rx_head = next_head;
        }
        else
        {
            s_camera_rx_overflow_count ++;
        }
    }
}

static uint8 camera_rx_ring_pop (uint8 *data)
{
    uint8 local_tail = s_camera_rx_tail;

    if(local_tail == s_camera_rx_head)
    {
        return 0U;
    }
    *data = s_camera_rx_ring[local_tail];
    s_camera_rx_tail = (uint8)((local_tail + 1U) & CAMERA_RX_RING_MASK);
    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析 13 字节视觉帧
// 帧格式       AA 55 01 FLAGS SEQ POS_L POS_H VEL_L VEL_H TIME_L TIME_H CRC 0D
// 单位         POS/VEL 为有符号整数，分别是 0.01cm、0.01cm/s；FLAGS bit0 表示识别有效
//-------------------------------------------------------------------------------------------------------------------
static void camera_accept_byte (uint8 data)
{
    uint8 sequence_delta;
    uint16 raw_value;

    if(0U == s_camera_frame_index)
    {
        if(CAMERA_FRAME_HEAD_0 == data)
        {
            s_camera_frame_buffer[0] = data;
            s_camera_frame_index = 1U;
        }
        return;
    }

    if(1U == s_camera_frame_index)
    {
        if(CAMERA_FRAME_HEAD_1 == data)
        {
            s_camera_frame_buffer[1] = data;
            s_camera_frame_index = 2U;
        }
        else if(CAMERA_FRAME_HEAD_0 != data)
        {
            s_camera_frame_index = 0U;
        }
        return;
    }

    s_camera_frame_buffer[s_camera_frame_index] = data;
    s_camera_frame_index ++;
    if(s_camera_frame_index < CAMERA_FRAME_LENGTH)
    {
        return;
    }
    s_camera_frame_index = 0U;

    if((CAMERA_FRAME_TAIL != s_camera_frame_buffer[12]) ||
       (CAMERA_FRAME_TYPE_BALL != s_camera_frame_buffer[2]) ||
       (s_camera_frame_buffer[11] !=
        camera_crc8_poly07(&s_camera_frame_buffer[2], 9U)))
    {
        s_camera_bad_frame_count ++;
        return;
    }

    if(s_camera_sequence_seen)
    {
        sequence_delta = (uint8)(s_camera_frame_buffer[4] - s_camera_sequence);
        if(sequence_delta > 1U)
        {
            s_camera_dropped_frame_count += (uint32)(sequence_delta - 1U);
        }
    }
    s_camera_sequence = s_camera_frame_buffer[4];
    s_camera_sequence_seen = 1U;
    s_camera_measurement_valid =
        (0U != (s_camera_frame_buffer[3] & 0x01U)) ? 1U : 0U;

    raw_value = (uint16)s_camera_frame_buffer[5] |
        ((uint16)s_camera_frame_buffer[6] << 8U);
    s_camera_position_x100 = (int32)(int16)raw_value;
    raw_value = (uint16)s_camera_frame_buffer[7] |
        ((uint16)s_camera_frame_buffer[8] << 8U);
    s_camera_velocity_x100 = (int32)(int16)raw_value;
    s_camera_capture_ms_low16 = (uint16)s_camera_frame_buffer[9] |
        ((uint16)s_camera_frame_buffer[10] << 8U);
    s_camera_link_age = 0U;
    s_camera_valid_frame_count ++;
}

static void camera_uart_process_received_data (void)
{
    uint8 data;

    while(camera_rx_ring_pop(&data))
    {
        camera_accept_byte(data);
    }
}

static uint16 auto_run_elapsed_ms (void)
{
    return (uint16)(s_camera_capture_ms_low16 - s_auto_run_start_ms);
}

static void auto_run_cancel (void)
{
    s_auto_run_phase = AUTO_RUN_IDLE;
    s_auto_run_start_ms = 0U;
    s_auto_run_right_ms = 0U;
    s_auto_run_right_position_x100 = 0;
    s_auto_run_timeout_reported = 0U;
    s_auto_run_final_stable_frames = 0U;
    s_cascade_profile = CASCADE_PROFILE_GENERIC_8926;
    s_auto_final_range_active = 0U;
    s_auto_final_range_seen = 0U;
    s_auto_final_far_guard_active = 0U;
    s_auto_final_far_guard_released = 0U;
    s_auto_final_far_guard_start_position_x100 = 0;
    s_auto_final_recovery_active = 0U;
    s_auto_final_recovery_frames = 0U;
    s_auto_final_recovery_cooldown_frames = 0U;
    s_auto_final_recovery_retry = 0U;
    s_auto_final_recovery_start_position_x100 = 0;
    s_auto_final_recovery_angle_x100 = 0;
}

static void landing_seed_support_integral (void)
{
    if(s_landing_move_direction > 0)
    {
        s_speed_i_term_x100 = POSITION_HOLD_RIGHT_SUPPORT_I_X100;
    }
    else
    {
        s_speed_i_term_x100 = POSITION_HOLD_LEFT_SUPPORT_I_X100;
    }
    s_speed_i_accumulator = s_speed_i_term_x100 * PID_INTEGRAL_SCALE;
}

static void cascade_pid_reset (void)
{
    int32 reset_position_error_x100 = s_ball_target_position_x100 -
        s_camera_position_x100;

    s_position_error_x100 = 0;
    s_position_p_term_x100 = 0;
    s_position_i_accumulator = 0;
    s_position_i_term_x100 = 0;
    s_position_d_term_x100 = 0;
    s_ball_target_velocity_x100 = 0;
    s_position_output_limited = 0U;
    s_position_previous_error_x100 = 0;
    s_position_error_seen = 0U;
    s_position_target_crossed = 0U;
    s_landing_capture_armed =
        (int32_abs_to_uint32(reset_position_error_x100) >=
            POSITION_LANDING_ARM_DISTANCE_X100) ? 1U : 0U;
    s_landing_hold_active = 0U;
    s_landing_move_direction = (reset_position_error_x100 > 0) ? 1 :
        ((reset_position_error_x100 < 0) ? -1 : 0);
    s_landing_catch_frames_remaining = 0U;
    s_landing_catch_angle_x100 = 0U;
    s_landing_catch_cancel_x100 = 0U;
    s_landing_best_error_x100 =
        int32_abs_to_uint32(reset_position_error_x100);
    s_landing_trim_active = 0U;
    s_landing_trim_frames = 0U;
    s_landing_trim_cooldown_frames = 0U;
    s_landing_trim_attempts = 0U;
    s_landing_trim_no_motion_retries = 0U;
    s_landing_trim_angle_x100 = 0;
    s_landing_trim_start_position_x100 = 0;
    s_landing_ceiling_guard_active = 0U;
    s_landing_guard_start_position_x100 = 0;
    s_landing_guard_motion_released = 0U;

    s_speed_error_x100 = 0;
    s_speed_p_term_x100 = 0;
    s_speed_i_accumulator = 0;
    s_speed_i_term_x100 = 0;
    s_speed_d_term_x100 = 0;
    s_speed_filtered_accel_x100 = 0;
    s_speed_last_velocity_x100 = s_camera_velocity_x100;
    s_vision_angle_command_x100 = 0;
    s_cascade_last_capture_ms = 0U;
    s_cascade_time_seen = 0U;
    s_speed_output_saturated = 0U;
    s_speed_stiction_detect_count = 0U;
    s_speed_stiction_active = 0U;
    s_speed_stiction_active_frames = 0U;
    s_speed_stiction_direction = 0;
    s_speed_stiction_start_position_x100 = 0;
    s_speed_stiction_launch_angle_x100 = 0;
    s_speed_stiction_last_launch_angle_x100 = 0;
    s_speed_stiction_retry_level = 0U;
    s_speed_stiction_blocked = 0U;
    s_speed_stiction_last_direction = 0;
    s_speed_stiction_event_count = 0U;
}

static int32 speed_stiction_launch_angle (int8 direction)
{
    int32 launch_angle_x100;
    int32 retry_extra_x100 =
        (int32)s_speed_stiction_retry_level * SPEED_STICTION_ANGLE_STEP_X100;

    if(direction > 0)
    {
        launch_angle_x100 = SPEED_STICTION_RIGHT_BASE_X100 -
            retry_extra_x100;
        if(launch_angle_x100 < SPEED_STICTION_RIGHT_MAX_X100)
        {
            launch_angle_x100 = SPEED_STICTION_RIGHT_MAX_X100;
        }
    }
    else
    {
        launch_angle_x100 = SPEED_STICTION_LEFT_BASE_X100 +
            retry_extra_x100;
        if(launch_angle_x100 > SPEED_STICTION_LEFT_MAX_X100)
        {
            launch_angle_x100 = SPEED_STICTION_LEFT_MAX_X100;
        }
    }
    return launch_angle_x100;
}

//-------------------------------------------------------------------------------------------------------------------
// Function brief      Detect a stationary ball that needs a measured breakaway angle
// Remarks             Directional velocity releases the boost immediately; every attempt also has a hard frame limit
//-------------------------------------------------------------------------------------------------------------------
static void speed_stiction_update (uint32 error_abs, uint32 velocity_abs)
{
    int32 directional_displacement_x100;
    int8 active_direction;
    int8 requested_direction = 0;
    uint8 directional_motion = 0U;
    uint8 launch_request_valid = 0U;

    if((s_position_error_x100 > 0) && (s_ball_target_velocity_x100 > 0))
    {
        requested_direction = 1;
    }
    else if((s_position_error_x100 < 0) &&
            (s_ball_target_velocity_x100 < 0))
    {
        requested_direction = -1;
    }

    launch_request_valid =
        (!s_position_target_crossed) &&
        (error_abs > SPEED_STICTION_POSITION_ERROR_X100) &&
        (int32_abs_to_uint32(s_ball_target_velocity_x100) >=
            SPEED_STICTION_TARGET_SPEED_X100) &&
        (0 != requested_direction);

    if((error_abs <= SPEED_STICTION_POSITION_ERROR_X100) ||
       ((0 != requested_direction) &&
        (0 != s_speed_stiction_last_direction) &&
        (requested_direction != s_speed_stiction_last_direction)))
    {
        s_speed_stiction_retry_level = 0U;
        s_speed_stiction_blocked = 0U;
    }

    if(s_speed_stiction_active)
    {
        s_speed_stiction_active_frames ++;
        directional_displacement_x100 = s_camera_position_x100 -
            s_speed_stiction_start_position_x100;
        directional_motion =
            (((s_speed_stiction_direction > 0) &&
                ((s_camera_velocity_x100 >= SPEED_STICTION_RELEASE_SPEED_X100) ||
                 (directional_displacement_x100 >=
                    SPEED_STICTION_RELEASE_DISTANCE_X100))) ||
             ((s_speed_stiction_direction < 0) &&
                ((s_camera_velocity_x100 <= -SPEED_STICTION_RELEASE_SPEED_X100) ||
                 (directional_displacement_x100 <=
                    -SPEED_STICTION_RELEASE_DISTANCE_X100)))) ?
                1U : 0U;

        if((!launch_request_valid) || directional_motion ||
           (requested_direction != s_speed_stiction_direction) ||
           (s_speed_stiction_active_frames >=
                SPEED_STICTION_MAX_ACTIVE_FRAMES))
        {
            active_direction = s_speed_stiction_direction;
            if((!directional_motion) && launch_request_valid &&
               (requested_direction == active_direction) &&
               (s_speed_stiction_active_frames >=
                    SPEED_STICTION_MAX_ACTIVE_FRAMES))
            {
                if(((active_direction > 0) &&
                    (s_speed_stiction_launch_angle_x100 <=
                        SPEED_STICTION_RIGHT_MAX_X100)) ||
                   ((active_direction < 0) &&
                    (s_speed_stiction_launch_angle_x100 >=
                        SPEED_STICTION_LEFT_MAX_X100)))
                {
                    // The maximum adaptive angle still produced no motion.
                    // Stop retrying until a new target/control reset.
                    s_speed_stiction_blocked = 1U;
                }
                else if(s_speed_stiction_retry_level < 255U)
                {
                    s_speed_stiction_retry_level ++;
                }
            }
            s_speed_stiction_active = 0U;
            s_speed_stiction_active_frames = 0U;
            s_speed_stiction_detect_count = 0U;
            s_speed_stiction_direction = 0;
            s_speed_stiction_start_position_x100 = 0;
            s_speed_stiction_launch_angle_x100 = 0;
        }
        return;
    }

    if(launch_request_valid && (!s_speed_stiction_blocked) &&
       (velocity_abs <= SPEED_STICTION_VELOCITY_X100))
    {
        if(s_speed_stiction_detect_count < SPEED_STICTION_DETECT_FRAMES)
        {
            s_speed_stiction_detect_count ++;
        }
        if(s_speed_stiction_detect_count >= SPEED_STICTION_DETECT_FRAMES)
        {
            s_speed_stiction_active = 1U;
            s_speed_stiction_active_frames = 0U;
            s_speed_stiction_detect_count = 0U;
            s_speed_stiction_direction = requested_direction;
            s_speed_stiction_start_position_x100 = s_camera_position_x100;
            s_speed_stiction_launch_angle_x100 =
                speed_stiction_launch_angle(requested_direction);
            s_speed_stiction_last_launch_angle_x100 =
                s_speed_stiction_launch_angle_x100;
            s_speed_stiction_last_direction = requested_direction;
            s_speed_stiction_event_count ++;

            // The launch-angle floor replaces slow integral buildup.  Clearing
            // the I term here lets braking take over cleanly after movement.
            s_speed_i_accumulator = 0;
            s_speed_i_term_x100 = 0;
        }
    }
    else
    {
        s_speed_stiction_detect_count = 0U;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串级双环 PID：位置 PID -> 目标球速，速度 PID -> 目标摆角
// 备注信息     仅在收到新视觉帧时调用；距离制动曲线和过零检测用于抑制冲过目标点
//-------------------------------------------------------------------------------------------------------------------
// Original 8926ff0 cascade controller.  It intentionally has no landing
// capture/final-range logic from later competition tuning.  Serial X/V and
// future non-KEY1 tasks use this path with their own PID profile.
static void cascade_pid_update_generic_8926 (void)
{
    const cascade_pid_profile_struct *pid_profile =
        &s_pid_profile_generic_8926;
    uint16 delta_ms;
    uint32 error_abs;
    uint32 velocity_abs;
    uint32 brake_speed_x100;
    int32 candidate_accumulator;
    int32 command_without_new_i;
    int32 desired_velocity_x100;
    int32 target_velocity_delta_x100;
    int32 max_velocity_increase_x100;
    int32 raw_accel_x100;
    int32 speed_effort_x100;
    int32 speed_integral_angle_limit_x100;
    int32 desired_angle_x100;
    int32 target_angle_delta_x100;
    int32 angle_slew_limit_x100;
    uint8 position_integral_allowed;
    uint8 speed_integral_allowed;
    uint8 settled;

    if(!s_cascade_time_seen)
    {
        delta_ms = CAMERA_NOMINAL_FRAME_PERIOD_MS;
        s_cascade_time_seen = 1U;
        s_speed_last_velocity_x100 = s_camera_velocity_x100;
    }
    else
    {
        delta_ms = (uint16)(s_camera_capture_ms_low16 -
            s_cascade_last_capture_ms);
        if(delta_ms < 5U)
        {
            delta_ms = 5U;
        }
        else if(delta_ms > 50U)
        {
            delta_ms = 50U;
        }
    }
    s_cascade_last_capture_ms = s_camera_capture_ms_low16;

    s_position_error_x100 = s_ball_target_position_x100 -
        s_camera_position_x100;
    error_abs = int32_abs_to_uint32(s_position_error_x100);
    velocity_abs = int32_abs_to_uint32(s_camera_velocity_x100);
    settled = ((error_abs <= POSITION_SETTLE_BAND_X100) &&
        (velocity_abs <= SPEED_SETTLE_BAND_X100)) ? 1U : 0U;

    s_position_target_crossed = 0U;
    if(s_position_error_seen &&
       (((s_position_previous_error_x100 > 0) &&
            (s_position_error_x100 < 0)) ||
        ((s_position_previous_error_x100 < 0) &&
            (s_position_error_x100 > 0))))
    {
        s_position_target_crossed = 1U;
        s_position_i_accumulator = 0;
        s_position_i_term_x100 = 0;
        s_speed_i_accumulator = 0;
        s_speed_i_term_x100 = 0;
        s_ball_target_velocity_x100 = 0;
    }
    if(0 != s_position_error_x100)
    {
        s_position_previous_error_x100 = s_position_error_x100;
        s_position_error_seen = 1U;
    }

    s_position_p_term_x100 =
        (s_position_error_x100 * pid_profile->position_kp_x100) / 100;
    s_position_d_term_x100 =
        -(s_camera_velocity_x100 * pid_profile->position_kd_right_x100) /
            100;

    brake_speed_x100 = integer_sqrt_u64(
        (uint64)2U * pid_profile->position_brake_right_x100 * error_abs);
    if(brake_speed_x100 >
       (uint32)pid_profile->position_speed_limit_x100)
    {
        brake_speed_x100 =
            (uint32)pid_profile->position_speed_limit_x100;
    }

    command_without_new_i = s_position_p_term_x100 +
        s_position_i_term_x100 + s_position_d_term_x100;
    position_integral_allowed =
        (error_abs > POSITION_INTEGRAL_FREEZE_X100) &&
        (int32_abs_to_uint32(command_without_new_i) <
            (uint32)pid_profile->position_speed_limit_x100) &&
        (int32_abs_to_uint32(command_without_new_i) < brake_speed_x100) &&
        (((s_position_error_x100 > 0) && (command_without_new_i >= 0)) ||
         ((s_position_error_x100 < 0) && (command_without_new_i <= 0)));

    if(position_integral_allowed)
    {
        candidate_accumulator = s_position_i_accumulator +
            s_position_error_x100 * pid_profile->position_ki_x100 *
                (int32)delta_ms;
        s_position_i_accumulator = int32_clamp(candidate_accumulator,
            -(pid_profile->position_i_limit_x100 * PID_INTEGRAL_SCALE),
            pid_profile->position_i_limit_x100 * PID_INTEGRAL_SCALE);
        s_position_i_term_x100 =
            s_position_i_accumulator / PID_INTEGRAL_SCALE;
    }

    desired_velocity_x100 = s_position_p_term_x100 +
        s_position_i_term_x100 + s_position_d_term_x100;
    s_position_output_limited = 0U;
    if(settled || s_position_target_crossed ||
       (error_abs <= POSITION_SETTLE_BAND_X100))
    {
        desired_velocity_x100 = 0;
        s_position_i_accumulator = 0;
        s_position_i_term_x100 = 0;
    }
    else if(s_position_error_x100 > 0)
    {
        if(desired_velocity_x100 < 0)
        {
            desired_velocity_x100 = 0;
            s_position_output_limited = 1U;
        }
        if(desired_velocity_x100 > (int32)brake_speed_x100)
        {
            desired_velocity_x100 = (int32)brake_speed_x100;
            s_position_output_limited = 1U;
        }
    }
    else
    {
        if(desired_velocity_x100 > 0)
        {
            desired_velocity_x100 = 0;
            s_position_output_limited = 1U;
        }
        if(desired_velocity_x100 < -(int32)brake_speed_x100)
        {
            desired_velocity_x100 = -(int32)brake_speed_x100;
            s_position_output_limited = 1U;
        }
    }
    desired_velocity_x100 = int32_clamp(desired_velocity_x100,
        -pid_profile->position_speed_limit_x100,
        pid_profile->position_speed_limit_x100);

    if((desired_velocity_x100 * s_ball_target_velocity_x100) < 0)
    {
        s_ball_target_velocity_x100 = 0;
    }
    else if(int32_abs_to_uint32(desired_velocity_x100) >
            int32_abs_to_uint32(s_ball_target_velocity_x100))
    {
        max_velocity_increase_x100 =
            (POSITION_SPEED_ACCEL_X100_PER_S * (int32)delta_ms) / 1000;
        if(max_velocity_increase_x100 < 1)
        {
            max_velocity_increase_x100 = 1;
        }
        target_velocity_delta_x100 = desired_velocity_x100 -
            s_ball_target_velocity_x100;
        target_velocity_delta_x100 = int32_clamp(target_velocity_delta_x100,
            -max_velocity_increase_x100, max_velocity_increase_x100);
        s_ball_target_velocity_x100 += target_velocity_delta_x100;
    }
    else
    {
        s_ball_target_velocity_x100 = desired_velocity_x100;
    }

    speed_stiction_update(error_abs, velocity_abs);

    raw_accel_x100 = ((s_camera_velocity_x100 -
        s_speed_last_velocity_x100) * 1000) / (int32)delta_ms;
    raw_accel_x100 = int32_clamp(raw_accel_x100,
        -pid_profile->speed_accel_limit_x100,
        pid_profile->speed_accel_limit_x100);
    s_speed_filtered_accel_x100 +=
        (raw_accel_x100 - s_speed_filtered_accel_x100) / 4;
    s_speed_last_velocity_x100 = s_camera_velocity_x100;

    s_speed_error_x100 = s_ball_target_velocity_x100 -
        s_camera_velocity_x100;
    s_speed_p_term_x100 =
        (s_speed_error_x100 * pid_profile->speed_kp_x100) / 100;
    s_speed_d_term_x100 =
        -(s_speed_filtered_accel_x100 * pid_profile->speed_kd_x100) / 100;

    command_without_new_i = s_speed_p_term_x100 +
        s_speed_i_term_x100 + s_speed_d_term_x100;
    speed_integral_angle_limit_x100 = (command_without_new_i < 0) ?
        pid_profile->speed_positive_angle_limit_x100 :
        pid_profile->speed_negative_angle_limit_x100;
    speed_integral_allowed = (!settled) &&
        (!s_speed_stiction_active) &&
        (int32_abs_to_uint32(command_without_new_i) <
            (uint32)speed_integral_angle_limit_x100);
    if(speed_integral_allowed)
    {
        candidate_accumulator = s_speed_i_accumulator +
            s_speed_error_x100 * pid_profile->speed_ki_x100 *
                (int32)delta_ms;
        s_speed_i_accumulator = int32_clamp(candidate_accumulator,
            -(pid_profile->speed_i_limit_x100 * PID_INTEGRAL_SCALE),
            pid_profile->speed_i_limit_x100 * PID_INTEGRAL_SCALE);
        s_speed_i_term_x100 = s_speed_i_accumulator / PID_INTEGRAL_SCALE;
    }

    if(settled)
    {
        s_speed_i_accumulator = 0;
        s_speed_i_term_x100 = 0;
        speed_effort_x100 = 0;
    }
    else
    {
        speed_effort_x100 = s_speed_p_term_x100 +
            s_speed_i_term_x100 + s_speed_d_term_x100;
    }

    desired_angle_x100 = -speed_effort_x100;
    if(s_speed_stiction_active)
    {
        if((s_speed_stiction_direction > 0) &&
           (desired_angle_x100 > s_speed_stiction_launch_angle_x100))
        {
            desired_angle_x100 = s_speed_stiction_launch_angle_x100;
        }
        else if((s_speed_stiction_direction < 0) &&
                (desired_angle_x100 < s_speed_stiction_launch_angle_x100))
        {
            desired_angle_x100 = s_speed_stiction_launch_angle_x100;
        }
    }
    s_speed_output_saturated = 0U;
    if(desired_angle_x100 > pid_profile->speed_positive_angle_limit_x100)
    {
        desired_angle_x100 = pid_profile->speed_positive_angle_limit_x100;
        s_speed_output_saturated = 1U;
    }
    else if(desired_angle_x100 <
            -pid_profile->speed_negative_angle_limit_x100)
    {
        desired_angle_x100 =
            -pid_profile->speed_negative_angle_limit_x100;
        s_speed_output_saturated = 1U;
    }
    s_vision_angle_command_x100 = desired_angle_x100;

    if(((desired_angle_x100 * s_camera_velocity_x100) > 0) ||
       (int32_abs_to_uint32(s_ball_target_velocity_x100) < velocity_abs) ||
       s_position_target_crossed)
    {
        angle_slew_limit_x100 = VISION_BRAKE_ANGLE_SLEW_X100;
    }
    else
    {
        angle_slew_limit_x100 = VISION_ACCEL_ANGLE_SLEW_X100;
    }
    target_angle_delta_x100 = desired_angle_x100 - s_stepper_target_x100;
    target_angle_delta_x100 = int32_clamp(target_angle_delta_x100,
        -angle_slew_limit_x100, angle_slew_limit_x100);
    s_stepper_target_x100 += target_angle_delta_x100;
    s_direction_check_active = 0U;
}

static void cascade_pid_update_enhanced (void)
{
    const cascade_pid_profile_struct *pid_profile;
    uint16 delta_ms;
    uint32 error_abs;
    uint32 velocity_abs;
    uint32 brake_speed_x100;
    uint32 brake_accel_x100;
    int32 candidate_accumulator;
    int32 command_without_new_i;
    int32 position_kd_x100;
    int32 desired_velocity_x100;
    int32 target_velocity_delta_x100;
    int32 max_velocity_increase_x100;
    int32 raw_accel_x100;
    int32 speed_effort_x100;
    int32 speed_integral_angle_limit_x100;
    int32 desired_angle_x100;
    int32 target_angle_delta_x100;
    int32 angle_slew_limit_x100;
    int32 landing_trim_displacement_x100;
    int32 hold_angle_limit_x100;
    uint8 position_integral_allowed;
    uint8 speed_integral_allowed;
    uint8 settled;
    uint8 landing_just_captured = 0U;
    uint8 landing_trim_stop_reached;
    uint8 landing_trim_directional_motion;
    uint8 landing_trim_start_valid;
    int32 landing_trim_base_x100;
    int32 landing_trim_step_x100;
    int32 landing_trim_max_x100;
    int32 auto_final_recovery_displacement_x100;

    pid_profile = cascade_pid_profile_get();

    if(!s_cascade_time_seen)
    {
        delta_ms = CAMERA_NOMINAL_FRAME_PERIOD_MS;
        s_cascade_time_seen = 1U;
        s_speed_last_velocity_x100 = s_camera_velocity_x100;
    }
    else
    {
        delta_ms = (uint16)(s_camera_capture_ms_low16 -
            s_cascade_last_capture_ms);
        if(delta_ms < 5U)
        {
            delta_ms = 5U;
        }
        else if(delta_ms > 50U)
        {
            delta_ms = 50U;
        }
    }
    s_cascade_last_capture_ms = s_camera_capture_ms_low16;

    s_position_error_x100 = s_ball_target_position_x100 -
        s_camera_position_x100;
    error_abs = int32_abs_to_uint32(s_position_error_x100);
    velocity_abs = int32_abs_to_uint32(s_camera_velocity_x100);

    // The S1 left endpoint is an accepted range, not a mathematical point.
    // After the first entry, this dedicated controller owns the endpoint both
    // inside and outside the range so the old catch/trim state cannot re-enter.
    if(cascade_profile_is_key1_fixed() &&
       ((AUTO_RUN_TO_LEFT == s_auto_run_phase) ||
        ((AUTO_RUN_COMPLETE == s_auto_run_phase) &&
         s_auto_final_range_seen)))
    {
        if((!s_auto_final_range_seen) &&
           (s_camera_position_x100 <= AUTO_FINAL_MODE_ENTER_X100))
        {
            s_auto_final_range_seen = 1U;
        }
        if(s_auto_final_range_seen)
        {
            s_auto_final_range_active =
                (s_camera_position_x100 <= AUTO_FINAL_RANGE_NEAR_X100) ?
                    1U : 0U;

            // Permanently suppress the legacy endpoint capture, catch, trim,
            // guard and general stiction loops after the range was first seen.
            s_landing_capture_armed = 0U;
            s_landing_hold_active = s_auto_final_range_active;
            s_landing_move_direction = -1;
            s_landing_catch_frames_remaining = 0U;
            s_landing_catch_angle_x100 = 0U;
            s_landing_catch_cancel_x100 = 0U;
            s_landing_trim_active = 0U;
            s_landing_trim_frames = 0U;
            s_landing_trim_cooldown_frames = 0U;
            s_landing_trim_attempts = 0U;
            s_landing_trim_no_motion_retries = 0U;
            s_landing_trim_angle_x100 = 0;
            s_landing_ceiling_guard_active = 0U;
            s_landing_guard_motion_released = 0U;
            s_speed_stiction_detect_count = 0U;
            s_speed_stiction_active = 0U;
            s_speed_stiction_active_frames = 0U;
            s_speed_stiction_direction = 0;
            s_speed_stiction_start_position_x100 = 0;
            s_speed_stiction_launch_angle_x100 = 0;
            s_speed_stiction_retry_level = 0U;
            s_speed_stiction_blocked = 0U;
            s_position_p_term_x100 = 0;
            s_position_i_accumulator = 0;
            s_position_i_term_x100 = 0;
            s_position_d_term_x100 = 0;
            s_ball_target_velocity_x100 = 0;
            s_position_output_limited = 0U;
            s_position_target_crossed = 0U;
            s_speed_error_x100 = -s_camera_velocity_x100;
            s_speed_i_accumulator = 0;
            s_speed_i_term_x100 = 0;
            s_speed_d_term_x100 = 0;

            desired_angle_x100 = AUTO_FINAL_SUPPORT_ANGLE_X100 +
                (s_camera_velocity_x100 * AUTO_FINAL_DAMPING_KP_X100) / 100;

            if(s_auto_final_range_active)
            {
                s_auto_final_recovery_active = 0U;
                s_auto_final_recovery_frames = 0U;
                s_auto_final_recovery_cooldown_frames = 0U;
                s_auto_final_recovery_retry = 0U;
                s_auto_final_recovery_start_position_x100 = 0;
                s_auto_final_recovery_angle_x100 = 0;

                if(s_auto_final_far_guard_released &&
                   (s_camera_position_x100 >= AUTO_FINAL_GUARD_REARM_X100))
                {
                    s_auto_final_far_guard_released = 0U;
                }
                if((!s_auto_final_far_guard_active) &&
                   (!s_auto_final_far_guard_released) &&
                   (s_camera_position_x100 <= AUTO_FINAL_GUARD_ENTER_X100))
                {
                    s_auto_final_far_guard_active = 1U;
                    s_auto_final_far_guard_start_position_x100 =
                        s_camera_position_x100;
                }
                else if(s_auto_final_far_guard_active &&
                        ((s_camera_position_x100 -
                            s_auto_final_far_guard_start_position_x100) >=
                                AUTO_FINAL_GUARD_RELEASE_X100) &&
                        (s_camera_position_x100 >=
                            (AUTO_FINAL_RANGE_FAR_X100 +
                                AUTO_FINAL_GUARD_RELEASE_X100)))
                {
                    s_auto_final_far_guard_active = 0U;
                    s_auto_final_far_guard_released = 1U;
                    s_auto_final_far_guard_start_position_x100 = 0;
                }

                desired_angle_x100 = int32_clamp(desired_angle_x100,
                    AUTO_FINAL_DAMPING_MIN_X100,
                    AUTO_FINAL_DAMPING_MAX_X100);
                if(s_auto_final_far_guard_active &&
                   (desired_angle_x100 > AUTO_FINAL_GUARD_ANGLE_X100))
                {
                    desired_angle_x100 = AUTO_FINAL_GUARD_ANGLE_X100;
                }
            }
            else
            {
                // Outside x=-4.50cm, first use support+bias damping to arrest
                // the rebound.  Once nearly still, apply a bounded breakaway
                // pulse and withdraw it after only 0.05cm of leftward motion.
                s_auto_final_far_guard_active = 0U;
                s_auto_final_far_guard_released = 0U;
                s_auto_final_far_guard_start_position_x100 = 0;
                if(s_auto_final_recovery_cooldown_frames > 0U)
                {
                    s_auto_final_recovery_cooldown_frames --;
                }
                if(s_auto_final_recovery_active)
                {
                    s_auto_final_recovery_frames ++;
                    auto_final_recovery_displacement_x100 =
                        s_camera_position_x100 -
                            s_auto_final_recovery_start_position_x100;
                    if((auto_final_recovery_displacement_x100 <=
                            -AUTO_FINAL_RECOVERY_RELEASE_X100) ||
                       (s_camera_velocity_x100 <=
                            -AUTO_FINAL_RECOVERY_SPEED_X100))
                    {
                        s_auto_final_recovery_active = 0U;
                        s_auto_final_recovery_frames = 0U;
                        s_auto_final_recovery_cooldown_frames =
                            AUTO_FINAL_RECOVERY_COOLDOWN_FRAMES;
                        s_auto_final_recovery_angle_x100 = 0;
                    }
                    else if(s_auto_final_recovery_frames >=
                            AUTO_FINAL_RECOVERY_ACTIVE_FRAMES)
                    {
                        s_auto_final_recovery_active = 0U;
                        s_auto_final_recovery_frames = 0U;
                        s_auto_final_recovery_cooldown_frames = 1U;
                        s_auto_final_recovery_angle_x100 = 0;
                        if(s_auto_final_recovery_retry < 255U)
                        {
                            s_auto_final_recovery_retry ++;
                        }
                    }
                }
                if((!s_auto_final_recovery_active) &&
                   (0U == s_auto_final_recovery_cooldown_frames) &&
                   (velocity_abs <= AUTO_FINAL_RECOVERY_SPEED_X100))
                {
                    s_auto_final_recovery_active = 1U;
                    s_auto_final_recovery_frames = 0U;
                    s_auto_final_recovery_start_position_x100 =
                        s_camera_position_x100;
                    s_auto_final_recovery_angle_x100 =
                        AUTO_FINAL_RECOVERY_BASE_X100 +
                        (int32)s_auto_final_recovery_retry *
                            AUTO_FINAL_RECOVERY_STEP_X100;
                    if(s_auto_final_recovery_angle_x100 >
                        AUTO_FINAL_RECOVERY_MAX_X100)
                    {
                        s_auto_final_recovery_angle_x100 =
                            AUTO_FINAL_RECOVERY_MAX_X100;
                    }
                }

                desired_angle_x100 = int32_clamp(desired_angle_x100,
                    AUTO_FINAL_DAMPING_MIN_X100,
                    AUTO_FINAL_RECOVERY_BASE_X100);
                if(s_auto_final_recovery_active)
                {
                    desired_angle_x100 = s_auto_final_recovery_angle_x100;
                }
            }

            s_speed_p_term_x100 = -desired_angle_x100;
            s_speed_output_saturated =
                (s_auto_final_far_guard_active ||
                 s_auto_final_recovery_active) ? 1U : 0U;
            s_vision_angle_command_x100 = desired_angle_x100;
            target_angle_delta_x100 = desired_angle_x100 -
                s_stepper_target_x100;
            target_angle_delta_x100 = int32_clamp(target_angle_delta_x100,
                -AUTO_FINAL_ANGLE_SLEW_X100,
                AUTO_FINAL_ANGLE_SLEW_X100);
            s_stepper_target_x100 += target_angle_delta_x100;
            s_speed_filtered_accel_x100 = 0;
            s_speed_last_velocity_x100 = s_camera_velocity_x100;
            s_direction_check_active = 0U;
            return;
        }
    }

    // Release a latched landing if an external disturbance moves the ball well
    // outside the capture band.  This prevents HOLD from masking a large error.
    if(s_landing_hold_active &&
       (error_abs > ((s_landing_move_direction < 0) ?
            POSITION_LANDING_RELEASE_LEFT_X100 :
            POSITION_LANDING_RELEASE_RIGHT_X100)))
    {
        s_landing_hold_active = 0U;
        s_landing_capture_armed = 1U;
        s_landing_catch_frames_remaining = 0U;
        s_landing_catch_angle_x100 = 0U;
        s_landing_catch_cancel_x100 = 0U;
        s_landing_best_error_x100 = error_abs;
        s_landing_trim_active = 0U;
        s_landing_trim_frames = 0U;
        s_landing_trim_cooldown_frames = 0U;
        s_landing_trim_attempts = 0U;
        s_landing_trim_no_motion_retries = 0U;
        s_landing_trim_angle_x100 = 0;
        s_landing_trim_start_position_x100 = 0;
        s_landing_ceiling_guard_active = 0U;
        s_landing_guard_start_position_x100 = 0;
        s_landing_guard_motion_released = 0U;
        s_speed_i_accumulator = 0;
        s_speed_i_term_x100 = 0;
    }

    if(s_landing_capture_armed &&
       (error_abs < s_landing_best_error_x100))
    {
        s_landing_best_error_x100 = error_abs;
    }

    // Camera velocity is filtered and can lag the physical turning point.  A
    // 0.05cm retreat from the best position therefore provides a second,
    // position-only landing detector while the ball is still inside 1.00cm.
    if(s_landing_capture_armed &&
       (((error_abs <= POSITION_LANDING_BAND_X100) &&
         (velocity_abs <= POSITION_LANDING_SPEED_X100)) ||
        ((s_landing_best_error_x100 <= POSITION_LANDING_BAND_X100) &&
         (error_abs >= (s_landing_best_error_x100 +
            POSITION_LANDING_RETREAT_X100)) &&
         (velocity_abs <= POSITION_LANDING_RETREAT_SPEED_X100) &&
         (error_abs <= POSITION_LANDING_MAX_ERROR_X100))))
    {
        s_landing_capture_armed = 0U;
        s_landing_hold_active = 1U;
        s_landing_trim_active = 0U;
        s_landing_trim_frames = 0U;
        s_landing_trim_cooldown_frames = 0U;
        s_landing_trim_attempts = 0U;
        s_landing_trim_no_motion_retries = 0U;
        s_landing_trim_angle_x100 = 0;
        s_landing_ceiling_guard_active = 0U;
        s_landing_guard_start_position_x100 = 0;
        s_landing_guard_motion_released = 0U;
        if(s_landing_move_direction > 0)
        {
            s_landing_catch_frames_remaining =
                POSITION_HOLD_CATCH_RIGHT_FRAMES;
            s_landing_catch_angle_x100 =
                POSITION_HOLD_CATCH_RIGHT_ANGLE_X100;
            s_landing_catch_cancel_x100 =
                POSITION_HOLD_CATCH_RIGHT_CANCEL_X100;
        }
        else
        {
            s_landing_catch_frames_remaining =
                POSITION_HOLD_CATCH_LEFT_FRAMES;
            s_landing_catch_angle_x100 =
                POSITION_HOLD_CATCH_LEFT_ANGLE_X100;
            s_landing_catch_cancel_x100 =
                POSITION_HOLD_CATCH_LEFT_CANCEL_X100;
        }
        landing_just_captured = 1U;
    }
    if(s_landing_hold_active)
    {
        // HOLD remains a quiet closed loop.  A small position request lets the
        // speed integrator learn the mechanism's real support angle instead of
        // assuming that encoder-relative 0deg is physically level.
        s_position_p_term_x100 =
            (s_position_error_x100 * POSITION_HOLD_KP_X100) / 100;
        s_position_i_accumulator = 0;
        s_position_i_term_x100 = 0;
        s_position_d_term_x100 = 0;
        s_ball_target_velocity_x100 = int32_clamp(s_position_p_term_x100,
            -POSITION_HOLD_SPEED_LIMIT_X100,
            POSITION_HOLD_SPEED_LIMIT_X100);
        s_position_output_limited =
            (s_ball_target_velocity_x100 != s_position_p_term_x100) ? 1U : 0U;
        s_position_target_crossed = 0U;

        s_speed_stiction_detect_count = 0U;
        s_speed_stiction_active = 0U;
        s_speed_stiction_active_frames = 0U;
        s_speed_stiction_direction = 0;
        s_speed_stiction_start_position_x100 = 0;
        s_speed_stiction_launch_angle_x100 = 0;
        s_speed_stiction_retry_level = 0U;
        s_speed_stiction_blocked = 0U;

        // Each endpoint uses a directional overshoot guard.  The same trim
        // state machine then applies short, position-released breakaway pulses
        // toward the target without changing the already tuned X+ parameters.
        if(s_landing_move_direction > 0)
        {
            if((!s_landing_ceiling_guard_active) &&
               (s_position_error_x100 <=
                    -POSITION_HOLD_RIGHT_CEILING_ENTER_X100))
            {
                s_landing_ceiling_guard_active = 1U;
                s_landing_guard_start_position_x100 =
                    s_camera_position_x100;
                s_landing_guard_motion_released = 0U;
                s_landing_catch_frames_remaining = 0U;
                s_landing_catch_angle_x100 = 0U;
                s_landing_catch_cancel_x100 = 0U;
                s_landing_trim_active = 0U;
                s_landing_trim_frames = 0U;
                s_landing_trim_no_motion_retries = 0U;
                s_landing_trim_angle_x100 = 0;
                s_speed_i_accumulator = 0;
                s_speed_i_term_x100 = 0;
            }
            else if(s_landing_ceiling_guard_active &&
                    (s_position_error_x100 >=
                        -POSITION_HOLD_RIGHT_CEILING_EXIT_X100))
            {
                s_landing_ceiling_guard_active = 0U;
                s_landing_guard_start_position_x100 = 0;
                s_landing_guard_motion_released = 0U;
                s_speed_i_accumulator = 0;
                s_speed_i_term_x100 = 0;
            }
        }
        else if(s_landing_move_direction < 0)
        {
            if(s_landing_guard_motion_released &&
               (s_position_error_x100 <=
                    POSITION_HOLD_LEFT_FLOOR_EXIT_X100))
            {
                s_landing_guard_motion_released = 0U;
            }
            if((!s_landing_ceiling_guard_active) &&
               (!s_landing_guard_motion_released) &&
               (s_position_error_x100 >=
                    POSITION_HOLD_LEFT_FLOOR_ENTER_X100))
            {
                s_landing_ceiling_guard_active = 1U;
                s_landing_guard_start_position_x100 =
                    s_camera_position_x100;
                s_landing_guard_motion_released = 0U;
                s_landing_catch_frames_remaining = 0U;
                s_landing_catch_angle_x100 = 0U;
                s_landing_catch_cancel_x100 = 0U;
                s_landing_trim_active = 0U;
                s_landing_trim_frames = 0U;
                s_landing_trim_no_motion_retries = 0U;
                s_landing_trim_angle_x100 = 0;
                s_speed_i_accumulator = 0;
                s_speed_i_term_x100 = 0;
            }
            else if(s_landing_ceiling_guard_active &&
                    ((s_camera_position_x100 -
                        s_landing_guard_start_position_x100) >=
                            POSITION_HOLD_LEFT_FLOOR_RELEASE_DISTANCE_X100))
            {
                s_landing_ceiling_guard_active = 0U;
                s_landing_guard_start_position_x100 = 0;
                s_landing_guard_motion_released = 1U;
                s_speed_i_accumulator = 0;
                s_speed_i_term_x100 = 0;
            }
            else if(s_landing_ceiling_guard_active &&
                    (s_position_error_x100 <=
                        POSITION_HOLD_LEFT_FLOOR_EXIT_X100))
            {
                s_landing_ceiling_guard_active = 0U;
                s_landing_guard_start_position_x100 = 0;
                s_landing_guard_motion_released = 0U;
                s_speed_i_accumulator = 0;
                s_speed_i_term_x100 = 0;
            }
        }

        if(s_landing_trim_cooldown_frames > 0U)
        {
            s_landing_trim_cooldown_frames --;
        }
        if(s_landing_trim_active)
        {
            s_landing_trim_frames ++;
            landing_trim_displacement_x100 = s_camera_position_x100 -
                s_landing_trim_start_position_x100;
            landing_trim_stop_reached =
                (((s_landing_move_direction > 0) &&
                    (s_position_error_x100 <=
                        POSITION_HOLD_RIGHT_TRIM_STOP_X100)) ||
                 ((s_landing_move_direction < 0) &&
                    (s_position_error_x100 >=
                        -POSITION_HOLD_LEFT_TRIM_STOP_X100))) ? 1U : 0U;
            landing_trim_directional_motion =
                (((s_landing_move_direction > 0) &&
                    ((landing_trim_displacement_x100 >=
                        POSITION_HOLD_RIGHT_TRIM_DISTANCE_X100) ||
                     (s_camera_velocity_x100 >=
                        POSITION_HOLD_RIGHT_TRIM_SPEED_X100))) ||
                 ((s_landing_move_direction < 0) &&
                    ((landing_trim_displacement_x100 <=
                        -POSITION_HOLD_RIGHT_TRIM_DISTANCE_X100) ||
                     (s_camera_velocity_x100 <=
                        -POSITION_HOLD_RIGHT_TRIM_SPEED_X100)))) ? 1U : 0U;
            if(landing_trim_stop_reached ||
               s_landing_ceiling_guard_active)
            {
                s_landing_trim_active = 0U;
                s_landing_trim_frames = 0U;
                s_landing_trim_cooldown_frames = 0U;
                s_landing_trim_no_motion_retries = 0U;
                s_landing_trim_angle_x100 = 0;
                landing_seed_support_integral();
            }
            else if(landing_trim_directional_motion)
            {
                // Measured motion toward the active endpoint is the primary
                // safety release for either direction.
                s_landing_trim_active = 0U;
                s_landing_trim_frames = 0U;
                s_landing_trim_cooldown_frames =
                    POSITION_HOLD_RIGHT_TRIM_COOLDOWN_FRAMES;
                s_landing_trim_angle_x100 = 0;
                // Keep the angle level that produced motion, and immediately
                // return to the measured support bias instead of dropping to
                // a weak proportional-only command that lets the ball roll back.
                landing_seed_support_integral();
            }
            else if(s_landing_trim_frames >=
                    POSITION_HOLD_RIGHT_TRIM_FRAMES)
            {
                // A no-motion attempt is also time bounded.  Retry after one
                // camera frame, up to the global attempt limit.
                s_landing_trim_active = 0U;
                s_landing_trim_frames = 0U;
                s_landing_trim_cooldown_frames = 1U;
                if(s_landing_trim_no_motion_retries < 255U)
                {
                    s_landing_trim_no_motion_retries ++;
                }
                s_landing_trim_angle_x100 = 0;
                s_speed_i_accumulator = 0;
                s_speed_i_term_x100 = 0;
            }
        }
        landing_trim_start_valid =
            (((s_landing_move_direction > 0) &&
                (s_position_error_x100 >
                    POSITION_HOLD_RIGHT_TRIM_START_X100) &&
                (s_camera_velocity_x100 <=
                    POSITION_HOLD_RIGHT_TRIM_SPEED_X100)) ||
             ((s_landing_move_direction < 0) &&
                (s_position_error_x100 <
                    -POSITION_HOLD_LEFT_TRIM_START_X100) &&
                (s_camera_velocity_x100 >=
                    -POSITION_HOLD_RIGHT_TRIM_SPEED_X100))) ? 1U : 0U;
        if((!s_landing_trim_active) &&
           (!s_landing_ceiling_guard_active) &&
           (0U == s_landing_catch_frames_remaining) &&
           (0U == s_landing_trim_cooldown_frames) &&
           landing_trim_start_valid &&
           (s_landing_trim_attempts <
                POSITION_HOLD_RIGHT_TRIM_MAX_ATTEMPTS))
        {
            s_landing_trim_active = 1U;
            s_landing_trim_frames = 0U;
            s_landing_trim_attempts ++;
            s_landing_trim_start_position_x100 = s_camera_position_x100;
            landing_trim_base_x100 = (s_landing_move_direction > 0) ?
                POSITION_HOLD_RIGHT_TRIM_BASE_X100 :
                POSITION_HOLD_LEFT_TRIM_BASE_X100;
            landing_trim_step_x100 = (s_landing_move_direction > 0) ?
                POSITION_HOLD_RIGHT_TRIM_STEP_X100 :
                POSITION_HOLD_LEFT_TRIM_STEP_X100;
            landing_trim_max_x100 = (s_landing_move_direction > 0) ?
                POSITION_HOLD_RIGHT_TRIM_MAX_X100 :
                POSITION_HOLD_LEFT_TRIM_MAX_X100;
            s_landing_trim_angle_x100 = landing_trim_base_x100 +
                (int32)s_landing_trim_no_motion_retries *
                    landing_trim_step_x100;
            if(s_landing_trim_angle_x100 > landing_trim_max_x100)
            {
                s_landing_trim_angle_x100 = landing_trim_max_x100;
            }
            s_speed_i_accumulator = 0;
            s_speed_i_term_x100 = 0;
        }

        if(landing_just_captured)
        {
            s_speed_i_accumulator = 0;
            s_speed_i_term_x100 = 0;
            s_speed_filtered_accel_x100 = 0;
            s_speed_last_velocity_x100 = s_camera_velocity_x100;
        }
        raw_accel_x100 = ((s_camera_velocity_x100 -
            s_speed_last_velocity_x100) * 1000) / (int32)delta_ms;
        raw_accel_x100 = int32_clamp(raw_accel_x100,
            -pid_profile->speed_accel_limit_x100,
            pid_profile->speed_accel_limit_x100);
        s_speed_filtered_accel_x100 +=
            (raw_accel_x100 - s_speed_filtered_accel_x100) / 4;
        s_speed_last_velocity_x100 = s_camera_velocity_x100;

        s_speed_error_x100 = s_ball_target_velocity_x100 -
            s_camera_velocity_x100;
        s_speed_p_term_x100 =
            (s_speed_error_x100 * pid_profile->speed_kp_x100) / 100;
        s_speed_d_term_x100 =
            -(s_speed_filtered_accel_x100 * pid_profile->speed_kd_x100) / 100;
        command_without_new_i = s_speed_p_term_x100 +
            s_speed_i_term_x100 + s_speed_d_term_x100;
        if(int32_abs_to_uint32(command_without_new_i) <
            POSITION_HOLD_ANGLE_LIMIT_X100)
        {
            candidate_accumulator = s_speed_i_accumulator +
                s_speed_error_x100 * pid_profile->speed_ki_x100 *
                    (int32)delta_ms;
            s_speed_i_accumulator = int32_clamp(candidate_accumulator,
                -(pid_profile->speed_i_limit_x100 * PID_INTEGRAL_SCALE),
                pid_profile->speed_i_limit_x100 * PID_INTEGRAL_SCALE);
            s_speed_i_term_x100 =
                s_speed_i_accumulator / PID_INTEGRAL_SCALE;
        }

        speed_effort_x100 = s_speed_p_term_x100 +
            s_speed_i_term_x100 + s_speed_d_term_x100;
        desired_angle_x100 = -speed_effort_x100;
        hold_angle_limit_x100 = POSITION_HOLD_ANGLE_LIMIT_X100;
        if(s_landing_catch_frames_remaining > 0U)
        {
            if(error_abs <= s_landing_catch_cancel_x100)
            {
                s_landing_catch_frames_remaining = 0U;
                s_landing_catch_angle_x100 = 0U;
                s_landing_catch_cancel_x100 = 0U;
            }
            else
            {
                hold_angle_limit_x100 = POSITION_HOLD_TRANSIENT_LIMIT_X100;
                if((s_position_error_x100 > 0) &&
                   (desired_angle_x100 > -(int32)s_landing_catch_angle_x100))
                {
                    desired_angle_x100 =
                        -(int32)s_landing_catch_angle_x100;
                }
                else if((s_position_error_x100 < 0) &&
                        (desired_angle_x100 <
                            (int32)s_landing_catch_angle_x100))
                {
                    desired_angle_x100 =
                        (int32)s_landing_catch_angle_x100;
                }
                s_landing_catch_frames_remaining --;
                if(0U == s_landing_catch_frames_remaining)
                {
                    s_landing_catch_angle_x100 = 0U;
                    s_landing_catch_cancel_x100 = 0U;
                }
            }
        }
        if(s_landing_trim_active)
        {
            desired_angle_x100 = (s_landing_move_direction > 0) ?
                -s_landing_trim_angle_x100 : s_landing_trim_angle_x100;
            hold_angle_limit_x100 = POSITION_HOLD_TRANSIENT_LIMIT_X100;
        }
        if(s_landing_ceiling_guard_active)
        {
            desired_angle_x100 = (s_landing_move_direction > 0) ?
                POSITION_HOLD_RIGHT_CEILING_ANGLE_X100 :
                POSITION_HOLD_LEFT_FLOOR_ANGLE_X100;
            hold_angle_limit_x100 = POSITION_HOLD_TRANSIENT_LIMIT_X100;
        }
        s_speed_output_saturated = 0U;
        if(desired_angle_x100 > hold_angle_limit_x100)
        {
            desired_angle_x100 = hold_angle_limit_x100;
            s_speed_output_saturated = 1U;
        }
        else if(desired_angle_x100 < -hold_angle_limit_x100)
        {
            desired_angle_x100 = -hold_angle_limit_x100;
            s_speed_output_saturated = 1U;
        }
        s_vision_angle_command_x100 = desired_angle_x100;
        target_angle_delta_x100 = desired_angle_x100 -
            s_stepper_target_x100;
        target_angle_delta_x100 = int32_clamp(target_angle_delta_x100,
            -VISION_BRAKE_ANGLE_SLEW_X100,
            VISION_BRAKE_ANGLE_SLEW_X100);
        s_stepper_target_x100 += target_angle_delta_x100;
        s_direction_check_active = 0U;
        return;
    }

    settled = ((error_abs <= POSITION_SETTLE_BAND_X100) &&
        (velocity_abs <= SPEED_SETTLE_BAND_X100)) ? 1U : 0U;

    s_position_target_crossed = 0U;
    if(s_position_error_seen &&
       (((s_position_previous_error_x100 > 0) &&
            (s_position_error_x100 < 0)) ||
        ((s_position_previous_error_x100 < 0) &&
            (s_position_error_x100 > 0))))
    {
        // Do not let either integrator keep pushing after crossing the target.
        s_position_target_crossed = 1U;
        s_position_i_accumulator = 0;
        s_position_i_term_x100 = 0;
        s_speed_i_accumulator = 0;
        s_speed_i_term_x100 = 0;
        s_ball_target_velocity_x100 = 0;
    }
    if(0 != s_position_error_x100)
    {
        s_position_previous_error_x100 = s_position_error_x100;
        s_position_error_seen = 1U;
    }

    s_position_p_term_x100 =
        (s_position_error_x100 * pid_profile->position_kp_x100) / 100;
    if(s_position_error_x100 >= 0)
    {
        position_kd_x100 = pid_profile->position_kd_right_x100;
        brake_accel_x100 = pid_profile->position_brake_right_x100;
    }
    else
    {
        position_kd_x100 = pid_profile->position_kd_left_x100;
        brake_accel_x100 = pid_profile->position_brake_left_x100;
    }
    // Derivative on measurement avoids a target-change derivative kick.
    s_position_d_term_x100 =
        -(s_camera_velocity_x100 * position_kd_x100) / 100;

    brake_speed_x100 = integer_sqrt_u64(
        (uint64)2U * brake_accel_x100 * error_abs);
    if(brake_speed_x100 >
       (uint32)pid_profile->position_speed_limit_x100)
    {
        brake_speed_x100 =
            (uint32)pid_profile->position_speed_limit_x100;
    }

    command_without_new_i = s_position_p_term_x100 +
        s_position_i_term_x100 + s_position_d_term_x100;
    position_integral_allowed =
        (error_abs > POSITION_INTEGRAL_FREEZE_X100) &&
        (int32_abs_to_uint32(command_without_new_i) <
            (uint32)pid_profile->position_speed_limit_x100) &&
        (int32_abs_to_uint32(command_without_new_i) < brake_speed_x100) &&
        (((s_position_error_x100 > 0) && (command_without_new_i >= 0)) ||
         ((s_position_error_x100 < 0) && (command_without_new_i <= 0)));

    if(position_integral_allowed)
    {
        candidate_accumulator = s_position_i_accumulator +
            s_position_error_x100 * pid_profile->position_ki_x100 *
                (int32)delta_ms;
        s_position_i_accumulator = int32_clamp(candidate_accumulator,
            -(pid_profile->position_i_limit_x100 * PID_INTEGRAL_SCALE),
            pid_profile->position_i_limit_x100 * PID_INTEGRAL_SCALE);
        s_position_i_term_x100 =
            s_position_i_accumulator / PID_INTEGRAL_SCALE;
    }

    desired_velocity_x100 = s_position_p_term_x100 +
        s_position_i_term_x100 + s_position_d_term_x100;
    s_position_output_limited = 0U;
    if(settled || s_position_target_crossed ||
       (error_abs <= POSITION_SETTLE_BAND_X100))
    {
        desired_velocity_x100 = 0;
        s_position_i_accumulator = 0;
        s_position_i_term_x100 = 0;
    }
    else if(s_position_error_x100 > 0)
    {
        if(desired_velocity_x100 < 0)
        {
            desired_velocity_x100 = 0;
            s_position_output_limited = 1U;
        }
        if(desired_velocity_x100 > (int32)brake_speed_x100)
        {
            desired_velocity_x100 = (int32)brake_speed_x100;
            s_position_output_limited = 1U;
        }
    }
    else
    {
        if(desired_velocity_x100 > 0)
        {
            desired_velocity_x100 = 0;
            s_position_output_limited = 1U;
        }
        if(desired_velocity_x100 < -(int32)brake_speed_x100)
        {
            desired_velocity_x100 = -(int32)brake_speed_x100;
            s_position_output_limited = 1U;
        }
    }
    desired_velocity_x100 = int32_clamp(desired_velocity_x100,
        -pid_profile->position_speed_limit_x100,
        pid_profile->position_speed_limit_x100);

    // Acceleration is ramped, but reductions in target speed take effect at once.
    // A requested direction reversal first commands zero for one vision frame.
    if((desired_velocity_x100 * s_ball_target_velocity_x100) < 0)
    {
        s_ball_target_velocity_x100 = 0;
    }
    else if(int32_abs_to_uint32(desired_velocity_x100) >
            int32_abs_to_uint32(s_ball_target_velocity_x100))
    {
        max_velocity_increase_x100 =
            (POSITION_SPEED_ACCEL_X100_PER_S * (int32)delta_ms) / 1000;
        if(max_velocity_increase_x100 < 1)
        {
            max_velocity_increase_x100 = 1;
        }
        target_velocity_delta_x100 = desired_velocity_x100 -
            s_ball_target_velocity_x100;
        target_velocity_delta_x100 = int32_clamp(target_velocity_delta_x100,
            -max_velocity_increase_x100, max_velocity_increase_x100);
        s_ball_target_velocity_x100 += target_velocity_delta_x100;
    }
    else
    {
        s_ball_target_velocity_x100 = desired_velocity_x100;
    }

    speed_stiction_update(error_abs, velocity_abs);

    raw_accel_x100 = ((s_camera_velocity_x100 -
        s_speed_last_velocity_x100) * 1000) / (int32)delta_ms;
    raw_accel_x100 = int32_clamp(raw_accel_x100,
        -pid_profile->speed_accel_limit_x100,
        pid_profile->speed_accel_limit_x100);
    s_speed_filtered_accel_x100 +=
        (raw_accel_x100 - s_speed_filtered_accel_x100) / 4;
    s_speed_last_velocity_x100 = s_camera_velocity_x100;

    s_speed_error_x100 = s_ball_target_velocity_x100 -
        s_camera_velocity_x100;
    s_speed_p_term_x100 =
        (s_speed_error_x100 * pid_profile->speed_kp_x100) / 100;
    s_speed_d_term_x100 =
        -(s_speed_filtered_accel_x100 * pid_profile->speed_kd_x100) / 100;

    command_without_new_i = s_speed_p_term_x100 +
        s_speed_i_term_x100 + s_speed_d_term_x100;
    // speed_effort and beam angle have opposite signs: negative effort
    // produces the positive beam angle used for motion toward X-.
    speed_integral_angle_limit_x100 = (command_without_new_i < 0) ?
        pid_profile->speed_positive_angle_limit_x100 :
        pid_profile->speed_negative_angle_limit_x100;
    speed_integral_allowed = (!settled) &&
        (!s_speed_stiction_active) &&
        (int32_abs_to_uint32(command_without_new_i) <
            (uint32)speed_integral_angle_limit_x100);
    if(speed_integral_allowed)
    {
        candidate_accumulator = s_speed_i_accumulator +
            s_speed_error_x100 * pid_profile->speed_ki_x100 *
                (int32)delta_ms;
        s_speed_i_accumulator = int32_clamp(candidate_accumulator,
            -(pid_profile->speed_i_limit_x100 * PID_INTEGRAL_SCALE),
            pid_profile->speed_i_limit_x100 * PID_INTEGRAL_SCALE);
        s_speed_i_term_x100 = s_speed_i_accumulator / PID_INTEGRAL_SCALE;
    }

    if(settled)
    {
        s_speed_i_accumulator = 0;
        s_speed_i_term_x100 = 0;
        speed_effort_x100 = 0;
    }
    else
    {
        speed_effort_x100 = s_speed_p_term_x100 +
            s_speed_i_term_x100 + s_speed_d_term_x100;
    }

    // Positive beam angle accelerates the ball left, hence the minus sign.
    desired_angle_x100 = -speed_effort_x100;
    if(s_speed_stiction_active)
    {
        if((s_speed_stiction_direction > 0) &&
           (desired_angle_x100 > s_speed_stiction_launch_angle_x100))
        {
            desired_angle_x100 = s_speed_stiction_launch_angle_x100;
        }
        else if((s_speed_stiction_direction < 0) &&
                (desired_angle_x100 < s_speed_stiction_launch_angle_x100))
        {
            desired_angle_x100 = s_speed_stiction_launch_angle_x100;
        }
    }
    s_speed_output_saturated = 0U;
    if(desired_angle_x100 > pid_profile->speed_positive_angle_limit_x100)
    {
        desired_angle_x100 = pid_profile->speed_positive_angle_limit_x100;
        s_speed_output_saturated = 1U;
    }
    else if(desired_angle_x100 <
            -pid_profile->speed_negative_angle_limit_x100)
    {
        desired_angle_x100 =
            -pid_profile->speed_negative_angle_limit_x100;
        s_speed_output_saturated = 1U;
    }
    s_vision_angle_command_x100 = desired_angle_x100;

    // When commanded acceleration opposes the measured velocity, use the
    // faster angle slew so braking starts immediately near the target.
    if(((desired_angle_x100 * s_camera_velocity_x100) > 0) ||
       (int32_abs_to_uint32(s_ball_target_velocity_x100) < velocity_abs) ||
       s_position_target_crossed)
    {
        angle_slew_limit_x100 = VISION_BRAKE_ANGLE_SLEW_X100;
    }
    else
    {
        angle_slew_limit_x100 = VISION_ACCEL_ANGLE_SLEW_X100;
    }
    target_angle_delta_x100 = desired_angle_x100 - s_stepper_target_x100;
    target_angle_delta_x100 = int32_clamp(target_angle_delta_x100,
        -angle_slew_limit_x100, angle_slew_limit_x100);
    s_stepper_target_x100 += target_angle_delta_x100;
    s_direction_check_active = 0U;
}

static void cascade_pid_update (void)
{
    if((APP_CONTROL_KEY1_FIXED == s_app_control_mode) ||
       (APP_CONTROL_KEY2_CENTER == s_app_control_mode))
    {
        cascade_pid_update_enhanced();
    }
    else
    {
        cascade_pid_update_generic_8926();
    }
}

static void camera_uart_init (void)
{
    uart_init(CAMERA_UART_INDEX, CAMERA_UART_BAUDRATE,
        CAMERA_UART_TX_PIN, CAMERA_UART_RX_PIN);
    uart_set_callback(CAMERA_UART_INDEX, camera_uart_rx_callback, NULL);
    uart_set_interrupt_config(CAMERA_UART_INDEX,
        UART_INTERRUPT_CONFIG_RX_ENABLE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 B12 的硬件 PWM，用于向 D36A 输出 STEP 脉冲
// 备注信息     TIMA0 固定为 1MHz 计数时钟，运行时只修改周期，不占用 CPU 翻转 GPIO
//-------------------------------------------------------------------------------------------------------------------
static void stepper_pwm_init (void)
{
    pwm_init(STEPPER_STEP_PWM, 1000U, 0U);

    TIMA0->COUNTERREGS.CTRCTL &= ~GPTIMER_CTRCTL_EN_MASK;
    TIMA0->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
    TIMA0->CLKDIV = 7U;                         // 80MHz / 8
    TIMA0->COMMONREGS.CPS = 9U;                 // 再除以 10，得到 1MHz
    TIMA0->COUNTERREGS.LOAD = 999U;
    TIMA0->COUNTERREGS.CC_01[STEPPER_PWM_CHANNEL_INDEX] = 500U;
    TIMA0->COUNTERREGS.CTR = 0U;
    TIMA0->COMMONREGS.ODIS |=
        (GPTIMER_ODIS_C0CCP0_CCP_OUTPUT_LOW << STEPPER_PWM_CHANNEL_INDEX);
    TIMA0->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
}

static void stepper_pwm_set_frequency (uint32 frequency_pps)
{
    uint32 period_counts;
    uint32 load_value;

    if(0U == frequency_pps)
    {
        TIMA0->COMMONREGS.ODIS |=
            (GPTIMER_ODIS_C0CCP0_CCP_OUTPUT_LOW << STEPPER_PWM_CHANNEL_INDEX);
        return;
    }

    if(frequency_pps > STEPPER_MAX_FREQUENCY_PPS)
    {
        frequency_pps = STEPPER_MAX_FREQUENCY_PPS;
    }

    period_counts = (STEPPER_PWM_CLOCK_HZ + frequency_pps / 2U) / frequency_pps;
    if(period_counts < 2U)
    {
        period_counts = 2U;
    }
    load_value = period_counts - 1U;

    TIMA0->COUNTERREGS.LOAD = (uint16)load_value;
    TIMA0->COUNTERREGS.CC_01[STEPPER_PWM_CHANNEL_INDEX] =
        (uint16)(period_counts / 2U);
    TIMA0->COMMONREGS.ODIS &=
        ~(GPTIMER_ODIS_C0CCP0_CCP_OUTPUT_LOW << STEPPER_PWM_CHANNEL_INDEX);
}

static void stepper_stop_pulses (void)
{
    stepper_pwm_set_frequency(0U);
    s_stepper_frequency_pps = 0;
}

static void stepper_apply_signed_frequency (int32 frequency_pps)
{
    int8 requested_direction;
    uint32 absolute_frequency;
    uint8 dir_level;

    if(0 == frequency_pps)
    {
        stepper_stop_pulses();
        return;
    }

    requested_direction = (frequency_pps > 0) ? 1 : -1;
    absolute_frequency = int32_abs_to_uint32(frequency_pps);

    // 改变 DIR 前先保持 STEP 为低至少一个控制周期，满足驱动器方向建立时间。
    if(requested_direction != s_stepper_applied_direction)
    {
        stepper_pwm_set_frequency(0U);
        dir_level = (requested_direction > 0) ? GPIO_HIGH : GPIO_LOW;
        if(s_stepper_direction_inverted)
        {
            dir_level = (GPIO_HIGH == dir_level) ? GPIO_LOW : GPIO_HIGH;
        }
        gpio_set_level(STEPPER_DIR_PIN, dir_level);
        s_stepper_applied_direction = requested_direction;
        return;
    }

    stepper_pwm_set_frequency(absolute_frequency);
}

static void stepper_disable_output (void)
{
    stepper_stop_pulses();
    gpio_low(STEPPER_EN_PIN);
    s_stepper_enabled = 0;
    s_direction_check_active = 0;
    s_direction_check_reference_valid = 0;
}

static void stepper_trip (stepper_fault_enum fault, const char *message)
{
    auto_run_cancel();
    s_app_control_mode = APP_CONTROL_IDLE;
    s_vision_control_active = 0U;
    s_vision_ball_lost = 1U;
    s_manual_pulse_active = 0U;
    cascade_pid_reset();
    stepper_disable_output();
    s_stepper_fault = fault;
    wireless_debug_printf("SAFETY STOP: %s\r\n", message);
}

static void stepper_start_direction_check (void)
{
    s_direction_check_active = 1;
    // The rotor may snap to the nearest electrical detent as EN rises.  That
    // motion is not caused by a STEP pulse and must not become the reference
    // for polarity checking.  Capture the baseline when the first real STEP
    // command is applied instead.
    s_direction_check_reference_valid = 0;
    s_direction_check_counter = 0;
    s_direction_check_reference_error = 0U;
}

static void manual_pulse_start (int8 direction)
{
    s_manual_pulse_active = 1U;
    s_manual_pulse_counter = 0U;
    s_manual_pulse_direction = direction;
    s_manual_pulse_limit = MANUAL_PULSE_MAX_LOOPS;
    s_manual_pulse_start_position_x100 = s_camera_position_x100;
    s_manual_pulse_last_sequence = s_camera_sequence;
    s_manual_pulse_phase = MANUAL_PULSE_PHASE_RAMP;
    s_manual_pulse_ramp_loops = 0U;
    s_manual_pulse_hold_counter = 0U;
    s_stepper_target_x100 = (direction > 0) ?
        MANUAL_PULSE_POSITIVE_X100 : MANUAL_PULSE_NEGATIVE_X100;
    s_direction_check_active = 0U;

    wireless_debug_printf(
        "PULSE START: %c%u.%02udeg; hold=%ums, total=%ums; S=emergency stop\r\n",
        (direction > 0) ? '+' : '-',
        int32_abs_to_uint32(s_stepper_target_x100) / 100U,
        int32_abs_to_uint32(s_stepper_target_x100) % 100U,
        MANUAL_PULSE_HOLD_LOOPS,
        s_manual_pulse_limit);
}

static void manual_pulse_finish (const char *reason)
{
    int8 completed_direction;
    int32 displacement_x100;
    int32 completed_relative_x100;
    uint32 displacement_abs;
    uint32 velocity_abs;
    uint32 relative_abs;
    uint32 elapsed_loops;
    uint32 ramp_loops;
    uint32 hold_loops;
    char displacement_sign;
    char velocity_sign;
    char relative_sign;
    const char *completed_phase;

    completed_direction = s_manual_pulse_direction;
    completed_relative_x100 = s_encoder_total_x100;
    elapsed_loops = s_manual_pulse_counter;
    if(MANUAL_PULSE_PHASE_HOLD == s_manual_pulse_phase)
    {
        completed_phase = "HOLD";
        ramp_loops = s_manual_pulse_ramp_loops;
        hold_loops = s_manual_pulse_hold_counter;
    }
    else
    {
        completed_phase = "RAMP";
        ramp_loops = elapsed_loops;
        hold_loops = 0U;
    }
    displacement_x100 = s_camera_position_x100 -
        s_manual_pulse_start_position_x100;
    displacement_abs = int32_abs_to_uint32(displacement_x100);
    velocity_abs = int32_abs_to_uint32(s_camera_velocity_x100);
    relative_abs = int32_abs_to_uint32(completed_relative_x100);
    displacement_sign = (displacement_x100 < 0) ? '-' : '+';
    velocity_sign = (s_camera_velocity_x100 < 0) ? '-' : '+';
    relative_sign = (completed_relative_x100 < 0) ? '-' : '+';

    s_manual_pulse_active = 0U;
    s_manual_pulse_counter = 0U;
    s_manual_pulse_direction = 0;
    s_manual_pulse_limit = 0U;
    s_manual_pulse_phase = MANUAL_PULSE_PHASE_IDLE;
    s_manual_pulse_ramp_loops = 0U;
    s_manual_pulse_hold_counter = 0U;
    s_stepper_target_x100 = 0;
    // Stop STEP immediately before printing so UART output cannot extend the kick.
    stepper_stop_pulses();
    if(s_stepper_enabled && s_encoder_feedback_valid)
    {
        stepper_start_direction_check();
    }

    wireless_debug_printf(
        "PULSE END: %s, phase=%s, drive=%c, ramp=%ums, hold=%ums, elapsed=%ums, rel=%c%u.%02udeg, dx=%c%u.%02ucm, v=%c%u.%02ucm/s\r\n",
        reason, completed_phase, (completed_direction > 0) ? '+' : '-',
        ramp_loops, hold_loops, elapsed_loops,
        relative_sign, relative_abs / 100U, relative_abs % 100U,
        displacement_sign, displacement_abs / 100U, displacement_abs % 100U,
        velocity_sign, velocity_abs / 100U, velocity_abs % 100U);
}

static void manual_pulse_update (void)
{
    int32 displacement_x100;
    int32 motion_indicator_x100;

    if(!s_manual_pulse_active)
    {
        return;
    }

    if(!s_stepper_enabled)
    {
        s_manual_pulse_active = 0U;
        s_manual_pulse_counter = 0U;
        s_manual_pulse_direction = 0;
        s_manual_pulse_limit = 0U;
        s_manual_pulse_phase = MANUAL_PULSE_PHASE_IDLE;
        s_manual_pulse_ramp_loops = 0U;
        s_manual_pulse_hold_counter = 0U;
        return;
    }

    if((!s_encoder_feedback_valid) || (!g_ms42_pwm_signal_ok) ||
       (s_encoder_feedback_age > STEPPER_FEEDBACK_TIMEOUT_LOOPS))
    {
        // Returning to level is unsafe without feedback, so disable the driver first.
        stepper_disable_output();
        s_stepper_fault = STEPPER_FAULT_ENCODER_LOST;
        manual_pulse_finish("ENCODER_LOST");
        return;
    }

    if(s_camera_link_age > VISION_LINK_TIMEOUT_LOOPS)
    {
        manual_pulse_finish("LINK_LOST");
        return;
    }

    if(s_camera_sequence != s_manual_pulse_last_sequence)
    {
        s_manual_pulse_last_sequence = s_camera_sequence;
        if(!s_camera_measurement_valid)
        {
            manual_pulse_finish("BALL_LOST");
            return;
        }

        displacement_x100 = s_camera_position_x100 -
            s_manual_pulse_start_position_x100;
        if((int32_abs_to_uint32(displacement_x100) >=
                MANUAL_PULSE_STOP_DISTANCE_X100) ||
           (int32_abs_to_uint32(s_camera_velocity_x100) >=
                MANUAL_PULSE_STOP_SPEED_X100))
        {
            motion_indicator_x100 =
                (int32_abs_to_uint32(displacement_x100) >=
                    MANUAL_PULSE_STOP_DISTANCE_X100) ?
                displacement_x100 : s_camera_velocity_x100;
            if(((s_manual_pulse_direction > 0) &&
                    (motion_indicator_x100 > 0)) ||
               ((s_manual_pulse_direction < 0) &&
                    (motion_indicator_x100 < 0)))
            {
                manual_pulse_finish("MOTION");
            }
            else
            {
                manual_pulse_finish("WRONG_DIR");
            }
            return;
        }
    }

    if(((s_manual_pulse_direction > 0) &&
            (s_encoder_total_x100 >= MANUAL_PULSE_POSITIVE_OVERRUN_X100)) ||
       ((s_manual_pulse_direction < 0) &&
            (s_encoder_total_x100 <= MANUAL_PULSE_NEGATIVE_OVERRUN_X100)))
    {
        manual_pulse_finish("ANGLE_OVERRUN");
        return;
    }

    if(MANUAL_PULSE_PHASE_RAMP == s_manual_pulse_phase)
    {
        if(((s_manual_pulse_direction > 0) &&
                (s_encoder_total_x100 >= MANUAL_PULSE_POSITIVE_HOLD_X100)) ||
           ((s_manual_pulse_direction < 0) &&
                (s_encoder_total_x100 <= MANUAL_PULSE_NEGATIVE_HOLD_X100)))
        {
            // Keep the original target applied; only start timing the hold here.
            s_manual_pulse_phase = MANUAL_PULSE_PHASE_HOLD;
            s_manual_pulse_ramp_loops = s_manual_pulse_counter;
            s_manual_pulse_hold_counter = 0U;
        }
    }
    else if(MANUAL_PULSE_PHASE_HOLD == s_manual_pulse_phase)
    {
        s_manual_pulse_hold_counter ++;
        if(s_manual_pulse_hold_counter >= MANUAL_PULSE_HOLD_LOOPS)
        {
            manual_pulse_finish("HOLD_TIMEOUT");
            return;
        }
    }

    s_manual_pulse_counter ++;
    if(s_manual_pulse_counter < s_manual_pulse_limit)
    {
        return;
    }
    manual_pulse_finish("TIMEOUT");
}

static void vision_command_level (uint8 start_direction_check)
{
    s_stepper_target_x100 = 0;
    if(start_direction_check && s_stepper_enabled && s_encoder_feedback_valid)
    {
        stepper_start_direction_check();
    }
}

static void vision_control_stop_to_level (const char *reason)
{
    auto_run_cancel();
    s_app_control_mode = APP_CONTROL_IDLE;
    s_vision_control_active = 0U;
    s_vision_ball_lost = 1U;
    s_vision_sequence_seen = 0U;
    s_vision_lost_frame_count = 0U;
    s_vision_reacquire_count = 0U;
    cascade_pid_reset();
    vision_command_level(1U);
    wireless_debug_printf("VISION SAFE: %s; target=+0.00deg, motor en=%u\r\n",
        reason, s_stepper_enabled);
}

static void auto_run_update (void)
{
    uint16 elapsed_ms;
    uint32 endpoint_error_abs;
    uint32 right_error_abs;
    uint8 result_pass;
    uint8 endpoint_stable;
    uint8 final_stable_required;
    uint32 final_speed_limit_x100;

    if((AUTO_RUN_TO_RIGHT != s_auto_run_phase) &&
       (AUTO_RUN_TO_LEFT != s_auto_run_phase))
    {
        return;
    }

    elapsed_ms = auto_run_elapsed_ms();
    final_speed_limit_x100 = cascade_profile_is_key1_fixed() ?
        S1_AUTO_RUN_FINAL_SPEED_X100 : AUTO_RUN_FINAL_SPEED_X100;
    final_stable_required = cascade_profile_is_key1_fixed() ?
        S1_AUTO_RUN_FINAL_STABLE_FRAMES : AUTO_RUN_FINAL_STABLE_FRAMES;
    if((elapsed_ms > AUTO_RUN_TIME_LIMIT_MS) &&
       (!s_auto_run_timeout_reported))
    {
        s_auto_run_timeout_reported = 1U;
        if(cascade_profile_is_key1_fixed())
        {
            wireless_debug_printf(
                "S1 TIME LIMIT EXCEEDED: %ums; control continues to left range\r\n",
                elapsed_ms);
        }
        else
        {
            wireless_debug_printf(
                "AUTO TIME LIMIT EXCEEDED: %ums; control continues to -5.00cm\r\n",
                elapsed_ms);
        }
    }

    if(AUTO_RUN_TO_RIGHT == s_auto_run_phase)
    {
        endpoint_error_abs = int32_abs_to_uint32(
            AUTO_RUN_RIGHT_TARGET_X100 - s_camera_position_x100);
        endpoint_stable = cascade_profile_is_key1_fixed() ?
            (s_landing_hold_active &&
             (endpoint_error_abs <= AUTO_RUN_ENDPOINT_ERROR_X100)) :
            ((endpoint_error_abs <= AUTO_RUN_ENDPOINT_ERROR_X100) &&
             (int32_abs_to_uint32(s_camera_velocity_x100) <=
                AUTO_RUN_FINAL_SPEED_X100) &&
             (!s_speed_stiction_active));
        if(endpoint_stable)
        {
            s_auto_run_right_ms = elapsed_ms;
            s_auto_run_right_position_x100 = s_camera_position_x100;
            if(cascade_profile_is_key1_fixed())
            {
                wireless_debug_printf(
                    "S1 TURN: x=+%u.%02ucm err=%u.%02ucm t=%ums; target=-5.20cm range=-6.00..-4.50cm\r\n",
                    int32_abs_to_uint32(s_camera_position_x100) / 100U,
                    int32_abs_to_uint32(s_camera_position_x100) % 100U,
                    endpoint_error_abs / 100U, endpoint_error_abs % 100U,
                    elapsed_ms);
            }
            else
            {
                wireless_debug_printf(
                    "AUTO TURN: x=+%u.%02ucm err=%u.%02ucm t=%ums; target=-5.00cm\r\n",
                    int32_abs_to_uint32(s_camera_position_x100) / 100U,
                    int32_abs_to_uint32(s_camera_position_x100) % 100U,
                    endpoint_error_abs / 100U, endpoint_error_abs % 100U,
                    elapsed_ms);
            }
            s_auto_run_phase = AUTO_RUN_TO_LEFT;
            s_ball_target_position_x100 = cascade_profile_is_key1_fixed() ?
                S1_AUTO_RUN_LEFT_TARGET_X100 : AUTO_RUN_LEFT_TARGET_X100;
            s_auto_final_range_active = 0U;
            s_auto_final_range_seen = 0U;
            s_auto_final_far_guard_active = 0U;
            s_auto_final_far_guard_released = 0U;
            s_auto_final_far_guard_start_position_x100 = 0;
            s_auto_final_recovery_active = 0U;
            s_auto_final_recovery_frames = 0U;
            s_auto_final_recovery_cooldown_frames = 0U;
            s_auto_final_recovery_retry = 0U;
            s_auto_final_recovery_start_position_x100 = 0;
            s_auto_final_recovery_angle_x100 = 0;
            cascade_pid_reset();
            s_vision_sequence_seen = 1U;
            s_vision_last_sequence = s_camera_sequence;
        }
        return;
    }

    endpoint_error_abs = int32_abs_to_uint32(
        AUTO_RUN_LEFT_NOMINAL_X100 - s_camera_position_x100);
    if(cascade_profile_is_key1_fixed())
    {
        endpoint_stable = s_auto_final_range_active &&
            (s_camera_position_x100 >= AUTO_FINAL_RANGE_FAR_X100) &&
            (s_camera_position_x100 <= AUTO_FINAL_RANGE_NEAR_X100) &&
            (int32_abs_to_uint32(s_camera_velocity_x100) <=
                final_speed_limit_x100) &&
            (0U == s_auto_final_far_guard_active);
    }
    else
    {
        endpoint_stable =
            (endpoint_error_abs <= AUTO_RUN_ENDPOINT_ERROR_X100) &&
            (int32_abs_to_uint32(s_camera_velocity_x100) <=
                final_speed_limit_x100) &&
            (!s_speed_stiction_active);
    }
    if(endpoint_stable)
    {
        if(s_auto_run_final_stable_frames <
            final_stable_required)
        {
            s_auto_run_final_stable_frames ++;
        }
    }
    else
    {
        s_auto_run_final_stable_frames = 0U;
    }
    if(s_auto_run_final_stable_frames >=
        final_stable_required)
    {
        right_error_abs = int32_abs_to_uint32(
            AUTO_RUN_RIGHT_TARGET_X100 -
                s_auto_run_right_position_x100);
        result_pass = ((elapsed_ms <= AUTO_RUN_TIME_LIMIT_MS) &&
            (right_error_abs <= AUTO_RUN_ENDPOINT_ERROR_X100) &&
            (cascade_profile_is_key1_fixed() ?
                ((s_camera_position_x100 >= AUTO_FINAL_RANGE_FAR_X100) &&
                 (s_camera_position_x100 <= AUTO_FINAL_RANGE_NEAR_X100)) :
                (endpoint_error_abs <= AUTO_RUN_ENDPOINT_ERROR_X100))) ? 1U : 0U;
        s_auto_run_phase = AUTO_RUN_COMPLETE;
        wireless_debug_printf(
            "AUTO %s: right_err=%u.%02ucm left_err=%u.%02ucm turn=%ums total=%ums\r\n",
            result_pass ? "PASS" : "FAIL",
            right_error_abs / 100U, right_error_abs % 100U,
            endpoint_error_abs / 100U, endpoint_error_abs % 100U,
            s_auto_run_right_ms, elapsed_ms);
    }
}

static void vision_control_update (void)
{
    if(!s_vision_control_active)
    {
        return;
    }

    if(!s_stepper_enabled)
    {
        s_app_control_mode = APP_CONTROL_IDLE;
        s_vision_control_active = 0U;
        s_vision_ball_lost = 1U;
        cascade_pid_reset();
        return;
    }

    if(s_camera_link_age > VISION_LINK_TIMEOUT_LOOPS)
    {
        vision_control_stop_to_level("camera link lost");
        return;
    }

    // Apply at most once for each camera packet, not once per 1ms motor loop.
    if(s_vision_sequence_seen &&
       (s_camera_sequence == s_vision_last_sequence))
    {
        return;
    }
    s_vision_last_sequence = s_camera_sequence;
    s_vision_sequence_seen = 1U;

    if((!s_camera_measurement_valid) ||
       (int32_abs_to_uint32(s_camera_position_x100) >
            VISION_POSITION_LIMIT_X100) ||
       (int32_abs_to_uint32(s_camera_velocity_x100) >
            VISION_VELOCITY_LIMIT_X100))
    {
        s_vision_reacquire_count = 0U;
        if(s_vision_lost_frame_count < VISION_BALL_LOST_FRAME_LIMIT)
        {
            s_vision_lost_frame_count ++;
        }
        if((s_vision_lost_frame_count >= VISION_BALL_LOST_FRAME_LIMIT) &&
           (!s_vision_ball_lost))
        {
            s_vision_ball_lost = 1U;
            cascade_pid_reset();
            vision_command_level(1U);
            wireless_debug_printf(
                "VISION BALL LOST: target=+0.00deg, waiting for %u valid frames\r\n",
                VISION_REACQUIRE_VALID_FRAMES);
        }
        return;
    }

    s_vision_lost_frame_count = 0U;
    if(s_vision_ball_lost)
    {
        vision_command_level(0U);
        if(s_vision_reacquire_count < VISION_REACQUIRE_VALID_FRAMES)
        {
            s_vision_reacquire_count ++;
        }
        if(s_vision_reacquire_count < VISION_REACQUIRE_VALID_FRAMES)
        {
            return;
        }

        s_vision_ball_lost = 0U;
        s_vision_reacquire_count = 0U;
        cascade_pid_reset();
        s_direction_check_active = 0U;
        wireless_debug_printf("VISION BALL REACQUIRED: control resumed\r\n");
    }

    cascade_pid_update();
    auto_run_update();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     约 1ms 调用一次的电机角度执行层（位于球位置/速度双环之后）
// 备注信息     将速度环给出的摆角目标转换为带符号 STEP 频率，并执行斜坡和硬件安全限位
//-------------------------------------------------------------------------------------------------------------------
static void stepper_control_update (void)
{
    int32 error_x100;
    int32 desired_frequency;
    int32 next_frequency;
    uint32 absolute_error;
    uint32 frequency_magnitude;

    if(!s_stepper_enabled)
    {
        stepper_stop_pulses();
        return;
    }

    if((!s_encoder_feedback_valid) || (!g_ms42_pwm_signal_ok) ||
       (s_encoder_feedback_age > STEPPER_FEEDBACK_TIMEOUT_LOOPS))
    {
        stepper_trip(STEPPER_FAULT_ENCODER_LOST, "encoder PWM lost");
        return;
    }

    if(s_encoder_absolute_violation_count >=
        STEPPER_ABSOLUTE_LIMIT_CONFIRM_SAMPLES)
    {
        stepper_trip(STEPPER_FAULT_ABSOLUTE_LIMIT, "absolute angle limit");
        return;
    }

    if(int32_abs_to_uint32(s_encoder_total_x100) > STEPPER_TEST_TRIP_LIMIT_X100)
    {
        stepper_trip(STEPPER_FAULT_TEST_RANGE, "initial-test +/-10deg range");
        return;
    }

    error_x100 = s_stepper_target_x100 - s_encoder_total_x100;
    absolute_error = int32_abs_to_uint32(error_x100);
    desired_frequency = 0;

    if(absolute_error > STEPPER_POSITION_DEADBAND_X100)
    {
        frequency_magnitude =
            (absolute_error * STEPPER_KP_PPS_PER_DEG) / 100U;
        if(frequency_magnitude < STEPPER_MIN_FREQUENCY_PPS)
        {
            frequency_magnitude = STEPPER_MIN_FREQUENCY_PPS;
        }
        if(frequency_magnitude > STEPPER_MAX_FREQUENCY_PPS)
        {
            frequency_magnitude = STEPPER_MAX_FREQUENCY_PPS;
        }
        desired_frequency = (error_x100 > 0) ?
            (int32)frequency_magnitude : -(int32)frequency_magnitude;
    }

    next_frequency = s_stepper_frequency_pps;

    // 反向时先减速到零，下一个周期再改变 DIR。
    if(((next_frequency > 0) && (desired_frequency < 0)) ||
       ((next_frequency < 0) && (desired_frequency > 0)))
    {
        desired_frequency = 0;
    }

    if(next_frequency < desired_frequency)
    {
        next_frequency += STEPPER_SLEW_PPS_PER_MS;
        if(next_frequency > desired_frequency)
        {
            next_frequency = desired_frequency;
        }
    }
    else if(next_frequency > desired_frequency)
    {
        next_frequency -= STEPPER_SLEW_PPS_PER_MS;
        if(next_frequency < desired_frequency)
        {
            next_frequency = desired_frequency;
        }
    }

    s_stepper_frequency_pps = next_frequency;
    stepper_apply_signed_frequency(next_frequency);

    if(s_direction_check_active && (0 != next_frequency))
    {
        if(!s_direction_check_reference_valid)
        {
            s_direction_check_reference_error = absolute_error;
            s_direction_check_reference_valid = 1U;
            s_direction_check_counter = 0U;
        }
        else
        {
            s_direction_check_counter ++;
            if(s_direction_check_counter >= STEPPER_DIRECTION_CHECK_LOOPS)
            {
                if(absolute_error >
                   (s_direction_check_reference_error + STEPPER_DIRECTION_ERROR_X100))
                {
                    stepper_trip(STEPPER_FAULT_DIRECTION,
                        "angle moved away from target; send I while disabled");
                    return;
                }

                s_direction_check_counter = 0U;
                s_direction_check_reference_error = absolute_error;
                if(absolute_error <= (STEPPER_POSITION_DEADBAND_X100 + 10U))
                {
                    s_direction_check_active = 0U;
                    s_direction_check_reference_valid = 0U;
                }
            }
        }
    }
    else if(s_direction_check_active && s_direction_check_reference_valid &&
            (absolute_error <= (STEPPER_POSITION_DEADBAND_X100 + 10U)))
    {
        // A short correction can finish before the 50ms direction window.
        // Close that check now so its old baseline cannot affect a later
        // target command.
        s_direction_check_active = 0U;
        s_direction_check_reference_valid = 0U;
        s_direction_check_counter = 0U;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 TIMG8，硬件同时捕获 PWM 高电平宽度和整周期
// 备注信息     TIMG8 使用 40MHz BUSCLK，2 分频后计数频率为 20MHz
//-------------------------------------------------------------------------------------------------------------------
static void ms42_pwm_capture_init (void)
{
    static const DL_Timer_ClockConfig clock_config =
    {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 1U,
    };
    static const DL_Timer_CaptureCombinedConfig capture_config =
    {
        .captureMode = DL_TIMER_CAPTURE_COMBINED_MODE_PULSE_WIDTH_AND_PERIOD,
        .period = MS42_PWM_TIMER_LOAD,
        .startTimer = DL_TIMER_STOP,
        .inputChan = DL_TIMER_INPUT_CHAN_0,
        .inputInvMode = DL_TIMER_CC_INPUT_INV_NOINVERT,
    };

    // PWM 是编码器推挽输出，因此输入端不使用内部上拉或下拉。
    afio_init(MS42_PWM_PIN, GPI, GPIO_AF3, GPI_FLOATING_IN);

    timer_clock_enable(TIM_G8);
    DL_TimerG_setClockConfig(TIMG8, &clock_config);
    DL_TimerG_initCaptureCombinedMode(TIMG8, &capture_config);
    DL_TimerG_setCoreHaltBehavior(TIMG8, DL_TIMER_CORE_HALT_IMMEDIATE);

    // CC0 捕获下降沿，CC1 捕获上升沿；ZERO 用来判断 PWM 信号丢失。
    DL_TimerG_clearInterruptStatus(TIMG8, TIMG8->CPU_INT.RIS);
    DL_TimerG_enableInterrupt(TIMG8,
        DL_TIMERG_INTERRUPT_CC0_DN_EVENT |
        DL_TIMERG_INTERRUPT_CC1_DN_EVENT |
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    interrupt_set_priority(TIMG8_INT_IRQn, 1);
    interrupt_enable(TIMG8_INT_IRQn);
    DL_TimerG_startCounter(TIMG8);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIMG8 中断的应用层处理函数，由 isr.c 调用
// 备注信息     使用连续时间戳作差，不在上升沿中断中重装计数器
//-------------------------------------------------------------------------------------------------------------------
void ms42_pwm_capture_irq_handler (void)
{
    uint32 current_rise;

    switch(DL_TimerG_getPendingInterrupt(TIMG8))
    {
        case DL_TIMERG_IIDX_CC0_DN:
        {
            s_ms42_pwm_last_fall =
                DL_TimerG_getCaptureCompareValue(TIMG8, DL_TIMER_CC_0_INDEX);
            s_ms42_pwm_fall_valid = 1;
            s_ms42_pwm_edge_seen = 1;
        }break;

        case DL_TIMERG_IIDX_CC1_DN:
        {
            current_rise =
                DL_TimerG_getCaptureCompareValue(TIMG8, DL_TIMER_CC_1_INDEX);
            s_ms42_pwm_edge_seen = 1;

            if(g_ms42_pwm_synced && s_ms42_pwm_fall_valid)
            {
                // TIMG8 向下计数，减法后取低 16 位即可自动处理计数器回绕。
                g_ms42_pwm_period_ticks =
                    (s_ms42_pwm_previous_rise - current_rise) & MS42_PWM_TIMER_LOAD;
                g_ms42_pwm_high_ticks =
                    (s_ms42_pwm_previous_rise - s_ms42_pwm_last_fall) & MS42_PWM_TIMER_LOAD;
                g_ms42_pwm_sample_ready = 1;
                g_ms42_pwm_signal_ok = 1;
            }

            s_ms42_pwm_previous_rise = current_rise;
            s_ms42_pwm_fall_valid = 0;
            g_ms42_pwm_synced = 1;
        }break;

        case DL_TIMERG_IIDX_ZERO:
        {
            // 一个完整的 16 位计数周期内都没有边沿，才判定 PWM 已断开。
            if(!s_ms42_pwm_edge_seen)
            {
                g_ms42_pwm_synced = 0;
                g_ms42_pwm_signal_ok = 0;
                s_ms42_pwm_fall_valid = 0;
            }
            s_ms42_pwm_edge_seen = 0;
        }break;

        default: break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将捕获值换算为 0~35999，即角度的 0.01 度单位
// 返回参数     1=数据有效，0=帧格式不合理
//-------------------------------------------------------------------------------------------------------------------
static uint8 ms42_pwm_get_angle_x100 (
    uint32 high_ticks, uint32 period_ticks, uint32 *angle_x100)
{
    uint32 high_clocks;
    uint32 angle_count;
    const uint32 minimum_high_clocks = MS42_PWM_HEADER_HIGH_CLOCKS;
    const uint32 maximum_high_clocks =
        MS42_PWM_HEADER_HIGH_CLOCKS + MS42_PWM_ANGLE_COUNTS - 1U;

    if((0U == period_ticks) || (high_ticks >= period_ticks) ||
       (!(((period_ticks >= MS42_PWM_FAST_PERIOD_MIN_TICKS) &&
             (period_ticks <= MS42_PWM_FAST_PERIOD_MAX_TICKS)) ||
           ((period_ticks >= MS42_PWM_SLOW_PERIOD_MIN_TICKS) &&
             (period_ticks <= MS42_PWM_SLOW_PERIOD_MAX_TICKS)))))
    {
        return 0;
    }

    // 用实测周期归一化，因此兼容 MT6816 的 971.1Hz 和 485.6Hz 两种 PWM 频率。
    high_clocks = (uint32)(((uint64)high_ticks * MS42_PWM_FRAME_CLOCKS +
        period_ticks / 2U) / period_ticks);
    // 输入同步和整数计数可能在边界产生约 1 个时钟的抖动。
    // 仅对固定帧头/帧尾附近给出小容差，明显错误的脉宽仍然会被拒绝。
    if(((high_clocks + MS42_PWM_EDGE_TOLERANCE_CLOCKS) < minimum_high_clocks) ||
        (high_clocks > (maximum_high_clocks + MS42_PWM_EDGE_TOLERANCE_CLOCKS)))
    {
        return 0;
    }

    if(high_clocks < minimum_high_clocks)
    {
        high_clocks = minimum_high_clocks;
    }
    else if(high_clocks > maximum_high_clocks)
    {
        high_clocks = maximum_high_clocks;
    }

    angle_count = high_clocks - MS42_PWM_HEADER_HIGH_CLOCKS;
    *angle_x100 = (uint32)(((uint64)angle_count * 36000U +
        MS42_PWM_ANGLE_COUNTS / 2U) / MS42_PWM_ANGLE_COUNTS);
    return 1;
}

static void ms42_resync_candidate_reset (void)
{
    s_encoder_resync_candidate_valid = 0U;
    s_encoder_resync_candidate_count = 0U;
}

static int32 ms42_wrapped_delta_x100 (uint32 current_x100,
    uint32 reference_x100)
{
    int32 delta_x100 = (int32)current_x100 - (int32)reference_x100;

    if(delta_x100 > 18000)
    {
        delta_x100 -= 36000;
    }
    else if(delta_x100 < -18000)
    {
        delta_x100 += 36000;
    }
    return delta_x100;
}

static uint8 ms42_process_sample (uint32 high_ticks, uint32 period_ticks)
{
    uint32 angle_x100;
    int32 raw_delta_x100;
    int32 delta_x100;
    int32 candidate_delta_x100;
    uint32 resync_jump_abs;
    char resync_jump_sign;

    if(!ms42_pwm_get_angle_x100(high_ticks, period_ticks, &angle_x100))
    {
        s_encoder_rejected_sample_count ++;
        ms42_resync_candidate_reset();
        return 0;
    }

    if(!s_encoder_multiturn_initialized)
    {
        s_encoder_last_x100 = angle_x100;
        s_encoder_multiturn_initialized = 1;
        ms42_resync_candidate_reset();
    }
    else
    {
        raw_delta_x100 = (int32)angle_x100 - (int32)s_encoder_last_x100;
        delta_x100 = ms42_wrapped_delta_x100(angle_x100,
            s_encoder_last_x100);

        if(int32_abs_to_uint32(delta_x100) >
            MS42_MAX_SAMPLE_DELTA_X100)
        {
            s_encoder_rejected_sample_count ++;

            // An isolated capture glitch must not move the encoder baseline.
            // If the absolute PWM instead remains at one new, mechanically
            // plausible angle for three frames, rebase once so rejection
            // cannot become permanent after a real shaft/encoder step.
            if((angle_x100 <= STEPPER_ABSOLUTE_MIN_X100) ||
               (angle_x100 >= STEPPER_ABSOLUTE_MAX_X100))
            {
                ms42_resync_candidate_reset();
                return 0;
            }

            if(!s_encoder_resync_candidate_valid)
            {
                s_encoder_resync_candidate_x100 = angle_x100;
                s_encoder_resync_candidate_count = 1U;
                s_encoder_resync_candidate_valid = 1U;
                return 0;
            }

            candidate_delta_x100 = ms42_wrapped_delta_x100(angle_x100,
                s_encoder_resync_candidate_x100);
            if(int32_abs_to_uint32(candidate_delta_x100) <=
                MS42_MAX_SAMPLE_DELTA_X100)
            {
                s_encoder_resync_candidate_x100 = angle_x100;
                if(s_encoder_resync_candidate_count <
                    MS42_RESYNC_REQUIRED_SAMPLES)
                {
                    s_encoder_resync_candidate_count ++;
                }
            }
            else
            {
                s_encoder_resync_candidate_x100 = angle_x100;
                s_encoder_resync_candidate_count = 1U;
            }

            if(s_encoder_resync_candidate_count <
                MS42_RESYNC_REQUIRED_SAMPLES)
            {
                return 0;
            }

            raw_delta_x100 = (int32)angle_x100 -
                (int32)s_encoder_last_x100;
            delta_x100 = ms42_wrapped_delta_x100(angle_x100,
                s_encoder_last_x100);
            if(int32_abs_to_uint32(delta_x100) >
                MS42_RESYNC_MAX_JUMP_X100)
            {
                ms42_resync_candidate_reset();
                return 0;
            }

            if(raw_delta_x100 > 18000)
            {
                s_encoder_wrap_count --;
            }
            else if(raw_delta_x100 < -18000)
            {
                s_encoder_wrap_count ++;
            }
            s_encoder_total_x100 += delta_x100;
            s_encoder_last_x100 = angle_x100;
            s_encoder_high_ticks = high_ticks;
            s_encoder_period_ticks = period_ticks;
            s_encoder_single_x100 = angle_x100;
            s_encoder_absolute_violation_count = 0U;
            s_encoder_feedback_valid = 1U;
            s_encoder_feedback_age = 0U;
            s_encoder_resync_event_count ++;
            ms42_resync_candidate_reset();

            // Freeze at the newly confirmed physical angle first.  The next
            // vision frame then resumes through the normal angle-slew limiter.
            stepper_stop_pulses();
            s_stepper_target_x100 = s_encoder_total_x100;
            s_manual_pulse_active = 0U;
            cascade_pid_reset();
            if(s_vision_control_active)
            {
                s_vision_sequence_seen = 1U;
                s_vision_last_sequence = s_camera_sequence;
            }

            resync_jump_abs = int32_abs_to_uint32(delta_x100);
            resync_jump_sign = (delta_x100 < 0) ? '-' : '+';
            wireless_debug_printf(
                "ENCODER RESYNC: abs=%u.%02u jump=%c%u.%02udeg events=%u; target frozen\r\n",
                angle_x100 / 100U, angle_x100 % 100U,
                resync_jump_sign, resync_jump_abs / 100U,
                resync_jump_abs % 100U, s_encoder_resync_event_count);
            return 1;
        }

        ms42_resync_candidate_reset();

        if(raw_delta_x100 > 18000)
        {
            s_encoder_wrap_count --;
        }
        else if(raw_delta_x100 < -18000)
        {
            s_encoder_wrap_count ++;
        }

        s_encoder_total_x100 += delta_x100;
        s_encoder_last_x100 = angle_x100;
    }

    s_encoder_high_ticks = high_ticks;
    s_encoder_period_ticks = period_ticks;
    s_encoder_single_x100 = angle_x100;

    if((angle_x100 <= STEPPER_ABSOLUTE_MIN_X100) ||
       (angle_x100 >= STEPPER_ABSOLUTE_MAX_X100))
    {
        if(s_encoder_absolute_violation_count <
            STEPPER_ABSOLUTE_LIMIT_CONFIRM_SAMPLES)
        {
            s_encoder_absolute_violation_count ++;
        }
    }
    else
    {
        s_encoder_absolute_violation_count = 0U;
    }

    s_encoder_feedback_valid = 1;
    s_encoder_feedback_age = 0;
    return 1;
}

static uint8 parse_signed_x100 (const char *text, int32 *value_x100)
{
    uint32 index = 0;
    uint32 whole = 0;
    uint32 fraction = 0;
    uint32 fraction_digits = 0;
    uint8 digit_seen = 0;
    int32 sign = 1;

    while((' ' == text[index]) || ('\t' == text[index]))
    {
        index ++;
    }

    if(('-' == text[index]) || ('+' == text[index]))
    {
        if('-' == text[index])
        {
            sign = -1;
        }
        index ++;
    }

    while((text[index] >= '0') && (text[index] <= '9'))
    {
        digit_seen = 1;
        whole = whole * 10U + (uint32)(text[index] - '0');
        if(whole > 100U)
        {
            return 0;
        }
        index ++;
    }

    if('.' == text[index])
    {
        index ++;
        while((text[index] >= '0') && (text[index] <= '9'))
        {
            digit_seen = 1;
            if(fraction_digits >= 2U)
            {
                return 0;
            }
            fraction = fraction * 10U + (uint32)(text[index] - '0');
            fraction_digits ++;
            index ++;
        }
    }

    while((' ' == text[index]) || ('\t' == text[index]))
    {
        index ++;
    }

    if((!digit_seen) || ('\0' != text[index]))
    {
        return 0;
    }

    if(1U == fraction_digits)
    {
        fraction *= 10U;
    }
    *value_x100 = sign * (int32)(whole * 100U + fraction);
    return 1;
}

static void wireless_print_status (void)
{
    int32 error_x100;
    uint32 relative_abs;
    uint32 target_abs;
    uint32 error_abs;
    uint32 frequency_abs;
    char relative_sign;
    char target_sign;
    char error_sign;
    char frequency_sign;

    if(!s_encoder_feedback_valid)
    {
        wireless_debug_printf("waiting for MS42CG PWM on B10...\r\n");
        return;
    }

    error_x100 = s_stepper_target_x100 - s_encoder_total_x100;
    relative_abs = int32_abs_to_uint32(s_encoder_total_x100);
    target_abs = int32_abs_to_uint32(s_stepper_target_x100);
    error_abs = int32_abs_to_uint32(error_x100);
    frequency_abs = int32_abs_to_uint32(s_stepper_frequency_pps);
    relative_sign = (s_encoder_total_x100 < 0) ? '-' : '+';
    target_sign = (s_stepper_target_x100 < 0) ? '-' : '+';
    error_sign = (error_x100 < 0) ? '-' : '+';
    frequency_sign = (s_stepper_frequency_pps < 0) ? '-' : '+';

    wireless_debug_printf(
        "abs=%3u.%02u, rel=%c%u.%02u, target=%c%u.%02u, err=%c%u.%02u, step=%c%upps, en=%u, fault=%u, age=%u, reject=%u, absbad=%u, high=%u, period=%u, rsync=%u, rcand=%u, txdrop=%u\r\n",
        s_encoder_single_x100 / 100U, s_encoder_single_x100 % 100U,
        relative_sign, relative_abs / 100U, relative_abs % 100U,
        target_sign, target_abs / 100U, target_abs % 100U,
        error_sign, error_abs / 100U, error_abs % 100U,
        frequency_sign, frequency_abs, s_stepper_enabled, s_stepper_fault,
        s_encoder_feedback_age, s_encoder_rejected_sample_count,
        s_encoder_absolute_violation_count,
        s_encoder_high_ticks, s_encoder_period_ticks,
        s_encoder_resync_event_count, s_encoder_resync_candidate_count,
        s_wireless_tx_dropped_message_count);
}

static void camera_print_status (void)
{
    uint32 position_abs;
    uint32 velocity_abs;
    char position_sign;
    char velocity_sign;

    if(s_camera_link_age > CAMERA_LINK_TIMEOUT_LOOPS)
    {
        wireless_debug_printf(
            "cam=LINK_LOST, frames=%u, bad=%u, drop=%u, ovf=%u\r\n",
            s_camera_valid_frame_count, s_camera_bad_frame_count,
            s_camera_dropped_frame_count, s_camera_rx_overflow_count);
        return;
    }

    if(!s_camera_measurement_valid)
    {
        wireless_debug_printf(
            "cam=BALL_LOST, seq=%u, age=%ums, frames=%u, bad=%u, drop=%u, ovf=%u\r\n",
            s_camera_sequence, s_camera_link_age,
            s_camera_valid_frame_count, s_camera_bad_frame_count,
            s_camera_dropped_frame_count, s_camera_rx_overflow_count);
        return;
    }

    position_abs = int32_abs_to_uint32(s_camera_position_x100);
    velocity_abs = int32_abs_to_uint32(s_camera_velocity_x100);
    position_sign = (s_camera_position_x100 < 0) ? '-' : '+';
    velocity_sign = (s_camera_velocity_x100 < 0) ? '-' : '+';
    wireless_debug_printf(
        "cam=VALID, seq=%u, pos=%c%u.%02ucm, vel=%c%u.%02ucm/s, age=%ums, t=%u, frames=%u, bad=%u, drop=%u, ovf=%u\r\n",
        s_camera_sequence,
        position_sign, position_abs / 100U, position_abs % 100U,
        velocity_sign, velocity_abs / 100U, velocity_abs % 100U,
        s_camera_link_age, s_camera_capture_ms_low16,
        s_camera_valid_frame_count, s_camera_bad_frame_count,
        s_camera_dropped_frame_count, s_camera_rx_overflow_count);
}

static void cascade_idle_print_status (void)
{
    int32 error_x100;
    uint32 target_abs;
    uint32 error_abs;
    char target_sign;
    char error_sign;

    target_abs = int32_abs_to_uint32(s_ball_target_position_x100);
    target_sign = (s_ball_target_position_x100 < 0) ? '-' : '+';
    if((s_camera_link_age > CAMERA_LINK_TIMEOUT_LOOPS) ||
       (!s_camera_measurement_valid))
    {
        wireless_debug_printf(
            "cascade=OFF, xref=%c%u.%02ucm, reason=%s\r\n",
            target_sign, target_abs / 100U, target_abs % 100U,
            (s_camera_link_age > CAMERA_LINK_TIMEOUT_LOOPS) ?
                "LINK_LOST" : "BALL_LOST");
        return;
    }

    error_x100 = s_ball_target_position_x100 - s_camera_position_x100;
    error_abs = int32_abs_to_uint32(error_x100);
    error_sign = (error_x100 < 0) ? '-' : '+';
    wireless_debug_printf(
        "cascade=OFF, xref=%c%u.%02ucm, error=%c%u.%02ucm; send V to arm\r\n",
        target_sign, target_abs / 100U, target_abs % 100U,
        error_sign, error_abs / 100U, error_abs % 100U);
}

static void vision_control_print_status (void)
{
    uint32 position_abs;
    uint32 velocity_abs;
    uint32 position_target_abs;
    uint32 position_error_abs;
    uint32 velocity_target_abs;
    uint32 speed_error_abs;
    uint32 angle_command_abs;
    uint32 relative_abs;
    uint32 angle_target_abs;
    uint32 frequency_abs;
    uint32 stiction_launch_abs;
    char position_sign;
    char velocity_sign;
    char position_target_sign;
    char position_error_sign;
    char velocity_target_sign;
    char speed_error_sign;
    char angle_command_sign;
    char relative_sign;
    char angle_target_sign;
    char frequency_sign;
    char stiction_direction;
    char stiction_launch_sign;
    const char *state;

    position_abs = int32_abs_to_uint32(s_camera_position_x100);
    velocity_abs = int32_abs_to_uint32(s_camera_velocity_x100);
    position_target_abs = int32_abs_to_uint32(s_ball_target_position_x100);
    position_error_abs = int32_abs_to_uint32(s_position_error_x100);
    velocity_target_abs = int32_abs_to_uint32(s_ball_target_velocity_x100);
    speed_error_abs = int32_abs_to_uint32(s_speed_error_x100);
    angle_command_abs = int32_abs_to_uint32(s_vision_angle_command_x100);
    relative_abs = int32_abs_to_uint32(s_encoder_total_x100);
    angle_target_abs = int32_abs_to_uint32(s_stepper_target_x100);
    frequency_abs = int32_abs_to_uint32(s_stepper_frequency_pps);
    stiction_launch_abs =
        int32_abs_to_uint32(s_speed_stiction_last_launch_angle_x100);
    position_sign = (s_camera_position_x100 < 0) ? '-' : '+';
    velocity_sign = (s_camera_velocity_x100 < 0) ? '-' : '+';
    position_target_sign = (s_ball_target_position_x100 < 0) ? '-' : '+';
    position_error_sign = (s_position_error_x100 < 0) ? '-' : '+';
    velocity_target_sign = (s_ball_target_velocity_x100 < 0) ? '-' : '+';
    speed_error_sign = (s_speed_error_x100 < 0) ? '-' : '+';
    angle_command_sign = (s_vision_angle_command_x100 < 0) ? '-' : '+';
    relative_sign = (s_encoder_total_x100 < 0) ? '-' : '+';
    angle_target_sign = (s_stepper_target_x100 < 0) ? '-' : '+';
    frequency_sign = (s_stepper_frequency_pps < 0) ? '-' : '+';
    stiction_direction = (s_speed_stiction_last_direction > 0) ? 'R' :
        ((s_speed_stiction_last_direction < 0) ? 'L' : '-');
    stiction_launch_sign =
        (s_speed_stiction_last_launch_angle_x100 < 0) ? '-' : '+';
    state = s_vision_ball_lost ? "REACQ" :
        (s_landing_hold_active ? "HOLD" : "ACTIVE");

    wireless_debug_printf(
        "ctrl=%s x=%c%u.%02u xref=%c%u.%02u ex=%c%u.%02u v=%c%u.%02u vref=%c%u.%02u ev=%c%u.%02u\r\n",
        state,
        position_sign, position_abs / 100U, position_abs % 100U,
        position_target_sign, position_target_abs / 100U, position_target_abs % 100U,
        position_error_sign, position_error_abs / 100U, position_error_abs % 100U,
        velocity_sign, velocity_abs / 100U, velocity_abs % 100U,
        velocity_target_sign, velocity_target_abs / 100U, velocity_target_abs % 100U,
        speed_error_sign, speed_error_abs / 100U, speed_error_abs % 100U);
    wireless_debug_printf(
        "angle_cmd=%c%u.%02u target=%c%u.%02u rel=%c%u.%02u step=%c%u lim=%u/%u cross=%u kick=%u/%c/%u events=%u boost=%c%u.%02u retry=%u block=%u catch=%u trim=%u/%u guard=%u range=%u/%u/%u age=%u\r\n",
        angle_command_sign, angle_command_abs / 100U, angle_command_abs % 100U,
        angle_target_sign, angle_target_abs / 100U, angle_target_abs % 100U,
        relative_sign, relative_abs / 100U, relative_abs % 100U,
        frequency_sign, frequency_abs, s_position_output_limited,
        s_speed_output_saturated, s_position_target_crossed,
        s_speed_stiction_active, stiction_direction,
        s_speed_stiction_active_frames, s_speed_stiction_event_count,
        stiction_launch_sign, stiction_launch_abs / 100U,
        stiction_launch_abs % 100U, s_speed_stiction_retry_level,
        s_speed_stiction_blocked, s_landing_catch_frames_remaining,
        s_landing_trim_active, s_landing_trim_attempts,
        s_landing_ceiling_guard_active, s_auto_final_range_active,
        s_auto_final_far_guard_active, s_auto_final_recovery_active,
        s_camera_link_age);
}

//-------------------------------------------------------------------------------------------------------------------
// Function brief      Store a compact sample for offline motion/braking analysis
// Remarks             Capture is RAM-only so blocking wireless UART output cannot disturb the control timing
//-------------------------------------------------------------------------------------------------------------------
static void diagnostic_log_capture_sample (void)
{
    diagnostic_sample_struct *sample;
    int32 position_error_x100;

    if(!s_diagnostic_log_enabled)
    {
        return;
    }
    if(s_diagnostic_sequence_seen &&
       (s_diagnostic_last_sequence == s_camera_sequence))
    {
        return;
    }

    s_diagnostic_last_sequence = s_camera_sequence;
    s_diagnostic_sequence_seen = 1U;
    if(s_diagnostic_sample_count >= DIAGNOSTIC_LOG_CAPACITY)
    {
        s_diagnostic_log_full = 1U;
        return;
    }

    position_error_x100 = s_ball_target_position_x100 -
        s_camera_position_x100;
    sample = &s_diagnostic_samples[s_diagnostic_sample_count];
    sample->capture_ms = s_camera_capture_ms_low16;
    sample->position_x100 = (int16)s_camera_position_x100;
    sample->velocity_x100 = (int16)s_camera_velocity_x100;
    sample->target_position_x100 = (int16)s_ball_target_position_x100;
    sample->position_error_x100 = (int16)position_error_x100;
    sample->target_velocity_x100 = (int16)s_ball_target_velocity_x100;
    sample->angle_command_x100 = (int16)s_vision_angle_command_x100;
    sample->relative_angle_x100 = (int16)s_encoder_total_x100;
    sample->step_frequency_pps = (int16)s_stepper_frequency_pps;
    sample->sequence = s_camera_sequence;
    sample->measurement_valid = s_camera_measurement_valid;
    sample->stiction_active = s_speed_stiction_active ||
        s_landing_trim_active || s_landing_ceiling_guard_active;
    sample->stiction_blocked = s_speed_stiction_blocked;
    sample->landing_hold = s_landing_hold_active;
    sample->position_limited = s_position_output_limited;
    sample->speed_limited = s_speed_output_saturated;
    sample->fault = (uint8)s_stepper_fault;
    s_diagnostic_sample_count ++;
}

static void diagnostic_log_dump (void)
{
    diagnostic_sample_struct *sample;
    uint16 index;

    wireless_debug_printf(
        "DLOG DUMP count=%u full=%u; x100 units\r\n"
        "D,t,seq,valid,x,v,xref,ex,vref,angle,rel,step,kick,block,hold,plim,slim,fault\r\n",
        s_diagnostic_sample_count, s_diagnostic_log_full);
    wireless_tx_flush_blocking();
    for(index = 0U; index < s_diagnostic_sample_count; index ++)
    {
        sample = &s_diagnostic_samples[index];
        wireless_debug_printf(
            "D,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u\r\n",
            sample->capture_ms, sample->sequence,
            sample->measurement_valid, sample->position_x100,
            sample->velocity_x100, sample->target_position_x100,
            sample->position_error_x100, sample->target_velocity_x100,
            sample->angle_command_x100, sample->relative_angle_x100,
            sample->step_frequency_pps, sample->stiction_active,
            sample->stiction_blocked, sample->landing_hold,
            sample->position_limited, sample->speed_limited, sample->fault);
        wireless_tx_flush_blocking();
    }
    wireless_debug_printf("DLOG DUMP END\r\n");
    wireless_tx_flush_blocking();
    s_diagnostic_sample_count = 0U;
    s_diagnostic_log_full = 0U;
}

static void wireless_print_help (void)
{
    wireless_debug_printf("Commands: Z=zero(level), E=enable, V=vision ON, M=manual/level\r\n");
    wireless_debug_printf("          P+=right pulse, P-=left pulse; motion/angle/max500ms stop\r\n");
    wireless_debug_printf("          X+5/X-5/X0=ball target(cm); T+10/T-10/T0=manual angle\r\n");
    wireless_debug_printf("          A=generic auto; S1=fixed competition; S2=hold O\r\n");
    wireless_debug_printf("          D1=25Hz RAM log ON; after test send S then D0 to dump CSV\r\n");
    wireless_debug_printf("S1 short=fixed task; S2 short=hold O; hold either 1s=STOP\r\n");
}

static void wireless_execute_command (const char *command)
{
    int32 requested_target;
    uint32 current_abs;
    char current_sign;

    switch(command[0])
    {
        case 'z':
        case 'Z':
        {
            if(s_stepper_enabled)
            {
                wireless_debug_printf("Z rejected: send S before resetting zero\r\n");
            }
            else if((!s_encoder_feedback_valid) || (!g_ms42_pwm_signal_ok))
            {
                wireless_debug_printf("Z rejected: encoder PWM is not valid\r\n");
            }
            else if((s_encoder_single_x100 <= STEPPER_ABSOLUTE_MIN_X100) ||
                    (s_encoder_single_x100 >= STEPPER_ABSOLUTE_MAX_X100))
            {
                wireless_debug_printf("Z rejected: absolute angle outside soft limits\r\n");
            }
            else
            {
                auto_run_cancel();
                s_encoder_total_x100 = 0;
                s_encoder_wrap_count = 0;
                s_encoder_last_x100 = s_encoder_single_x100;
                s_encoder_multiturn_initialized = 1;
                s_stepper_target_x100 = 0;
                s_vision_control_active = 0U;
                s_vision_ball_lost = 1U;
                s_manual_pulse_active = 0U;
                cascade_pid_reset();
                s_stepper_zero_set = 1;
                s_stepper_fault = STEPPER_FAULT_NONE;
                s_direction_check_active = 0;
                wireless_debug_printf("ZERO SET: abs=%u.%02u deg, rel=+0.00 deg\r\n",
                    s_encoder_single_x100 / 100U,
                    s_encoder_single_x100 % 100U);
            }
        }break;

        case 'e':
        case 'E':
        {
            if(s_stepper_enabled)
            {
                wireless_debug_printf("Motor already enabled\r\n");
            }
            else if(!s_stepper_zero_set)
            {
                wireless_debug_printf("E rejected: put mechanism level and send Z first\r\n");
            }
            else if((!s_encoder_feedback_valid) || (!g_ms42_pwm_signal_ok) ||
                    (s_encoder_feedback_age > STEPPER_FEEDBACK_TIMEOUT_LOOPS))
            {
                wireless_debug_printf("E rejected: encoder PWM is not valid\r\n");
            }
            else if((s_encoder_single_x100 <= STEPPER_ABSOLUTE_MIN_X100) ||
                    (s_encoder_single_x100 >= STEPPER_ABSOLUTE_MAX_X100) ||
                    (int32_abs_to_uint32(s_encoder_total_x100) >
                        STEPPER_TEST_TRIP_LIMIT_X100))
            {
                wireless_debug_printf("E rejected: current angle outside test range\r\n");
            }
            else
            {
                stepper_stop_pulses();
                s_stepper_fault = STEPPER_FAULT_NONE;
                gpio_high(STEPPER_EN_PIN);
                s_stepper_enabled = 1;
                stepper_start_direction_check();
                wireless_debug_printf("MOTOR ENABLED: target retained, keep hands clear\r\n");
            }
        }break;

        case 's':
        case 'S':
        {
            auto_run_cancel();
            s_app_control_mode = APP_CONTROL_IDLE;
            s_competition_phase = COMPETITION_IDLE;
            s_vision_control_active = 0U;
            s_vision_ball_lost = 1U;
            s_manual_pulse_active = 0U;
            cascade_pid_reset();
            stepper_disable_output();
            if(s_encoder_feedback_valid && s_stepper_zero_set)
            {
                s_stepper_target_x100 = s_encoder_total_x100;
            }
            s_stepper_fault = STEPPER_FAULT_NONE;
            wireless_debug_printf("MOTOR STOPPED / DISABLED\r\n");
        }break;

        case 'i':
        case 'I':
        {
            if(s_stepper_enabled)
            {
                wireless_debug_printf("I rejected: send S before changing DIR polarity\r\n");
            }
            else
            {
                s_stepper_direction_inverted = !s_stepper_direction_inverted;
                if(s_encoder_feedback_valid && s_stepper_zero_set)
                {
                    s_stepper_target_x100 = s_encoder_total_x100;
                }
                wireless_debug_printf("DIR polarity inverted=%u\r\n",
                    s_stepper_direction_inverted);
            }
        }break;

        case 't':
        case 'T':
        {
            if(!s_stepper_zero_set)
            {
                wireless_debug_printf("T rejected: send Z at level position first\r\n");
            }
            else if(s_vision_control_active)
            {
                wireless_debug_printf("T rejected: send M to leave vision control first\r\n");
            }
            else if(s_manual_pulse_active)
            {
                wireless_debug_printf("T rejected: pulse is active; send S if needed\r\n");
            }
            else if(!parse_signed_x100(&command[1], &requested_target))
            {
                wireless_debug_printf("T format error; examples: T+1, T-1, T0\r\n");
            }
            else if(int32_abs_to_uint32(requested_target) >
                    STEPPER_TARGET_LIMIT_X100)
            {
                wireless_debug_printf("T rejected: safe range is +/-20.00 deg\r\n");
            }
            else
            {
                s_stepper_target_x100 = requested_target;
                stepper_start_direction_check();
                current_abs = int32_abs_to_uint32(requested_target);
                current_sign = (requested_target < 0) ? '-' : '+';
                wireless_debug_printf("TARGET SET: %c%u.%02u deg%s\r\n",
                    current_sign, current_abs / 100U, current_abs % 100U,
                    s_stepper_enabled ? "" : " (motor disabled)");
            }
        }break;

        case 'x':
        case 'X':
        {
            if(!parse_signed_x100(&command[1], &requested_target))
            {
                wireless_debug_printf("X format error; examples: X+5, X-5, X0\r\n");
            }
            else if(int32_abs_to_uint32(requested_target) >
                    VISION_TARGET_POSITION_LIMIT_X100)
            {
                wireless_debug_printf("X rejected: calibrated range is +/-10.00cm\r\n");
            }
            else
            {
                auto_run_cancel();
                if((APP_CONTROL_KEY1_FIXED != s_app_control_mode) &&
                   (APP_CONTROL_KEY2_CENTER != s_app_control_mode))
                {
                    s_app_control_mode = APP_CONTROL_GENERIC_8926;
                }
                s_ball_target_position_x100 = requested_target;
                cascade_pid_reset();
                if(s_vision_control_active)
                {
                    // Wait for the next camera frame before applying the new target.
                    s_vision_sequence_seen = 1U;
                    s_vision_last_sequence = s_camera_sequence;
                }
                current_abs = int32_abs_to_uint32(requested_target);
                current_sign = (requested_target < 0) ? '-' : '+';
                wireless_debug_printf(
                    "BALL TARGET SET: %c%u.%02ucm, cascade=%s\r\n",
                    current_sign, current_abs / 100U, current_abs % 100U,
                    s_vision_control_active ? "ACTIVE" : "OFF");
            }
        }break;

        case 'a':
        case 'A':
        {
            uint8 s1_profile_requested =
                (COMPETITION_HANDS_OFF == s_competition_phase) ? 1U : 0U;

            if('\0' != command[1])
            {
                wireless_debug_printf("A format error; use A only\r\n");
            }
            else if((!s_stepper_enabled) || (!s_vision_control_active) ||
                    s_vision_ball_lost)
            {
                wireless_debug_printf(
                    "A rejected: send E and V, then wait for ACTIVE\r\n");
            }
            else if((s_camera_link_age > VISION_LINK_TIMEOUT_LOOPS) ||
                    (!s_camera_measurement_valid))
            {
                wireless_debug_printf("A rejected: camera/ball is not valid\r\n");
            }
            else if(int32_abs_to_uint32(s_camera_position_x100) >
                    AUTO_RUN_START_POSITION_X100)
            {
                wireless_debug_printf(
                    "A rejected: place ball within +/-0.50cm of O\r\n");
            }
            else if(int32_abs_to_uint32(s_camera_velocity_x100) >
                    AUTO_RUN_START_SPEED_X100)
            {
                wireless_debug_printf(
                    "A rejected: wait until ball speed <=0.50cm/s\r\n");
            }
            else
            {
                auto_run_cancel();
                s_app_control_mode = s1_profile_requested ?
                    APP_CONTROL_KEY1_FIXED : APP_CONTROL_GENERIC_8926;
                s_cascade_profile = s1_profile_requested ?
                    CASCADE_PROFILE_KEY1_FIXED :
                    CASCADE_PROFILE_GENERIC_8926;
                s_auto_run_phase = AUTO_RUN_TO_RIGHT;
                s_auto_run_start_ms = s_camera_capture_ms_low16;
                s_ball_target_position_x100 =
                    AUTO_RUN_RIGHT_TARGET_X100;
                cascade_pid_reset();
                s_vision_sequence_seen = 1U;
                s_vision_last_sequence = s_camera_sequence;
                if(cascade_profile_is_key1_fixed())
                {
                    wireless_debug_printf(
                        "S1 START: fixed O->+5.00cm->left range, aim=-5.20cm, limit=5000ms\r\n");
                }
                else
                {
                    wireless_debug_printf(
                        "AUTO START: generic O->+5.00cm->-5.00cm, limit=5000ms\r\n");
                }
            }
        }break;

        case 'p':
        case 'P':
        {
            if(((command[1] != '+') && (command[1] != '-')) ||
               ('\0' != command[2]))
            {
                wireless_debug_printf("P format error; use P+ or P-\r\n");
            }
            else if(s_vision_control_active)
            {
                wireless_debug_printf("P rejected: send M to leave vision control first\r\n");
            }
            else if(s_manual_pulse_active)
            {
                wireless_debug_printf("P rejected: another pulse is active\r\n");
            }
            else if(!s_stepper_enabled)
            {
                wireless_debug_printf("P rejected: send E first\r\n");
            }
            else if((!s_stepper_zero_set) || (!s_encoder_feedback_valid) ||
                    (!g_ms42_pwm_signal_ok))
            {
                wireless_debug_printf("P rejected: zero/encoder is not valid\r\n");
            }
            else if((s_camera_link_age > VISION_LINK_TIMEOUT_LOOPS) ||
                    (!s_camera_measurement_valid) ||
                    (int32_abs_to_uint32(s_camera_velocity_x100) >
                        VISION_VELOCITY_LIMIT_X100))
            {
                wireless_debug_printf("P rejected: camera link/ball is not valid\r\n");
            }
            else if(int32_abs_to_uint32(s_camera_velocity_x100) >
                    MANUAL_PULSE_START_SPEED_X100)
            {
                wireless_debug_printf("P rejected: wait until |velocity| <= 0.30cm/s\r\n");
            }
            else if(((command[1] == '+') &&
                        (s_camera_position_x100 >
                            -MANUAL_PULSE_POSITION_GATE_X100)) ||
                    ((command[1] == '-') &&
                        (s_camera_position_x100 <
                            MANUAL_PULSE_POSITION_GATE_X100)))
            {
                wireless_debug_printf(
                    "P rejected: P+ requires x<=-0.30cm; P- requires x>=+0.30cm\r\n");
            }
            else if((int32_abs_to_uint32(s_encoder_total_x100) >
                        MANUAL_PULSE_START_ANGLE_X100) ||
                    (int32_abs_to_uint32(s_stepper_target_x100) >
                        STEPPER_POSITION_DEADBAND_X100))
            {
                wireless_debug_printf("P rejected: send M and wait for rel/target near 0\r\n");
            }
            else
            {
                manual_pulse_start((command[1] == '+') ? 1 : -1);
            }
        }break;

        case 'v':
        case 'V':
        {
            if(s_vision_control_active)
            {
                wireless_debug_printf("VISION CONTROL already active\r\n");
            }
            else if(s_manual_pulse_active)
            {
                wireless_debug_printf("V rejected: wait for pulse completion\r\n");
            }
            else if(!s_stepper_enabled)
            {
                wireless_debug_printf("V rejected: send E first\r\n");
            }
            else if((!s_stepper_zero_set) || (!s_encoder_feedback_valid) ||
                    (!g_ms42_pwm_signal_ok))
            {
                wireless_debug_printf("V rejected: zero/encoder is not valid\r\n");
            }
            else if(int32_abs_to_uint32(s_encoder_total_x100) >
                    VISION_START_ANGLE_LIMIT_X100)
            {
                wireless_debug_printf("V rejected: return mechanism within +/-3.00deg first\r\n");
            }
            else if((s_camera_link_age > VISION_LINK_TIMEOUT_LOOPS) ||
                    (!s_camera_measurement_valid) ||
                    (int32_abs_to_uint32(s_camera_velocity_x100) >
                        VISION_VELOCITY_LIMIT_X100))
            {
                wireless_debug_printf("V rejected: camera link/ball is not valid\r\n");
            }
            else if((int32_abs_to_uint32(s_camera_position_x100) >
                        VISION_POSITION_LIMIT_X100) ||
                    (int32_abs_to_uint32(s_ball_target_position_x100 -
                        s_camera_position_x100) > VISION_START_ERROR_LIMIT_X100))
            {
                wireless_debug_printf("V rejected: ball/target is outside calibrated travel\r\n");
            }
            else
            {
                auto_run_cancel();
                if((APP_CONTROL_KEY1_FIXED != s_app_control_mode) &&
                   (APP_CONTROL_KEY2_CENTER != s_app_control_mode))
                {
                    s_app_control_mode = APP_CONTROL_GENERIC_8926;
                }
                vision_command_level(0U);
                s_vision_control_active = 1U;
                s_vision_ball_lost = 1U;
                s_vision_sequence_seen = 1U;
                s_vision_last_sequence = s_camera_sequence;
                s_vision_lost_frame_count = 0U;
                s_vision_reacquire_count = 0U;
                cascade_pid_reset();
                s_direction_check_active = 0U;
                wireless_debug_printf(
                    "CASCADE PID ARMED: position->speed, angle<=+/-8deg, X sets target\r\n");
            }
        }break;

        case 'm':
        case 'M':
        {
            auto_run_cancel();
            s_app_control_mode = APP_CONTROL_IDLE;
            s_vision_control_active = 0U;
            s_vision_ball_lost = 1U;
            s_vision_sequence_seen = 0U;
            s_vision_lost_frame_count = 0U;
            s_vision_reacquire_count = 0U;
            s_manual_pulse_active = 0U;
            cascade_pid_reset();
            vision_command_level(1U);
            wireless_debug_printf("MANUAL/LEVEL MODE: target=+0.00deg, motor en=%u\r\n",
                s_stepper_enabled);
        }break;

        case 'd':
        case 'D':
        {
            if((command[1] == '1') && ('\0' == command[2]))
            {
                s_diagnostic_log_enabled = 1U;
                s_diagnostic_sequence_seen = 0U;
                s_diagnostic_sample_count = 0U;
                s_diagnostic_log_full = 0U;
                wireless_debug_printf(
                    "DLOG RECORDING 25Hz to RAM; capacity=896 samples; control unchanged\r\n");
            }
            else if((command[1] == '0') && ('\0' == command[2]))
            {
                s_diagnostic_log_enabled = 0U;
                s_diagnostic_sequence_seen = 0U;
                if(s_stepper_enabled)
                {
                    wireless_debug_printf(
                        "DLOG STOPPED count=%u; send S, then D0 again to dump safely\r\n",
                        s_diagnostic_sample_count);
                }
                else if(s_diagnostic_sample_count > 0U)
                {
                    diagnostic_log_dump();
                }
                else
                {
                    wireless_debug_printf("DLOG OFF: no stored samples\r\n");
                }
            }
            else
            {
                wireless_debug_printf("D format error; use D1 or D0\r\n");
            }
        }break;

        case 'h':
        case 'H':
        {
            wireless_print_help();
        }break;

        default:
        {
            wireless_debug_printf("Unknown command: %s\r\n", command);
        }break;
    }
}

static void wireless_finish_buffered_command (void)
{
    if(0U == s_wireless_command_length)
    {
        return;
    }

    s_wireless_command_buffer[s_wireless_command_length] = '\0';
    wireless_execute_command(s_wireless_command_buffer);
    s_wireless_command_length = 0;
    s_wireless_command_idle = 0;
}

static void wireless_accept_character (uint8 character)
{
    char immediate_command[2];

    if(0x00U == character)
    {
        return;
    }

    if(('\r' == character) || ('\n' == character))
    {
        wireless_finish_buffered_command();
        return;
    }

    // 单字符安全命令立即执行，即使串口助手没有附加回车换行。
    if((0U == s_wireless_command_length) &&
       (('Z' == character) || ('z' == character) ||
        ('E' == character) || ('e' == character) ||
         ('S' == character) || ('s' == character) ||
         ('I' == character) || ('i' == character) ||
         ('V' == character) || ('v' == character) ||
         ('M' == character) || ('m' == character) ||
         ('H' == character) || ('h' == character)))
    {
        immediate_command[0] = (char)character;
        immediate_command[1] = '\0';
        wireless_execute_command(immediate_command);
        return;
    }

    if((character >= 0x20U) && (character <= 0x7EU))
    {
        if(s_wireless_command_length < (WIRELESS_COMMAND_BUFFER_SIZE - 1U))
        {
            s_wireless_command_buffer[s_wireless_command_length] = (char)character;
            s_wireless_command_length ++;
            s_wireless_command_idle = 0;
        }
        else
        {
            s_wireless_command_length = 0;
            s_wireless_command_idle = 0;
            wireless_debug_printf("Command too long\r\n");
        }
    }
}

static void competition_prepare_reset (void)
{
    s_competition_sequence_seen = 1U;
    s_competition_last_sequence = s_camera_sequence;
    s_competition_stable_frames = 0U;
    s_competition_hands_off_start_ms = 0U;
}

static uint8 competition_camera_origin_valid (void)
{
    return ((s_camera_link_age <= VISION_LINK_TIMEOUT_LOOPS) &&
            s_camera_measurement_valid &&
            (int32_abs_to_uint32(s_camera_position_x100) <=
                AUTO_RUN_START_POSITION_X100) &&
            (int32_abs_to_uint32(s_camera_velocity_x100) <=
                AUTO_RUN_START_SPEED_X100)) ? 1U : 0U;
}

static uint8 competition_new_camera_frame (void)
{
    if((!s_competition_sequence_seen) ||
       (s_camera_sequence != s_competition_last_sequence))
    {
        s_competition_sequence_seen = 1U;
        s_competition_last_sequence = s_camera_sequence;
        return 1U;
    }
    return 0U;
}

static void competition_start_from_button (void)
{
    if(COMPETITION_IDLE != s_competition_phase)
    {
        wireless_debug_printf(
            "S1 ignored: competition sequence is already active; hold S1 to stop\r\n");
        return;
    }
    if(s_stepper_enabled)
    {
        wireless_debug_printf(
            "S1 rejected: motor is already enabled; hold S1 to stop first\r\n");
        return;
    }
    if((!s_encoder_feedback_valid) || (!g_ms42_pwm_signal_ok) ||
       (s_encoder_feedback_age > STEPPER_FEEDBACK_TIMEOUT_LOOPS))
    {
        wireless_debug_printf("S1 rejected: encoder PWM is not valid\r\n");
        return;
    }
    if(!competition_camera_origin_valid())
    {
        wireless_debug_printf(
            "S1 rejected: camera invalid or ball not still within O +/-0.50cm\r\n");
        return;
    }

    // The operator has physically levelled the beam.  Reuse the tested
    // command paths so serial and button operation have identical guards.
    s_app_control_mode = APP_CONTROL_KEY1_FIXED;
    wireless_execute_command("Z");
    if(!s_stepper_zero_set)
    {
        s_app_control_mode = APP_CONTROL_IDLE;
        return;
    }
    wireless_execute_command("X0");
    wireless_execute_command("E");
    if(!s_stepper_enabled)
    {
        s_app_control_mode = APP_CONTROL_IDLE;
        return;
    }

    s_competition_phase = COMPETITION_WAIT_LEVEL;
    competition_prepare_reset();
    wireless_debug_printf(
        "S1 PREP: zero captured; waiting for enable snap to return level\r\n");
}

static void competition_button_update (void)
{
    key_state_enum key_state;
    uint8 new_camera_frame;
    uint16 hands_off_elapsed_ms;

    key_state = key_get_state(KEY_1);
    if(KEY_LONG_PRESS == key_state)
    {
        if(!s_competition_key_long_handled)
        {
            s_competition_key_long_handled = 1U;
            wireless_execute_command("S");
            s_competition_phase = COMPETITION_IDLE;
            competition_prepare_reset();
            wireless_debug_printf("S1 EMERGENCY STOP\r\n");
        }
        return;
    }
    if(KEY_RELEASE == key_state)
    {
        s_competition_key_long_handled = 0U;
    }
    else if(KEY_SHORT_PRESS == key_state)
    {
        key_clear_state(KEY_1);
        competition_start_from_button();
        return;
    }

    new_camera_frame = competition_new_camera_frame();
    switch(s_competition_phase)
    {
        case COMPETITION_WAIT_LEVEL:
        {
            if(!s_stepper_enabled)
            {
                s_competition_phase = COMPETITION_IDLE;
                s_app_control_mode = APP_CONTROL_IDLE;
                wireless_debug_printf(
                    "S1 PREP ABORTED: motor disabled or safety fault\r\n");
                break;
            }
            if(!new_camera_frame)
            {
                break;
            }
            if(competition_camera_origin_valid() &&
               (int32_abs_to_uint32(s_encoder_total_x100) <=
                    COMPETITION_LEVEL_BAND_X100) &&
               (0 == s_stepper_frequency_pps))
            {
                if(s_competition_stable_frames <
                    COMPETITION_PREP_STABLE_FRAMES)
                {
                    s_competition_stable_frames ++;
                }
            }
            else
            {
                s_competition_stable_frames = 0U;
            }

            if(s_competition_stable_frames >=
               COMPETITION_PREP_STABLE_FRAMES)
            {
                wireless_execute_command("V");
                if(s_vision_control_active)
                {
                    s_competition_phase = COMPETITION_WAIT_VISION;
                    competition_prepare_reset();
                    wireless_debug_printf(
                        "S1 PREP: level stable; waiting for vision ACTIVE\r\n");
                }
                else
                {
                    s_competition_phase = COMPETITION_IDLE;
                    s_app_control_mode = APP_CONTROL_IDLE;
                    wireless_debug_printf("S1 PREP ABORTED: vision arm rejected\r\n");
                }
            }
        }break;

        case COMPETITION_WAIT_VISION:
        {
            if((!s_stepper_enabled) || (!s_vision_control_active))
            {
                s_competition_phase = COMPETITION_IDLE;
                s_app_control_mode = APP_CONTROL_IDLE;
                wireless_debug_printf("S1 PREP ABORTED: vision/motor stopped\r\n");
                break;
            }
            if(!new_camera_frame)
            {
                break;
            }
            if((!s_vision_ball_lost) && competition_camera_origin_valid())
            {
                if(s_competition_stable_frames <
                    COMPETITION_PREP_STABLE_FRAMES)
                {
                    s_competition_stable_frames ++;
                }
            }
            else
            {
                s_competition_stable_frames = 0U;
            }

            if(s_competition_stable_frames >=
               COMPETITION_PREP_STABLE_FRAMES)
            {
                s_competition_phase = COMPETITION_HANDS_OFF;
                s_competition_hands_off_start_ms =
                    s_camera_capture_ms_low16;
                wireless_debug_printf(
                    "S1 READY: hands off; auto starts in %ums\r\n",
                    COMPETITION_HANDS_OFF_DELAY_MS);
            }
        }break;

        case COMPETITION_HANDS_OFF:
        {
            if((!s_stepper_enabled) || (!s_vision_control_active))
            {
                s_competition_phase = COMPETITION_IDLE;
                s_app_control_mode = APP_CONTROL_IDLE;
                wireless_debug_printf("S1 PREP ABORTED: vision/motor stopped\r\n");
                break;
            }
            if(new_camera_frame &&
               ((!s_vision_ball_lost) && competition_camera_origin_valid()))
            {
                hands_off_elapsed_ms = (uint16)(s_camera_capture_ms_low16 -
                    s_competition_hands_off_start_ms);
                if(hands_off_elapsed_ms >= COMPETITION_HANDS_OFF_DELAY_MS)
                {
                    wireless_execute_command("A");
                    if(AUTO_RUN_TO_RIGHT == s_auto_run_phase)
                    {
                        s_competition_phase = COMPETITION_RUNNING;
                        wireless_debug_printf(
                            "S1 RUNNING: no more key presses required\r\n");
                    }
                    else
                    {
                        s_competition_phase = COMPETITION_IDLE;
                        s_app_control_mode = APP_CONTROL_IDLE;
                        wireless_debug_printf("S1 PREP ABORTED: auto start rejected\r\n");
                    }
                }
            }
            else if(new_camera_frame)
            {
                s_competition_phase = COMPETITION_WAIT_VISION;
                competition_prepare_reset();
                wireless_debug_printf(
                    "S1 WAIT: ball moved before start; waiting at O again\r\n");
            }
        }break;

        case COMPETITION_RUNNING:
        {
            if(AUTO_RUN_COMPLETE == s_auto_run_phase)
            {
                s_competition_phase = COMPETITION_COMPLETE;
                wireless_debug_printf(
                    "S1 COMPLETE: holding -6.00..-4.50cm range; hold S1/S2 to stop\r\n");
            }
            else if((AUTO_RUN_TO_RIGHT != s_auto_run_phase) &&
                    (AUTO_RUN_TO_LEFT != s_auto_run_phase))
            {
                s_competition_phase = COMPETITION_IDLE;
                s_app_control_mode = APP_CONTROL_IDLE;
                wireless_debug_printf("S1 RUN ABORTED\r\n");
            }
        }break;

        case COMPETITION_IDLE:
        case COMPETITION_COMPLETE:
        default:
        {
        }break;
    }
}

static void center_hold_prepare_reset (void)
{
    s_center_hold_sequence_seen = 1U;
    s_center_hold_last_sequence = s_camera_sequence;
    s_center_hold_stable_frames = 0U;
}

static void center_hold_state_reset (void)
{
    s_center_hold_phase = CENTER_HOLD_IDLE;
    center_hold_prepare_reset();
    if(APP_CONTROL_KEY2_CENTER == s_app_control_mode)
    {
        s_app_control_mode = APP_CONTROL_IDLE;
    }
    if(CASCADE_PROFILE_KEY2_CENTER == s_cascade_profile)
    {
        s_cascade_profile = CASCADE_PROFILE_GENERIC_8926;
    }
}

static uint8 center_hold_new_camera_frame (void)
{
    if((!s_center_hold_sequence_seen) ||
       (s_camera_sequence != s_center_hold_last_sequence))
    {
        s_center_hold_sequence_seen = 1U;
        s_center_hold_last_sequence = s_camera_sequence;
        return 1U;
    }
    return 0U;
}

static uint8 center_hold_camera_start_valid (void)
{
    return ((s_camera_link_age <= VISION_LINK_TIMEOUT_LOOPS) &&
            s_camera_measurement_valid &&
            (int32_abs_to_uint32(s_camera_position_x100) <=
                VISION_POSITION_LIMIT_X100) &&
            (int32_abs_to_uint32(s_camera_velocity_x100) <=
                AUTO_RUN_START_SPEED_X100)) ? 1U : 0U;
}

static void center_hold_start_from_button (void)
{
    if(CENTER_HOLD_IDLE != s_center_hold_phase)
    {
        wireless_debug_printf(
            "S2 ignored: center hold is already active; hold S2 to stop\r\n");
        return;
    }
    if((COMPETITION_IDLE != s_competition_phase) ||
       (APP_CONTROL_KEY1_FIXED == s_app_control_mode))
    {
        wireless_debug_printf(
            "S2 ignored: S1 sequence is active; hold S1/S2 to stop\r\n");
        return;
    }
    if(s_stepper_enabled || s_vision_control_active)
    {
        wireless_debug_printf(
            "S2 rejected: motor/vision is already active; send S first\r\n");
        return;
    }
    if((!s_encoder_feedback_valid) || (!g_ms42_pwm_signal_ok) ||
       (s_encoder_feedback_age > STEPPER_FEEDBACK_TIMEOUT_LOOPS))
    {
        wireless_debug_printf("S2 rejected: encoder PWM is not valid\r\n");
        return;
    }
    if(!center_hold_camera_start_valid())
    {
        wireless_debug_printf(
            "S2 rejected: camera invalid or ball is not still on the rail\r\n");
        return;
    }

    s_app_control_mode = APP_CONTROL_KEY2_CENTER;
    wireless_execute_command("Z");
    if(!s_stepper_zero_set)
    {
        center_hold_state_reset();
        return;
    }
    wireless_execute_command("X0");
    wireless_execute_command("E");
    if(!s_stepper_enabled)
    {
        center_hold_state_reset();
        return;
    }

    s_center_hold_phase = CENTER_HOLD_WAIT_LEVEL;
    center_hold_prepare_reset();
    wireless_debug_printf(
        "S2 PREP: zero captured; waiting for enable snap to return level\r\n");
}

static void center_hold_button_update (void)
{
    key_state_enum key_state;
    uint8 new_camera_frame;

    key_state = key_get_state(KEY_2);
    if(KEY_LONG_PRESS == key_state)
    {
        if(!s_center_hold_key_long_handled)
        {
            s_center_hold_key_long_handled = 1U;
            wireless_execute_command("S");
            center_hold_state_reset();
            wireless_debug_printf("S2 EMERGENCY STOP\r\n");
        }
        return;
    }
    if(KEY_RELEASE == key_state)
    {
        s_center_hold_key_long_handled = 0U;
    }
    else if(KEY_SHORT_PRESS == key_state)
    {
        key_clear_state(KEY_2);
        center_hold_start_from_button();
        return;
    }

    new_camera_frame = center_hold_new_camera_frame();
    switch(s_center_hold_phase)
    {
        case CENTER_HOLD_WAIT_LEVEL:
        {
            if((!s_stepper_enabled) ||
               (APP_CONTROL_KEY2_CENTER != s_app_control_mode))
            {
                center_hold_state_reset();
                wireless_debug_printf(
                    "S2 PREP ABORTED: motor disabled or mode changed\r\n");
                break;
            }
            if(!new_camera_frame)
            {
                break;
            }
            if(center_hold_camera_start_valid() &&
               (int32_abs_to_uint32(s_encoder_total_x100) <=
                    COMPETITION_LEVEL_BAND_X100) &&
               (0 == s_stepper_frequency_pps))
            {
                if(s_center_hold_stable_frames <
                   COMPETITION_PREP_STABLE_FRAMES)
                {
                    s_center_hold_stable_frames ++;
                }
            }
            else
            {
                s_center_hold_stable_frames = 0U;
            }

            if(s_center_hold_stable_frames >=
               COMPETITION_PREP_STABLE_FRAMES)
            {
                wireless_execute_command("V");
                if(s_vision_control_active)
                {
                    s_center_hold_phase = CENTER_HOLD_WAIT_VISION;
                    center_hold_prepare_reset();
                    wireless_debug_printf(
                        "S2 PREP: level stable; waiting for vision ACTIVE\r\n");
                }
                else
                {
                    center_hold_state_reset();
                    wireless_debug_printf(
                        "S2 PREP ABORTED: vision arm rejected\r\n");
                }
            }
        }break;

        case CENTER_HOLD_WAIT_VISION:
        {
            if((!s_stepper_enabled) || (!s_vision_control_active) ||
               (APP_CONTROL_KEY2_CENTER != s_app_control_mode))
            {
                center_hold_state_reset();
                wireless_debug_printf(
                    "S2 PREP ABORTED: vision/motor stopped\r\n");
                break;
            }
            if(!new_camera_frame)
            {
                break;
            }
            if((!s_vision_ball_lost) &&
               (s_camera_link_age <= VISION_LINK_TIMEOUT_LOOPS) &&
               s_camera_measurement_valid)
            {
                if(s_center_hold_stable_frames <
                   COMPETITION_PREP_STABLE_FRAMES)
                {
                    s_center_hold_stable_frames ++;
                }
            }
            else
            {
                s_center_hold_stable_frames = 0U;
            }

            if(s_center_hold_stable_frames >=
               COMPETITION_PREP_STABLE_FRAMES)
            {
                auto_run_cancel();
                s_app_control_mode = APP_CONTROL_KEY2_CENTER;
                s_cascade_profile = CASCADE_PROFILE_KEY2_CENTER;
                s_ball_target_position_x100 = 0;
                cascade_pid_reset();
                s_vision_sequence_seen = 1U;
                s_vision_last_sequence = s_camera_sequence;
                s_center_hold_phase = CENTER_HOLD_ACTIVE;
                wireless_debug_printf(
                    "S2 CENTER HOLD: target=0.00cm; hold S1/S2 to stop\r\n");
            }
        }break;

        case CENTER_HOLD_ACTIVE:
        {
            if((!s_stepper_enabled) || (!s_vision_control_active))
            {
                center_hold_state_reset();
                wireless_debug_printf("S2 CENTER HOLD ABORTED\r\n");
            }
            else if((APP_CONTROL_KEY2_CENTER != s_app_control_mode) ||
                    (CASCADE_PROFILE_KEY2_CENTER != s_cascade_profile) ||
                    (0 != s_ball_target_position_x100) ||
                    (AUTO_RUN_IDLE != s_auto_run_phase))
            {
                // S2 owns the zero target until an explicit S/long press.
                auto_run_cancel();
                s_app_control_mode = APP_CONTROL_KEY2_CENTER;
                s_cascade_profile = CASCADE_PROFILE_KEY2_CENTER;
                s_ball_target_position_x100 = 0;
                cascade_pid_reset();
                s_vision_sequence_seen = 1U;
                s_vision_last_sequence = s_camera_sequence;
            }
        }break;

        case CENTER_HOLD_IDLE:
        default:
        {
        }break;
    }
}


int main (void)
{
    uint32 high_ticks;
    uint32 period_ticks;
    uint32 primask;
    uint8 wireless_rx_buffer[WIRELESS_UART_BUFFER_SIZE];
    uint32 wireless_rx_length;
    uint32 wireless_rx_index;
    uint32 status_counter = 0;
    uint32 key_scan_counter = 0;
    uint8 valid_sample_this_loop;

    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>

    // 上电安全状态：先拉低 DIR/EN，再初始化 STEP 硬件 PWM 且强制输出低电平。
    gpio_init(STEPPER_DIR_PIN,  GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(STEPPER_EN_PIN,   GPO, GPIO_LOW, GPO_PUSH_PULL);
    stepper_pwm_init();

    if(wireless_uart_init())
    {
        // 自动波特率关闭时一般不会进入这里；初始化失败则保持电机失能。
        while(true)
        {
            system_delay_ms(100);
        }
    }

    camera_uart_init();
    ms42_pwm_capture_init();
    key_init(COMPETITION_KEY_SCAN_PERIOD_MS);
    interrupt_global_enable(0);

    wireless_debug_printf("\r\nMS42CG encoder + D36A angle-loop test\r\n");
    wireless_debug_printf("Wireless UART: 115200, B6=TX, B7=RX, B2=RTS\r\n");
    wireless_debug_printf("Encoder PWM=B10, STEP=B12, DIR=B13, EN=B8(high=enable)\r\n");
    wireless_debug_printf("Vision UART2: B15=TX, B16=RX, 115200\r\n");
    wireless_debug_printf("Cascade PID: position -> target speed -> beam angle\r\n");
    wireless_debug_printf("Generic 8926ff0: Pos Kp=3.50 Ki=0.08 Kd=1.00, brake=15\r\n");
    wireless_debug_printf("S1 fixed profile: X- Kd=0.55, brake=15, internal target=-5.20cm\r\n");
    wireless_debug_printf("Modes: KEY1=fixed task; KEY2=center hold; serial X/V/A=generic 8926ff0\r\n");
    wireless_debug_printf("S2 profile: Pos Kp=3.50 Ki=0.08 Kd=0.30/1.00, target=0.00cm\r\n");
    wireless_debug_printf("Position speed limit=20cm/s\r\n");
    wireless_debug_printf("Speed Kp=0.80 Ki=0.10 Kd=0.03, angle limit=+/-8deg\r\n");
    wireless_debug_printf("Fast response: vref accel=200cm/s2, angle slew=1.2/2.0deg/frame\r\n");
    wireless_tx_flush_blocking();
    wireless_debug_printf("Landing capture: <=0.95cm; low speed or retreat at <=2.00cm/s\r\n");
    wireless_debug_printf("Landing catch: X+=4.60deg, X-=6.50deg; max500ms to +/-0.40cm\r\n");
    wireless_debug_printf("X+ HOLD trim: adaptive 4.60..5.40deg, 0.05cm release, max16\r\n");
    wireless_debug_printf("X+ trim release seeds 2.50deg I support to prevent rollback\r\n");
    wireless_debug_printf("X+ ceiling guard: target+0.40cm enter, target+0.20cm exit\r\n");
    wireless_debug_printf("X- HOLD trim: adaptive 6.50..8.00deg, 0.05cm release\r\n");
    wireless_debug_printf("X- floor guard: -4.60deg, release after 0.05cm rightward motion\r\n");
    wireless_debug_printf("Generic A: O->+5->-5 point targets; independent of S1 final controller\r\n");
    wireless_debug_printf("Landing hold: soft v<=1.5cm/s angle<=4deg; no kick\r\n");
    wireless_debug_printf("Landing release: drift >1.20cm or next X target\r\n");
    wireless_debug_printf("Auto stiction: adaptive +/-4.6..6.5deg, step=0.25deg\r\n");
    wireless_debug_printf("S1 left range: support=+2.80deg, damping=-2.5..+4.0deg\r\n");
    wireless_debug_printf("S1 left recovery: +6.50..8.00deg, 0.05cm release; far guard=-3.0deg\r\n");
    wireless_tx_flush_blocking();
    wireless_debug_printf("Encoder filter: jump reject + 3-frame <=25deg resync; abs limit 5 samples\r\n");
    wireless_debug_printf("Wireless TX: queued, runtime chunks <=2 bytes while motor enabled\r\n");
    wireless_debug_printf("Pulse test: visual stop, rel stop +4.4/-4.2deg, max500ms\r\n");
    wireless_debug_printf("Boot: motor/vision OFF. Level beam; S1 also requires ball at O.\r\n");
    wireless_debug_printf("S1=fixed task; S2=return/hold O; hold either key 1s=STOP.\r\n");
    wireless_debug_printf("Limits: target +/-20deg, test trip +/-25deg, absolute 121~208deg\r\n");
    wireless_print_help();
    wireless_tx_flush_blocking();

    while(true)
    {
        valid_sample_this_loop = 0;

        if(g_ms42_pwm_sample_ready)
        {
            // 复制一组彼此对应的中断数据，避免复制过程中恰好被下一帧更新。
            primask = interrupt_global_disable();
            high_ticks = g_ms42_pwm_high_ticks;
            period_ticks = g_ms42_pwm_period_ticks;
            g_ms42_pwm_sample_ready = 0;
            interrupt_global_enable(primask);

            if(ms42_process_sample(high_ticks, period_ticks))
            {
                valid_sample_this_loop = 1;
            }
        }

        if((!valid_sample_this_loop) &&
           (s_encoder_feedback_age <= STEPPER_FEEDBACK_TIMEOUT_LOOPS))
        {
            s_encoder_feedback_age ++;
        }

        wireless_rx_length = wireless_uart_read_buffer(
            wireless_rx_buffer, WIRELESS_UART_BUFFER_SIZE);
        for(wireless_rx_index = 0;
            wireless_rx_index < wireless_rx_length;
            wireless_rx_index ++)
        {
            wireless_accept_character(wireless_rx_buffer[wireless_rx_index]);
        }

        if((0U == wireless_rx_length) && (s_wireless_command_length > 0U))
        {
            s_wireless_command_idle ++;
            if(s_wireless_command_idle >= WIRELESS_COMMAND_IDLE_LOOPS)
            {
                wireless_finish_buffered_command();
            }
        }

        camera_uart_process_received_data();
        if(s_camera_link_age <= CAMERA_LINK_TIMEOUT_LOOPS)
        {
            s_camera_link_age ++;
        }
        key_scan_counter ++;
        if(key_scan_counter >= COMPETITION_KEY_SCAN_PERIOD_MS)
        {
            key_scan_counter = 0U;
            key_scanner();
        }
        competition_button_update();
        center_hold_button_update();
        vision_control_update();
        manual_pulse_update();

        stepper_control_update();

        status_counter ++;
        if(s_manual_pulse_active)
        {
            // Avoid blocking wireless status output while timing a short pulse.
            status_counter = 0;
        }
        else if(s_diagnostic_log_enabled)
        {
            if(status_counter >= WIRELESS_DIAGNOSTIC_PERIOD_LOOPS)
            {
                status_counter = 0;
                diagnostic_log_capture_sample();
            }
        }
        else if(s_vision_control_active)
        {
            if(status_counter >= WIRELESS_ACTIVE_STATUS_PERIOD_LOOPS)
            {
                status_counter = 0;
                vision_control_print_status();
            }
        }
        else if(status_counter >= WIRELESS_STATUS_PERIOD_LOOPS)
        {
            status_counter = 0;
            wireless_print_status();
            camera_print_status();
            cascade_idle_print_status();
        }

        wireless_tx_flush_one_chunk();
        system_delay_ms(1);
    }
}
