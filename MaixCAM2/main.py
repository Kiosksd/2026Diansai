"""MaixCAM2 standalone steel-ball position debugger.

This version intentionally has no RTSP/JPEG/WebRTC server.  Ball position and
velocity are sent to the MSPM0G3507 over a dedicated serial link; the 3507
firmware only monitors these packets and does not drive the motor from them.
"""

from maix import app, camera, display, err, image, nn, pinmap, sys, time, uart

from ball_position import (
    AdaptiveAlphaBetaFilter,
    axis_point,
    position_from_pixel,
    validate_calibration,
)


# ------------------------------ 调试配置 ------------------------------

# 每隔一段时间向 MaixVision 终端打印一次状态，不进行逐帧刷屏。
DEBUG_STATUS_ENABLE = True
DEBUG_STATUS_PERIOD_MS = 500

# False 表示关闭模型双缓冲：检测结果对应当前画面，延迟更低，适合闭环控制。
LOW_LATENCY_MODE = True

LENS_CORR_ENABLE = False
LENS_CORR_STRENGTH = 0.6

DETECTION_CONFIDENCE = 0.40
DETECTION_IOU = 0.45

# MaixCAM2 专用 YOLO26 模型。此程序不再自动选择 MaixCAM 模型。
MODEL_PATH = "models/yolo26_all_maixcam2_yolo26_640_160/yolo26_all.mud"

# 相机固定后的三点标定。旧坐标系中，实际 -10/0/+10cm 分别测得
# -8.84/-0.075/+9.18cm；换算到 640x160 原图后得到以下球心像素。
# 使用左右分段比例，避免只做统一缩放后中心产生约 0.27cm 偏差。
AXIS_START_PX = (97, 122)   # 刻度 -10cm
AXIS_ZERO_PX = (318, 123)   # 刻度   0cm
AXIS_END_PX = (551, 124)    # 刻度 +10cm
AXIS_START_CM = -10.0
AXIS_ZERO_CM = 0.0
AXIS_END_CM = 10.0
TARGET_CM = 0.0

# MaixCAM2 right-side B2/B3 pins (UART3) are dedicated to the vision link.
# The existing wireless
# debug UART on the MSPM0G3507 remains on B6/B7 and is not affected.
VISION_UART_DEVICE = "/dev/ttyS3"
VISION_UART_BAUDRATE = 115200
VISION_UART_TX_PERIOD_MS = 20

VISION_FRAME_HEAD_0 = 0xAA
VISION_FRAME_HEAD_1 = 0x55
VISION_FRAME_TYPE_BALL = 0x01
VISION_FRAME_TAIL = 0x0D


def require_maixcam2():
    device_name = sys.device_name().strip().lower()
    if device_name != "maixcam2":
        raise RuntimeError(
            "This debug program requires MaixCAM2, current device: {}".format(
                sys.device_name()
            )
        )


def signed_text(value, digits=2):
    return ("{:+." + str(digits) + "f}").format(value)


def crc8_poly07(data):
    """CRC-8 polynomial 0x07, initial value 0x00."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def signed_x100_bytes(value):
    """Round and saturate a physical value to signed int16, little-endian."""
    raw = int(round(value * 100.0))
    raw = max(-32768, min(32767, raw))
    if raw < 0:
        raw += 0x10000
    return raw & 0xFF, (raw >> 8) & 0xFF


def make_vision_frame(valid, sequence, position_cm, velocity_cm_s, capture_ms):
    if not valid:
        position_cm = 0.0
        velocity_cm_s = 0.0

    pos_low, pos_high = signed_x100_bytes(position_cm)
    vel_low, vel_high = signed_x100_bytes(velocity_cm_s)
    timestamp = int(capture_ms) & 0xFFFF
    payload = bytes(
        (
            VISION_FRAME_TYPE_BALL,
            0x01 if valid else 0x00,
            sequence & 0xFF,
            pos_low,
            pos_high,
            vel_low,
            vel_high,
            timestamp & 0xFF,
            (timestamp >> 8) & 0xFF,
        )
    )
    return (
        bytes((VISION_FRAME_HEAD_0, VISION_FRAME_HEAD_1))
        + payload
        + bytes((crc8_poly07(payload), VISION_FRAME_TAIL))
    )


require_maixcam2()

detector = nn.YOLO26(
    model=MODEL_PATH,
    dual_buff=not LOW_LATENCY_MODE,
)

input_width = detector.input_width()
input_height = detector.input_height()

if AXIS_START_PX is None:
    AXIS_START_PX = (5, input_height // 2)
if AXIS_END_PX is None:
    AXIS_END_PX = (input_width - 5, input_height // 2)

validate_calibration(
    AXIS_START_PX,
    AXIS_END_PX,
    AXIS_START_CM,
    AXIS_END_CM,
    input_width,
    input_height,
)

if not (
    0 <= AXIS_ZERO_PX[0] < input_width
    and 0 <= AXIS_ZERO_PX[1] < input_height
):
    raise ValueError("zero calibration point is outside the image")

# 中心点在线段中的比例决定左右两段的独立换算斜率。
_, axis_zero_ratio, axis_zero_distance_px = position_from_pixel(
    AXIS_ZERO_PX,
    AXIS_START_PX,
    AXIS_END_PX,
    0.0,
    1.0,
)
if not 0.0 < axis_zero_ratio < 1.0:
    raise ValueError("zero calibration point must be between both endpoints")
if axis_zero_distance_px > 2.0:
    raise ValueError("zero calibration point is too far from the calibrated axis")


def calibrated_position_from_ratio(ratio):
    """Convert projected axis ratio with independent left/right calibration."""
    if ratio <= axis_zero_ratio:
        return AXIS_START_CM + (ratio / axis_zero_ratio) * (
            AXIS_ZERO_CM - AXIS_START_CM
        )
    return AXIS_ZERO_CM + (
        (ratio - axis_zero_ratio) / (1.0 - axis_zero_ratio)
    ) * (AXIS_END_CM - AXIS_ZERO_CM)


def calibrated_ratio_from_position(position_cm):
    """Inverse of calibrated_position_from_ratio for drawing the target."""
    if position_cm <= AXIS_ZERO_CM:
        return (
            (position_cm - AXIS_START_CM)
            / (AXIS_ZERO_CM - AXIS_START_CM)
            * axis_zero_ratio
        )
    return axis_zero_ratio + (
        (position_cm - AXIS_ZERO_CM)
        / (AXIS_END_CM - AXIS_ZERO_CM)
        * (1.0 - axis_zero_ratio)
    )


axis_dx = AXIS_END_PX[0] - AXIS_START_PX[0]
axis_dy = AXIS_END_PX[1] - AXIS_START_PX[1]
axis_length_squared = float(axis_dx * axis_dx + axis_dy * axis_dy)


def calibrated_velocity_from_pixel_velocity(ratio, velocity_x, velocity_y):
    """Project filter velocity onto the calibrated axis and return cm/s."""
    ratio_per_second = (
        velocity_x * axis_dx + velocity_y * axis_dy
    ) / axis_length_squared
    if ratio <= axis_zero_ratio:
        cm_per_ratio = (
            (AXIS_ZERO_CM - AXIS_START_CM) / axis_zero_ratio
        )
    else:
        cm_per_ratio = (
            (AXIS_END_CM - AXIS_ZERO_CM) / (1.0 - axis_zero_ratio)
        )
    return ratio_per_second * cm_per_ratio

if not min(AXIS_START_CM, AXIS_END_CM) <= TARGET_CM <= max(
    AXIS_START_CM, AXIS_END_CM
):
    raise ValueError("TARGET_CM must be inside the calibrated range")

cam = camera.Camera(input_width, input_height, detector.input_format())
disp = display.Display()
position_filter = AdaptiveAlphaBetaFilter()

err.check_raise(
    pinmap.set_pin_function("B2", "UART3_TX"),
    "failed to map MaixCAM2 B2 to UART3_TX",
)
err.check_raise(
    pinmap.set_pin_function("B3", "UART3_RX"),
    "failed to map MaixCAM2 B3 to UART3_RX",
)
vision_uart = uart.UART(VISION_UART_DEVICE, VISION_UART_BAUDRATE)

axis_color = image.Color.from_rgb(0, 220, 255)
target_color = image.Color.from_rgb(255, 220, 0)
panel_color = image.Color.from_rgb(0, 0, 0)
filtered_color = image.Color.from_rgb(0, 255, 0)


def draw_calibration_axis(img):
    img.draw_line(
        AXIS_START_PX[0],
        AXIS_START_PX[1],
        AXIS_END_PX[0],
        AXIS_END_PX[1],
        axis_color,
        2,
    )
    img.draw_cross(AXIS_START_PX[0], AXIS_START_PX[1], axis_color, 7, 2)
    img.draw_cross(AXIS_END_PX[0], AXIS_END_PX[1], axis_color, 7, 2)

    target_ratio = calibrated_ratio_from_position(TARGET_CM)
    target_x, target_y = axis_point(
        target_ratio, AXIS_START_PX, AXIS_END_PX
    )
    img.draw_cross(int(target_x), int(target_y), target_color, 10, 2)


print("MaixCAM2 steel-ball standalone debugger")
print("network streaming: OFF")
print("vision link: Maix UART3 B2=TX/B3=RX -> 3507 UART2, 115200 8N1")
print("vision link mode: monitor only; it cannot command the motor")
print("model input: {}x{}".format(input_width, input_height))
print("axis: {} -> {}".format(AXIS_START_PX, AXIS_END_PX))
print(
    "axis zero: {} ratio={:.4f}".format(AXIS_ZERO_PX, axis_zero_ratio)
)
print(
    "physical range: {:.2f}cm -> {:.2f}cm, target={:.2f}cm".format(
        AXIS_START_CM, AXIS_END_CM, TARGET_CM
    )
)

last_loop_ms = time.ticks_ms()
status_start_ms = last_loop_ms
status_frame_count = 0
display_fps = 0.0
vision_sequence = 0
last_vision_tx_ms = last_loop_ms

while not app.need_exit():
    frame_start_ms = time.ticks_ms()
    loop_ms = frame_start_ms - last_loop_ms
    last_loop_ms = frame_start_ms

    img = cam.read()
    if LENS_CORR_ENABLE:
        img = img.lens_corr(strength=LENS_CORR_STRENGTH)

    detect_start_ms = time.ticks_ms()
    objects = detector.detect(
        img,
        conf_th=DETECTION_CONFIDENCE,
        iou_th=DETECTION_IOU,
    )
    detect_ms = time.ticks_ms() - detect_start_ms
    now_ms = time.ticks_ms()

    draw_calibration_axis(img)

    # 模型只有 ball 类别；多框时只采用置信度最高的一个。
    ball = max(objects, key=lambda obj: obj.score) if objects else None
    debug_state = "ball=0"
    measurement_valid = False
    position_cm = 0.0
    velocity_cm_s = 0.0

    img.draw_rect(0, 0, input_width, 47, panel_color, -1)

    if ball is not None:
        raw_x = ball.x + ball.w * 0.5
        raw_y = ball.y + ball.h * 0.5
        filtered_x, filtered_y = position_filter.update(raw_x, raw_y, now_ms)

        _, axis_ratio, axis_distance_px = position_from_pixel(
            (filtered_x, filtered_y),
            AXIS_START_PX,
            AXIS_END_PX,
            0.0,
            1.0,
        )
        position_cm = calibrated_position_from_ratio(axis_ratio)
        velocity_cm_s = calibrated_velocity_from_pixel_velocity(
            axis_ratio, position_filter.vx, position_filter.vy
        )
        position_error_cm = TARGET_CM - position_cm
        measurement_valid = True
        projected_x, projected_y = axis_point(
            axis_ratio, AXIS_START_PX, AXIS_END_PX
        )

        img.draw_rect(
            ball.x,
            ball.y,
            ball.w,
            ball.h,
            color=image.COLOR_GREEN,
            thickness=2,
        )
        img.draw_cross(
            int(filtered_x), int(filtered_y), filtered_color, 10, 2
        )
        img.draw_cross(
            int(projected_x), int(projected_y), target_color, 6, 2
        )

        img.draw_string(
            8,
            4,
            "BALL {} cm  ERR {} cm".format(
                signed_text(position_cm), signed_text(position_error_cm)
            ),
            color=image.COLOR_WHITE,
            scale=1.2,
            thickness=2,
        )
        img.draw_string(
            8,
            25,
            "conf={:.2f}  axis_dist={:.1f}px".format(
                ball.score, axis_distance_px
            ),
            color=image.COLOR_GREEN,
            scale=1.0,
            thickness=1,
        )

        debug_state = (
            "ball=1 pos={:+.2f}cm err={:+.2f}cm "
            "raw=({:.1f},{:.1f}) filt=({:.1f},{:.1f}) "
            "vx={:+.1f}px/s conf={:.2f} axis_dist={:.1f}px"
        ).format(
            position_cm,
            position_error_cm,
            raw_x,
            raw_y,
            filtered_x,
            filtered_y,
            position_filter.vx,
            ball.score,
            axis_distance_px,
        )
    else:
        position_filter.mark_missing(now_ms)
        img.draw_string(
            8,
            4,
            "BALL LOST",
            color=image.COLOR_RED,
            scale=1.4,
            thickness=2,
        )

    # 20ms nominal period.  Advancing by the nominal period instead of by
    # `now_ms` gives an average 50Hz stream even with a roughly 60FPS loop.
    if now_ms - last_vision_tx_ms >= VISION_UART_TX_PERIOD_MS:
        if now_ms - last_vision_tx_ms > 200:
            last_vision_tx_ms = now_ms
        else:
            last_vision_tx_ms += VISION_UART_TX_PERIOD_MS
        frame = make_vision_frame(
            measurement_valid,
            vision_sequence,
            position_cm,
            velocity_cm_s,
            now_ms,
        )
        vision_uart.write(frame)
        vision_sequence = (vision_sequence + 1) & 0xFF

    fps_text = "FPS:{:.1f}".format(display_fps)
    fps_size = image.string_size(fps_text, scale=1.2, thickness=2)
    img.draw_string(
        img.width() - fps_size.width() - 8,
        4,
        fps_text,
        color=image.COLOR_GREEN,
        scale=1.2,
        thickness=2,
    )

    # 不启动网络图传。disp.show 会显示到本机屏幕，并在连接 MaixVision 时供调试查看。
    disp.show(img)

    status_frame_count += 1
    status_elapsed_ms = now_ms - status_start_ms
    if status_elapsed_ms >= DEBUG_STATUS_PERIOD_MS:
        display_fps = status_frame_count * 1000.0 / max(1, status_elapsed_ms)
        if DEBUG_STATUS_ENABLE:
            print(
                "fps={:.1f} loop={}ms detect={}ms {}".format(
                    display_fps, loop_ms, detect_ms, debug_state
                )
            )
        status_start_ms = now_ms
        status_frame_count = 0
