"""
Stable black/grey target-frame tracker for MaixCAM2 (MaixPy v4).

The target is located from its rectangular edges, so the printed black border
may look grey in the camera image.  This version intentionally does NOT try to
pair inner and outer edges: MaixPy often reports only one of them per frame.
Their centres are effectively the same, so temporal tracking gives a steadier
and more useful result than an unreliable edge pair.
"""

from maix import app, camera, display, image


# Keep the image-processing path at 320 x 240: 640 x 480 find_rects can
# exhaust MaixCAM2's fast frame buffer.  Long-distance support comes from
# accepting smaller valid rectangles, not from risking that allocation.
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FPS = 30
FRAME_EDGE_THRESHOLD = 1800
FRAME_HOLD_FRAMES = 15

# The old values effectively required at least 64 x 48 pixels / 5485 px^2,
# which rejects this target before tracking begins.  These limits retain a
# small distant target while aspect ratio and temporal tracking reject noise.
MIN_FRAME_WIDTH = 24
MIN_FRAME_HEIGHT = 16
MIN_FRAME_AREA = 500

# The target's distinctive feature is several thin, faded red rings.  A red
# traffic cone is a large filled region, so both quadrant coverage and density
# are used to distinguish the target from the background.
RED_RING_LAB = [5, 95, 8, 100, -10, 90]
MIN_RED_RING_PIXELS = 8
MAX_RED_RING_DENSITY = 0.22

COLOR_YELLOW = image.Color.from_rgb(255, 255, 0)
COLOR_CYAN = image.Color.from_rgb(0, 255, 255)
COLOR_RED = image.Color.from_rgb(255, 0, 0)


def useful_rectangles(rectangles, image_width, image_height):
    """Filter obvious noise before choosing the printed target frame."""
    candidates = []
    for rect in rectangles:
        width = rect.w()
        height = rect.h()
        area = width * height
        if width < MIN_FRAME_WIDTH or height < MIN_FRAME_HEIGHT:
            continue
        if area < MIN_FRAME_AREA:
            continue
        # Reject the image boundary if it appears as a rectangle.
        if width >= image_width * 0.98 and height >= image_height * 0.98:
            continue
        # The target is landscape; this rejects the portrait background screen
        # and makes initial long-distance acquisition more reliable.
        aspect = width / height
        if 1.05 <= aspect <= 2.15:
            candidates.append(rect)
    return candidates


def rect_centre(rect):
    return rect.x() + rect.w() // 2, rect.y() + rect.h() // 2


def red_ring_signature(img, rect):
    """Score thin red ring pixels distributed through a rectangle's interior."""
    x, y, width, height = rect.x(), rect.y(), rect.w(), rect.h()
    blobs = img.find_blobs(
        [RED_RING_LAB],
        roi=[x, y, width, height],
        x_stride=1,
        y_stride=1,
        area_threshold=1,
        pixels_threshold=1,
        merge=False,
    )

    red_pixels = 0
    quadrants = [False, False, False, False]
    for blob in blobs:
        red_pixels += blob.pixels()
        bx, by = blob.cx(), blob.cy()
        # find_blobs normally returns image coordinates.  Keep this fallback
        # for firmware that returns coordinates relative to the ROI.
        if bx < x or bx >= x + width:
            bx += x
        if by < y or by >= y + height:
            by += y
        column = 1 if bx >= x + width // 2 else 0
        row = 1 if by >= y + height // 2 else 0
        quadrants[row * 2 + column] = True

    density = red_pixels / max(1, width * height)
    coverage = sum(1 for covered in quadrants if covered)
    is_target_like = (
        red_pixels >= MIN_RED_RING_PIXELS and
        coverage >= 3 and
        density <= MAX_RED_RING_DENSITY
    )
    return is_target_like, coverage, red_pixels, density


def choose_rectangle(img, rectangles, previous_centre, previous_area):
    """Select only a rectangle verified by the target's red-ring pattern."""
    if not rectangles:
        return None, None

    best = None
    best_score = None
    best_signature = None
    for rect in rectangles:
        is_target_like, coverage, red_pixels, density = red_ring_signature(img, rect)
        if not is_target_like:
            continue
        centre = rect_centre(rect)
        area = rect.w() * rect.h()
        if previous_centre is None:
            # Initial acquisition: ring coverage is more important than area.
            score = coverage * 100000 + red_pixels * 50 + rect.magnitude()
        else:
            centre_error = abs(centre[0] - previous_centre[0]) + \
                           abs(centre[1] - previous_centre[1])
            area_error = abs(area - previous_area) * 100 // max(1, previous_area)
            # Keep the same verified target through motion, but never allow a
            # background rectangle with no ring signature to win.
            score = coverage * 100000 + red_pixels * 50 + rect.magnitude() - \
                    centre_error * 8 - area_error * 3
        if best_score is None or score > best_score:
            best = rect
            best_score = score
            best_signature = (coverage, red_pixels, density)

    if best is None:
        return None, None
    return best, best_signature


def rect_corners(rect):
    return [(int(x), int(y)) for x, y in rect.corners()]


def smooth_corners(previous, current):
    """Use 75% of the previous frame and 25% of the current measurement."""
    if previous is None:
        return current
    return [
        ((previous[i][0] * 3 + current[i][0]) // 4,
         (previous[i][1] * 3 + current[i][1]) // 4)
        for i in range(4)
    ]


def centre_of_corners(corners):
    return sum(point[0] for point in corners) // 4, \
           sum(point[1] for point in corners) // 4


def draw_quad(img, corners):
    for index in range(4):
        x1, y1 = corners[index]
        x2, y2 = corners[(index + 1) % 4]
        img.draw_line(x1, y1, x2, y2, color=COLOR_YELLOW, thickness=2)


def draw_cross(img, x, y):
    img.draw_circle(x, y, 7, color=COLOR_CYAN, thickness=2)
    img.draw_line(x - 12, y, x + 12, y, color=COLOR_CYAN, thickness=2)
    img.draw_line(x, y - 12, x, y + 12, color=COLOR_CYAN, thickness=2)


def main():
    cam = camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT, fps=CAMERA_FPS)
    disp = display.Display()
    cam.skip_frames(30)

    edge_detection_enabled = True
    stable_corners = None
    stable_area = None
    missing_frames = FRAME_HOLD_FRAMES

    while not app.need_exit():
        img = cam.read()
        measured = None
        ring_signature = None

        if edge_detection_enabled:
            try:
                candidates = useful_rectangles(
                    img.find_rects(threshold=FRAME_EDGE_THRESHOLD),
                    img.width(), img.height()
                )
                previous_centre = centre_of_corners(stable_corners) \
                    if stable_corners is not None else None
                measured, ring_signature = choose_rectangle(
                    img, candidates, previous_centre, stable_area
                )
            except MemoryError:
                edge_detection_enabled = False
                print("frame edge detector disabled: fast frame buffer is full")

        if measured is not None:
            stable_corners = smooth_corners(stable_corners, rect_corners(measured))
            measured_area = measured.w() * measured.h()
            stable_area = measured_area if stable_area is None else \
                (stable_area * 3 + measured_area) // 4
            missing_frames = 0
            frame_state = "MEASURED R{} P{}".format(
                ring_signature[0], ring_signature[1]
            )
        elif stable_corners is not None and missing_frames < FRAME_HOLD_FRAMES:
            missing_frames += 1
            frame_state = "HOLD {}/{}".format(missing_frames, FRAME_HOLD_FRAMES)
        else:
            stable_corners = None
            stable_area = None
            frame_state = "NOT FOUND"

        if stable_corners is not None:
            draw_quad(img, stable_corners)
            cx, cy = centre_of_corners(stable_corners)
            draw_cross(img, cx, cy)
            img.draw_string(8, 8, "TARGET FRAME centre:({}, {})".format(cx, cy),
                            color=COLOR_CYAN, scale=1.1)
            img.draw_string(8, 28, "FRAME: {}".format(frame_state),
                            color=COLOR_YELLOW, scale=1.0)
        else:
            img.draw_string(8, 8, "TARGET NOT FOUND", color=COLOR_RED, scale=1.2)
            img.draw_string(8, 28, "FRAME: check target is fully visible",
                            color=COLOR_YELLOW, scale=0.9)

        disp.show(img)


main()
