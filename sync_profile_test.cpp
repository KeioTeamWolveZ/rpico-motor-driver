#include <cmath>
#include <cstdio>
#include <cstdlib>

static const double SYNC_KP = 0.30;
static const double SYNC_TOLERANCE_DEG = 3.0;
static const double SYNC_MIN_SPEED_DEG_S = 20.0;
static const double SYNC_MAX_SPEED_DEG_S = 180.0;
static const double SYNC_SLOWDOWN_GAIN = 1.2;

static double clamp_double(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static double min_double(double a, double b) {
    return a < b ? a : b;
}

static double calculate_sync_wheel_base_speed(double remaining_deg,
                                              double requested_speed_deg_s) {
    if (remaining_deg <= 0.0) {
        return 0.0;
    }

    double base_speed =
        min_double(requested_speed_deg_s, SYNC_SLOWDOWN_GAIN * remaining_deg);
    if (base_speed < 0.0) {
        base_speed = 0.0;
    }
    if (remaining_deg > SYNC_TOLERANCE_DEG && base_speed < SYNC_MIN_SPEED_DEG_S) {
        base_speed = SYNC_MIN_SPEED_DEG_S;
    }
    return base_speed;
}

struct SpeedResult {
    double left_abs;
    double right_abs;
    bool done;
};

static SpeedResult calculate_sync_speeds(double target,
                                         double left_progress,
                                         double right_progress,
                                         double requested_speed) {
    SpeedResult result = {};
    result.done = left_progress >= target - SYNC_TOLERANCE_DEG &&
                  right_progress >= target - SYNC_TOLERANCE_DEG;
    if (result.done) {
        return result;
    }

    double left_remaining = target - left_progress;
    double right_remaining = target - right_progress;
    double requested = clamp_double(requested_speed, 0.0, SYNC_MAX_SPEED_DEG_S);
    double left_base = calculate_sync_wheel_base_speed(left_remaining, requested);
    double right_base = calculate_sync_wheel_base_speed(right_remaining, requested);
    double correction = SYNC_KP * (left_progress - right_progress);

    result.left_abs = clamp_double(left_base - correction, 0.0, SYNC_MAX_SPEED_DEG_S);
    result.right_abs = clamp_double(right_base + correction, 0.0, SYNC_MAX_SPEED_DEG_S);
    if (left_remaining <= 0.0) {
        result.left_abs = 0.0;
    }
    if (right_remaining <= 0.0) {
        result.right_abs = 0.0;
    }
    return result;
}

static void expect_close(const char* name, double actual, double expected) {
    if (std::fabs(actual - expected) > 0.001) {
        std::printf("FAIL %s actual=%.6f expected=%.6f\n", name, actual, expected);
        std::exit(1);
    }
}

static void expect_true(const char* name, bool value) {
    if (!value) {
        std::printf("FAIL %s\n", name);
        std::exit(1);
    }
}

int main() {
    SpeedResult r = calculate_sync_speeds(5.0, 0.0, 0.0, 180.0);
    expect_close("zero left", r.left_abs, 20.0);
    expect_close("zero right", r.right_abs, 20.0);

    r = calculate_sync_speeds(5.0, 0.0, 4.8, 180.0);
    expect_close("right lead left", r.left_abs, 21.44);
    expect_close("right lead right", r.right_abs, 0.0);
    expect_true("right lead avoids old left speed", r.left_abs > 1.68);

    r = calculate_sync_speeds(5.0, 4.8, 0.0, 180.0);
    expect_close("left lead left", r.left_abs, 0.0);
    expect_close("left lead right", r.right_abs, 21.44);

    r = calculate_sync_speeds(5.0, 0.185, 3.138, 180.0);
    expect_close("case 4 left", r.left_abs, 20.8859);
    expect_close("case 4 right", r.right_abs, 1.3485);

    r = calculate_sync_speeds(5.0, 1.477, 4.154, 180.0);
    expect_close("case 5 left", r.left_abs, 20.8031);
    expect_close("case 5 right", r.right_abs, 0.2121);

    r = calculate_sync_speeds(30.0, 12.0, 12.0, 180.0);
    expect_close("same progress left", r.left_abs, 21.6);
    expect_close("same progress right", r.right_abs, 21.6);

    r = calculate_sync_speeds(5.0, 6.0, 0.0, 180.0);
    expect_close("left overshoot", r.left_abs, 0.0);
    expect_true("left overshoot right moves", r.right_abs > 0.0);

    r = calculate_sync_speeds(5.0, 0.0, 6.0, 180.0);
    expect_true("right overshoot left moves", r.left_abs > 0.0);
    expect_close("right overshoot", r.right_abs, 0.0);

    r = calculate_sync_speeds(5.0, 100.0, 0.0, 180.0);
    expect_close("correction clamp left", r.left_abs, 0.0);
    expect_true("correction clamp right", r.right_abs <= SYNC_MAX_SPEED_DEG_S);

    double positive_left = calculate_sync_speeds(5.0, 0.0, 4.8, 180.0).left_abs;
    double negative_left = calculate_sync_speeds(5.0, 0.0, 4.8, 180.0).left_abs;
    expect_close("direction symmetry", positive_left, negative_left);

    r = calculate_sync_speeds(5.0, 2.031, 4.8, 180.0);
    expect_true("done tolerance unchanged", r.done);
    expect_close("done left speed", r.left_abs, 0.0);
    expect_close("done right speed", r.right_abs, 0.0);

    std::printf("sync_profile_test OK\n");
    return 0;
}
