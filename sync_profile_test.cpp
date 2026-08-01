#include "firmware_logic.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>

using SpeedResult = firmware::SyncSpeedResult;

static SpeedResult calculate_sync_speeds(double target,
                                         double left_progress,
                                         double right_progress,
                                         double requested_speed,
                                         bool left_boost_input = false,
                                         bool right_boost_input = false,
                                         int left_directed_progress_count = 0,
                                         int right_directed_progress_count = 0,
                                         uint64_t elapsed_us = 0) {
    return firmware::calculate_sync_speeds(
        target,
        left_progress,
        right_progress,
        requested_speed,
        left_boost_input,
        right_boost_input,
        left_directed_progress_count,
        right_directed_progress_count,
        elapsed_us);
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

static void expect_false(const char* name, bool value) {
    if (value) {
        std::printf("FAIL %s\n", name);
        std::exit(1);
    }
}

static void expect_boundary(double target,
                            double expected_tolerance,
                            double expected_threshold) {
    SpeedResult r = calculate_sync_speeds(target, 0.0, 0.0, 180.0);
    char name[80];
    std::snprintf(name, sizeof(name), "tolerance %.3f", target);
    expect_close(name, r.tolerance, expected_tolerance);
    std::snprintf(name, sizeof(name), "threshold %.3f", target);
    expect_close(name, r.threshold, expected_threshold);
}

static void expect_min_speed(double target, double expected_min_speed) {
    SpeedResult r = calculate_sync_speeds(target, 0.0, 0.0, 180.0);
    char name[80];
    std::snprintf(name, sizeof(name), "min speed %.3f", target);
    expect_close(name, r.min_speed, expected_min_speed);
}

static double encoder_counts_to_deg(int count) {
    return static_cast<double>(count) * 360.0 / 3900.0;
}

int main() {
    expect_close("zero target tolerance",
                 firmware::calculate_sync_tolerance_deg(0.0), 0.0);
    expect_close("negative target tolerance",
                 firmware::calculate_sync_tolerance_deg(-1.0), 0.0);
    expect_close("max tolerance",
                 firmware::calculate_sync_tolerance_deg(409.256), 3.0);

    expect_boundary(1.0, 0.2, 0.8);
    expect_boundary(2.0, 0.4, 1.6);
    expect_boundary(3.0, 0.6, 2.4);
    expect_boundary(4.0, 0.8, 3.2);
    expect_boundary(5.0, 1.0, 4.0);
    expect_boundary(6.0, 1.2, 4.8);
    expect_boundary(9.0, 1.8, 7.2);
    expect_boundary(15.0, 3.0, 12.0);
    expect_boundary(30.0, 3.0, 27.0);
    expect_boundary(180.0, 3.0, 177.0);
    expect_boundary(409.256, 3.0, 406.256);

    expect_min_speed(1.0, 30.0);
    expect_min_speed(3.0, 30.0);
    expect_min_speed(5.0, 30.0);
    expect_min_speed(9.0, 30.0);
    expect_min_speed(15.0, 30.0);
    expect_min_speed(15.0001, 20.0);
    expect_min_speed(30.0, 20.0);
    expect_min_speed(180.0, 20.0);
    expect_min_speed(409.256, 20.0);

    expect_true("target 5 boost eligible",
                calculate_sync_speeds(5.0, 0.0, 0.0, 180.0).boost_eligible);
    expect_true("target 15 boost eligible",
                calculate_sync_speeds(15.0, 0.0, 0.0, 180.0).boost_eligible);
    expect_false("target 15.0001 boost disabled",
                 calculate_sync_speeds(15.0001, 0.0, 0.0, 180.0).boost_eligible);

    expect_false("target 1 start not done",
                 calculate_sync_speeds(1.0, 0.0, 0.0, 180.0).done);
    expect_false("target 2 start not done",
                 calculate_sync_speeds(2.0, 0.0, 0.0, 180.0).done);
    expect_false("target 3 start not done",
                 calculate_sync_speeds(3.0, 0.0, 0.0, 180.0).done);

    SpeedResult r = calculate_sync_speeds(5.0, 0.0, 0.0, 180.0);
    expect_false("case 1 not done", r.done);
    expect_close("zero left", r.left_abs, 30.0);
    expect_close("zero right", r.right_abs, 30.0);

    r = calculate_sync_speeds(5.0, 0.0, 0.0, 180.0, true, true, 0, 0, 0);
    expect_true("boost start left active", r.left_boost_active);
    expect_true("boost start right active", r.right_boost_active);
    expect_false("small target startup assist left inactive",
                 r.left_startup_assist_active);
    expect_false("small target startup assist right inactive",
                 r.right_startup_assist_active);
    expect_close("boost start left", r.left_abs, 80.0);
    expect_close("boost start right", r.right_abs, 80.0);

    r = calculate_sync_speeds(5.0, 0.0, 0.0, 180.0, true, true, 1, 0, 10000);
    expect_false("left movement releases left boost", r.left_boost_active);
    expect_true("right boost continues", r.right_boost_active);
    expect_close("left released speed", r.left_abs, 30.0);
    expect_close("right still boosted speed", r.right_abs, 80.0);

    r = calculate_sync_speeds(5.0, 0.0, 0.0, 180.0, true, true, 0, 1, 10000);
    expect_true("left boost continues", r.left_boost_active);
    expect_false("right movement releases right boost", r.right_boost_active);
    expect_close("left still boosted speed", r.left_abs, 80.0);
    expect_close("right released speed", r.right_abs, 30.0);

    r = calculate_sync_speeds(5.0, 0.0, 0.0, 180.0, true, true, 1, 1, 10000);
    expect_false("both movement releases left boost", r.left_boost_active);
    expect_false("both movement releases right boost", r.right_boost_active);
    expect_close("both released left", r.left_abs, 30.0);
    expect_close("both released right", r.right_abs, 30.0);

    r = calculate_sync_speeds(5.0, 0.0, 0.0, 180.0, true, true, 0, 0, 299999);
    expect_true("boost before max left", r.left_boost_active);
    expect_true("boost before max right", r.right_boost_active);

    r = calculate_sync_speeds(5.0, 0.0, 0.0, 180.0, true, true, 0, 0, 300000);
    expect_false("boost max releases left", r.left_boost_active);
    expect_false("boost max releases right", r.right_boost_active);
    expect_close("boost max left speed", r.left_abs, 30.0);
    expect_close("boost max right speed", r.right_abs, 30.0);

    r = calculate_sync_speeds(5.0, 0.0, 0.0, 180.0, true, true, 0, 0, 300001);
    expect_false("boost after max releases left", r.left_boost_active);
    expect_false("boost after max releases right", r.right_boost_active);

    r = calculate_sync_speeds(5.0, 0.0, 0.0, 180.0, true, true, -1, -1, 10000);
    expect_true("reverse count keeps left boost", r.left_boost_active);
    expect_true("reverse count keeps right boost", r.right_boost_active);
    expect_close("reverse count boosted left", r.left_abs, 80.0);
    expect_close("reverse count boosted right", r.right_abs, 80.0);

    r = calculate_sync_speeds(5.0, 0.0, 4.8, 180.0);
    expect_false("case 2 not done", r.done);
    expect_close("right lead left", r.left_abs, 31.44);
    expect_close("right lead right", r.right_abs, 0.0);
    expect_true("right lead avoids old left speed", r.left_abs > 1.68);

    r = calculate_sync_speeds(5.0, 0.0, 4.8, 180.0, true, true, 0, 0, 10000);
    expect_true("lagging left boost active", r.left_boost_active);
    expect_false("leading right boost inactive in tolerance", r.right_boost_active);
    expect_close("boosted right lead left", r.left_abs, 81.44);
    expect_close("boosted right lead right", r.right_abs, 0.0);

    r = calculate_sync_speeds(5.0, 3.5, 4.8, 180.0);
    expect_false("case 3 not done", r.done);
    expect_close("case 3 left", r.left_abs, 30.39);
    expect_close("case 3 right", r.right_abs, 0.0);

    r = calculate_sync_speeds(5.0, 3.5, 4.8, 180.0, true, true, 0, 0, 10000);
    expect_true("lagging left boost remains active", r.left_boost_active);
    expect_false("right tolerance disables boost", r.right_boost_active);
    expect_close("boosted case 3 left", r.left_abs, 80.39);
    expect_close("boosted case 3 right", r.right_abs, 0.0);

    r = calculate_sync_speeds(5.0, 4.0, 4.8, 180.0);
    expect_true("case 4 done", r.done);
    expect_close("case 4 done left", r.left_abs, 0.0);
    expect_close("case 4 done right", r.right_abs, 0.0);

    r = calculate_sync_speeds(5.0, 4.2, 4.1, 180.0);
    expect_true("case 5 done", r.done);

    r = calculate_sync_speeds(5.0, 3.99, 5.2, 180.0);
    expect_false("case 6 not done", r.done);
    expect_close("case 6 left", r.left_abs, 30.363);
    expect_close("case 6 right", r.right_abs, 0.0);

    r = calculate_sync_speeds(5.0, 3.99, 5.2, 180.0, true, true, 0, 0, 10000);
    expect_true("case 6 left boost active", r.left_boost_active);
    expect_false("case 6 right boost inactive", r.right_boost_active);
    expect_close("boosted case 6 left", r.left_abs, 80.363);
    expect_close("boosted case 6 right", r.right_abs, 0.0);

    r = calculate_sync_speeds(5.0, 5.5, 3.0, 180.0);
    expect_false("case 7 not done", r.done);
    expect_close("case 7 left", r.left_abs, 0.0);
    expect_close("case 7 right", r.right_abs, 30.75);

    r = calculate_sync_speeds(5.0, 5.5, 3.0, 180.0, true, true, 0, 0, 10000);
    expect_false("overshoot left boost inactive", r.left_boost_active);
    expect_true("right boost active after left overshoot", r.right_boost_active);
    expect_close("boosted case 7 left", r.left_abs, 0.0);
    expect_close("boosted case 7 right", r.right_abs, 80.75);

    r = calculate_sync_speeds(5.0, 4.8, 0.0, 180.0);
    expect_close("left lead left", r.left_abs, 0.0);
    expect_close("left lead right", r.right_abs, 31.44);

    r = calculate_sync_speeds(5.0, 0.185, 3.138, 180.0);
    expect_close("case 4 left", r.left_abs, 30.8859);
    expect_close("case 4 right", r.right_abs, 29.1141);

    r = calculate_sync_speeds(5.0, 1.477, 4.154, 180.0);
    expect_close("case 5 left", r.left_abs, 30.8031);
    expect_close("case 5 right", r.right_abs, 0.0);

    r = calculate_sync_speeds(
        30.0,
        12.0,
        12.0,
        180.0,
        false,
        false,
        3,
        3,
        firmware::kSyncSmallTargetStartBoostMaxUs);
    expect_close("same progress left", r.left_abs, 21.6);
    expect_close("same progress right", r.right_abs, 21.6);

    r = calculate_sync_speeds(30.0, 0.0, 0.0, 180.0);
    expect_true("target 30 startup assist eligible",
                r.startup_assist_eligible);
    expect_true("target 30 initial left assist active",
                r.left_startup_assist_active);
    expect_true("target 30 initial right assist active",
                r.right_startup_assist_active);
    expect_close("target 30 initial left", r.left_abs, 50.0);
    expect_close("target 30 initial right", r.right_abs, 50.0);

    r = calculate_sync_speeds(30.0, 0.0, 0.0, 180.0, true, true, 0, 0, 0);
    expect_false("target 30 left small boost inactive", r.left_boost_active);
    expect_false("target 30 right small boost inactive", r.right_boost_active);
    expect_close("target 30 startup assist ignores small boost left",
                 r.left_abs, 50.0);
    expect_close("target 30 startup assist ignores small boost right",
                 r.right_abs, 50.0);

    r = calculate_sync_speeds(30.0, 0.0, 0.0, 180.0, false, false, 0, 0,
                              firmware::kSyncSmallTargetStartBoostMaxUs);
    expect_false("target 30 assist releases at window left",
                 r.left_startup_assist_active);
    expect_false("target 30 assist releases at window right",
                 r.right_startup_assist_active);
    expect_close("target 30 profile after window left", r.left_abs, 36.0);
    expect_close("target 30 profile after window right", r.right_abs, 36.0);

    const double startup_targets[] = {15.090, 16.168, 21.557, 26.946};
    for (double startup_target : startup_targets) {
        r = calculate_sync_speeds(startup_target, 0.0, 0.0, 180.0);
        char name[96];
        std::snprintf(name, sizeof(name),
                      "startup assist target %.3f left", startup_target);
        expect_close(name, r.left_abs, firmware::kSyncStartupAssistMinSpeedDegS);
        std::snprintf(name, sizeof(name),
                      "startup assist target %.3f right", startup_target);
        expect_close(name, r.right_abs, firmware::kSyncStartupAssistMinSpeedDegS);
        expect_true("startup assist left active",
                    r.left_startup_assist_active);
        expect_true("startup assist right active",
                    r.right_startup_assist_active);
    }

    r = calculate_sync_speeds(21.557, 0.0, 0.0, 180.0);
    expect_close("positive physical command preserves direction",
                 1.0 * r.left_abs, 50.0);
    expect_close("negative physical command preserves direction",
                 -1.0 * r.right_abs, -50.0);

    r = calculate_sync_speeds(
        21.557,
        encoder_counts_to_deg(1),
        encoder_counts_to_deg(1),
        180.0,
        false,
        false,
        1,
        1,
        10000);
    expect_true("one count keeps left startup assist",
                r.left_startup_assist_active);
    expect_true("one count keeps right startup assist",
                r.right_startup_assist_active);
    expect_close("one count left speed", r.left_abs, 50.0);
    expect_close("one count right speed", r.right_abs, 50.0);

    r = calculate_sync_speeds(
        21.557,
        encoder_counts_to_deg(3),
        encoder_counts_to_deg(3),
        180.0,
        false,
        false,
        3,
        3,
        10000);
    expect_false("three counts releases left startup assist",
                 r.left_startup_assist_active);
    expect_false("three counts releases right startup assist",
                 r.right_startup_assist_active);
    expect_close("three counts left returns profile",
                 r.left_abs,
                 1.2 * (21.557 - encoder_counts_to_deg(3)));
    expect_close("three counts right returns profile",
                 r.right_abs,
                 1.2 * (21.557 - encoder_counts_to_deg(3)));

    r = calculate_sync_speeds(
        21.557,
        -encoder_counts_to_deg(3),
        -encoder_counts_to_deg(3),
        180.0,
        false,
        false,
        -3,
        -3,
        10000);
    expect_true("reverse counts keep left startup assist",
                r.left_startup_assist_active);
    expect_true("reverse counts keep right startup assist",
                r.right_startup_assist_active);
    expect_close("reverse counts left speed", r.left_abs, 50.0);
    expect_close("reverse counts right speed", r.right_abs, 50.0);

    r = calculate_sync_speeds(
        21.557,
        0.0,
        0.0,
        180.0,
        false,
        false,
        0,
        0,
        firmware::kSyncSmallTargetStartBoostMaxUs);
    expect_false("assist window end left inactive",
                 r.left_startup_assist_active);
    expect_false("assist window end right inactive",
                 r.right_startup_assist_active);
    expect_close("assist window end left profile", r.left_abs, 25.8684);
    expect_close("assist window end right profile", r.right_abs, 25.8684);

    r = calculate_sync_speeds(50.0, 0.0, 0.0, 180.0);
    expect_true("target 50 assist active but does not lower left",
                r.left_startup_assist_active);
    expect_true("target 50 assist active but does not lower right",
                r.right_startup_assist_active);
    expect_close("target 50 profile remains left", r.left_abs, 60.0);
    expect_close("target 50 profile remains right", r.right_abs, 60.0);

    r = calculate_sync_speeds(21.557, 18.6, 0.0, 180.0);
    expect_false("reached left startup assist inactive",
                 r.left_startup_assist_active);
    expect_true("unreached right startup assist active",
                r.right_startup_assist_active);
    expect_close("reached left remains stopped", r.left_abs, 0.0);
    expect_close("unreached right gets startup assist", r.right_abs, 50.0);

    r = calculate_sync_speeds(45.0, encoder_counts_to_deg(2), 0.0, 180.0);
    expect_true("correction case left startup assist active",
                r.left_startup_assist_active);
    expect_true("correction case right startup assist active",
                r.right_startup_assist_active);
    expect_true("correction changes left speed above assist floor",
                r.left_abs < 54.0);
    expect_true("correction changes right speed above assist floor",
                r.right_abs > 54.0);
    expect_true("correction keeps left below max",
                r.left_abs <= firmware::kSyncMaxSpeedDegS);
    expect_true("correction keeps right below max",
                r.right_abs <= firmware::kSyncMaxSpeedDegS);

    r = calculate_sync_speeds(180.0, 0.0, 0.0, 300.0);
    expect_close("target 180 initial left",
                 r.left_abs, firmware::kSyncMaxSpeedDegS);
    expect_close("target 180 initial right",
                 r.right_abs, firmware::kSyncMaxSpeedDegS);

    r = calculate_sync_speeds(409.256, 0.0, 0.0, 300.0);
    expect_close("long target max left",
                 r.left_abs, firmware::kSyncMaxSpeedDegS);
    expect_close("long target max right",
                 r.right_abs, firmware::kSyncMaxSpeedDegS);

    r = calculate_sync_speeds(5.0, 6.0, 0.0, 180.0);
    expect_close("left overshoot", r.left_abs, 0.0);
    expect_true("left overshoot right moves", r.right_abs > 0.0);

    r = calculate_sync_speeds(5.0, 0.0, 6.0, 180.0);
    expect_true("right overshoot left moves", r.left_abs > 0.0);
    expect_close("right overshoot", r.right_abs, 0.0);

    r = calculate_sync_speeds(
        409.256,
        -500.0,
        0.0,
        180.0,
        false,
        false,
        3,
        3,
        firmware::kSyncSmallTargetStartBoostMaxUs);
    expect_close("correction clamp left",
                 r.left_abs, firmware::kSyncMaxSpeedDegS);
    expect_close("correction clamp right", r.right_abs, 30.0);

    SpeedResult positive_profile = calculate_sync_speeds(5.0, 0.0, 4.8, 180.0);
    SpeedResult negative_profile = calculate_sync_speeds(5.0, 0.0, 4.8, 180.0);
    expect_close("direction symmetry left",
                 positive_profile.left_abs,
                 negative_profile.left_abs);
    expect_close("direction symmetry right",
                 positive_profile.right_abs,
                 negative_profile.right_abs);

    r = calculate_sync_speeds(5.0, 2.031, 4.8, 180.0);
    expect_false("old tolerance no longer done", r.done);
    expect_true("old tolerance left keeps moving",
                r.left_abs >= firmware::kSyncSmallTargetMinSpeedDegS);

    r = calculate_sync_speeds(5.0, 0.0, 0.0, 10.0);
    expect_close("small target min beats requested left", r.left_abs, 30.0);
    expect_close("small target min beats requested right", r.right_abs, 30.0);

    r = calculate_sync_speeds(5.0, 0.0, 0.0, 10.0, true, true, 0, 0, 0);
    expect_close("boost beats requested left", r.left_abs, 80.0);
    expect_close("boost beats requested right", r.right_abs, 80.0);

    r = calculate_sync_speeds(
        30.0,
        20.0,
        20.0,
        10.0,
        false,
        false,
        3,
        3,
        firmware::kSyncSmallTargetStartBoostMaxUs);
    expect_close("normal target min beats requested left", r.left_abs, 20.0);
    expect_close("normal target min beats requested right", r.right_abs, 20.0);

    r = calculate_sync_speeds(5.0, 4.0, 4.8, 180.0);
    expect_true("done at effective threshold", r.done);
    expect_close("done left speed", r.left_abs, 0.0);
    expect_close("done right speed", r.right_abs, 0.0);

    expect_close("startup assist min constant",
                 firmware::kSyncStartupAssistMinSpeedDegS, 50.0);
    expect_true("startup assist progress counts constant",
                firmware::kSyncStartupAssistProgressCounts == 3);
    expect_true("startup assist window uses existing boost window",
                firmware::kSyncSmallTargetStartBoostMaxUs == 300000);
    expect_true("stall timeout unchanged",
                firmware::kSyncStallTimeoutUs == 1500000);

    std::printf("sync_profile_test OK\n");
    return 0;
}
