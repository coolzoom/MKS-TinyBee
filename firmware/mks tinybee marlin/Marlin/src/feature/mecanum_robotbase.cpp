/**
 * Mecanum Robot Base - Protocol implementation
 * Protocol: firmware/robotbase/README.md
 * XYZA = four wheels. Motion via G91 G1 relative moves.
 */
#include "../inc/MarlinConfig.h"

#if ENABLED(MECANUM_ROBOTBASE)

#include "mecanum_robotbase.h"
#include "../gcode/queue.h"
#include "../module/planner.h"
#include "../module/motion.h"
#include "../MarlinCore.h"

#ifndef MECANUM_DEFAULT_SPEED_MM_S
  #define MECANUM_DEFAULT_SPEED_MM_S 50.0f
#endif
#ifndef MECANUM_ROTATION_RADIUS_MM
  #define MECANUM_ROTATION_RADIUS_MM 150.0f
#endif
#ifndef MECANUM_PI
  #define MECANUM_PI 3.14159265f
#endif
#define MECANUM_TRANSFORM_FACTOR 0.7071f  // √2/2 for lateral
#define MECANUM_CONTINUOUS_MM   10000.0f // long move until S
#define ROBOTBASE_SPEED_MIN     0.0f
#define ROBOTBASE_SPEED_MAX     200.0f
#define ROBOTBASE_DISTANCE_MAX  10000.0f
#define ROBOTBASE_ANGLE_MAX     360.0f

// PWM remote tuning (microseconds)
#ifndef RB_REMOTE_ENABLE_FB
  #define RB_REMOTE_ENABLE_FB         1
#endif
#ifndef RB_REMOTE_ENABLE_LR
  #define RB_REMOTE_ENABLE_LR         1
#endif
#ifndef RB_REMOTE_ENABLE_ROT
  #define RB_REMOTE_ENABLE_ROT        1
#endif
#ifndef RB_REMOTE_ENABLE_RAY
  #define RB_REMOTE_ENABLE_RAY        1
#endif
#ifndef RB_PWM_CENTER_FB_US
  #define RB_PWM_CENTER_FB_US         1500
#endif
#ifndef RB_PWM_CENTER_LR_US
  #define RB_PWM_CENTER_LR_US         1500
#endif
#ifndef RB_PWM_CENTER_ROT_US
  #define RB_PWM_CENTER_ROT_US        1500
#endif
#ifndef RB_PWM_CENTER_RAY_US
  #define RB_PWM_CENTER_RAY_US        1500
#endif
#ifndef RB_PWM_DEADBAND_FB_US
  #define RB_PWM_DEADBAND_FB_US       120
#endif
#ifndef RB_PWM_DEADBAND_LR_US
  #define RB_PWM_DEADBAND_LR_US       120
#endif
#ifndef RB_PWM_DEADBAND_ROT_US
  #define RB_PWM_DEADBAND_ROT_US      120
#endif
#ifndef RB_PWM_DEADBAND_RAY_US
  #define RB_PWM_DEADBAND_RAY_US      120
#endif
#ifndef RB_ANALOG_POLL_MS
  #define RB_ANALOG_POLL_MS           20
#endif
#ifndef RB_PWM_STREAM_INTERVAL_MS
  #define RB_PWM_STREAM_INTERVAL_MS   50
#endif
#ifndef RB_PWM_AUTO_CENTER
  #define RB_PWM_AUTO_CENTER          1
#endif
#ifndef RB_PWM_AUTO_CENTER_MS
  #define RB_PWM_AUTO_CENTER_MS       2000
#endif
#ifndef RB_PWM_AUTO_CENTER_MIN_SAMPLES
  #define RB_PWM_AUTO_CENTER_MIN_SAMPLES 20
#endif
#ifndef RB_PWM_SIGNAL_TIMEOUT_MS
  #define RB_PWM_SIGNAL_TIMEOUT_MS    120
#endif
#ifndef RB_REMOTE_SPEED_MM_S
  #define RB_REMOTE_SPEED_MM_S        60.0f
#endif
#ifndef RB_RAY_CORRECT_SPEED_MM_S
  #define RB_RAY_CORRECT_SPEED_MM_S   35.0f
#endif

static uint8_t robotbase_stepflage = 0;   // 0=stopped, 1=moving
static uint8_t robotbase_serial_controlled = 0;
static uint8_t robotbase_distance_controlled = 0;
static float   robotbase_current_speed = 0.0f;
static uint8_t robotbase_raytracing_enabled = 1;
static uint8_t robotbase_centered = 0;
static bool    robotbase_profile_applied = false;
static uint8_t robotbase_remote_mode = 0;
static bool    rb_remote_apply_busy = false;
static uint8_t rb_pwm_stream_enabled = 0;
enum : uint8_t { RB_CH_FB = 0, RB_CH_LR, RB_CH_ROT, RB_CH_RAY };
static volatile uint32_t rb_pwm_rise_us[4] = { 0, 0, 0, 0 };
static volatile uint32_t rb_pwm_updates[4] = { 0, 0, 0, 0 };
static volatile uint32_t rb_pwm_last_valid_us[4] = { 0, 0, 0, 0 };
static volatile int      rb_pwm_us[4] = {
  RB_PWM_CENTER_FB_US, RB_PWM_CENTER_LR_US, RB_PWM_CENTER_ROT_US, RB_PWM_CENTER_RAY_US
};
static int rb_pwm_fb = RB_PWM_CENTER_FB_US;
static int rb_pwm_lr = RB_PWM_CENTER_LR_US;
static int rb_pwm_rot = RB_PWM_CENTER_ROT_US;
static int rb_pwm_ray = RB_PWM_CENTER_RAY_US;
static int rb_pwm_center_fb = RB_PWM_CENTER_FB_US;
static int rb_pwm_center_lr = RB_PWM_CENTER_LR_US;
static int rb_pwm_center_rot = RB_PWM_CENTER_ROT_US;
static int rb_pwm_center_ray = RB_PWM_CENTER_RAY_US;
static bool rb_pwm_center_ready = false;
static millis_t rb_pwm_center_until_ms = 0;
static uint32_t rb_pwm_center_sum[4] = { 0, 0, 0, 0 };
static uint16_t rb_pwm_center_cnt[4] = { 0, 0, 0, 0 };

static void reply_ack() { SERIAL_ECHOLNPGM("ACK"); }
static void reply_ok()  { SERIAL_ECHOLNPGM("ok"); }

static void report_pwm_snapshot(const bool include_thresholds) {
  const int pwm_low_fb = rb_pwm_center_fb - RB_PWM_DEADBAND_FB_US;
  const int pwm_high_fb = rb_pwm_center_fb + RB_PWM_DEADBAND_FB_US;
  const int pwm_low_lr = rb_pwm_center_lr - RB_PWM_DEADBAND_LR_US;
  const int pwm_high_lr = rb_pwm_center_lr + RB_PWM_DEADBAND_LR_US;
  const int pwm_low_rot = rb_pwm_center_rot - RB_PWM_DEADBAND_ROT_US;
  const int pwm_high_rot = rb_pwm_center_rot + RB_PWM_DEADBAND_ROT_US;
  const int pwm_low_ray = rb_pwm_center_ray - RB_PWM_DEADBAND_RAY_US;
  const int pwm_high_ray = rb_pwm_center_ray + RB_PWM_DEADBAND_RAY_US;

  SERIAL_ECHOPGM("PWM:FB=");
  SERIAL_ECHO(rb_pwm_fb);
  SERIAL_ECHOPGM(",LR=");
  SERIAL_ECHO(rb_pwm_lr);
  SERIAL_ECHOPGM(",ROT=");
  SERIAL_ECHO(rb_pwm_rot);
  SERIAL_ECHOPGM(",RAY=");
  SERIAL_ECHO(rb_pwm_ray);
  SERIAL_ECHOPGM(",enFB=");
  SERIAL_ECHO(int(RB_REMOTE_ENABLE_FB));
  SERIAL_ECHOPGM(",enLR=");
  SERIAL_ECHO(int(RB_REMOTE_ENABLE_LR));
  SERIAL_ECHOPGM(",enROT=");
  SERIAL_ECHO(int(RB_REMOTE_ENABLE_ROT));
  SERIAL_ECHOPGM(",enRAY=");
  SERIAL_ECHO(int(RB_REMOTE_ENABLE_RAY));
  SERIAL_ECHOPGM(",stream=");
  SERIAL_ECHO(int(rb_pwm_stream_enabled));
  SERIAL_ECHOPGM(",autoCenter=");
  SERIAL_ECHO(int(rb_pwm_center_ready));
  SERIAL_ECHOPGM(",updFB=");
  SERIAL_ECHO(rb_pwm_updates[RB_CH_FB]);
  SERIAL_ECHOPGM(",updLR=");
  SERIAL_ECHO(rb_pwm_updates[RB_CH_LR]);
  SERIAL_ECHOPGM(",updROT=");
  SERIAL_ECHO(rb_pwm_updates[RB_CH_ROT]);
  SERIAL_ECHOPGM(",updRAY=");
  SERIAL_ECHO(rb_pwm_updates[RB_CH_RAY]);
  if (!include_thresholds) { SERIAL_EOL(); return; }

  // Thresholds / calibration details
  if (include_thresholds) {
    SERIAL_ECHOPGM(",cFB=");
    SERIAL_ECHO(rb_pwm_center_fb);
    SERIAL_ECHOPGM(",dFB=");
    SERIAL_ECHO(RB_PWM_DEADBAND_FB_US);
    SERIAL_ECHOPGM(",lowFB=");
    SERIAL_ECHO(pwm_low_fb);
    SERIAL_ECHOPGM(",highFB=");
    SERIAL_ECHO(pwm_high_fb);
    SERIAL_ECHOPGM(",cLR=");
    SERIAL_ECHO(rb_pwm_center_lr);
    SERIAL_ECHOPGM(",dLR=");
    SERIAL_ECHO(RB_PWM_DEADBAND_LR_US);
    SERIAL_ECHOPGM(",lowLR=");
    SERIAL_ECHO(pwm_low_lr);
    SERIAL_ECHOPGM(",highLR=");
    SERIAL_ECHO(pwm_high_lr);
    SERIAL_ECHOPGM(",cROT=");
    SERIAL_ECHO(rb_pwm_center_rot);
    SERIAL_ECHOPGM(",dROT=");
    SERIAL_ECHO(RB_PWM_DEADBAND_ROT_US);
    SERIAL_ECHOPGM(",lowROT=");
    SERIAL_ECHO(pwm_low_rot);
    SERIAL_ECHOPGM(",highROT=");
    SERIAL_ECHO(pwm_high_rot);
    SERIAL_ECHOPGM(",cRAY=");
    SERIAL_ECHO(rb_pwm_center_ray);
    SERIAL_ECHOPGM(",dRAY=");
    SERIAL_ECHO(RB_PWM_DEADBAND_RAY_US);
    SERIAL_ECHOPGM(",lowRAY=");
    SERIAL_ECHO(pwm_low_ray);
    SERIAL_ECHOPGM(",highRAY=");
    SERIAL_ECHO(pwm_high_ray);
  }
  SERIAL_EOL();
}

enum : uint8_t {
  RB_REMOTE_STOP = 0,
  RB_REMOTE_FB_FWD,
  RB_REMOTE_FB_BWD,
  RB_REMOTE_SLIDE_L,
  RB_REMOTE_SLIDE_R,
  RB_REMOTE_ROT_L,
  RB_REMOTE_ROT_R,
  RB_REMOTE_RAY_L,
  RB_REMOTE_RAY_R
};

static inline void rb_pwm_edge(const uint8_t ch, const int pin) {
  const uint32_t now = micros();
  if (digitalRead(pin)) rb_pwm_rise_us[ch] = now;
  else {
    const uint32_t rise = rb_pwm_rise_us[ch];
    if (rise) {
      const uint32_t w = now - rise;
      if (w >= 700 && w <= 2500) {
        rb_pwm_us[ch] = int(w);
        rb_pwm_updates[ch]++;
        rb_pwm_last_valid_us[ch] = now;
      }
    }
  }
}

#ifdef RB_REMOTE_FB_PIN
  static void rb_isr_fb()  { rb_pwm_edge(RB_CH_FB, RB_REMOTE_FB_PIN); }
#endif
#ifdef RB_REMOTE_LR_PIN
  static void rb_isr_lr()  { rb_pwm_edge(RB_CH_LR, RB_REMOTE_LR_PIN); }
#endif
#ifdef RB_REMOTE_ROT_PIN
  static void rb_isr_rot() { rb_pwm_edge(RB_CH_ROT, RB_REMOTE_ROT_PIN); }
#endif
#ifdef RB_RAY_TRACK_PIN
  static void rb_isr_ray() { rb_pwm_edge(RB_CH_RAY, RB_RAY_TRACK_PIN); }
#endif

static void apply_robotbase_motion_profile() {
  if (robotbase_profile_applied) return;

  // Override possible EEPROM leftovers so speed commands behave predictably.
  planner.settings.axis_steps_per_mm[X_AXIS] = 80.0f;
  planner.settings.axis_steps_per_mm[Y_AXIS] = 80.0f;
  planner.settings.axis_steps_per_mm[Z_AXIS] = 80.0f;
  planner.settings.axis_steps_per_mm[I_AXIS] = 80.0f;
  planner.refresh_positioning();

  planner.set_max_feedrate(X_AXIS, 300.0f);
  planner.set_max_feedrate(Y_AXIS, 300.0f);
  planner.set_max_feedrate(Z_AXIS, 300.0f);
  planner.set_max_feedrate(I_AXIS, 300.0f);

  planner.set_max_acceleration(X_AXIS, 200.0f);
  planner.set_max_acceleration(Y_AXIS, 200.0f);
  planner.set_max_acceleration(Z_AXIS, 200.0f);
  planner.set_max_acceleration(I_AXIS, 200.0f);

  robotbase_profile_applied = true;
}

static float clamp_speed(float v) {
  if (v < ROBOTBASE_SPEED_MIN || v != v) return MECANUM_DEFAULT_SPEED_MM_S;
  if (v > ROBOTBASE_SPEED_MAX) return ROBOTBASE_SPEED_MAX;
  return v;
}
static float clamp_distance(float v) {
  if (v < 0 || v != v) return 0;
  if (v > ROBOTBASE_DISTANCE_MAX) return ROBOTBASE_DISTANCE_MAX;
  return v;
}
static float clamp_angle(float v) {
  if (v < 0 || v != v) return 0;
  if (v > ROBOTBASE_ANGLE_MAX) return ROBOTBASE_ANGLE_MAX;
  return v;
}

// Inject G91 + G1 + G90, wait for completion. F = mm/min = speed_mm_s * 60. Uses queue.inject() (max 63 chars).
static void mecanum_move_distance(float dx, float dy, float dz, float da, float speed_mm_s) {
  char buf[64], sx[10], sy[10], sz[10], sa[10];
  const float f = speed_mm_s * 60.0f; // mm/min
  (void)sprintf(buf, "G91\nG1 X%s Y%s Z%s A%s F%ld\nG90",
    dtostrf(dx, 1, 2, sx),
    dtostrf(dy, 1, 2, sy),
    dtostrf(dz, 1, 2, sz),
    dtostrf(da, 1, 2, sa),
    (long)(f + 0.5f)
  );
  queue.inject(buf);
  planner.synchronize();
  robotbase_stepflage = 0;
  robotbase_distance_controlled = 0;
  reply_ok();
}

// Continuous move: inject long move (no wait)
static void mecanum_move_continuous(float dx, float dy, float dz, float da, float speed_mm_s, const bool serial_owner=true) {
  char buf[64], sx[10], sy[10], sz[10], sa[10];
  const float f = speed_mm_s * 60.0f;
  (void)sprintf(buf, "G91\nG1 X%s Y%s Z%s A%s F%ld\nG90",
    dtostrf(dx, 1, 2, sx),
    dtostrf(dy, 1, 2, sy),
    dtostrf(dz, 1, 2, sz),
    dtostrf(da, 1, 2, sa),
    (long)(f + 0.5f)
  );
  queue.inject(buf);
  robotbase_stepflage = 1;
  robotbase_serial_controlled = serial_owner ? 1 : 0;
  robotbase_distance_controlled = 0;
  robotbase_current_speed = speed_mm_s;
}

// Restart continuous movement so updated speed takes effect immediately.
static void mecanum_restart_continuous(float dx, float dy, float dz, float da, float speed_mm_s, const bool serial_owner=true) {
  if (robotbase_stepflage) quickstop_stepper();
  mecanum_move_continuous(dx, dy, dz, da, speed_mm_s, serial_owner);
}

static void robotbase_remote_stop() {
  if (robotbase_stepflage) quickstop_stepper();
  robotbase_stepflage = 0;
  robotbase_distance_controlled = 0;
  robotbase_current_speed = 0;
  robotbase_remote_mode = RB_REMOTE_STOP;
}

static void robotbase_remote_apply(const uint8_t mode, const float speed_mm_s) {
  if (rb_remote_apply_busy) return; // Prevent re-entry via quickstop->idle()->task()
  if (mode == robotbase_remote_mode && robotbase_stepflage && speed_mm_s == robotbase_current_speed) return;

  rb_remote_apply_busy = true;
  // Set target mode first so nested task() calls (during quickstop) won't retrigger mode switch.
  robotbase_remote_mode = mode;

  switch (mode) {
    case RB_REMOTE_FB_FWD:
      mecanum_restart_continuous(-MECANUM_CONTINUOUS_MM,  MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM,  MECANUM_CONTINUOUS_MM, speed_mm_s, false);
      break;
    case RB_REMOTE_FB_BWD:
      mecanum_restart_continuous( MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM,  MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, speed_mm_s, false);
      break;
    case RB_REMOTE_SLIDE_L:
    case RB_REMOTE_RAY_L:
      mecanum_restart_continuous(-MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM,  MECANUM_CONTINUOUS_MM,  MECANUM_CONTINUOUS_MM, speed_mm_s, false);
      break;
    case RB_REMOTE_SLIDE_R:
    case RB_REMOTE_RAY_R:
      mecanum_restart_continuous( MECANUM_CONTINUOUS_MM,  MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, speed_mm_s, false);
      break;
    case RB_REMOTE_ROT_L:
      mecanum_restart_continuous( MECANUM_CONTINUOUS_MM,  MECANUM_CONTINUOUS_MM,  MECANUM_CONTINUOUS_MM,  MECANUM_CONTINUOUS_MM, speed_mm_s, false);
      break;
    case RB_REMOTE_ROT_R:
      mecanum_restart_continuous(-MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, speed_mm_s, false);
      break;
    default:
      robotbase_remote_stop();
      rb_remote_apply_busy = false;
      return;
  }
  rb_remote_apply_busy = false;
}

void mecanum_robotbase_init() {
  rb_pwm_center_fb = RB_PWM_CENTER_FB_US;
  rb_pwm_center_lr = RB_PWM_CENTER_LR_US;
  rb_pwm_center_rot = RB_PWM_CENTER_ROT_US;
  rb_pwm_center_ray = RB_PWM_CENTER_RAY_US;
  rb_pwm_center_ready = !RB_PWM_AUTO_CENTER;
  rb_pwm_center_until_ms = millis() + RB_PWM_AUTO_CENTER_MS;
  rb_pwm_center_sum[RB_CH_FB] = rb_pwm_center_sum[RB_CH_LR] = rb_pwm_center_sum[RB_CH_ROT] = rb_pwm_center_sum[RB_CH_RAY] = 0;
  rb_pwm_center_cnt[RB_CH_FB] = rb_pwm_center_cnt[RB_CH_LR] = rb_pwm_center_cnt[RB_CH_ROT] = rb_pwm_center_cnt[RB_CH_RAY] = 0;

  #ifdef RB_REMOTE_FB_PIN
    if (RB_REMOTE_FB_PIN >= 0) {
      SET_INPUT(RB_REMOTE_FB_PIN);
      attachInterrupt(digitalPinToInterrupt(RB_REMOTE_FB_PIN), rb_isr_fb, CHANGE);
    }
  #endif
  #ifdef RB_REMOTE_LR_PIN
    if (RB_REMOTE_LR_PIN >= 0) {
      SET_INPUT(RB_REMOTE_LR_PIN);
      attachInterrupt(digitalPinToInterrupt(RB_REMOTE_LR_PIN), rb_isr_lr, CHANGE);
    }
  #endif
  #ifdef RB_REMOTE_ROT_PIN
    if (RB_REMOTE_ROT_PIN >= 0) {
      SET_INPUT(RB_REMOTE_ROT_PIN);
      attachInterrupt(digitalPinToInterrupt(RB_REMOTE_ROT_PIN), rb_isr_rot, CHANGE);
    }
  #endif
  #ifdef RB_RAY_TRACK_PIN
    if (RB_RAY_TRACK_PIN >= 0) {
      SET_INPUT(RB_RAY_TRACK_PIN);
      attachInterrupt(digitalPinToInterrupt(RB_RAY_TRACK_PIN), rb_isr_ray, CHANGE);
    }
  #endif
}

void mecanum_robotbase_task() {
  static millis_t next_poll_ms = 0;
  if (!ELAPSED(millis(), next_poll_ms)) return;
  next_poll_ms = millis() + RB_ANALOG_POLL_MS;

  const int pwm_low_fb = rb_pwm_center_fb - RB_PWM_DEADBAND_FB_US;
  const int pwm_high_fb = rb_pwm_center_fb + RB_PWM_DEADBAND_FB_US;
  const int pwm_low_lr = rb_pwm_center_lr - RB_PWM_DEADBAND_LR_US;
  const int pwm_high_lr = rb_pwm_center_lr + RB_PWM_DEADBAND_LR_US;
  const int pwm_low_rot = rb_pwm_center_rot - RB_PWM_DEADBAND_ROT_US;
  const int pwm_high_rot = rb_pwm_center_rot + RB_PWM_DEADBAND_ROT_US;
  const int pwm_low_ray = rb_pwm_center_ray - RB_PWM_DEADBAND_RAY_US;
  const int pwm_high_ray = rb_pwm_center_ray + RB_PWM_DEADBAND_RAY_US;
  constexpr float remote_speed_mm_s = RB_REMOTE_SPEED_MM_S;
  constexpr float ray_speed_mm_s = RB_RAY_CORRECT_SPEED_MM_S;

  rb_pwm_fb = rb_pwm_us[RB_CH_FB];
  rb_pwm_lr = rb_pwm_us[RB_CH_LR];
  rb_pwm_rot = rb_pwm_us[RB_CH_ROT];
  rb_pwm_ray = rb_pwm_us[RB_CH_RAY];
  const uint32_t now_us = micros();
  const uint32_t timeout_us = uint32_t(RB_PWM_SIGNAL_TIMEOUT_MS) * 1000UL;
  const bool fb_fresh = (now_us - rb_pwm_last_valid_us[RB_CH_FB]) <= timeout_us;
  const bool lr_fresh = (now_us - rb_pwm_last_valid_us[RB_CH_LR]) <= timeout_us;
  const bool rot_fresh = (now_us - rb_pwm_last_valid_us[RB_CH_ROT]) <= timeout_us;
  const bool ray_fresh = (now_us - rb_pwm_last_valid_us[RB_CH_RAY]) <= timeout_us;
  if (!fb_fresh) rb_pwm_fb = rb_pwm_center_fb;
  if (!lr_fresh) rb_pwm_lr = rb_pwm_center_lr;
  if (!rot_fresh) rb_pwm_rot = rb_pwm_center_rot;
  if (!ray_fresh) rb_pwm_ray = rb_pwm_center_ray;

  if (!rb_pwm_center_ready && RB_PWM_AUTO_CENTER) {
    const millis_t ms = millis();
    if (ELAPSED(ms, rb_pwm_center_until_ms)) {
      if (rb_pwm_center_cnt[RB_CH_FB] >= RB_PWM_AUTO_CENTER_MIN_SAMPLES) rb_pwm_center_fb = int(rb_pwm_center_sum[RB_CH_FB] / rb_pwm_center_cnt[RB_CH_FB]);
      if (rb_pwm_center_cnt[RB_CH_LR] >= RB_PWM_AUTO_CENTER_MIN_SAMPLES) rb_pwm_center_lr = int(rb_pwm_center_sum[RB_CH_LR] / rb_pwm_center_cnt[RB_CH_LR]);
      if (rb_pwm_center_cnt[RB_CH_ROT] >= RB_PWM_AUTO_CENTER_MIN_SAMPLES) rb_pwm_center_rot = int(rb_pwm_center_sum[RB_CH_ROT] / rb_pwm_center_cnt[RB_CH_ROT]);
      if (rb_pwm_center_cnt[RB_CH_RAY] >= RB_PWM_AUTO_CENTER_MIN_SAMPLES) rb_pwm_center_ray = int(rb_pwm_center_sum[RB_CH_RAY] / rb_pwm_center_cnt[RB_CH_RAY]);
      rb_pwm_center_ready = true;
    } else {
      if (rb_pwm_fb >= 700 && rb_pwm_fb <= 2500) { rb_pwm_center_sum[RB_CH_FB] += rb_pwm_fb; rb_pwm_center_cnt[RB_CH_FB]++; }
      if (rb_pwm_lr >= 700 && rb_pwm_lr <= 2500) { rb_pwm_center_sum[RB_CH_LR] += rb_pwm_lr; rb_pwm_center_cnt[RB_CH_LR]++; }
      if (rb_pwm_rot >= 700 && rb_pwm_rot <= 2500) { rb_pwm_center_sum[RB_CH_ROT] += rb_pwm_rot; rb_pwm_center_cnt[RB_CH_ROT]++; }
      if (rb_pwm_ray >= 700 && rb_pwm_ray <= 2500) { rb_pwm_center_sum[RB_CH_RAY] += rb_pwm_ray; rb_pwm_center_cnt[RB_CH_RAY]++; }
    }
  }

  // Hold still during auto-center window to avoid false motion at startup.
  if (!rb_pwm_center_ready) {
    robotbase_remote_stop();
    return;
  }

  if (rb_pwm_stream_enabled) {
    static millis_t next_stream_ms = 0;
    if (ELAPSED(millis(), next_stream_ms)) {
      report_pwm_snapshot(false);
      next_stream_ms = millis() + RB_PWM_STREAM_INTERVAL_MS;
    }
  }

  // Serial command has priority over analog remote control.
  // Keep PWM debug output running even when serial motion is active.
  if (robotbase_serial_controlled) return;

  apply_robotbase_motion_profile();

  // Inputs not yet captured / invalid: avoid accidental movement.
  if (rb_pwm_fb == 0 && rb_pwm_lr == 0 && rb_pwm_rot == 0 && rb_pwm_ray == 0) {
    robotbase_remote_stop();
    return;
  }

  const bool fb_fwd = RB_REMOTE_ENABLE_FB && fb_fresh && rb_pwm_fb > pwm_high_fb, fb_bwd = RB_REMOTE_ENABLE_FB && fb_fresh && rb_pwm_fb < pwm_low_fb;
  const bool slide_r = RB_REMOTE_ENABLE_LR && lr_fresh && rb_pwm_lr > pwm_high_lr, slide_l = RB_REMOTE_ENABLE_LR && lr_fresh && rb_pwm_lr < pwm_low_lr;
  const bool rot_r = RB_REMOTE_ENABLE_ROT && rot_fresh && rb_pwm_rot > pwm_high_rot, rot_l = RB_REMOTE_ENABLE_ROT && rot_fresh && rb_pwm_rot < pwm_low_rot;

  if (fb_fwd || fb_bwd) {
    if (robotbase_raytracing_enabled && RB_REMOTE_ENABLE_RAY && ray_fresh) {
      if (rb_pwm_ray > pwm_high_ray) { robotbase_remote_apply(RB_REMOTE_RAY_R, ray_speed_mm_s); return; }
      if (rb_pwm_ray < pwm_low_ray)  { robotbase_remote_apply(RB_REMOTE_RAY_L, ray_speed_mm_s); return; }
    }
    robotbase_remote_apply(fb_fwd ? RB_REMOTE_FB_FWD : RB_REMOTE_FB_BWD, remote_speed_mm_s);
    return;
  }

  if (slide_l || slide_r) {
    robotbase_remote_apply(slide_l ? RB_REMOTE_SLIDE_L : RB_REMOTE_SLIDE_R, remote_speed_mm_s);
    return;
  }

  if (rot_l || rot_r) {
    robotbase_remote_apply(rot_l ? RB_REMOTE_ROT_L : RB_REMOTE_ROT_R, remote_speed_mm_s);
    return;
  }

  robotbase_remote_stop();
}

static bool is_robotbase_command(const char *cmd) {
  if (!cmd || !*cmd) return false;
  char c = cmd[0];
  if ((c == 'A' && strncmp(cmd, "ADC", 3) == 0) || (c == 'P' && strncmp(cmd, "PWM", 3) == 0)) return true;
  if (strcmp(cmd, "PWMON") == 0 || strcmp(cmd, "PWMOFF") == 0) return true;
  if (c == 'S') {
    if (cmd[1] == '\0') return true; // S
    if (cmd[1] == 'T' && strncmp(cmd, "STATUS", 6) == 0) return true;
    if (cmd[1] == 'D' && (cmd[2] == 'L' || cmd[2] == 'R')) return true; // SDL, SDR
    if (cmd[1] == 'L' || cmd[1] == 'R') return true; // SL, SR
  }
  if (c == 'F' || c == 'B' || c == 'L' || c == 'R') return true;
  if (strcmp(cmd, "FINDRAY") == 0 || strcmp(cmd, "DISABLERAY") == 0) return true;
  if (strcmp(cmd, "ID") == 0 || strcmp(cmd, "?") == 0) return true;
  return false;
}

bool process_robotbase_command(char *command) {
  if (!command || !*command) return false;
  while (*command == ' ') command++;
  if (!is_robotbase_command(command)) return false;
  apply_robotbase_motion_profile();

  // ID / ? : reply identity only (no ACK per README)
  if (strcmp(command, "ID") == 0 || strcmp(command, "?") == 0) {
    SERIAL_ECHOLNPGM("robotbase");
    return true;
  }

  // S - stop (highest priority)
  if (strcmp(command, "S") == 0) {
    reply_ack();
    quickstop_stepper();
    robotbase_stepflage = 0;
    robotbase_serial_controlled = 0;
    robotbase_distance_controlled = 0;
    reply_ok();
    return true;
  }

  reply_ack();

  // STATUS
  if (strncmp(command, "STATUS", 6) == 0) {
    const int pwm_low_fb = rb_pwm_center_fb - RB_PWM_DEADBAND_FB_US;
    const int pwm_high_fb = rb_pwm_center_fb + RB_PWM_DEADBAND_FB_US;
    const int pwm_low_lr = rb_pwm_center_lr - RB_PWM_DEADBAND_LR_US;
    const int pwm_high_lr = rb_pwm_center_lr + RB_PWM_DEADBAND_LR_US;
    const int pwm_low_rot = rb_pwm_center_rot - RB_PWM_DEADBAND_ROT_US;
    const int pwm_high_rot = rb_pwm_center_rot + RB_PWM_DEADBAND_ROT_US;
    const int pwm_low_ray = rb_pwm_center_ray - RB_PWM_DEADBAND_RAY_US;
    const int pwm_high_ray = rb_pwm_center_ray + RB_PWM_DEADBAND_RAY_US;
    SERIAL_ECHOPGM("STATUS:stepflage=");
    SERIAL_ECHO(robotbase_stepflage);
    SERIAL_ECHOPGM(",isSerialControlled=");
    SERIAL_ECHO(robotbase_serial_controlled);
    SERIAL_ECHOPGM(",isDistanceControlled=");
    SERIAL_ECHO(robotbase_distance_controlled);
    SERIAL_ECHOPGM(",currentSpeed=");
    SERIAL_ECHO(robotbase_current_speed);
    SERIAL_ECHOPGM(",isRaytracingEnabled=");
    SERIAL_ECHO(robotbase_raytracing_enabled);
    SERIAL_ECHOPGM(",isCentered=");
    SERIAL_ECHO(robotbase_centered);
    SERIAL_ECHOPGM(",pwmFB=");
    SERIAL_ECHO(rb_pwm_fb);
    SERIAL_ECHOPGM(",pwmLR=");
    SERIAL_ECHO(rb_pwm_lr);
    SERIAL_ECHOPGM(",pwmROT=");
    SERIAL_ECHO(rb_pwm_rot);
    SERIAL_ECHOPGM(",pwmRAY=");
    SERIAL_ECHO(rb_pwm_ray);
    SERIAL_ECHOPGM(",pwmFBLow=");
    SERIAL_ECHO(pwm_low_fb);
    SERIAL_ECHOPGM(",pwmFBHigh=");
    SERIAL_ECHO(pwm_high_fb);
    SERIAL_ECHOPGM(",pwmLRLow=");
    SERIAL_ECHO(pwm_low_lr);
    SERIAL_ECHOPGM(",pwmLRHigh=");
    SERIAL_ECHO(pwm_high_lr);
    SERIAL_ECHOPGM(",pwmROTLow=");
    SERIAL_ECHO(pwm_low_rot);
    SERIAL_ECHOPGM(",pwmROTHigh=");
    SERIAL_ECHO(pwm_high_rot);
    SERIAL_ECHOPGM(",pwmRAYLow=");
    SERIAL_ECHO(pwm_low_ray);
    SERIAL_ECHOPGM(",pwmRAYHigh=");
    SERIAL_ECHO(pwm_high_ray);
    SERIAL_ECHOPGM(",pwmStream=");
    SERIAL_ECHO(int(rb_pwm_stream_enabled));
    SERIAL_ECHOPGM(",autoCenter=");
    SERIAL_ECHOLN(int(rb_pwm_center_ready));
    return true;
  }

  // PWM stream control
  if (strcmp(command, "PWMON") == 0) {
    rb_pwm_stream_enabled = 1;
    rb_pwm_updates[RB_CH_FB] = rb_pwm_updates[RB_CH_LR] = rb_pwm_updates[RB_CH_ROT] = rb_pwm_updates[RB_CH_RAY] = 0;
    SERIAL_ECHOLNPGM("pwm stream on");
    return true;
  }
  if (strcmp(command, "PWMOFF") == 0) {
    rb_pwm_stream_enabled = 0;
    SERIAL_ECHOLNPGM("pwm stream off");
    return true;
  }

  // PWM / ADC(legacy) - print raw pulse widths and thresholds
  if (strncmp(command, "PWM", 3) == 0 || strncmp(command, "ADC", 3) == 0) {
    report_pwm_snapshot(true);
    return true;
  }

  // FINDRAY / DISABLERAY
  if (strcmp(command, "FINDRAY") == 0) {
    robotbase_raytracing_enabled = 1;
    SERIAL_ECHOLNPGM("raytracking enabled");
    return true;
  }
  if (strcmp(command, "DISABLERAY") == 0) {
    robotbase_raytracing_enabled = 0;
    SERIAL_ECHOLNPGM("raytracking disabled");
    return true;
  }

  char cmd_char = command[0];
  float speed = MECANUM_DEFAULT_SPEED_MM_S;
  float distance = 0.0f;
  bool distance_cmd = false;
  bool is_angle = false; // LD, RD

  const char *p = command + 1;
  if (cmd_char == 'S' && command[1] == 'D' && (command[2] == 'L' || command[2] == 'R')) {
    distance_cmd = true;
    p = command + 3;
  } else if (cmd_char == 'F' && command[1] == 'D') {
    distance_cmd = true;
    p = command + 2;
  } else if (cmd_char == 'B' && command[1] == 'D') {
    distance_cmd = true;
    p = command + 2;
  } else if (cmd_char == 'L' && command[1] == 'D') {
    distance_cmd = true;
    is_angle = true;
    p = command + 2;
  } else if (cmd_char == 'R' && command[1] == 'D') {
    distance_cmd = true;
    is_angle = true;
    p = command + 2;
  } else if (cmd_char == 'S' && (command[1] == 'L' || command[1] == 'R')) {
    p = command + 2;
  } else if (cmd_char == 'F' || cmd_char == 'B' || cmd_char == 'L' || cmd_char == 'R') {
    p = command + 1;
  }

  if (*p) {
    const char *colon = strchr(p, ':');
    if (distance_cmd && colon) {
      char *end;
      distance = is_angle ? clamp_angle(strtof(p, &end)) : clamp_distance(strtof(p, &end));
      speed = clamp_speed(strtof(colon + 1, &end));
    } else if (distance_cmd) {
      char *end;
      distance = is_angle ? clamp_angle(strtof(p, &end)) : clamp_distance(strtof(p, &end));
    } else {
      char *end;
      speed = clamp_speed(strtof(p, &end));
    }
  }

  if (distance_cmd && distance <= 0 && !is_angle) return true;
  if (distance_cmd && is_angle && distance <= 0) return true;

  robotbase_serial_controlled = 1;
  robotbase_current_speed = speed;

  // Convert rotation angle to wheel travel (mm)
  float d = distance;
  if (is_angle) {
    d = (MECANUM_ROTATION_RADIUS_MM * 2.0f * MECANUM_PI) * (distance / 360.0f);
  }

  switch (cmd_char) {
    case 'F':
      if (distance_cmd) {
        robotbase_stepflage = 1;
        robotbase_distance_controlled = 1;
        mecanum_move_distance(-d, d, -d, d, speed);
      } else {
        mecanum_restart_continuous(-MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, speed);
      }
      break;
    case 'B':
      if (distance_cmd) {
        robotbase_stepflage = 1;
        robotbase_distance_controlled = 1;
        mecanum_move_distance(d, -d, d, -d, speed);
      } else {
        mecanum_restart_continuous(MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, speed);
      }
      break;
    case 'L':
      if (distance_cmd && is_angle) {
        robotbase_stepflage = 1;
        robotbase_distance_controlled = 1;
        mecanum_move_distance(d, d, d, d, speed);
      } else if (distance_cmd) {
        float lateral = d / MECANUM_TRANSFORM_FACTOR;
        robotbase_stepflage = 1;
        robotbase_distance_controlled = 1;
        mecanum_move_distance(-lateral, -lateral, lateral, lateral, speed);
      } else {
        // L / R are turn commands (rotation in place), not slides.
        mecanum_restart_continuous(MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, speed);
      }
      break;
    case 'R':
      if (distance_cmd && is_angle) {
        robotbase_stepflage = 1;
        robotbase_distance_controlled = 1;
        mecanum_move_distance(-d, -d, -d, -d, speed);
      } else if (distance_cmd) {
        float lateral = d / MECANUM_TRANSFORM_FACTOR;
        robotbase_stepflage = 1;
        robotbase_distance_controlled = 1;
        mecanum_move_distance(lateral, lateral, -lateral, -lateral, speed);
      } else {
        // R is right turn (rotation in place).
        mecanum_restart_continuous(-MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, speed);
      }
      break;
    case 'S':
      if (command[1] == 'D' && command[2] == 'L') {
        float lateral = d / MECANUM_TRANSFORM_FACTOR;
        robotbase_stepflage = 1;
        robotbase_distance_controlled = 1;
        mecanum_move_distance(-lateral, -lateral, lateral, lateral, speed);
      } else if (command[1] == 'D' && command[2] == 'R') {
        float lateral = d / MECANUM_TRANSFORM_FACTOR;
        robotbase_stepflage = 1;
        robotbase_distance_controlled = 1;
        mecanum_move_distance(lateral, lateral, -lateral, -lateral, speed);
      } else if (command[1] == 'L') {
        mecanum_restart_continuous(-MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, speed);
      } else if (command[1] == 'R') {
        mecanum_restart_continuous(MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, speed);
      }
      break;
    default:
      break;
  }
  return true;
}

#endif // MECANUM_ROBOTBASE
