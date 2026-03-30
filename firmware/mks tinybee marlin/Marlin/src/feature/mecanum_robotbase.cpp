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

static uint8_t robotbase_stepflage = 0;   // 0=stopped, 1=moving
static uint8_t robotbase_serial_controlled = 0;
static uint8_t robotbase_distance_controlled = 0;
static float   robotbase_current_speed = 0.0f;
static uint8_t robotbase_raytracing_enabled = 1;
static uint8_t robotbase_centered = 0;
static bool    robotbase_profile_applied = false;

static void reply_ack() { SERIAL_ECHOLNPGM("ACK"); }
static void reply_ok()  { SERIAL_ECHOLNPGM("ok"); }

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
static void mecanum_move_continuous(float dx, float dy, float dz, float da, float speed_mm_s) {
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
  robotbase_serial_controlled = 1;
  robotbase_distance_controlled = 0;
  robotbase_current_speed = speed_mm_s;
}

// Restart continuous movement so updated speed takes effect immediately.
static void mecanum_restart_continuous(float dx, float dy, float dz, float da, float speed_mm_s) {
  if (robotbase_stepflage) quickstop_stepper();
  mecanum_move_continuous(dx, dy, dz, da, speed_mm_s);
}

static bool is_robotbase_command(const char *cmd) {
  if (!cmd || !*cmd) return false;
  char c = cmd[0];
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
    SERIAL_ECHOLN(robotbase_centered);
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
      distance = clamp_distance(strtof(p, &end));
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
        mecanum_restart_continuous(-MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, speed);
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
        mecanum_restart_continuous(MECANUM_CONTINUOUS_MM, MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, -MECANUM_CONTINUOUS_MM, speed);
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
