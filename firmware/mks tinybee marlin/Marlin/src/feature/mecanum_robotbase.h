/**
 * Mecanum Robot Base - Serial protocol for Mecanum wheel car
 * XYZA axes map to four wheels. Protocol per firmware/robotbase/README.md
 */
#pragma once

#if ENABLED(MECANUM_ROBOTBASE)

/**
 * Process a line as robotbase protocol command.
 * Return true if the line was handled (do not enqueue to G-code).
 */
bool process_robotbase_command(char *command);

/**
 * Initialize optional robotbase peripherals (analog remote inputs).
 */
void mecanum_robotbase_init();

/**
 * Periodic robotbase task (remote sampling / motion arbitration).
 */
void mecanum_robotbase_task();

#endif // MECANUM_ROBOTBASE
