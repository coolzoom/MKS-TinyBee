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

#endif // MECANUM_ROBOTBASE
