/*
 * log_capture.h -- the HAL log sink for host tests
 */

#ifndef LOG_CAPTURE_H
#define LOG_CAPTURE_H

typedef struct {
    int warns;
    int level;
    char msg[256];
} log_capture_t;

extern log_capture_t logged;

void log_capture_reset(void);

#endif
