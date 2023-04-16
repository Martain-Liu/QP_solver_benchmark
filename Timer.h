#ifndef TIME_H_
#define TIME_H_

#include <assert.h>
#include <stdint.h>
#include <time.h>

/*!
 * Timer for measuring time elapsed with clock_monotonic
 */
class Timer {
 public:
  /*!
   * Construct and start timer
   */
  explicit Timer() { Start(); }

  /*!
   * Start the timer
   */
  void Start() { clock_gettime(CLOCK_MONOTONIC, &_startTime); }

  /*!
   * Get milliseconds elapsed
   */
  double GetMs() const { return (double)GetNs() / 1.e6; }

  /*!
   * Get nanoseconds elapsed
   */
  int64_t GetNs() const {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)(now.tv_nsec - _startTime.tv_nsec) + 1000000000 * (now.tv_sec - _startTime.tv_sec);
  }

  /*!
   * Get seconds elapsed
   */
  double GetSeconds() { return (double)GetNs() / 1.e9; }

  struct timespec _startTime;
};

#endif  // PX_QUADRUPED_MODULES_SAFE_CHECK_TIMER_H_
