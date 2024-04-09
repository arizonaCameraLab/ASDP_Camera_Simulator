/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */
 
/**
* @file Trigger.h
* @brief Apache Strap-Down Pilotage BSP helper class to implement triggers.
*
* @author ReliaSolve.
* @date April 9th, 2024.
*/

#pragma once

#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <list>
#include <string>
#include <ASDP_SpinFreeAccurateTimer.h>

/// @brief Implements Trigger behavior.
/// @details This class handles the firing of triggers.  It will be used to implement both
/// software and hardware triggers.  It includes both one-shot and periodic triggers.
/// Each trigger can be tied to zero or one software triggers. There are no external
/// hardware triggers in this implementation, so they are not checked for and anything
/// tied to one behaves as if it were unsynchronized.
class Trigger {
public:
  /// @brief Construct a trigger object.
  /// @param timer The timer object to avoid spin-waiting inside every thread.
  Trigger(std::shared_ptr<asdp::SpinFreeAccurateTimer> timer);

  /// @brief Destroy a trigger object.
  ~Trigger();

  /// @brief Configure the trigger.
  /// @param Mode The mode of the trigger: 0 = disabled, 1 = unsynchronized, 2 = one-shot
  /// software, 3 = periodic software, 4 = one - shot hardware, 5 = periodic hardware.
  /// @param Period The period of the trigger in seconds.
  /// @param TrackingFactor The tracking factor for the trigger.
  void Configure(uint8_t Mode, float Period, float TrackingFactor);

  /// @brief Register a client that will be notified when the trigger fires.
  /// @param cv The condition variable to register.
  void RegisterClient(std::shared_ptr<std::condition_variable> cv);

  /// @brief Unregister a client that will be notified when the trigger fires.
  /// @param cv The condition variable to unregister.
  /// @return true if the client was found and unregistered, false otherwise.
  bool UnregisterClient(std::shared_ptr<std::condition_variable> cv);

  /// @brief Fire the trigger, presumably based on a software trigger request.
  /// @details This function is called when a trigger should be fired.  For periodic
  /// triggers, it will cause the time to adjust according to the tracking factor.
  /// This function does not need to be called for periodic triggers, as they will
  /// fire automatically.  It does need to be called for one-shot triggers.  If it is
  /// called for periodic triggers, it will adjust the time to the next trigger.
  /// @param time The time at which the trigger should be fired.
  /// @return true if the trigger was fired, false otherwise.
  bool Fire(std::chrono::steady_clock::time_point time);

  /// @brief Return the period of the trigger.
  /// @return The period of the trigger in seconds.
  float GetPeriod() const { return m_period; }

  /// @brief Test the trigger class.
  /// @return An empty string if the test passes, an error message otherwise.
  static std::string Test();

protected:
  // Configuration parameters for the trigger.
  uint8_t m_mode = 0;                 ///< The mode of the trigger.
  float m_period = 0;                 ///< The period of the trigger in seconds.
  float m_trackingFactor = 1;         ///< The tracking factor for the trigger.
  std::chrono::steady_clock::time_point m_lastRequestedTriggerTime = {}; ///< The last time the trigger was requested.

  /// The timer object to avoid spin-waiting inside every thread.
  std::shared_ptr<asdp::SpinFreeAccurateTimer> m_timer;

  /// The condition variable to signal when the trigger is fired.
  std::shared_ptr<std::condition_variable> m_condition;

  /// Clients who are registered with us.  Each is a condition variable to be signaled.
  std::list< std::shared_ptr<std::condition_variable> > m_clients;

  /// The mutex to protect the condition variable and other internal state.
  std::mutex m_mutex;

  /// Variable to tell the thread to stop.
  std::atomic<bool> m_stop = false;

  /// Thread to handle the triggering.
  std::thread m_thread;
  std::atomic_bool m_running = false; ///< Used to wait for the thread to start.

  /// Implements the trigger thread.
  void TriggerThread();
};
