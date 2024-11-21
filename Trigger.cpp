/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <Trigger.h>
#include <vector>
#include <iostream>

Trigger::Trigger(std::shared_ptr<asdp::SpinFreeAccurateTimer<std::chrono::steady_clock::time_point> > timer)
  : m_timer(timer)
{
  // Create a condition variable for the trigger.
  m_queue = std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >();

  // Start our thread to handle the triggering.
  m_thread = std::thread(&Trigger::TriggerThread, this);
  while (!m_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void Trigger::Configure(uint8_t Mode, double Period, float TrackingFactor)
{
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  uint8_t oldMode = m_mode;
  m_mode = Mode;
  m_period = Period;
  m_trackingFactor = TrackingFactor;

  // See if we need to request a trigger depending on the mode.
  /// @todo In the case where the trigger had a very long period and we're setting it to a short
  /// period, this may take an entire old period to come into effect.  So we need to replace the
  /// old entry if there is one.  Unfortunately, the timer may be handling multiple triggers, so
  /// we can't be sure to get the correct entry.
  if ((m_mode & 0x01) && !(oldMode & 0x01)) {
    // We're a periodic trigger, so request a trigger at the specified interval in the future unless
    // we were already periodic.
    std::chrono::steady_clock::time_point nextTriggerTime = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(m_period));
    m_timer->AddEntry(nextTriggerTime, nextTriggerTime, m_queue);
    m_lastRequestedTriggerTime = nextTriggerTime;
  }
}

Trigger::~Trigger()
{
  m_stop = true;
  m_thread.join();
}

void Trigger::RegisterClient(std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > queue)
{
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  m_clients.push_back(queue);
}

bool Trigger::UnregisterClient(std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > queue)
{
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (auto client : m_clients) {
    if (client.get() == queue.get()) {
      m_clients.remove(client);
      return true;
    }
  }
  return false;
}

bool Trigger::Fire(std::chrono::steady_clock::time_point time)
{
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  // If we are disabled or configured as a hardware trigger, do nothing.
  if ((m_mode == 0) || ((m_mode & 0x04) != 0)) {
    return false;
  }

  if (!(m_mode & 0x01)) {
    // We are a one-shot trigger; we need to request a trigger at the specified time and we're done.
    m_timer->AddEntry(time, time, m_queue);
  } else {
    // We're a periodic trigger, so we need to adjust the time to the next trigger based on the tracking factor
    // and the relative phase of the requested time and the ongoing sequence of triggers.
    // We first find the relative phase of the requested time and the ongoing sequence of triggers, in
    // terms of the change in time from the last requested trigger time (in the range -half a period to
    // +half a period).
    // We then scale the offset by the tracking factor and add it to the last requested time (a factor
    // of 1 will shift all the way, a factor of 0 will not adjust it) to shift it to where we would
    // have liked it to be.
    double offset = std::chrono::duration<double>(time - m_lastRequestedTriggerTime).count();
    long periods = static_cast<long>(offset / m_period);
    double shift = offset - periods * m_period;
    // Don't shift more than half an interval into the past, so we don't miss a trigger.
    if (shift < -0.5 * m_period) {
      shift += m_period;
    }
    double adjustedShift = shift * m_trackingFactor;
    std::chrono::steady_clock::time_point nextTriggerTime = m_lastRequestedTriggerTime +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(adjustedShift));
    m_lastRequestedTriggerTime = nextTriggerTime;
  }

  return true;
}

void Trigger::TriggerThread()
{
  m_running = true;
  while (!m_stop && m_queue) {
    // See if we have been triggered. Timeout so that we can shut down even
    // if we are not triggered.
    std::chrono::steady_clock::time_point triggerTime;
    // Handle only one trigger so that we always check for being done.
    if (m_queue->dequeue(triggerTime, std::chrono::milliseconds(10))) {
      // At this point, we have been triggered.  We need to notify all clients.
      // We're going to look at our internal state, so we need to lock.
      std::lock_guard<std::recursive_mutex> lock(m_mutex);
      for (auto client : m_clients) {
        client->enqueue(triggerTime);
      }

      // See if we need to request a new trigger depending on the mode.
      if (m_mode & 0x01) {
        // We're a periodic trigger, so request a new one.
        std::chrono::steady_clock::time_point nextTriggerTime = m_lastRequestedTriggerTime +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(m_period));
        m_timer->AddEntry(nextTriggerTime, nextTriggerTime, m_queue);

        // Update the last trigger time
        m_lastRequestedTriggerTime = nextTriggerTime;
      }
    }
  }
}

std::string Trigger::Test()
{
  // Timer to use for our tests.
  std::shared_ptr< asdp::SpinFreeAccurateTimer<std::chrono::steady_clock::time_point> > timer(
    new asdp::SpinFreeAccurateTimer< std::chrono::steady_clock::time_point>());

  // Create a Trigger and check client registration and unregistration.
  {
    Trigger trigger(timer);
    std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > q1 =
      std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >();
    std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > q2 =
      std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >();
    trigger.RegisterClient(q1);
    trigger.RegisterClient(q2);
    if (trigger.UnregisterClient(q1) == false) {
      return "Failed to unregister client 1";
    }
    if (trigger.UnregisterClient(q1) == true) {
      return "Unregistered client 1 twice";
    }
    if (trigger.UnregisterClient(q2) == false) {
      return "Failed to unregister client 2";
    }
  }

  // Create a Trigger and try to fire it without configuring it.
  {
    Trigger trigger(timer);
    if (trigger.Fire(std::chrono::steady_clock::now()) == true) {
      return "Fired unconfigured trigger";
    }
  }

  // Create a Trigger and configure it as a one-shot software trigger.
  // Then register a client and fire it and make sure the trigger was received by the client.
  // Then configure as a hardware trigger and ensure that firing does not work.
  {
    Trigger trigger(timer);
    std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > queue =
      std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >();
    trigger.RegisterClient(queue);
    trigger.Configure(2, 0, 0);
    if (!trigger.Fire(std::chrono::steady_clock::now())) {
      return "Failed to fire one-shot software trigger";
    }
    std::chrono::steady_clock::time_point triggerTime;
    if (!queue->dequeue(triggerTime, std::chrono::milliseconds(500))) {
      return "Failed to receive one-shot software trigger";
    }
  }
  {
    Trigger trigger(timer);
    std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > queue =
      std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >();
    trigger.RegisterClient(queue);
    trigger.Configure(4, 0, 0);
    if (trigger.Fire(std::chrono::steady_clock::now())) {
      return "Fired one-shot hardware trigger";
    }
  }

  // Create a Trigger and configure it as a periodic trigger.  Then make sure that it fires 10
  // times.
  {
    Trigger trigger(timer);
    std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > queue =
      std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >();
    trigger.RegisterClient(queue);
    trigger.Configure(3, 1e-3, 1);
    for (int i = 0; i < 10; i++) {
      std::chrono::steady_clock::time_point triggerTime;
      if (!queue->dequeue(triggerTime, std::chrono::milliseconds(500))) {
        return "Failed to receive periodic software trigger";
      }
    }
  }
  {
    Trigger trigger(timer);
    std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > queue =
      std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >();
    trigger.RegisterClient(queue);
    trigger.Configure(5, 1e-3, 1);
    for (int i = 0; i < 10; i++) {
      std::chrono::steady_clock::time_point triggerTime;
      if (!queue->dequeue(triggerTime, std::chrono::milliseconds(500))) {
        return "Failed to receive periodic hardware trigger";
      }
    }
  }

  // Create a Trigger and configure it as a 60fps periodic trigger.  Then make sure that it fires
  // for two seconds.
  {
    Trigger trigger(timer);
    std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > queue =
      std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >();
    trigger.RegisterClient(queue);
    trigger.Configure(3, 1.0/60, 1);
    auto start = std::chrono::steady_clock::now();
    do {
      std::chrono::steady_clock::time_point triggerTime;
      if (!queue->dequeue(triggerTime, std::chrono::milliseconds(50))) {
        return "Failed to receive periodic software trigger over period of 2 seconds";
      }
    } while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2));
  }

  // Test the tracking factor and the ability of the system to shift phase based on software triggers.
  // We will configure a periodic trigger with a period of 10 ms and a tracking factor of 0.5.  We will
  // watch the reports for awhile to determine the offset from 0 time in period units and then start
  // firing the software trigger halfway into the period.  We should see the trigger shift towards the
  // desired offset.
  {
    // Configure the trigger with a period of 10ms and a tracking factor of 0.5.
    Trigger trigger(timer);
    std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > queue =
      std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >();
    trigger.RegisterClient(queue);
    float period = 10e-3;
    trigger.Configure(3, period, 0.5);

    // Find the average offset from 0 time in period units.
    double offsetFrac = 0;
    for (int i = 0; i < 10; i++) {
      std::chrono::steady_clock::time_point triggerTime;
      if (!queue->dequeue(triggerTime, std::chrono::milliseconds(500))) {
        return "Failed to receive periodic software trigger";
      }
      std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
      double periods = std::chrono::duration<double>(now.time_since_epoch()).count() / period;
      offsetFrac += periods - static_cast<long>(periods);
    }
    offsetFrac /= 10;
    double desiredOffsetFrac = offsetFrac + 0.5;
    if (desiredOffsetFrac > 1) {
      desiredOffsetFrac -= 1;
    }

    // Now fire the trigger at the desired offset fraction times the period and track the offset.
    // Do this relative to the offset from a particular trigger and send the same time for multiple
    // iterations to see how the offset changes.
    {
      std::chrono::steady_clock::time_point triggerTime;
      if (!queue->dequeue(triggerTime, std::chrono::milliseconds(500))) {
        return "Failed to receive periodic software trigger";
      }
    }
    std::chrono::steady_clock::time_point time = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(period/2));
    std::vector<double> offsetFracs;
    for (size_t j = 0; j < 10; j++) {
      // Send a trigger request at the desired offset fraction times the period.
      trigger.Fire(time);

      // Track how the offset changes
      double of = 0;
      for (int i = 0; i < 10; i++) {
        std::chrono::steady_clock::time_point triggerTime;
        if (!queue->dequeue(triggerTime, std::chrono::milliseconds(500))) {
          return "Failed to receive periodic software trigger";
        }
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        double periods = std::chrono::duration<double>(now.time_since_epoch()).count() / period;
        of += periods - static_cast<long>(periods);
      }
      offsetFracs.push_back(of / 10);
    }

    // We expect the offset to converge to close to the desired offset, which is around 0.5 periods
    // away from the original. The first adustment should be further from the target than the last
    // and the last should be quite close.
    if (std::abs(offsetFracs[0] - desiredOffsetFrac) <= std::abs(offsetFracs[9] - desiredOffsetFrac)) {
      return "Initial offset was not further from desired offset than final";
    }
    if (std::abs(offsetFracs[9] - desiredOffsetFrac) > 0.05) {
      return "Final offset was not close to desired offset";
    }
  }

  // All the tests passed.
  return "";
}
