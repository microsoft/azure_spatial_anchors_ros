#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <system_error>

template <typename T>
class WaitForValue {
  std::mutex m_mutex;
  std::condition_variable m_conditionVariable;
  T m_value;
  bool m_set = false;

 public:
  T Wait(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(m_mutex);
    if (!m_conditionVariable.wait_for(lock, timeout,
                                      [this]() { return m_set; })) {
      throw std::runtime_error("Timeout.");
    }

    return m_value;
  }

  void Set(T value) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_set = true;
    m_value = std::move(value);
    m_conditionVariable.notify_all();
  }
};
