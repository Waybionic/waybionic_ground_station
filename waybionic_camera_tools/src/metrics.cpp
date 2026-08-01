#include "waybionic_camera_tools/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace waybionic_camera_tools
{

  ImageLatencyTracker::ImageLatencyTracker(
      const double expected_frame_rate,
      const double stale_timeout_sec,
      const std::size_t history_size,
      const double drop_gap_factor)
      : stale_timeout_sec_(stale_timeout_sec),
        history_size_(history_size),
        drop_gap_factor_(drop_gap_factor)
  {
    if (history_size_ == 0U)
    {
      throw std::invalid_argument("history_size must be positive");
    }
    if (stale_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("stale_timeout_sec must be positive");
    }
    if (drop_gap_factor_ < 1.0)
    {
      throw std::invalid_argument("drop_gap_factor must be at least 1.0");
    }

    if (expected_frame_rate > 0.0)
    {
      expected_period_ns_ = static_cast<std::int64_t>(1'000'000'000.0 / expected_frame_rate);
    }
  }

  FrameRecord ImageLatencyTracker::observe(
      const std::int64_t receive_ns,
      const std::optional<std::int64_t> capture_ns)
  {
    if (receive_ns < 0)
    {
      throw std::invalid_argument("receive_ns must be non-negative");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    ++total_frames_;

    const bool missing_header = !capture_ns.has_value() || capture_ns.value() <= 0;
    bool stale_header = false;
    bool negative_latency = false;
    std::optional<double> latency_ms;
    std::optional<double> capture_gap_ms;
    std::int64_t estimated_drops = 0;

    std::optional<double> receive_gap_ms;
    if (last_receive_ns_.has_value())
    {
      const auto gap_ns = std::max<std::int64_t>(0, receive_ns - last_receive_ns_.value());
      receive_gap_ms = static_cast<double>(gap_ns) / 1'000'000.0;
    }

    if (!missing_header)
    {
      const auto capture_value = capture_ns.value();
      const auto latency_ns = receive_ns - capture_value;
      negative_latency = latency_ns < 0;
      if (!negative_latency)
      {
        latency_ms = static_cast<double>(latency_ns) / 1'000'000.0;
      }

      if (last_capture_ns_.has_value())
      {
        const auto capture_delta_ns = capture_value - last_capture_ns_.value();
        stale_header = capture_delta_ns <= 0;
        if (!stale_header)
        {
          capture_gap_ms = static_cast<double>(capture_delta_ns) / 1'000'000.0;
          estimated_drops = estimate_drops(capture_delta_ns);
        }
      }

      if (!stale_header)
      {
        last_capture_ns_ = capture_value;
      }
    }

    FrameRecord record{
        receive_ns,
        capture_ns,
        latency_ms,
        receive_gap_ms,
        capture_gap_ms,
        estimated_drops,
        stale_header,
        missing_header,
        negative_latency};

    if (records_.size() == history_size_)
    {
      records_.pop_front();
    }
    records_.push_back(record);
    last_receive_ns_ = receive_ns;
    return record;
  }

  StreamSnapshot ImageLatencyTracker::snapshot(const std::int64_t now_ns) const
  {
    if (now_ns < 0)
    {
      throw std::invalid_argument("now_ns must be non-negative");
    }

    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<double> valid_latencies_ms;
    valid_latencies_ms.reserve(records_.size());

    std::int64_t estimated_dropped_frames = 0;
    std::size_t stale_or_repeated_frames = 0;
    std::size_t missing_header_frames = 0;
    std::size_t negative_latency_frames = 0;

    for (const auto &record : records_)
    {
      if (record.latency_ms.has_value())
      {
        valid_latencies_ms.push_back(record.latency_ms.value());
      }
      estimated_dropped_frames += record.estimated_drops;
      stale_or_repeated_frames += record.stale_header ? 1U : 0U;
      missing_header_frames += record.missing_header ? 1U : 0U;
      negative_latency_frames += record.negative_latency ? 1U : 0U;
    }

    std::optional<double> seconds_since_last_frame;
    std::string state = "waiting";
    if (last_receive_ns_.has_value())
    {
      const auto elapsed_ns = std::max<std::int64_t>(0, now_ns - last_receive_ns_.value());
      seconds_since_last_frame = static_cast<double>(elapsed_ns) / 1'000'000'000.0;
      state = seconds_since_last_frame.value() > stale_timeout_sec_ ? "stale" : "connected";
    }

    std::optional<double> latency_avg_ms;
    std::optional<double> latency_min_ms;
    std::optional<double> latency_max_ms;
    if (!valid_latencies_ms.empty())
    {
      const double sum = std::accumulate(valid_latencies_ms.begin(), valid_latencies_ms.end(), 0.0);
      latency_avg_ms = sum / static_cast<double>(valid_latencies_ms.size());
      const auto [minimum, maximum] = std::minmax_element(
          valid_latencies_ms.begin(), valid_latencies_ms.end());
      latency_min_ms = *minimum;
      latency_max_ms = *maximum;
    }

    return StreamSnapshot{
        state,
        total_frames_,
        records_.size(),
        valid_latencies_ms.size(),
        compute_frame_rate_hz(records_),
        latency_avg_ms,
        latency_min_ms,
        latency_max_ms,
        compute_jitter_ms(valid_latencies_ms),
        estimated_dropped_frames,
        stale_or_repeated_frames,
        missing_header_frames,
        negative_latency_frames,
        seconds_since_last_frame};
  }

  std::int64_t ImageLatencyTracker::estimate_drops(const std::int64_t capture_delta_ns) const
  {
    if (!expected_period_ns_.has_value())
    {
      return 0;
    }

    const auto threshold = static_cast<std::int64_t>(
        static_cast<double>(expected_period_ns_.value()) * drop_gap_factor_);
    if (capture_delta_ns <= threshold)
    {
      return 0;
    }

    const auto periods_elapsed = static_cast<std::int64_t>(std::floor(
        (static_cast<double>(capture_delta_ns) /
         static_cast<double>(expected_period_ns_.value())) +
        0.5));
    return std::max<std::int64_t>(0, periods_elapsed - 1);
  }

  double ImageLatencyTracker::compute_frame_rate_hz(
      const std::deque<FrameRecord> &records)
  {
    if (records.size() < 2U)
    {
      return 0.0;
    }

    const auto duration_ns = records.back().receive_ns - records.front().receive_ns;
    if (duration_ns <= 0)
    {
      return 0.0;
    }

    return static_cast<double>(records.size() - 1U) /
           (static_cast<double>(duration_ns) / 1'000'000'000.0);
  }

  std::optional<double> ImageLatencyTracker::compute_jitter_ms(
      const std::vector<double> &latencies_ms)
  {
    if (latencies_ms.empty())
    {
      return std::nullopt;
    }
    if (latencies_ms.size() == 1U)
    {
      return 0.0;
    }

    const double mean = std::accumulate(latencies_ms.begin(), latencies_ms.end(), 0.0) /
                        static_cast<double>(latencies_ms.size());

    double squared_difference_sum = 0.0;
    for (const double latency : latencies_ms)
    {
      const double difference = latency - mean;
      squared_difference_sum += difference * difference;
    }

    // Population standard deviation, equivalent to Python statistics.pstdev().
    return std::sqrt(squared_difference_sum / static_cast<double>(latencies_ms.size()));
  }

} // namespace waybionic_camera_tools
