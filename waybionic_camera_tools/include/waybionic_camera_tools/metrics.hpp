#ifndef WAYBIONIC_CAMERA_TOOLS__METRICS_HPP_
#define WAYBIONIC_CAMERA_TOOLS__METRICS_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace waybionic_camera_tools
{

  struct FrameRecord
  {
    std::int64_t receive_ns{};
    std::optional<std::int64_t> capture_ns;
    std::optional<double> latency_ms;
    std::optional<double> receive_gap_ms;
    std::optional<double> capture_gap_ms;
    std::int64_t estimated_drops{};
    bool stale_header{};
    bool missing_header{};
    bool negative_latency{};
  };

  struct StreamSnapshot
  {
    std::string state{"waiting"};
    std::uint64_t total_frames{};
    std::size_t frames_in_window{};
    std::size_t valid_latency_frames{};
    double frame_rate_hz{};
    std::optional<double> latency_avg_ms;
    std::optional<double> latency_min_ms;
    std::optional<double> latency_max_ms;
    std::optional<double> jitter_ms;
    std::int64_t estimated_dropped_frames{};
    std::size_t stale_or_repeated_frames{};
    std::size_t missing_header_frames{};
    std::size_t negative_latency_frames{};
    std::optional<double> seconds_since_last_frame;
  };

  class ImageLatencyTracker
  {
  public:
    explicit ImageLatencyTracker(
        double expected_frame_rate = 0.0,
        double stale_timeout_sec = 1.0,
        std::size_t history_size = 180,
        double drop_gap_factor = 1.5);

    FrameRecord observe(
        std::int64_t receive_ns,
        std::optional<std::int64_t> capture_ns);

    [[nodiscard]] StreamSnapshot snapshot(std::int64_t now_ns) const;

  private:
    [[nodiscard]] std::int64_t estimate_drops(std::int64_t capture_delta_ns) const;
    [[nodiscard]] static double compute_frame_rate_hz(const std::deque<FrameRecord> &records);
    [[nodiscard]] static std::optional<double> compute_jitter_ms(
        const std::vector<double> &latencies_ms);

    std::optional<std::int64_t> expected_period_ns_;
    double stale_timeout_sec_;
    std::size_t history_size_;
    double drop_gap_factor_;

    mutable std::mutex mutex_;
    std::deque<FrameRecord> records_;
    std::uint64_t total_frames_{0};
    std::optional<std::int64_t> last_receive_ns_;
    std::optional<std::int64_t> last_capture_ns_;
  };

} // namespace waybionic_camera_tools

#endif // WAYBIONIC_CAMERA_TOOLS__METRICS_HPP_
