#include "waybionic_camera_tools/metrics.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace waybionic_camera_tools
{
    struct StreamMetadata
    {
        std::uint32_t width{0};
        std::uint32_t height{0};
        std::string encoding;
        std::string frame_id;
    };

    class ImageLatencyMonitor : public rclcpp::Node
    {
    public:
        ImageLatencyMonitor()
            : Node("image_latency_monitor")
        {
            // We can change these later to include right and left camera views?
            image_topic_ = declare_parameter<std::string>(
                "image_topic", "/doctor_view/image_raw");

            camera_info_topic_ = declare_parameter<std::string>(
                "camera_info_topic", "/doctor_view/camera_info");

            diagnostics_topic_ = declare_parameter<std::string>(
                "diagnostics_topic", "/diagnostics");

            camera_signal_name_ = declare_parameter<std::string>(
                "camera_signal_name", "camera.primary");

            const double expected_frame_rate = declare_parameter<double>(
                "expected_frame_rate", 30.0);

            expected_width_ = declare_parameter<std::int64_t>("expected_width", 1920);

            expected_height_ = declare_parameter<std::int64_t>("expected_height", 1080);

            optical_frame_id_ = declare_parameter<std::string>(
                "optical_frame_id", "doctor_view_optical_frame");

            timestamp_source_ = declare_parameter<std::string>(
                "timestamp_source", "image.header.stamp");

            const double stale_timeout_sec = declare_parameter<double>(
                "stale_timeout_sec", 1.0);

            const std::int64_t history_size = declare_parameter<std::int64_t>("history_size", 180);

            const double drop_gap_factor = declare_parameter<double>(
                "drop_gap_factor", 1.5);

            const double diagnostic_period_sec = declare_parameter<double>(
                "diagnostic_period_sec", 1.0);

            log_period_sec_ = declare_parameter<double>("log_period_sec", 5.0);

            if (history_size <= 0)
            {
                throw std::invalid_argument("history_size must be positive");
            }
            if (diagnostic_period_sec <= 0.0)
            {
                throw std::invalid_argument("diagnostic_period_sec must be positive");
            }
            if (log_period_sec_ <= 0.0)
            {
                throw std::invalid_argument("log_period_sec must be positive");
            }

            tracker_ = std::make_unique<ImageLatencyTracker>(
                expected_frame_rate,
                stale_timeout_sec,
                static_cast<std::size_t>(history_size),
                drop_gap_factor);

            auto diagnostics_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                diagnostics_topic_, diagnostics_qos);

            image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(&ImageLatencyMonitor::image_callback, this, std::placeholders::_1));

            const auto diagnostic_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(diagnostic_period_sec));
            diagnostic_timer_ = create_wall_timer(
                diagnostic_period,
                std::bind(&ImageLatencyMonitor::publish_diagnostics, this));

            RCLCPP_INFO(
                get_logger(),
                "Monitoring %s -> %s with signal root %s",
                image_topic_.c_str(),
                diagnostics_topic_.c_str(),
                camera_signal_name_.c_str());
        }

    private:
        using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
        using KeyValue = diagnostic_msgs::msg::KeyValue;
        using Image = sensor_msgs::msg::Image;

        void image_callback(const Image::ConstSharedPtr message)
        {
            const rclcpp::Time receive_time = now();
            const auto capture_time = extract_capture_time(*message);

            metadata_ = StreamMetadata{
                message->width,
                message->height,
                message->encoding,
                message->header.frame_id};

            tracker_->observe(
                receive_time.nanoseconds(),
                capture_time.has_value() ? std::optional<std::int64_t>(capture_time->nanoseconds()) : std::nullopt);
        }

        [[nodiscard]] std::optional<rclcpp::Time> extract_capture_time(const Image &message) const
        {
            if (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0U)
            {
                return std::nullopt;
            }
            return rclcpp::Time(message.header.stamp, get_clock()->get_clock_type());
        }

        void publish_diagnostics()
        {
            const rclcpp::Time current_time = now();
            const StreamSnapshot snapshot = tracker_->snapshot(current_time.nanoseconds());
            const std::uint8_t level = status_level(snapshot);
            const std::string message = status_message(snapshot);

            diagnostic_msgs::msg::DiagnosticArray diagnostics;
            diagnostics.header.stamp = current_time;
            diagnostics.status = build_statuses(snapshot, level, message);
            diagnostics_publisher_->publish(diagnostics);

            if (should_log(current_time))
            {
                RCLCPP_INFO(get_logger(), "%s", log_line(snapshot, message).c_str());
                last_log_time_ = current_time;
            }
        }

        [[nodiscard]] std::vector<DiagnosticStatus> build_statuses(
            const StreamSnapshot &snapshot,
            const std::uint8_t level,
            const std::string &message) const
        {
            std::vector<DiagnosticStatus> statuses;
            statuses.reserve(7);

            statuses.push_back(make_status(
                camera_signal_name_,
                level,
                message,
                format_number(snapshot.latency_avg_ms),
                "ms",
                {
                    {"state", snapshot.state},
                    {"image_topic", image_topic_},
                    {"camera_info_topic", camera_info_topic_},
                    {"optical_frame_id", optical_frame_id_},
                    {"timestamp_source", timestamp_source_},
                    {"resolution", resolution_string()},
                    {"encoding", metadata_.encoding.empty() ? "n/a" : metadata_.encoding},
                    {"frame_id", metadata_.frame_id.empty() ? "n/a" : metadata_.frame_id},
                    {"expected_resolution", expected_resolution_string()},
                    {"frame_rate_hz", format_number(snapshot.frame_rate_hz)},
                    {"latency_min_ms", format_number(snapshot.latency_min_ms)},
                    {"latency_max_ms", format_number(snapshot.latency_max_ms)},
                    {"jitter_ms", format_number(snapshot.jitter_ms)},
                    {"dropped_frames", std::to_string(snapshot.estimated_dropped_frames)},
                    {"stale_frames", std::to_string(snapshot.stale_or_repeated_frames)},
                }));

            statuses.push_back(make_status(
                camera_signal_name_ + ".frame_rate", level, message,
                format_number(snapshot.frame_rate_hz), "Hz"));
            statuses.push_back(make_status(
                camera_signal_name_ + ".latency_min", level, message,
                format_number(snapshot.latency_min_ms), "ms"));
            statuses.push_back(make_status(
                camera_signal_name_ + ".latency_max", level, message,
                format_number(snapshot.latency_max_ms), "ms"));
            statuses.push_back(make_status(
                camera_signal_name_ + ".jitter", level, message,
                format_number(snapshot.jitter_ms), "ms"));
            statuses.push_back(make_status(
                camera_signal_name_ + ".dropped_frames", level, message,
                std::to_string(snapshot.estimated_dropped_frames), "frames"));
            statuses.push_back(make_status(
                camera_signal_name_ + ".stale_frames", level, message,
                std::to_string(snapshot.stale_or_repeated_frames), "frames"));

            return statuses;
        }

        [[nodiscard]] DiagnosticStatus make_status(
            const std::string &name,
            const std::uint8_t level,
            const std::string &message,
            const std::optional<std::string> &value,
            const std::string &unit,
            const std::vector<std::pair<std::string, std::optional<std::string>>> &extra_values = {}) const
        {
            DiagnosticStatus status;
            status.level = level;
            status.name = name;
            status.message = message;
            status.hardware_id = image_topic_;
            KeyValue value_entry;
            value_entry.key = "value";
            value_entry.value = text_or_na(value);
            status.values.push_back(value_entry);

            KeyValue unit_entry;
            unit_entry.key = "unit";
            unit_entry.value = text_or_na(unit);
            status.values.push_back(unit_entry);

            for (const auto &[key, item] : extra_values)
            {
                KeyValue entry;
                entry.key = key;
                entry.value = text_or_na(item);
                status.values.push_back(entry);
            }
            return status;
        }

        [[nodiscard]] std::uint8_t status_level(const StreamSnapshot &snapshot) const
        {
            if (snapshot.state == "stale")
            {
                return DiagnosticStatus::STALE;
            }
            if (snapshot.state == "waiting")
            {
                return DiagnosticStatus::WARN;
            }
            if (
                snapshot.estimated_dropped_frames > 0 ||
                snapshot.stale_or_repeated_frames > 0U ||
                snapshot.missing_header_frames > 0U ||
                snapshot.negative_latency_frames > 0U ||
                has_resolution_mismatch())
            {
                return DiagnosticStatus::WARN;
            }
            return DiagnosticStatus::OK;
        }

        [[nodiscard]] std::string status_message(const StreamSnapshot &snapshot) const
        {
            if (snapshot.state == "waiting")
            {
                return "Waiting for frames on " + image_topic_;
            }

            if (snapshot.state == "stale")
            {
                std::ostringstream stream;
                stream << "Stream stale; last frame " << std::fixed << std::setprecision(2)
                       << snapshot.seconds_since_last_frame.value_or(0.0)
                       << "s ago on " << image_topic_;
                return stream.str();
            }

            std::vector<std::string> issues;
            if (snapshot.estimated_dropped_frames > 0)
            {
                issues.push_back(
                    std::to_string(snapshot.estimated_dropped_frames) + " estimated dropped frame(s)");
            }
            if (snapshot.stale_or_repeated_frames > 0U)
            {
                issues.push_back(
                    std::to_string(snapshot.stale_or_repeated_frames) +
                    " repeated-or-out-of-order header(s)");
            }
            if (snapshot.missing_header_frames > 0U)
            {
                issues.push_back(
                    std::to_string(snapshot.missing_header_frames) +
                    " frame(s) missing header timestamps");
            }
            if (snapshot.negative_latency_frames > 0U)
            {
                issues.push_back(
                    std::to_string(snapshot.negative_latency_frames) +
                    " frame(s) had negative latency");
            }
            if (has_resolution_mismatch())
            {
                issues.push_back(
                    "resolution mismatch, expected " + expected_resolution_string() +
                    " but saw " + resolution_string());
            }

            if (!issues.empty())
            {
                std::ostringstream stream;
                stream << "Connected with ";
                for (std::size_t index = 0; index < issues.size(); ++index)
                {
                    if (index > 0U)
                    {
                        stream << "; ";
                    }
                    stream << issues[index];
                }
                return stream.str();
            }

            return "Connected to " + image_topic_;
        }

        [[nodiscard]] bool should_log(const rclcpp::Time &current_time) const
        {
            if (!last_log_time_.has_value())
            {
                return true;
            }
            return (current_time - last_log_time_.value()).nanoseconds() >=
                   static_cast<std::int64_t>(log_period_sec_ * 1e9);
        }

        [[nodiscard]] std::string log_line(
            const StreamSnapshot &snapshot,
            const std::string &message) const
        {
            std::ostringstream stream;
            stream << camera_signal_name_ << " on " << image_topic_
                   << ": fps=" << format_metric(snapshot.frame_rate_hz, "Hz")
                   << " latency(avg/min/max)=" << format_metric(snapshot.latency_avg_ms, "ms")
                   << "/" << format_metric(snapshot.latency_min_ms, "ms")
                   << "/" << format_metric(snapshot.latency_max_ms, "ms")
                   << " jitter=" << format_metric(snapshot.jitter_ms, "ms")
                   << " dropped=" << snapshot.estimated_dropped_frames
                   << " stale=" << snapshot.stale_or_repeated_frames
                   << " missing_header=" << snapshot.missing_header_frames
                   << " | " << message;
            return stream.str();
        }

        [[nodiscard]] bool has_resolution_mismatch() const
        {
            if (metadata_.width == 0U || metadata_.height == 0U)
            {
                return false;
            }
            return expected_width_ > 0 && expected_height_ > 0 &&
                   (metadata_.width != static_cast<std::uint32_t>(expected_width_) ||
                    metadata_.height != static_cast<std::uint32_t>(expected_height_));
        }

        [[nodiscard]] std::string resolution_string() const
        {
            if (metadata_.width == 0U || metadata_.height == 0U)
            {
                return "n/a";
            }
            return std::to_string(metadata_.width) + "x" + std::to_string(metadata_.height);
        }

        [[nodiscard]] std::string expected_resolution_string() const
        {
            if (expected_width_ <= 0 || expected_height_ <= 0)
            {
                return "n/a";
            }
            return std::to_string(expected_width_) + "x" + std::to_string(expected_height_);
        }

        [[nodiscard]] static std::optional<std::string> format_number(
            const std::optional<double> &value)
        {
            if (!value.has_value())
            {
                return std::nullopt;
            }
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(2) << value.value();
            return stream.str();
        }

        [[nodiscard]] static std::optional<std::string> format_number(const double value)
        {
            return format_number(std::optional<double>(value));
        }

        [[nodiscard]] static std::string format_metric(
            const std::optional<double> &value,
            const std::string &unit)
        {
            const auto formatted = format_number(value);
            return formatted.has_value() ? formatted.value() + unit : "n/a";
        }

        [[nodiscard]] static std::string format_metric(const double value, const std::string &unit)
        {
            return format_metric(std::optional<double>(value), unit);
        }

        [[nodiscard]] static std::string text_or_na(const std::optional<std::string> &value)
        {
            return !value.has_value() || value->empty() ? "n/a" : value.value();
        }

        [[nodiscard]] static std::string text_or_na(const std::string &value)
        {
            return value.empty() ? "n/a" : value;
        }

        std::string image_topic_;
        std::string camera_info_topic_;
        std::string diagnostics_topic_;
        std::string camera_signal_name_;
        std::int64_t expected_width_{0};
        std::int64_t expected_height_{0};
        std::string optical_frame_id_;
        std::string timestamp_source_;
        double log_period_sec_{5.0};

        std::unique_ptr<ImageLatencyTracker> tracker_;
        StreamMetadata metadata_;
        std::optional<rclcpp::Time> last_log_time_;

        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
        rclcpp::Subscription<Image>::SharedPtr image_subscription_;
        rclcpp::TimerBase::SharedPtr diagnostic_timer_;
    };

} // namespace waybionic_camera_tools

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<waybionic_camera_tools::ImageLatencyMonitor>());
    rclcpp::shutdown();
    return 0;
}