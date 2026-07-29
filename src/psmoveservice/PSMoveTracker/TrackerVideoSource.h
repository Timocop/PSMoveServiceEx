#ifndef TRACKER_VIDEO_SOURCE_H
#define TRACKER_VIDEO_SOURCE_H

#include <opencv2/core/core.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

// Describes one native camera mode. The subtype is stored as a canonical GUID
// string so that it can be persisted without leaking platform types into
// configuration files or non-Windows builds.
struct TrackerVideoMediaMode
{
    uint32_t width;
    uint32_t height;
    uint32_t frame_rate_numerator;
    uint32_t frame_rate_denominator;
    std::string subtype_guid;
    std::string subtype_name;
    int32_t default_stride;
    uint32_t interlace_mode;
    bool compressed;
    bool bgr_conversion_supported;

    TrackerVideoMediaMode()
    {
        clear();
    }

    void clear()
    {
        width = 0;
        height = 0;
        frame_rate_numerator = 0;
        frame_rate_denominator = 0;
        subtype_guid.clear();
        subtype_name.clear();
        default_stride = 0;
        interlace_mode = 0;
        compressed = false;
        bgr_conversion_supported = false;
    }

    bool isValid() const
    {
        return width > 0
            && height > 0
            && frame_rate_numerator > 0
            && frame_rate_denominator > 0
            && !subtype_guid.empty();
    }

    std::string getModeKey() const
    {
        std::string normalized_subtype = subtype_guid;
        std::transform(
            normalized_subtype.begin(),
            normalized_subtype.end(),
            normalized_subtype.begin(),
            [](unsigned char value) { return static_cast<char>(std::tolower(value)); });

        std::ostringstream stream;
        stream << width << "x" << height
               << "@" << frame_rate_numerator << "/" << frame_rate_denominator
               << ":" << normalized_subtype
               << ":i" << interlace_mode;
        return stream.str();
    }

    bool operator==(const TrackerVideoMediaMode &other) const
    {
        return getModeKey() == other.getModeKey();
    }

    bool operator!=(const TrackerVideoMediaMode &other) const
    {
        return !(*this == other);
    }
};

struct TrackerVideoFrame
{
    cv::Mat bgr;
    uint64_t sequence;
    std::chrono::steady_clock::time_point capture_time;
    std::chrono::steady_clock::time_point arrival_time;
    int64_t source_timestamp_100ns;
    bool source_timestamp_valid;
    uint64_t dropped_frames_before_this;

    TrackerVideoFrame()
        : sequence(0)
        , source_timestamp_100ns(0)
        , source_timestamp_valid(false)
        , dropped_frames_before_this(0)
    {
    }

    bool isValid() const
    {
        return !bgr.empty()
            && bgr.type() == CV_8UC3
            && bgr.isContinuous();
    }
};

// Maps Media Foundation's presentation timestamps (100 ns units) onto the
// process-local steady clock. The first timestamp is anchored to frame arrival;
// subsequent timestamps preserve the source clock delta. Timestamp
// discontinuities are deliberately re-anchored rather than leaking a clock
// jump into pose prediction.
class TrackerVideoTimestampMapper
{
public:
    explicit TrackerVideoTimestampMapper(
        std::chrono::steady_clock::duration latency_offset =
            std::chrono::steady_clock::duration::zero())
        : m_latencyOffset(latency_offset)
        , m_initialized(false)
        , m_sourceAnchor100ns(0)
        , m_lastSource100ns(0)
    {
    }

    void reset()
    {
        m_initialized = false;
        m_sourceAnchor100ns = 0;
        m_lastSource100ns = 0;
        m_captureAnchor = std::chrono::steady_clock::time_point();
    }

    void setLatencyOffset(std::chrono::steady_clock::duration latency_offset)
    {
        m_latencyOffset = latency_offset;
        reset();
    }

    bool isInitialized() const
    {
        return m_initialized;
    }

    std::chrono::steady_clock::time_point map(
        int64_t source_timestamp_100ns,
        const std::chrono::steady_clock::time_point &arrival_time,
        bool source_timestamp_valid,
        bool discontinuity,
        bool *used_source_timestamp = nullptr)
    {
        if (used_source_timestamp != nullptr)
        {
            *used_source_timestamp = false;
        }

        if (!source_timestamp_valid)
        {
            reset();
            return arrival_time - m_latencyOffset;
        }

        if (discontinuity
            || (m_initialized && source_timestamp_100ns <= m_lastSource100ns))
        {
            reset();
        }

        if (!m_initialized)
        {
            m_initialized = true;
            m_sourceAnchor100ns = source_timestamp_100ns;
            m_lastSource100ns = source_timestamp_100ns;
            m_captureAnchor = arrival_time - m_latencyOffset;

            if (used_source_timestamp != nullptr)
            {
                *used_source_timestamp = true;
            }
            return m_captureAnchor;
        }

        const int64_t source_delta_100ns =
            source_timestamp_100ns - m_sourceAnchor100ns;
        if (source_delta_100ns < 0
            || source_delta_100ns
                > (std::numeric_limits<int64_t>::max)() / 100)
        {
            reset();
            return map(
                source_timestamp_100ns,
                arrival_time,
                true,
                false,
                used_source_timestamp);
        }

        const std::chrono::steady_clock::time_point mapped_time =
            m_captureAnchor
            + std::chrono::nanoseconds(source_delta_100ns * 100);

        // A source clock that maps far into the future is no longer compatible
        // with the steady-clock anchor (device reset, suspend/resume, or a
        // malformed timestamp). Re-anchor immediately.
        const std::chrono::milliseconds maximum_future_lead(250);
        if (mapped_time > arrival_time + maximum_future_lead)
        {
            reset();
            return map(
                source_timestamp_100ns,
                arrival_time,
                true,
                false,
                used_source_timestamp);
        }

        if (used_source_timestamp != nullptr)
        {
            *used_source_timestamp = true;
        }
        m_lastSource100ns = source_timestamp_100ns;
        return mapped_time;
    }

private:
    std::chrono::steady_clock::duration m_latencyOffset;
    bool m_initialized;
    int64_t m_sourceAnchor100ns;
    int64_t m_lastSource100ns;
    std::chrono::steady_clock::time_point m_captureAnchor;
};

// Single-slot producer/consumer queue. Camera callbacks never wait for image
// processing: an unread frame is replaced by the newest frame and reported via
// dropped_frames_before_this.
class TrackerVideoLatestFrameBuffer
{
public:
    TrackerVideoLatestFrameBuffer()
        : m_generation(0)
        , m_consumedGeneration(0)
        , m_pendingDroppedFrames(0)
        , m_totalDroppedFrames(0)
    {
    }

    bool push(TrackerVideoFrame frame)
    {
        if (frame.bgr.empty() || frame.bgr.type() != CV_8UC3)
        {
            return false;
        }

        if (!frame.bgr.isContinuous())
        {
            frame.bgr = frame.bgr.clone();
        }

        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_generation != m_consumedGeneration)
        {
            ++m_pendingDroppedFrames;
            ++m_totalDroppedFrames;
        }

        frame.dropped_frames_before_this = m_pendingDroppedFrames;
        m_latestFrame = std::move(frame);
        ++m_generation;
        return true;
    }

    bool hasNewFrame() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_generation != m_consumedGeneration;
    }

    bool tryGetLatestFrame(TrackerVideoFrame &frame)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_generation == m_consumedGeneration)
        {
            return false;
        }

        frame = m_latestFrame;
        m_consumedGeneration = m_generation;
        m_pendingDroppedFrames = 0;
        return true;
    }

    uint64_t getTotalDroppedFrameCount() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_totalDroppedFrames;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_latestFrame = TrackerVideoFrame();
        m_generation = 0;
        m_consumedGeneration = 0;
        m_pendingDroppedFrames = 0;
        m_totalDroppedFrames = 0;
    }

private:
    mutable std::mutex m_mutex;
    TrackerVideoFrame m_latestFrame;
    uint64_t m_generation;
    uint64_t m_consumedGeneration;
    uint64_t m_pendingDroppedFrames;
    uint64_t m_totalDroppedFrames;
};

class TrackerVideoControlNormalization
{
public:
    static uint8_t fromDeviceValue(
        int32_t value,
        int32_t minimum,
        int32_t maximum)
    {
        if (maximum <= minimum)
        {
            return 1;
        }

        const int64_t clamped = (std::max)(
            static_cast<int64_t>(minimum),
            (std::min)(
                static_cast<int64_t>(maximum),
                static_cast<int64_t>(value)));
        const int64_t range =
            static_cast<int64_t>(maximum) - static_cast<int64_t>(minimum);
        const int64_t offset = clamped - static_cast<int64_t>(minimum);
        return static_cast<uint8_t>(
            1 + (offset * 254 + range / 2) / range);
    }

    static int32_t toDeviceValue(
        uint8_t normalized_value,
        int32_t minimum,
        int32_t maximum,
        int32_t step)
    {
        if (maximum <= minimum)
        {
            return minimum;
        }

        // Zero denotes automatic control mode and therefore has no manual
        // device value. Clamp it to the bottom of the manual range so this
        // helper remains total; callers still select the automatic flag.
        const uint8_t manual_value =
            normalized_value == 0 ? 1 : normalized_value;
        const int64_t range =
            static_cast<int64_t>(maximum) - static_cast<int64_t>(minimum);
        int64_t raw_value =
            static_cast<int64_t>(minimum)
            + (static_cast<int64_t>(manual_value - 1) * range + 127) / 254;

        const int64_t positive_step = step > 0 ? step : 1;
        const int64_t offset = raw_value - static_cast<int64_t>(minimum);
        raw_value =
            static_cast<int64_t>(minimum)
            + ((offset + positive_step / 2) / positive_step) * positive_step;
        raw_value = (std::max)(
            static_cast<int64_t>(minimum),
            (std::min)(static_cast<int64_t>(maximum), raw_value));
        return static_cast<int32_t>(raw_value);
    }
};

class ITrackerVideoSource
{
public:
    virtual ~ITrackerVideoSource()
    {
    }

    virtual bool isOpen() const = 0;
    virtual void close() = 0;
    virtual bool hasNewFrame() const = 0;
    virtual bool tryGetLatestFrame(TrackerVideoFrame &frame) = 0;
    virtual TrackerVideoMediaMode getActiveMode() const = 0;
    virtual std::string getDeviceStableId() const = 0;
    virtual std::string getLastError() const = 0;

    // Controls are normalized for persisted tracker settings. Zero requests
    // automatic mode; values 1..255 span the device's reported manual range.
    // actual_value reports hardware quantization when supplied.
    virtual bool getExposure(uint8_t &value) = 0;
    virtual bool setExposure(
        uint8_t requested_value,
        uint8_t *actual_value = nullptr) = 0;
    virtual bool getGain(uint8_t &value) = 0;
    virtual bool setGain(
        uint8_t requested_value,
        uint8_t *actual_value = nullptr) = 0;
};

#endif // TRACKER_VIDEO_SOURCE_H
