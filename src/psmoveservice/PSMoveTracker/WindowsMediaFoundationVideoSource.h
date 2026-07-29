#ifndef WINDOWS_MEDIA_FOUNDATION_VIDEO_SOURCE_H
#define WINDOWS_MEDIA_FOUNDATION_VIDEO_SOURCE_H

#include "GenericWebcamEnumerator.h"
#include "TrackerVideoSource.h"

#include <chrono>
#include <memory>

// Asynchronous, latest-frame-only webcam capture for tracker inputs. Device
// selection uses the Media Foundation symbolic link and an exact native media
// mode; it never relies on the process-global OpenCV camera index.
class WindowsMediaFoundationVideoSource final : public ITrackerVideoSource
{
public:
    WindowsMediaFoundationVideoSource();
    ~WindowsMediaFoundationVideoSource() override;

    WindowsMediaFoundationVideoSource(
        const WindowsMediaFoundationVideoSource &) = delete;
    WindowsMediaFoundationVideoSource &operator=(
        const WindowsMediaFoundationVideoSource &) = delete;

    bool open(
        const GenericWebcamDeviceInfo &device,
        const TrackerVideoMediaMode &mode,
        std::chrono::milliseconds latency_offset =
            std::chrono::milliseconds::zero());

    bool isOpen() const override;
    void close() override;
    bool hasNewFrame() const override;
    bool tryGetLatestFrame(TrackerVideoFrame &frame) override;
    TrackerVideoMediaMode getActiveMode() const override;
    std::string getDeviceStableId() const override;
    std::string getLastError() const override;
    bool getExposure(uint8_t &value) override;
    bool setExposure(
        uint8_t requested_value,
        uint8_t *actual_value = nullptr) override;
    bool getGain(uint8_t &value) override;
    bool setGain(
        uint8_t requested_value,
        uint8_t *actual_value = nullptr) override;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

#endif // WINDOWS_MEDIA_FOUNDATION_VIDEO_SOURCE_H
