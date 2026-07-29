#ifndef GENERIC_WEBCAM_ENUMERATOR_H
#define GENERIC_WEBCAM_ENUMERATOR_H

#include "DeviceEnumerator.h"
#include "TrackerVideoSource.h"

#include <cstddef>
#include <string>
#include <vector>

struct GenericWebcamDeviceInfo
{
    // Short, deterministic identifier suitable for configuration files.
    std::string stable_id;
    std::string friendly_name;
    std::string symbolic_link_utf8;

    // Media Foundation requires the original UTF-16 symbolic link when the
    // device is opened. It is intentionally retained alongside the display
    // strings instead of resolving the camera by its unstable numeric index.
    std::wstring symbolic_link;

    bool hardware_source;
    int vendor_id;
    int product_id;
    std::vector<TrackerVideoMediaMode> media_modes;
    std::string mode_enumeration_error;

    GenericWebcamDeviceInfo()
        : hardware_source(false)
        , vendor_id(-1)
        , product_id(-1)
    {
    }
};

class GenericWebcamEnumerator : public DeviceEnumerator
{
public:
    // Generic webcams are opt-in: an empty enabled_stable_ids list produces no
    // active devices. Use enumerateAllDevices() to populate a settings UI.
    explicit GenericWebcamEnumerator(
        const std::vector<std::string> &enabled_stable_ids,
        bool include_media_modes = false);
    virtual ~GenericWebcamEnumerator();

    bool is_valid() const override;
    bool next() override;
    int get_vendor_id() const override;
    int get_product_id() const override;
    const char *get_path() const override;

    const GenericWebcamDeviceInfo *get_device_info() const;
    const std::string &get_last_error() const;

    static bool enumerateAllDevices(
        std::vector<GenericWebcamDeviceInfo> &devices,
        bool include_media_modes,
        std::string *error_message = nullptr);

    static std::string makeStableId(const std::wstring &symbolic_link);

private:
    std::vector<GenericWebcamDeviceInfo> m_devices;
    std::size_t m_deviceIndex;
    std::string m_lastError;
};

#endif // GENERIC_WEBCAM_ENUMERATOR_H
