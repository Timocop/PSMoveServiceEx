#include "GenericWebcamEnumerator.h"

#include <algorithm>
#include <cctype>
#include <cwctype>
#include <iomanip>
#include <set>
#include <sstream>

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <objbase.h>
#include <wrl/client.h>

namespace
{
    using Microsoft::WRL::ComPtr;

    class MediaFoundationScope
    {
    public:
        MediaFoundationScope()
            : m_comInitialized(false)
            , m_mediaFoundationStarted(false)
            , m_result(S_OK)
        {
            const HRESULT com_result =
                CoInitializeEx(nullptr, COINIT_MULTITHREADED);
            if (SUCCEEDED(com_result))
            {
                m_comInitialized = true;
            }
            else if (com_result != RPC_E_CHANGED_MODE)
            {
                m_result = com_result;
                return;
            }

            m_result = MFStartup(MF_VERSION, MFSTARTUP_FULL);
            m_mediaFoundationStarted = SUCCEEDED(m_result);
        }

        ~MediaFoundationScope()
        {
            if (m_mediaFoundationStarted)
            {
                MFShutdown();
            }
            if (m_comInitialized)
            {
                CoUninitialize();
            }
        }

        HRESULT result() const
        {
            return m_result;
        }

    private:
        bool m_comInitialized;
        bool m_mediaFoundationStarted;
        HRESULT m_result;
    };

    std::string wideToUtf8(const std::wstring &value)
    {
        if (value.empty())
        {
            return std::string();
        }

        const int byte_count = WideCharToMultiByte(
            CP_UTF8,
            WC_ERR_INVALID_CHARS,
            value.data(),
            static_cast<int>(value.size()),
            nullptr,
            0,
            nullptr,
            nullptr);
        if (byte_count <= 0)
        {
            return std::string();
        }

        std::string result(static_cast<std::size_t>(byte_count), '\0');
        if (WideCharToMultiByte(
                CP_UTF8,
                WC_ERR_INVALID_CHARS,
                value.data(),
                static_cast<int>(value.size()),
                &result[0],
                byte_count,
                nullptr,
                nullptr)
            != byte_count)
        {
            return std::string();
        }
        return result;
    }

    std::string formatHRESULT(const char *operation, HRESULT result)
    {
        std::ostringstream stream;
        stream << operation << " failed (HRESULT 0x"
               << std::hex << std::uppercase << std::setw(8)
               << std::setfill('0')
               << static_cast<unsigned long>(result) << ")";

        wchar_t *message = nullptr;
        const DWORD message_length = FormatMessageW(
            FORMAT_MESSAGE_ALLOCATE_BUFFER
                | FORMAT_MESSAGE_FROM_SYSTEM
                | FORMAT_MESSAGE_IGNORE_INSERTS,
            nullptr,
            static_cast<DWORD>(result),
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            reinterpret_cast<wchar_t *>(&message),
            0,
            nullptr);
        if (message_length > 0 && message != nullptr)
        {
            std::wstring wide_message(message, message_length);
            while (!wide_message.empty()
                   && (wide_message.back() == L'\r'
                       || wide_message.back() == L'\n'
                       || wide_message.back() == L' '))
            {
                wide_message.pop_back();
            }
            const std::string utf8_message = wideToUtf8(wide_message);
            if (!utf8_message.empty())
            {
                stream << ": " << utf8_message;
            }
        }
        if (message != nullptr)
        {
            LocalFree(message);
        }
        return stream.str();
    }

    bool getAllocatedString(
        IMFAttributes *attributes,
        REFGUID key,
        std::wstring &value)
    {
        wchar_t *allocated_value = nullptr;
        UINT32 length = 0;
        const HRESULT result =
            attributes->GetAllocatedString(key, &allocated_value, &length);
        if (FAILED(result) || allocated_value == nullptr)
        {
            return false;
        }

        value.assign(allocated_value, length);
        CoTaskMemFree(allocated_value);
        return true;
    }

    std::wstring guidToString(REFGUID guid)
    {
        wchar_t buffer[64] = {};
        const int length =
            StringFromGUID2(guid, buffer, static_cast<int>(_countof(buffer)));
        if (length <= 1)
        {
            return std::wstring();
        }

        std::wstring result(buffer, static_cast<std::size_t>(length - 1));
        std::transform(
            result.begin(),
            result.end(),
            result.begin(),
            [](wchar_t value) { return static_cast<wchar_t>(std::towlower(value)); });
        return result;
    }

    int parseHexDeviceIdentifier(
        const std::wstring &normalized_symbolic_link,
        const wchar_t *prefix)
    {
        const std::wstring prefix_string(prefix);
        const std::size_t position =
            normalized_symbolic_link.find(prefix_string);
        if (position == std::wstring::npos
            || position + prefix_string.size() + 4
                > normalized_symbolic_link.size())
        {
            return -1;
        }

        unsigned int value = 0;
        for (std::size_t offset = 0; offset < 4; ++offset)
        {
            const wchar_t digit =
                normalized_symbolic_link[position + prefix_string.size() + offset];
            value <<= 4;
            if (digit >= L'0' && digit <= L'9')
            {
                value |= static_cast<unsigned int>(digit - L'0');
            }
            else if (digit >= L'a' && digit <= L'f')
            {
                value |= static_cast<unsigned int>(digit - L'a' + 10);
            }
            else
            {
                return -1;
            }
        }
        return static_cast<int>(value);
    }

    void describeSubtype(
        REFGUID subtype,
        TrackerVideoMediaMode &mode)
    {
        mode.compressed = false;
        mode.bgr_conversion_supported = true;

        if (IsEqualGUID(subtype, MFVideoFormat_RGB32))
        {
            mode.subtype_name = "RGB32";
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_ARGB32))
        {
            mode.subtype_name = "ARGB32";
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_RGB24))
        {
            mode.subtype_name = "RGB24";
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_YUY2))
        {
            mode.subtype_name = "YUY2";
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_UYVY))
        {
            mode.subtype_name = "UYVY";
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_NV12))
        {
            mode.subtype_name = "NV12";
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_MJPG))
        {
            mode.subtype_name = "MJPG";
            mode.compressed = true;
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_I420))
        {
            mode.subtype_name = "I420";
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_IYUV))
        {
            mode.subtype_name = "IYUV";
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_YV12))
        {
            mode.subtype_name = "YV12";
        }
        else
        {
            mode.subtype_name = "UNKNOWN";
            mode.bgr_conversion_supported = false;
        }
    }

    bool mediaTypeToMode(
        IMFMediaType *media_type,
        TrackerVideoMediaMode &mode)
    {
        mode.clear();

        UINT32 width = 0;
        UINT32 height = 0;
        UINT32 rate_numerator = 0;
        UINT32 rate_denominator = 0;
        GUID subtype = GUID_NULL;

        if (FAILED(MFGetAttributeSize(
                media_type,
                MF_MT_FRAME_SIZE,
                &width,
                &height))
            || FAILED(MFGetAttributeRatio(
                media_type,
                MF_MT_FRAME_RATE,
                &rate_numerator,
                &rate_denominator))
            || FAILED(media_type->GetGUID(MF_MT_SUBTYPE, &subtype)))
        {
            return false;
        }

        mode.width = width;
        mode.height = height;
        mode.frame_rate_numerator = rate_numerator;
        mode.frame_rate_denominator = rate_denominator;
        mode.subtype_guid = wideToUtf8(guidToString(subtype));

        UINT32 interlace_mode = 0;
        if (SUCCEEDED(media_type->GetUINT32(
                MF_MT_INTERLACE_MODE,
                &interlace_mode)))
        {
            mode.interlace_mode = interlace_mode;
        }

        UINT32 stride = 0;
        if (SUCCEEDED(media_type->GetUINT32(
                MF_MT_DEFAULT_STRIDE,
                &stride)))
        {
            mode.default_stride = static_cast<int32_t>(stride);
        }
        else
        {
            LONG calculated_stride = 0;
            if (SUCCEEDED(MFGetStrideForBitmapInfoHeader(
                    subtype.Data1,
                    mode.width,
                    &calculated_stride)))
            {
                mode.default_stride = calculated_stride;
            }
        }

        describeSubtype(subtype, mode);
        if (mode.interlace_mode != 0
            && mode.interlace_mode != MFVideoInterlace_Progressive)
        {
            mode.bgr_conversion_supported = false;
        }
        return mode.isValid();
    }

    bool enumerateMediaModes(
        IMFActivate *activation,
        std::vector<TrackerVideoMediaMode> &modes,
        std::string &error_message)
    {
        modes.clear();
        error_message.clear();

        ComPtr<IMFMediaSource> media_source;
        HRESULT result = activation->ActivateObject(
            __uuidof(IMFMediaSource),
            reinterpret_cast<void **>(media_source.GetAddressOf()));
        if (FAILED(result))
        {
            error_message =
                formatHRESULT("IMFActivate::ActivateObject", result);
            return false;
        }

        ComPtr<IMFSourceReader> reader;
        result = MFCreateSourceReaderFromMediaSource(
            media_source.Get(),
            nullptr,
            reader.GetAddressOf());
        if (FAILED(result))
        {
            error_message =
                formatHRESULT("MFCreateSourceReaderFromMediaSource", result);
            media_source->Shutdown();
            activation->ShutdownObject();
            return false;
        }

        std::set<std::string> seen_modes;
        for (DWORD type_index = 0; ; ++type_index)
        {
            ComPtr<IMFMediaType> media_type;
            result = reader->GetNativeMediaType(
                MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                type_index,
                media_type.GetAddressOf());
            if (result == MF_E_NO_MORE_TYPES)
            {
                result = S_OK;
                break;
            }
            if (FAILED(result))
            {
                error_message =
                    formatHRESULT("IMFSourceReader::GetNativeMediaType", result);
                break;
            }

            TrackerVideoMediaMode mode;
            if (mediaTypeToMode(media_type.Get(), mode)
                && seen_modes.insert(mode.getModeKey()).second)
            {
                modes.push_back(mode);
            }
        }

        std::sort(
            modes.begin(),
            modes.end(),
            [](const TrackerVideoMediaMode &left,
               const TrackerVideoMediaMode &right)
            {
                if (left.width != right.width)
                {
                    return left.width < right.width;
                }
                if (left.height != right.height)
                {
                    return left.height < right.height;
                }
                const uint64_t left_rate =
                    static_cast<uint64_t>(left.frame_rate_numerator)
                    * right.frame_rate_denominator;
                const uint64_t right_rate =
                    static_cast<uint64_t>(right.frame_rate_numerator)
                    * left.frame_rate_denominator;
                if (left_rate != right_rate)
                {
                    return left_rate < right_rate;
                }
                const int left_interlace_priority =
                    left.interlace_mode == MFVideoInterlace_Progressive
                        ? 0
                        : (left.interlace_mode == 0 ? 1 : 2);
                const int right_interlace_priority =
                    right.interlace_mode == MFVideoInterlace_Progressive
                        ? 0
                        : (right.interlace_mode == 0 ? 1 : 2);
                if (left_interlace_priority != right_interlace_priority)
                {
                    return left_interlace_priority < right_interlace_priority;
                }
                return left.subtype_guid < right.subtype_guid;
            });

        media_source->Shutdown();
        activation->ShutdownObject();
        if (SUCCEEDED(result) && modes.empty())
        {
            error_message =
                "The webcam did not expose any complete native video modes.";
            return false;
        }
        return SUCCEEDED(result);
    }

    bool populateMediaModesForSelectedDevices(
        std::vector<GenericWebcamDeviceInfo> &devices,
        std::string &error_message)
    {
        error_message.clear();
        if (devices.empty())
        {
            return true;
        }

        MediaFoundationScope runtime;
        if (FAILED(runtime.result()))
        {
            error_message =
                formatHRESULT("Media Foundation initialization", runtime.result());
            return false;
        }

        ComPtr<IMFAttributes> attributes;
        HRESULT result = MFCreateAttributes(attributes.GetAddressOf(), 1);
        if (SUCCEEDED(result))
        {
            result = attributes->SetGUID(
                MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        }
        if (FAILED(result))
        {
            error_message =
                formatHRESULT("Create webcam enumeration attributes", result);
            return false;
        }

        IMFActivate **activations = nullptr;
        UINT32 activation_count = 0;
        result = MFEnumDeviceSources(
            attributes.Get(),
            &activations,
            &activation_count);
        if (FAILED(result))
        {
            error_message = formatHRESULT("MFEnumDeviceSources", result);
            return false;
        }

        std::set<std::wstring> found_symbolic_links;
        for (UINT32 activation_index = 0;
             activation_index < activation_count;
             ++activation_index)
        {
            IMFActivate *activation = activations[activation_index];
            std::wstring symbolic_link;
            if (getAllocatedString(
                    activation,
                    MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
                    symbolic_link))
            {
                const auto selected_device = std::find_if(
                    devices.begin(),
                    devices.end(),
                    [&symbolic_link](const GenericWebcamDeviceInfo &device)
                    {
                        return device.symbolic_link == symbolic_link;
                    });
                if (selected_device != devices.end())
                {
                    found_symbolic_links.insert(symbolic_link);
                    enumerateMediaModes(
                        activation,
                        selected_device->media_modes,
                        selected_device->mode_enumeration_error);
                }
            }
            activation->Release();
        }
        CoTaskMemFree(activations);

        for (GenericWebcamDeviceInfo &device : devices)
        {
            if (found_symbolic_links.count(device.symbolic_link) == 0)
            {
                device.mode_enumeration_error =
                    "The webcam disappeared before its modes could be read.";
            }
        }
        return true;
    }
}

#endif // _WIN32

namespace
{
    std::string normalizedAscii(const std::string &value)
    {
        std::string result = value;
        std::transform(
            result.begin(),
            result.end(),
            result.begin(),
            [](unsigned char character)
            {
                return static_cast<char>(std::tolower(character));
            });
        return result;
    }
}

GenericWebcamEnumerator::GenericWebcamEnumerator(
    const std::vector<std::string> &enabled_stable_ids,
    bool include_media_modes)
    : DeviceEnumerator(CommonDeviceState::PS3EYE)
    , m_deviceIndex(0)
{
    m_deviceType = CommonDeviceState::PS3EYE;

    if (enabled_stable_ids.empty())
    {
        return;
    }

    std::vector<GenericWebcamDeviceInfo> discovered_devices;
    if (!enumerateAllDevices(
            discovered_devices,
            false,
            &m_lastError))
    {
        return;
    }

    std::set<std::string> enabled_ids;
    for (const std::string &stable_id : enabled_stable_ids)
    {
        enabled_ids.insert(normalizedAscii(stable_id));
    }

    for (const GenericWebcamDeviceInfo &device : discovered_devices)
    {
        if (enabled_ids.count(normalizedAscii(device.stable_id)) > 0)
        {
            m_devices.push_back(device);
        }
    }

#ifdef _WIN32
    if (include_media_modes && !m_devices.empty())
    {
        std::string mode_population_error;
        if (!populateMediaModesForSelectedDevices(
                m_devices,
                mode_population_error))
        {
            m_lastError = mode_population_error;
        }
    }
#else
    (void)include_media_modes;
#endif
}

GenericWebcamEnumerator::~GenericWebcamEnumerator()
{
}

bool GenericWebcamEnumerator::is_valid() const
{
    return m_deviceIndex < m_devices.size();
}

bool GenericWebcamEnumerator::next()
{
    if (m_deviceIndex < m_devices.size())
    {
        ++m_deviceIndex;
    }
    return is_valid();
}

int GenericWebcamEnumerator::get_vendor_id() const
{
    const GenericWebcamDeviceInfo *device = get_device_info();
    return device != nullptr ? device->vendor_id : -1;
}

int GenericWebcamEnumerator::get_product_id() const
{
    const GenericWebcamDeviceInfo *device = get_device_info();
    return device != nullptr ? device->product_id : -1;
}

const char *GenericWebcamEnumerator::get_path() const
{
    const GenericWebcamDeviceInfo *device = get_device_info();
    return device != nullptr ? device->stable_id.c_str() : nullptr;
}

const GenericWebcamDeviceInfo *
GenericWebcamEnumerator::get_device_info() const
{
    return is_valid() ? &m_devices[m_deviceIndex] : nullptr;
}

const std::string &GenericWebcamEnumerator::get_last_error() const
{
    return m_lastError;
}

std::string GenericWebcamEnumerator::makeStableId(
    const std::wstring &symbolic_link)
{
    // Two independently seeded FNV-1a hashes make collisions negligible while
    // retaining a short identifier acceptable to legacy C APIs.
    uint64_t hash_a = UINT64_C(14695981039346656037);
    uint64_t hash_b = UINT64_C(7809847782465536322);
    const uint64_t fnv_prime = UINT64_C(1099511628211);

    for (wchar_t character : symbolic_link)
    {
        const uint32_t normalized =
            static_cast<uint32_t>(std::towlower(character));
        for (unsigned int byte_index = 0;
             byte_index < sizeof(wchar_t);
             ++byte_index)
        {
            const uint8_t byte_value = static_cast<uint8_t>(
                (normalized >> (byte_index * 8)) & 0xffu);
            hash_a ^= byte_value;
            hash_a *= fnv_prime;

            hash_b ^= static_cast<uint8_t>(byte_value + 0x9du);
            hash_b *= fnv_prime;
        }
    }

    std::ostringstream stream;
    stream << "wmf_" << std::hex << std::nouppercase << std::setfill('0')
           << std::setw(16) << hash_a
           << std::setw(16) << hash_b;
    return stream.str();
}

bool GenericWebcamEnumerator::enumerateAllDevices(
    std::vector<GenericWebcamDeviceInfo> &devices,
    bool include_media_modes,
    std::string *error_message)
{
    devices.clear();
    if (error_message != nullptr)
    {
        error_message->clear();
    }

#ifdef _WIN32
    MediaFoundationScope runtime;
    if (FAILED(runtime.result()))
    {
        if (error_message != nullptr)
        {
            *error_message =
                formatHRESULT("Media Foundation initialization", runtime.result());
        }
        return false;
    }

    ComPtr<IMFAttributes> attributes;
    HRESULT result = MFCreateAttributes(attributes.GetAddressOf(), 1);
    if (SUCCEEDED(result))
    {
        result = attributes->SetGUID(
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
    }
    if (FAILED(result))
    {
        if (error_message != nullptr)
        {
            *error_message =
                formatHRESULT("Create webcam enumeration attributes", result);
        }
        return false;
    }

    IMFActivate **activations = nullptr;
    UINT32 activation_count = 0;
    result = MFEnumDeviceSources(
        attributes.Get(),
        &activations,
        &activation_count);
    if (FAILED(result))
    {
        if (error_message != nullptr)
        {
            *error_message = formatHRESULT("MFEnumDeviceSources", result);
        }
        return false;
    }

    for (UINT32 activation_index = 0;
         activation_index < activation_count;
         ++activation_index)
    {
        IMFActivate *activation = activations[activation_index];
        GenericWebcamDeviceInfo device;

        std::wstring friendly_name;
        std::wstring symbolic_link;
        const bool has_name = getAllocatedString(
            activation,
            MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
            friendly_name);
        const bool has_symbolic_link = getAllocatedString(
            activation,
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
            symbolic_link);

        if (has_symbolic_link && !symbolic_link.empty())
        {
            device.symbolic_link = symbolic_link;
            device.symbolic_link_utf8 = wideToUtf8(symbolic_link);
            device.friendly_name =
                has_name ? wideToUtf8(friendly_name) : "Unnamed webcam";
            device.stable_id = makeStableId(symbolic_link);

            std::wstring normalized_link = symbolic_link;
            std::transform(
                normalized_link.begin(),
                normalized_link.end(),
                normalized_link.begin(),
                [](wchar_t value)
                {
                    return static_cast<wchar_t>(std::towlower(value));
                });
            device.vendor_id =
                parseHexDeviceIdentifier(normalized_link, L"vid_");
            device.product_id =
                parseHexDeviceIdentifier(normalized_link, L"pid_");

            UINT32 hardware_source = 0;
            if (SUCCEEDED(activation->GetUINT32(
                    MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_HW_SOURCE,
                    &hardware_source)))
            {
                device.hardware_source = hardware_source != 0;
            }

            if (include_media_modes)
            {
                enumerateMediaModes(
                    activation,
                    device.media_modes,
                    device.mode_enumeration_error);
            }
            devices.push_back(device);
        }

        activation->Release();
    }
    CoTaskMemFree(activations);

    std::sort(
        devices.begin(),
        devices.end(),
        [](const GenericWebcamDeviceInfo &left,
           const GenericWebcamDeviceInfo &right)
        {
            return left.stable_id < right.stable_id;
        });
    return true;
#else
    (void)include_media_modes;
    if (error_message != nullptr)
    {
        *error_message =
            "Generic webcam enumeration is available only on Windows.";
    }
    return false;
#endif
}
