#include "WindowsMediaFoundationVideoSource.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cwctype>
#include <cstring>
#include <exception>
#include <iomanip>
#include <limits>
#include <mutex>
#include <new>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <dshow.h>
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <objbase.h>
#include <wrl/client.h>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace
{
    using Microsoft::WRL::ComPtr;

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

    std::string guidToString(REFGUID guid)
    {
        wchar_t buffer[64] = {};
        const int length =
            StringFromGUID2(guid, buffer, static_cast<int>(_countof(buffer)));
        if (length <= 1)
        {
            return std::string();
        }

        std::wstring value(buffer, static_cast<std::size_t>(length - 1));
        std::transform(
            value.begin(),
            value.end(),
            value.begin(),
            [](wchar_t character)
            {
                return static_cast<wchar_t>(std::towlower(character));
            });
        return wideToUtf8(value);
    }

    bool stringToGuid(const std::string &value, GUID &guid)
    {
        std::wstring wide_value;
        wide_value.reserve(value.size());
        for (unsigned char character : value)
        {
            if (character > 0x7f)
            {
                return false;
            }
            wide_value.push_back(static_cast<wchar_t>(character));
        }

        return SUCCEEDED(CLSIDFromString(
            const_cast<wchar_t *>(wide_value.c_str()),
            &guid));
    }

    void describeSubtype(REFGUID subtype, TrackerVideoMediaMode &mode)
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

        GUID subtype = GUID_NULL;
        if (FAILED(MFGetAttributeSize(
                media_type,
                MF_MT_FRAME_SIZE,
                &mode.width,
                &mode.height))
            || FAILED(MFGetAttributeRatio(
                media_type,
                MF_MT_FRAME_RATE,
                &mode.frame_rate_numerator,
                &mode.frame_rate_denominator))
            || FAILED(media_type->GetGUID(MF_MT_SUBTYPE, &subtype)))
        {
            return false;
        }

        mode.subtype_guid = guidToString(subtype);
        UINT32 interlace_mode = 0;
        if (SUCCEEDED(media_type->GetUINT32(
                MF_MT_INTERLACE_MODE,
                &interlace_mode)))
        {
            mode.interlace_mode = interlace_mode;
        }

        UINT32 stride = 0;
        if (SUCCEEDED(media_type->GetUINT32(MF_MT_DEFAULT_STRIDE, &stride)))
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

    bool modesMatch(
        const TrackerVideoMediaMode &requested,
        const TrackerVideoMediaMode &candidate)
    {
        return requested == candidate;
    }

    template <typename ControlInterface>
    bool getNormalizedControl(
        ControlInterface *control,
        long property,
        long automatic_flag,
        long manual_flag,
        const char *control_name,
        uint8_t &value,
        std::string &error_message)
    {
        if (control == nullptr)
        {
            error_message =
                std::string(control_name)
                + " is not supported by this webcam.";
            return false;
        }

        long minimum = 0;
        long maximum = 0;
        long step = 0;
        long default_value = 0;
        long capabilities = 0;
        HRESULT result = control->GetRange(
            property,
            &minimum,
            &maximum,
            &step,
            &default_value,
            &capabilities);
        if (FAILED(result))
        {
            error_message =
                formatHRESULT(
                    (std::string("Read ") + control_name + " range").c_str(),
                    result);
            return false;
        }

        long raw_value = 0;
        long flags = 0;
        result = control->Get(property, &raw_value, &flags);
        if (FAILED(result))
        {
            error_message =
                formatHRESULT(
                    (std::string("Read ") + control_name).c_str(),
                    result);
            return false;
        }

        if ((flags & automatic_flag) != 0)
        {
            if ((capabilities & automatic_flag) == 0)
            {
                error_message =
                    std::string(control_name)
                    + " reported automatic mode without advertising it.";
                return false;
            }
            value = 0;
            return true;
        }
        if ((capabilities & manual_flag) == 0)
        {
            error_message =
                std::string(control_name)
                + " does not expose a readable manual mode.";
            return false;
        }

        value = TrackerVideoControlNormalization::fromDeviceValue(
            raw_value,
            minimum,
            maximum);
        return true;
    }

    template <typename ControlInterface>
    bool setNormalizedControl(
        ControlInterface *control,
        long property,
        long automatic_flag,
        long manual_flag,
        const char *control_name,
        uint8_t requested_value,
        uint8_t *actual_value,
        std::string &error_message)
    {
        if (control == nullptr)
        {
            error_message =
                std::string(control_name)
                + " is not supported by this webcam.";
            return false;
        }

        long minimum = 0;
        long maximum = 0;
        long step = 0;
        long default_value = 0;
        long capabilities = 0;
        HRESULT result = control->GetRange(
            property,
            &minimum,
            &maximum,
            &step,
            &default_value,
            &capabilities);
        if (FAILED(result))
        {
            error_message =
                formatHRESULT(
                    (std::string("Read ") + control_name + " range").c_str(),
                    result);
            return false;
        }

        long raw_value = default_value;
        long requested_flag = automatic_flag;
        if (requested_value == 0)
        {
            if ((capabilities & automatic_flag) == 0)
            {
                error_message =
                    std::string(control_name)
                    + " does not support automatic mode.";
                return false;
            }
        }
        else
        {
            if ((capabilities & manual_flag) == 0)
            {
                error_message =
                    std::string(control_name)
                    + " does not support manual mode.";
                return false;
            }
            requested_flag = manual_flag;
            raw_value = TrackerVideoControlNormalization::toDeviceValue(
                requested_value,
                minimum,
                maximum,
                step);
        }

        result = control->Set(property, raw_value, requested_flag);
        if (FAILED(result))
        {
            error_message =
                formatHRESULT(
                    (std::string("Set ") + control_name).c_str(),
                    result);
            return false;
        }

        uint8_t observed_value = 0;
        if (!getNormalizedControl(
                control,
                property,
                automatic_flag,
                manual_flag,
                control_name,
                observed_value,
                error_message))
        {
            return false;
        }
        if (actual_value != nullptr)
        {
            *actual_value = observed_value;
        }
        return true;
    }

    class MediaBufferLock
    {
    public:
        explicit MediaBufferLock(IMFMediaBuffer *buffer)
            : m_buffer(buffer)
            , m_data(nullptr)
            , m_currentLength(0)
            , m_stride(0)
            , m_usesTwoDimensionalLock(false)
            , m_result(E_POINTER)
        {
            if (m_buffer == nullptr)
            {
                return;
            }

            if (SUCCEEDED(m_buffer->QueryInterface(
                    __uuidof(IMF2DBuffer),
                    reinterpret_cast<void **>(
                        m_twoDimensionalBuffer.GetAddressOf()))))
            {
                m_result =
                    m_twoDimensionalBuffer->Lock2D(&m_data, &m_stride);
                if (SUCCEEDED(m_result))
                {
                    m_usesTwoDimensionalLock = true;
                    DWORD contiguous_length = 0;
                    if (SUCCEEDED(
                            m_twoDimensionalBuffer->GetContiguousLength(
                                &contiguous_length)))
                    {
                        m_currentLength = contiguous_length;
                    }
                    else
                    {
                        m_buffer->GetCurrentLength(&m_currentLength);
                    }
                    return;
                }
                m_twoDimensionalBuffer.Reset();
            }

            DWORD maximum_length = 0;
            m_result = m_buffer->Lock(
                &m_data,
                &maximum_length,
                &m_currentLength);
        }

        ~MediaBufferLock()
        {
            if (SUCCEEDED(m_result))
            {
                if (m_usesTwoDimensionalLock)
                {
                    m_twoDimensionalBuffer->Unlock2D();
                }
                else
                {
                    m_buffer->Unlock();
                }
            }
        }

        HRESULT result() const
        {
            return m_result;
        }

        const BYTE *data() const
        {
            return m_data;
        }

        DWORD length() const
        {
            return m_currentLength;
        }

        bool usesTwoDimensionalLock() const
        {
            return m_usesTwoDimensionalLock;
        }

        LONG stride() const
        {
            return m_stride;
        }

    private:
        IMFMediaBuffer *m_buffer;
        ComPtr<IMF2DBuffer> m_twoDimensionalBuffer;
        BYTE *m_data;
        DWORD m_currentLength;
        LONG m_stride;
        bool m_usesTwoDimensionalLock;
        HRESULT m_result;
    };

    bool copyPackedRows(
        const BYTE *source,
        std::size_t source_length,
        uint32_t height,
        std::size_t row_bytes,
        int32_t stride,
        bool source_points_to_scanline_zero,
        bool validate_source_length,
        cv::Mat &destination,
        std::string &error_message)
    {
        if (source == nullptr || height == 0 || row_bytes == 0)
        {
            error_message = "The camera returned an empty video buffer.";
            return false;
        }

        if (stride == 0)
        {
            if (row_bytes
                > static_cast<std::size_t>((std::numeric_limits<int32_t>::max)()))
            {
                error_message = "The camera row is too wide to address safely.";
                return false;
            }
            stride = static_cast<int32_t>(row_bytes);
        }

        const int64_t signed_stride = stride;
        const uint64_t absolute_stride =
            signed_stride < 0
                ? static_cast<uint64_t>(-signed_stride)
                : static_cast<uint64_t>(signed_stride);
        if (absolute_stride < row_bytes)
        {
            error_message =
                "The camera media stride is smaller than one image row.";
            return false;
        }

        const uint64_t required_bytes =
            (static_cast<uint64_t>(height) - 1) * absolute_stride + row_bytes;
        if (validate_source_length && required_bytes > source_length)
        {
            error_message =
                "The camera media buffer is shorter than its declared mode.";
            return false;
        }

        const BYTE *top_row = source;
        if (!source_points_to_scanline_zero && signed_stride < 0)
        {
            top_row =
                source + (static_cast<uint64_t>(height) - 1) * absolute_stride;
        }
        for (uint32_t row_index = 0; row_index < height; ++row_index)
        {
            const std::ptrdiff_t row_offset =
                static_cast<std::ptrdiff_t>(row_index)
                * static_cast<std::ptrdiff_t>(
                    source_points_to_scanline_zero
                        ? signed_stride
                        : (signed_stride < 0
                            ? -static_cast<int64_t>(absolute_stride)
                            : static_cast<int64_t>(absolute_stride)));
            const BYTE *source_row = top_row + row_offset;
            std::memcpy(
                destination.ptr(static_cast<int>(row_index)),
                source_row,
                row_bytes);
        }
        return true;
    }

    bool convertSampleToBgr(
        IMFSample *sample,
        const TrackerVideoMediaMode &mode,
        REFGUID subtype,
        cv::Mat &bgr,
        std::string &error_message)
    {
        bgr.release();
        error_message.clear();

        ComPtr<IMFMediaBuffer> buffer;
        HRESULT result =
            sample->ConvertToContiguousBuffer(buffer.GetAddressOf());
        if (FAILED(result))
        {
            error_message =
                formatHRESULT("IMFSample::ConvertToContiguousBuffer", result);
            return false;
        }

        MediaBufferLock lock(buffer.Get());
        if (FAILED(lock.result()))
        {
            error_message =
                formatHRESULT("IMFMediaBuffer::Lock", lock.result());
            return false;
        }

        const uint32_t width = mode.width;
        const uint32_t height = mode.height;
        if (width == 0 || height == 0
            || width > static_cast<uint32_t>((std::numeric_limits<int>::max)())
            || height > static_cast<uint32_t>((std::numeric_limits<int>::max)()))
        {
            error_message = "The selected camera dimensions are invalid.";
            return false;
        }

        const int32_t input_stride =
            lock.usesTwoDimensionalLock()
                ? static_cast<int32_t>(lock.stride())
                : mode.default_stride;

        if (IsEqualGUID(subtype, MFVideoFormat_MJPG))
        {
            if (lock.length() == 0
                || lock.length()
                    > static_cast<DWORD>((std::numeric_limits<int>::max)()))
            {
                error_message = "The MJPG webcam sample length is invalid.";
                return false;
            }
            const cv::Mat encoded(
                1,
                static_cast<int>(lock.length()),
                CV_8UC1,
                const_cast<BYTE *>(lock.data()));
            bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_RGB24))
        {
            bgr.create(static_cast<int>(height), static_cast<int>(width), CV_8UC3);
            if (!copyPackedRows(
                    lock.data(),
                    lock.length(),
                    height,
                    static_cast<std::size_t>(width) * 3,
                    input_stride,
                    lock.usesTwoDimensionalLock(),
                    !lock.usesTwoDimensionalLock(),
                    bgr,
                    error_message))
            {
                return false;
            }
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_RGB32)
                 || IsEqualGUID(subtype, MFVideoFormat_ARGB32))
        {
            cv::Mat bgra(
                static_cast<int>(height),
                static_cast<int>(width),
                CV_8UC4);
            if (!copyPackedRows(
                    lock.data(),
                    lock.length(),
                    height,
                    static_cast<std::size_t>(width) * 4,
                    input_stride,
                    lock.usesTwoDimensionalLock(),
                    !lock.usesTwoDimensionalLock(),
                    bgra,
                    error_message))
            {
                return false;
            }
            cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_YUY2)
                 || IsEqualGUID(subtype, MFVideoFormat_UYVY))
        {
            cv::Mat packed(
                static_cast<int>(height),
                static_cast<int>(width),
                CV_8UC2);
            if (!copyPackedRows(
                    lock.data(),
                    lock.length(),
                    height,
                    static_cast<std::size_t>(width) * 2,
                    input_stride,
                    lock.usesTwoDimensionalLock(),
                    !lock.usesTwoDimensionalLock(),
                    packed,
                    error_message))
            {
                return false;
            }
            cv::cvtColor(
                packed,
                bgr,
                IsEqualGUID(subtype, MFVideoFormat_YUY2)
                    ? cv::COLOR_YUV2BGR_YUY2
                    : cv::COLOR_YUV2BGR_UYVY);
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_NV12))
        {
            if ((width & 1u) != 0 || (height & 1u) != 0)
            {
                error_message = "NV12 camera modes require even dimensions.";
                return false;
            }

            const int32_t stride =
                input_stride == 0
                    ? static_cast<int32_t>(width)
                    : input_stride;
            if (stride < static_cast<int32_t>(width))
            {
                error_message =
                    "Unsupported negative or undersized NV12 camera stride.";
                return false;
            }

            const uint64_t required_bytes =
                static_cast<uint64_t>(stride)
                * (height + height / 2);
            if (!lock.usesTwoDimensionalLock()
                && required_bytes > lock.length())
            {
                error_message =
                    "The NV12 camera buffer is shorter than its declared mode.";
                return false;
            }

            cv::Mat yuv(
                static_cast<int>(height + height / 2),
                static_cast<int>(width),
                CV_8UC1);
            for (uint32_t row = 0; row < height; ++row)
            {
                std::memcpy(
                    yuv.ptr(static_cast<int>(row)),
                    lock.data() + static_cast<uint64_t>(row) * stride,
                    width);
            }
            const BYTE *uv_source =
                lock.data() + static_cast<uint64_t>(stride) * height;
            for (uint32_t row = 0; row < height / 2; ++row)
            {
                std::memcpy(
                    yuv.ptr(static_cast<int>(height + row)),
                    uv_source + static_cast<uint64_t>(row) * stride,
                    width);
            }
            cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
        }
        else if (IsEqualGUID(subtype, MFVideoFormat_I420)
                 || IsEqualGUID(subtype, MFVideoFormat_IYUV)
                 || IsEqualGUID(subtype, MFVideoFormat_YV12))
        {
            if ((width & 1u) != 0 || (height & 1u) != 0)
            {
                error_message =
                    "Planar YUV camera modes require even dimensions.";
                return false;
            }

            const int32_t luma_stride =
                input_stride == 0
                    ? static_cast<int32_t>(width)
                    : input_stride;
            if (luma_stride < static_cast<int32_t>(width))
            {
                error_message =
                    "Unsupported negative or undersized planar YUV stride.";
                return false;
            }
            const int32_t chroma_stride = (luma_stride + 1) / 2;
            const uint64_t luma_bytes =
                static_cast<uint64_t>(luma_stride) * height;
            const uint64_t chroma_plane_bytes =
                static_cast<uint64_t>(chroma_stride) * (height / 2);
            if (!lock.usesTwoDimensionalLock()
                && luma_bytes + 2 * chroma_plane_bytes > lock.length())
            {
                error_message =
                    "The planar YUV buffer is shorter than its declared mode.";
                return false;
            }

            cv::Mat yuv(
                static_cast<int>(height + height / 2),
                static_cast<int>(width),
                CV_8UC1);
            for (uint32_t row = 0; row < height; ++row)
            {
                std::memcpy(
                    yuv.ptr(static_cast<int>(row)),
                    lock.data() + static_cast<uint64_t>(row) * luma_stride,
                    width);
            }

            BYTE *chroma_destination =
                yuv.ptr(static_cast<int>(height));
            const BYTE *first_chroma_source = lock.data() + luma_bytes;
            const BYTE *second_chroma_source =
                first_chroma_source + chroma_plane_bytes;
            const std::size_t chroma_row_bytes = width / 2;
            const uint32_t chroma_height = height / 2;
            for (uint32_t row = 0; row < chroma_height; ++row)
            {
                std::memcpy(
                    chroma_destination
                        + static_cast<uint64_t>(row) * chroma_row_bytes,
                    first_chroma_source
                        + static_cast<uint64_t>(row) * chroma_stride,
                    chroma_row_bytes);
                std::memcpy(
                    chroma_destination
                        + static_cast<uint64_t>(chroma_height + row)
                            * chroma_row_bytes,
                    second_chroma_source
                        + static_cast<uint64_t>(row) * chroma_stride,
                    chroma_row_bytes);
            }

            cv::cvtColor(
                yuv,
                bgr,
                IsEqualGUID(subtype, MFVideoFormat_YV12)
                    ? cv::COLOR_YUV2BGR_YV12
                    : cv::COLOR_YUV2BGR_I420);
        }
        else
        {
            error_message =
                "The selected webcam pixel format is not supported.";
            return false;
        }

        if (bgr.empty()
            || bgr.type() != CV_8UC3
            || bgr.cols != static_cast<int>(width)
            || bgr.rows != static_cast<int>(height))
        {
            error_message =
                "The decoded webcam frame does not match the selected mode.";
            bgr.release();
            return false;
        }
        if (!bgr.isContinuous())
        {
            bgr = bgr.clone();
        }
        return true;
    }

    struct CaptureState
    {
        explicit CaptureState(std::chrono::milliseconds latency_offset)
            : timestamp_mapper(latency_offset)
            , stopping(false)
            , open(false)
            , sequence(0)
            , callbacks_in_flight(0)
            , flush_received(false)
            , source_timestamp_seen(false)
            , last_source_timestamp(0)
            , subtype(GUID_NULL)
        {
        }

        void setError(const std::string &message)
        {
            std::lock_guard<std::mutex> lock(status_mutex);
            last_error = message;
        }

        std::string getError() const
        {
            std::lock_guard<std::mutex> lock(status_mutex);
            return last_error;
        }

        void callbackEntered()
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            ++callbacks_in_flight;
        }

        void callbackExited()
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            --callbacks_in_flight;
            callback_condition.notify_all();
        }

        void prepareForFlush()
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            flush_received = false;
        }

        void flushReceived()
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            flush_received = true;
            callback_condition.notify_all();
        }

        bool waitForCallbacks(std::chrono::milliseconds timeout)
        {
            std::unique_lock<std::mutex> lock(callback_mutex);
            return callback_condition.wait_for(
                lock,
                timeout,
                [this]()
                {
                    return flush_received && callbacks_in_flight == 0;
                });
        }

        void fail(const std::string &message)
        {
            setError(message);
            open.store(false);
            stopping.store(true);
        }

        TrackerVideoLatestFrameBuffer frames;
        TrackerVideoTimestampMapper timestamp_mapper;
        TrackerVideoMediaMode mode;
        std::string stable_id;
        std::atomic<bool> stopping;
        std::atomic<bool> open;
        std::atomic<uint64_t> sequence;

        mutable std::mutex status_mutex;
        std::string last_error;

        std::mutex callback_mutex;
        std::condition_variable callback_condition;
        unsigned int callbacks_in_flight;
        bool flush_received;

        bool source_timestamp_seen;
        int64_t last_source_timestamp;
        GUID subtype;
    };

    class CallbackGuard
    {
    public:
        explicit CallbackGuard(const std::shared_ptr<CaptureState> &state)
            : m_state(state)
        {
            m_state->callbackEntered();
        }

        ~CallbackGuard()
        {
            m_state->callbackExited();
        }

    private:
        std::shared_ptr<CaptureState> m_state;
    };

    class SourceReaderCallback final : public IMFSourceReaderCallback
    {
    public:
        explicit SourceReaderCallback(
            const std::shared_ptr<CaptureState> &state)
            : m_referenceCount(1)
            , m_state(state)
            , m_reader(nullptr)
        {
        }

        void setReader(IMFSourceReader *reader)
        {
            std::lock_guard<std::mutex> lock(m_readerMutex);
            m_reader = reader;
        }

        STDMETHODIMP QueryInterface(REFIID interface_id, void **object) override
        {
            if (object == nullptr)
            {
                return E_POINTER;
            }
            if (interface_id == __uuidof(IUnknown)
                || interface_id == __uuidof(IMFSourceReaderCallback))
            {
                *object = static_cast<IMFSourceReaderCallback *>(this);
                AddRef();
                return S_OK;
            }
            *object = nullptr;
            return E_NOINTERFACE;
        }

        STDMETHODIMP_(ULONG) AddRef() override
        {
            return ++m_referenceCount;
        }

        STDMETHODIMP_(ULONG) Release() override
        {
            const ULONG remaining = --m_referenceCount;
            if (remaining == 0)
            {
                delete this;
            }
            return remaining;
        }

        STDMETHODIMP OnReadSample(
            HRESULT status,
            DWORD stream_index,
            DWORD stream_flags,
            LONGLONG timestamp,
            IMFSample *sample) override
        {
            (void)stream_index;
            CallbackGuard guard(m_state);
            const std::chrono::steady_clock::time_point arrival_time =
                std::chrono::steady_clock::now();

            try
            {
            if (m_state->stopping.load())
            {
                return S_OK;
            }

            if (FAILED(status))
            {
                m_state->fail(
                    formatHRESULT("Asynchronous webcam read", status));
                return S_OK;
            }
            if ((stream_flags & MF_SOURCE_READERF_ERROR) != 0)
            {
                m_state->fail(
                    "Media Foundation reported an asynchronous webcam error.");
                return S_OK;
            }
            if ((stream_flags & MF_SOURCE_READERF_ENDOFSTREAM) != 0)
            {
                m_state->fail("The webcam video stream ended unexpectedly.");
                return S_OK;
            }
            if ((stream_flags
                 & (MF_SOURCE_READERF_NATIVEMEDIATYPECHANGED
                    | MF_SOURCE_READERF_CURRENTMEDIATYPECHANGED))
                != 0)
            {
                m_state->fail(
                    "The webcam changed media mode while tracking. "
                    "Re-open the device and recalibrate.");
                return S_OK;
            }

            const bool stream_discontinuity =
                (stream_flags & MF_SOURCE_READERF_STREAMTICK) != 0;
            if (stream_discontinuity)
            {
                m_state->timestamp_mapper.reset();
                m_state->source_timestamp_seen = false;
            }

            if (sample != nullptr)
            {
                cv::Mat bgr;
                std::string conversion_error;
                if (!convertSampleToBgr(
                        sample,
                        m_state->mode,
                        m_state->subtype,
                        bgr,
                        conversion_error))
                {
                    m_state->fail(conversion_error);
                    return S_OK;
                }

                bool timestamp_discontinuity = stream_discontinuity;
                const bool source_timestamp_valid = timestamp >= 0;
                if (source_timestamp_valid
                    && m_state->source_timestamp_seen
                    && timestamp <= m_state->last_source_timestamp)
                {
                    timestamp_discontinuity = true;
                }

                bool used_source_timestamp = false;
                const std::chrono::steady_clock::time_point capture_time =
                    m_state->timestamp_mapper.map(
                        timestamp,
                        arrival_time,
                        source_timestamp_valid,
                        timestamp_discontinuity,
                        &used_source_timestamp);

                if (source_timestamp_valid)
                {
                    m_state->source_timestamp_seen = true;
                    m_state->last_source_timestamp = timestamp;
                }
                else
                {
                    m_state->source_timestamp_seen = false;
                }

                TrackerVideoFrame frame;
                frame.bgr = std::move(bgr);
                frame.sequence = ++m_state->sequence;
                frame.capture_time = capture_time;
                frame.arrival_time = arrival_time;
                frame.source_timestamp_100ns = timestamp;
                frame.source_timestamp_valid = used_source_timestamp;
                if (!m_state->frames.push(std::move(frame)))
                {
                    m_state->fail(
                        "The webcam frame could not be queued as BGR8.");
                    return S_OK;
                }
            }

            if (!m_state->stopping.load())
            {
                IMFSourceReader *reader = acquireReader();
                if (reader == nullptr)
                {
                    m_state->fail(
                        "The webcam source reader was released unexpectedly.");
                    return S_OK;
                }

                const HRESULT read_result = reader->ReadSample(
                    MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                    0,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr);
                reader->Release();
                if (FAILED(read_result))
                {
                    m_state->fail(formatHRESULT(
                        "IMFSourceReader::ReadSample",
                        read_result));
                }
            }
            return S_OK;
            }
            catch (const cv::Exception &exception)
            {
                m_state->fail(
                    std::string("OpenCV failed while decoding a webcam frame: ")
                    + exception.what());
                return S_OK;
            }
            catch (const std::exception &exception)
            {
                m_state->fail(
                    std::string("Unexpected webcam callback failure: ")
                    + exception.what());
                return S_OK;
            }
            catch (...)
            {
                m_state->fail(
                    "Unknown failure in the asynchronous webcam callback.");
                return S_OK;
            }
        }

        STDMETHODIMP OnFlush(DWORD stream_index) override
        {
            (void)stream_index;
            CallbackGuard guard(m_state);
            m_state->flushReceived();
            return S_OK;
        }

        STDMETHODIMP OnEvent(
            DWORD stream_index,
            IMFMediaEvent *event) override
        {
            (void)stream_index;
            (void)event;
            CallbackGuard guard(m_state);
            return S_OK;
        }

    private:
        IMFSourceReader *acquireReader()
        {
            std::lock_guard<std::mutex> lock(m_readerMutex);
            if (m_reader != nullptr)
            {
                m_reader->AddRef();
            }
            return m_reader;
        }

        std::atomic<ULONG> m_referenceCount;
        std::shared_ptr<CaptureState> m_state;
        std::mutex m_readerMutex;
        IMFSourceReader *m_reader;
    };
}

#endif // _WIN32

class WindowsMediaFoundationVideoSource::Impl
{
public:
    Impl()
#ifdef _WIN32
        : m_comInitialized(false)
        , m_mediaFoundationStarted(false)
        , m_openThreadId(0)
#endif
    {
    }

    ~Impl()
    {
        close();
    }

    bool open(
        const GenericWebcamDeviceInfo &device,
        const TrackerVideoMediaMode &requested_mode,
        std::chrono::milliseconds latency_offset)
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
        closeLocked();
        m_lastError.clear();

#ifdef _WIN32
        m_state = std::make_shared<CaptureState>(latency_offset);
        m_state->stable_id = device.stable_id;

        if (device.stable_id.empty() || device.symbolic_link.empty())
        {
            m_state->setError(
                "A webcam descriptor with a Media Foundation symbolic link "
                "is required.");
            return false;
        }
        if (!requested_mode.isValid())
        {
            m_state->setError("An explicit, valid webcam media mode is required.");
            return false;
        }
        if (!requested_mode.bgr_conversion_supported)
        {
            m_state->setError(
                "The selected webcam pixel format cannot be converted to BGR8.");
            return false;
        }
        if (requested_mode.interlace_mode != 0
            && requested_mode.interlace_mode != MFVideoInterlace_Progressive)
        {
            m_state->setError(
                "Interlaced webcam modes are not supported for optical tracking.");
            return false;
        }

        GUID requested_subtype = GUID_NULL;
        if (!stringToGuid(requested_mode.subtype_guid, requested_subtype))
        {
            m_state->setError(
                "The selected webcam mode has an invalid subtype GUID.");
            return false;
        }

        m_openThreadId = GetCurrentThreadId();
        HRESULT result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        if (SUCCEEDED(result))
        {
            m_comInitialized = true;
        }
        else if (result != RPC_E_CHANGED_MODE)
        {
            m_state->setError(formatHRESULT("CoInitializeEx", result));
            return false;
        }

        result = MFStartup(MF_VERSION, MFSTARTUP_FULL);
        if (FAILED(result))
        {
            m_state->setError(formatHRESULT("MFStartup", result));
            shutdownRuntimeLocked();
            return false;
        }
        m_mediaFoundationStarted = true;

        ComPtr<IMFAttributes> source_attributes;
        result = MFCreateAttributes(source_attributes.GetAddressOf(), 2);
        if (SUCCEEDED(result))
        {
            result = source_attributes->SetGUID(
                MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        }
        if (SUCCEEDED(result))
        {
            result = source_attributes->SetString(
                MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
                device.symbolic_link.c_str());
        }
        if (SUCCEEDED(result))
        {
            result = MFCreateDeviceSource(
                source_attributes.Get(),
                m_mediaSource.GetAddressOf());
        }
        if (FAILED(result))
        {
            m_state->setError(
                formatHRESULT("Open webcam by symbolic link", result));
            closeCaptureLocked();
            shutdownRuntimeLocked();
            return false;
        }

        // UVC Media Foundation sources commonly expose the DirectShow camera
        // control interfaces through QueryInterface. Their absence is a
        // per-capability condition, not a capture-open failure.
        m_mediaSource.As(&m_cameraControl);
        m_mediaSource.As(&m_videoProcAmp);

        SourceReaderCallback *callback =
            new (std::nothrow) SourceReaderCallback(m_state);
        if (callback == nullptr)
        {
            m_state->setError(
                "Unable to allocate the asynchronous webcam callback.");
            closeCaptureLocked();
            shutdownRuntimeLocked();
            return false;
        }
        m_callback.Attach(callback);

        ComPtr<IMFAttributes> reader_attributes;
        result = MFCreateAttributes(reader_attributes.GetAddressOf(), 1);
        if (SUCCEEDED(result))
        {
            result = reader_attributes->SetUnknown(
                MF_SOURCE_READER_ASYNC_CALLBACK,
                m_callback.Get());
        }
        if (SUCCEEDED(result))
        {
            result = MFCreateSourceReaderFromMediaSource(
                m_mediaSource.Get(),
                reader_attributes.Get(),
                m_reader.GetAddressOf());
        }
        if (FAILED(result))
        {
            m_state->setError(
                formatHRESULT(
                    "Create asynchronous webcam source reader",
                    result));
            closeCaptureLocked();
            shutdownRuntimeLocked();
            return false;
        }

        m_reader->SetStreamSelection(MF_SOURCE_READER_ALL_STREAMS, FALSE);
        result = m_reader->SetStreamSelection(
            MF_SOURCE_READER_FIRST_VIDEO_STREAM,
            TRUE);
        if (FAILED(result))
        {
            m_state->setError(
                formatHRESULT("Select webcam video stream", result));
            closeCaptureLocked();
            shutdownRuntimeLocked();
            return false;
        }

        ComPtr<IMFMediaType> selected_media_type;
        for (DWORD type_index = 0; ; ++type_index)
        {
            ComPtr<IMFMediaType> candidate_media_type;
            result = m_reader->GetNativeMediaType(
                MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                type_index,
                candidate_media_type.GetAddressOf());
            if (result == MF_E_NO_MORE_TYPES)
            {
                break;
            }
            if (FAILED(result))
            {
                m_state->setError(formatHRESULT(
                    "Enumerate webcam native modes",
                    result));
                closeCaptureLocked();
                shutdownRuntimeLocked();
                return false;
            }

            TrackerVideoMediaMode candidate_mode;
            if (mediaTypeToMode(candidate_media_type.Get(), candidate_mode)
                && modesMatch(requested_mode, candidate_mode))
            {
                selected_media_type = candidate_media_type;
                break;
            }
        }

        if (selected_media_type == nullptr)
        {
            m_state->setError(
                "The selected webcam no longer exposes the requested native "
                "media mode. Refresh the device list.");
            closeCaptureLocked();
            shutdownRuntimeLocked();
            return false;
        }

        result = m_reader->SetCurrentMediaType(
            MF_SOURCE_READER_FIRST_VIDEO_STREAM,
            nullptr,
            selected_media_type.Get());
        if (FAILED(result))
        {
            m_state->setError(
                formatHRESULT("Set exact webcam media mode", result));
            closeCaptureLocked();
            shutdownRuntimeLocked();
            return false;
        }

        ComPtr<IMFMediaType> actual_media_type;
        result = m_reader->GetCurrentMediaType(
            MF_SOURCE_READER_FIRST_VIDEO_STREAM,
            actual_media_type.GetAddressOf());
        TrackerVideoMediaMode actual_mode;
        if (FAILED(result)
            || !mediaTypeToMode(actual_media_type.Get(), actual_mode)
            || !modesMatch(requested_mode, actual_mode))
        {
            m_state->setError(
                FAILED(result)
                    ? formatHRESULT("Verify webcam media mode", result)
                    : "The webcam did not accept the requested native media mode.");
            closeCaptureLocked();
            shutdownRuntimeLocked();
            return false;
        }

        GUID actual_subtype = GUID_NULL;
        result = actual_media_type->GetGUID(MF_MT_SUBTYPE, &actual_subtype);
        if (FAILED(result))
        {
            m_state->setError(
                formatHRESULT("Read active webcam subtype", result));
            closeCaptureLocked();
            shutdownRuntimeLocked();
            return false;
        }

        m_state->mode = actual_mode;
        m_state->subtype = actual_subtype;
        m_callback->setReader(m_reader.Get());
        m_state->stopping.store(false);
        m_state->open.store(true);

        result = m_reader->ReadSample(
            MF_SOURCE_READER_FIRST_VIDEO_STREAM,
            0,
            nullptr,
            nullptr,
            nullptr,
            nullptr);
        if (FAILED(result))
        {
            m_state->fail(
                formatHRESULT("Start asynchronous webcam capture", result));
            closeCaptureLocked();
            shutdownRuntimeLocked();
            return false;
        }
        return true;
#else
        (void)device;
        (void)requested_mode;
        (void)latency_offset;
        m_lastError =
            "Windows Media Foundation webcam capture is available only on Windows.";
        return false;
#endif
    }

    bool isOpen() const
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
#ifdef _WIN32
        return m_state != nullptr && m_state->open.load();
#else
        return false;
#endif
    }

    void close()
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
        closeLocked();
    }

    bool hasNewFrame() const
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
#ifdef _WIN32
        return m_state != nullptr && m_state->frames.hasNewFrame();
#else
        return false;
#endif
    }

    bool tryGetLatestFrame(TrackerVideoFrame &frame)
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
#ifdef _WIN32
        return m_state != nullptr && m_state->frames.tryGetLatestFrame(frame);
#else
        (void)frame;
        return false;
#endif
    }

    TrackerVideoMediaMode getActiveMode() const
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
#ifdef _WIN32
        return m_state != nullptr
            ? m_state->mode
            : TrackerVideoMediaMode();
#else
        return TrackerVideoMediaMode();
#endif
    }

    std::string getDeviceStableId() const
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
#ifdef _WIN32
        return m_state != nullptr ? m_state->stable_id : std::string();
#else
        return std::string();
#endif
    }

    std::string getLastError() const
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
#ifdef _WIN32
        return m_state != nullptr ? m_state->getError() : m_lastError;
#else
        return m_lastError;
#endif
    }

    bool getExposure(uint8_t &value)
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
#ifdef _WIN32
        std::string error_message;
        if (!getNormalizedControl(
                m_cameraControl.Get(),
                CameraControl_Exposure,
                CameraControl_Flags_Auto,
                CameraControl_Flags_Manual,
                "webcam exposure",
                value,
                error_message))
        {
            if (m_state != nullptr)
            {
                m_state->setError(error_message);
            }
            else
            {
                m_lastError = error_message;
            }
            return false;
        }
        return true;
#else
        (void)value;
        m_lastError =
            "Webcam exposure control is available only on Windows.";
        return false;
#endif
    }

    bool setExposure(uint8_t requested_value, uint8_t *actual_value)
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
#ifdef _WIN32
        std::string error_message;
        if (!setNormalizedControl(
                m_cameraControl.Get(),
                CameraControl_Exposure,
                CameraControl_Flags_Auto,
                CameraControl_Flags_Manual,
                "webcam exposure",
                requested_value,
                actual_value,
                error_message))
        {
            if (m_state != nullptr)
            {
                m_state->setError(error_message);
            }
            else
            {
                m_lastError = error_message;
            }
            return false;
        }
        return true;
#else
        (void)requested_value;
        (void)actual_value;
        m_lastError =
            "Webcam exposure control is available only on Windows.";
        return false;
#endif
    }

    bool getGain(uint8_t &value)
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
#ifdef _WIN32
        std::string error_message;
        if (!getNormalizedControl(
                m_videoProcAmp.Get(),
                VideoProcAmp_Gain,
                VideoProcAmp_Flags_Auto,
                VideoProcAmp_Flags_Manual,
                "webcam gain",
                value,
                error_message))
        {
            if (m_state != nullptr)
            {
                m_state->setError(error_message);
            }
            else
            {
                m_lastError = error_message;
            }
            return false;
        }
        return true;
#else
        (void)value;
        m_lastError = "Webcam gain control is available only on Windows.";
        return false;
#endif
    }

    bool setGain(uint8_t requested_value, uint8_t *actual_value)
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
#ifdef _WIN32
        std::string error_message;
        if (!setNormalizedControl(
                m_videoProcAmp.Get(),
                VideoProcAmp_Gain,
                VideoProcAmp_Flags_Auto,
                VideoProcAmp_Flags_Manual,
                "webcam gain",
                requested_value,
                actual_value,
                error_message))
        {
            if (m_state != nullptr)
            {
                m_state->setError(error_message);
            }
            else
            {
                m_lastError = error_message;
            }
            return false;
        }
        return true;
#else
        (void)requested_value;
        (void)actual_value;
        m_lastError = "Webcam gain control is available only on Windows.";
        return false;
#endif
    }

private:
    void closeLocked()
    {
#ifdef _WIN32
        closeCaptureLocked();
        shutdownRuntimeLocked();
#endif
    }

#ifdef _WIN32
    void closeCaptureLocked()
    {
        if (m_state != nullptr)
        {
            m_state->stopping.store(true);
            m_state->open.store(false);
        }

        if (m_reader != nullptr && m_state != nullptr)
        {
            m_state->prepareForFlush();
            const HRESULT flush_result =
                m_reader->Flush(MF_SOURCE_READER_FIRST_VIDEO_STREAM);
            if (FAILED(flush_result))
            {
                m_state->setError(
                    formatHRESULT("Flush webcam source reader", flush_result));
            }
            else if (!m_state->waitForCallbacks(std::chrono::seconds(2)))
            {
                m_state->setError(
                    "Timed out waiting for webcam callbacks to stop.");
            }
        }

        if (m_callback != nullptr)
        {
            m_callback->setReader(nullptr);
        }
        m_reader.Reset();
        m_callback.Reset();
        m_cameraControl.Reset();
        m_videoProcAmp.Reset();

        if (m_mediaSource != nullptr)
        {
            const HRESULT shutdown_result = m_mediaSource->Shutdown();
            if (FAILED(shutdown_result)
                && shutdown_result != MF_E_SHUTDOWN
                && m_state != nullptr)
            {
                m_state->setError(
                    formatHRESULT(
                        "Shut down webcam media source",
                        shutdown_result));
            }
            m_mediaSource.Reset();
        }

        if (m_state != nullptr)
        {
            m_state->frames.clear();
        }
    }

    void shutdownRuntimeLocked()
    {
        if (m_mediaFoundationStarted)
        {
            const HRESULT result = MFShutdown();
            if (FAILED(result) && m_state != nullptr)
            {
                m_state->setError(formatHRESULT("MFShutdown", result));
            }
            m_mediaFoundationStarted = false;
        }

        if (m_comInitialized)
        {
            if (GetCurrentThreadId() == m_openThreadId)
            {
                CoUninitialize();
            }
            else if (m_state != nullptr)
            {
                m_state->setError(
                    "The webcam was closed on a different thread; the "
                    "opening thread's COM apartment could not be released.");
            }
            m_comInitialized = false;
        }
        m_openThreadId = 0;
    }

    std::shared_ptr<CaptureState> m_state;
    ComPtr<IMFMediaSource> m_mediaSource;
    ComPtr<IMFSourceReader> m_reader;
    ComPtr<SourceReaderCallback> m_callback;
    ComPtr<IAMCameraControl> m_cameraControl;
    ComPtr<IAMVideoProcAmp> m_videoProcAmp;
    bool m_comInitialized;
    bool m_mediaFoundationStarted;
    DWORD m_openThreadId;
#endif

    std::string m_lastError;
    mutable std::mutex m_lifecycleMutex;
};

WindowsMediaFoundationVideoSource::WindowsMediaFoundationVideoSource()
    : m_impl(new Impl())
{
}

WindowsMediaFoundationVideoSource::~WindowsMediaFoundationVideoSource()
{
}

bool WindowsMediaFoundationVideoSource::open(
    const GenericWebcamDeviceInfo &device,
    const TrackerVideoMediaMode &mode,
    std::chrono::milliseconds latency_offset)
{
    return m_impl->open(device, mode, latency_offset);
}

bool WindowsMediaFoundationVideoSource::isOpen() const
{
    return m_impl->isOpen();
}

void WindowsMediaFoundationVideoSource::close()
{
    m_impl->close();
}

bool WindowsMediaFoundationVideoSource::hasNewFrame() const
{
    return m_impl->hasNewFrame();
}

bool WindowsMediaFoundationVideoSource::tryGetLatestFrame(
    TrackerVideoFrame &frame)
{
    return m_impl->tryGetLatestFrame(frame);
}

TrackerVideoMediaMode
WindowsMediaFoundationVideoSource::getActiveMode() const
{
    return m_impl->getActiveMode();
}

std::string WindowsMediaFoundationVideoSource::getDeviceStableId() const
{
    return m_impl->getDeviceStableId();
}

std::string WindowsMediaFoundationVideoSource::getLastError() const
{
    return m_impl->getLastError();
}

bool WindowsMediaFoundationVideoSource::getExposure(uint8_t &value)
{
    return m_impl->getExposure(value);
}

bool WindowsMediaFoundationVideoSource::setExposure(
    uint8_t requested_value,
    uint8_t *actual_value)
{
    return m_impl->setExposure(requested_value, actual_value);
}

bool WindowsMediaFoundationVideoSource::getGain(uint8_t &value)
{
    return m_impl->getGain(value);
}

bool WindowsMediaFoundationVideoSource::setGain(
    uint8_t requested_value,
    uint8_t *actual_value)
{
    return m_impl->setGain(requested_value, actual_value);
}
