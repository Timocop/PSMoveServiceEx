// -- includes -----
#include "TrackerDeviceEnumerator.h"
#include "VirtualTrackerEnumerator.h"
#include "ServerUtility.h"
#include "USBDeviceManager.h"
#include "ServerLog.h"
#include "assert.h"
#include "string.h"

// -- private definitions -----
#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define MAX_CAMERA_TYPE_INDEX               GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_CAMERA_TYPE_COUNT) - 1 // -1 ignore virtual tracker

// -- globals -----
// NOTE: This list must match the tracker order in CommonDeviceState::eDeviceType
USBDeviceFilter k_supported_tracker_infos[MAX_CAMERA_TYPE_INDEX] = {
    { 0x1415, 0x2000 }, // PS3Eye
    //{ 0x05a9, 0x058a }, // PS4 Camera - TODO
};

// -- private prototypes -----
static bool is_tracker_supported(USBDeviceEnumerator* enumerator, CommonDeviceState::eDeviceType device_type_filter, CommonDeviceState::eDeviceType &out_device_type);

// -- methods -----
TrackerDeviceEnumerator::TrackerDeviceEnumerator(
	eAPIType _apiType)
	: DeviceEnumerator()
	, api_type(_apiType)
	, enumerator_count(0)
	, enumerator_index(0)
    , m_cameraIndex(-1)
{
	switch (_apiType)
	{
	case eAPIType::CommunicationType_HID:
	{
		{
			AnyDeviceEnumerator _enumerator;
			_enumerator.m_usb_enumerator = usb_device_enumerator_allocate();
			_enumerator.enumerator = nullptr;
			enumerators.push_back(_enumerator);
		}
		enumerator_count = 1;
		break;
	}
	case eAPIType::CommunicationType_VIRTUAL:
	{
		{
			AnyDeviceEnumerator _enumerator;
			_enumerator.m_usb_enumerator = nullptr;
			_enumerator.enumerator = new VirtualTrackerEnumerator();
			enumerators.push_back(_enumerator);
		}
		enumerator_count = 1;
		break;
	}
	case eAPIType::CommunicationType_ALL:
	{
		{
			AnyDeviceEnumerator _enumerator;
			_enumerator.m_usb_enumerator = usb_device_enumerator_allocate();
			_enumerator.enumerator = nullptr;
			enumerators.push_back(_enumerator);
		}
		{
			AnyDeviceEnumerator _enumerator;
			_enumerator.m_usb_enumerator = nullptr;
			_enumerator.enumerator = new VirtualTrackerEnumerator();
			enumerators.push_back(_enumerator);
		}
		enumerator_count = 2;
		break;
	}
	}

	if (enumerators[enumerator_index].m_usb_enumerator != nullptr)
	{
		m_deviceType = CommonDeviceState::PS3EYE;

		// If the first USB device handle isn't a tracker, move on to the next device
		if (testUSBEnumerator())
		{
			m_cameraIndex = 0;
		}
		else
		{
			next();
		}
	}
	else if (enumerators[enumerator_index].enumerator != nullptr)
	{
		if (is_valid())
		{
			m_deviceType = enumerators[enumerator_index].enumerator->get_device_type();
			m_cameraIndex = 0;
		}
		else
		{
			next();
		}
	}
	else
	{
		m_deviceType = CommonDeviceState::SUPPORTED_CAMERA_TYPE_COUNT; // invalid
	}
}

TrackerDeviceEnumerator::~TrackerDeviceEnumerator()
{
	for (int index = 0; index < enumerator_count; ++index)
	{
		if (enumerators[index].m_usb_enumerator != nullptr)
		{
			usb_device_enumerator_free(enumerators[index].m_usb_enumerator);
		}
		else if (enumerators[index].enumerator != nullptr)
		{
			delete enumerators[index].enumerator;
		}
	}
}

int TrackerDeviceEnumerator::get_vendor_id() const
{
	if (enumerator_index < enumerator_count)
	{
		if (enumerators[enumerator_index].m_usb_enumerator != nullptr)
		{
			USBDeviceFilter devInfo;
			int vendor_id = -1;

			if (is_valid() && usb_device_enumerator_get_filter(enumerators[enumerator_index].m_usb_enumerator, devInfo))
			{
				vendor_id = devInfo.vendor_id;
			}

			return vendor_id;
		}
		else if (enumerators[enumerator_index].enumerator != nullptr)
		{
			return  enumerators[enumerator_index].enumerator->get_vendor_id();
		}
	}

	return -1;
}

int TrackerDeviceEnumerator::get_product_id() const
{
	if (enumerator_index < enumerator_count)
	{
		if (enumerators[enumerator_index].m_usb_enumerator != nullptr)
		{
			USBDeviceFilter devInfo;
			int product_id = -1;

			if (is_valid() && usb_device_enumerator_get_filter(enumerators[enumerator_index].m_usb_enumerator, devInfo))
			{
				product_id = devInfo.product_id;
			}

			return product_id;
		}
		else if (enumerators[enumerator_index].enumerator != nullptr)
		{
			return  enumerators[enumerator_index].enumerator->get_product_id();
		}
	}

	return -1;
}

const char *TrackerDeviceEnumerator::get_path() const
{
	if (enumerator_index < enumerator_count)
	{
		if (enumerators[enumerator_index].m_usb_enumerator != nullptr)
		{
			const char *result = nullptr;

			if (is_valid())
			{
				// Return a pointer to our member variable that has the path cached
				result = m_currentUSBPath;
			}

			return result;
		}
		else if (enumerators[enumerator_index].enumerator != nullptr)
		{
			return enumerators[enumerator_index].enumerator->get_path();
		}
	}

	return nullptr;
}

const USBDeviceEnumerator *TrackerDeviceEnumerator::get_hid_tracker_enumerator() const
{
	USBDeviceEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<USBDeviceEnumerator *>(enumerators[0].m_usb_enumerator) : nullptr;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == 0) ? static_cast<USBDeviceEnumerator *>(enumerators[0].m_usb_enumerator) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

const VirtualTrackerEnumerator *TrackerDeviceEnumerator::get_virtual_tracker_enumerator() const
{
	VirtualTrackerEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<VirtualTrackerEnumerator *>(enumerators[0].enumerator) : nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == 1) ? static_cast<VirtualTrackerEnumerator *>(enumerators[1].enumerator) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

bool TrackerDeviceEnumerator::is_valid() const
{
	if (enumerator_index < enumerator_count)
	{
		if (enumerators[enumerator_index].m_usb_enumerator != nullptr)
		{
			return usb_device_enumerator_is_valid(enumerators[enumerator_index].m_usb_enumerator);
		}
		else if (enumerators[enumerator_index].enumerator != nullptr)
		{
			return enumerators[enumerator_index].enumerator->is_valid();
		}
	}

	return false;
}

bool TrackerDeviceEnumerator::next()
{
	bool foundValid = false;

	while (!foundValid && enumerator_index < enumerator_count)
	{
		if (enumerators[enumerator_index].m_usb_enumerator != nullptr)
		{
			while (is_valid() && !foundValid)
			{
				usb_device_enumerator_next(enumerators[enumerator_index].m_usb_enumerator);

				if (testUSBEnumerator())
				{
					foundValid = true;
				}
			}

			if (foundValid)
			{
				m_deviceType = CommonDeviceState::PS3EYE;
				++m_cameraIndex;
			}
			else
			{
				++enumerator_index;
			}
		}
		else if (enumerators[enumerator_index].enumerator != nullptr)
		{
			while (!foundValid && enumerator_index < enumerator_count)
			{
				if (enumerators[enumerator_index].enumerator->is_valid())
				{
					enumerators[enumerator_index].enumerator->next();
					foundValid = enumerators[enumerator_index].enumerator->is_valid();
				}
				else
				{
					++enumerator_index;

					if (enumerator_index < enumerator_count)
					{
						foundValid = enumerators[enumerator_index].enumerator->is_valid();
					}
				}
			}

			if (foundValid)
			{
				m_deviceType = enumerators[enumerator_index].enumerator->get_device_type();
				++m_cameraIndex;
			}
			else
			{
				m_deviceType = CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT; // invalid
			}
		}
	}

	return foundValid;
}

bool TrackerDeviceEnumerator::testUSBEnumerator()
{
	bool foundValid = false;

	if (enumerator_index < enumerator_count)
	{
		if (enumerators[enumerator_index].m_usb_enumerator != nullptr)
		{
			if (is_valid() && is_tracker_supported(enumerators[enumerator_index].m_usb_enumerator, m_deviceTypeFilter, m_deviceType))
			{
				char USBPath[256];

				// Cache the path to the device
				usb_device_enumerator_get_path(enumerators[enumerator_index].m_usb_enumerator, USBPath, sizeof(USBPath));

				// Test open the device
				char errorReason[256];
				if (usb_device_can_be_opened(enumerators[enumerator_index].m_usb_enumerator, errorReason, sizeof(errorReason)))
				{
					// Remember the last successfully opened tracker path
					strncpy(m_currentUSBPath, USBPath, sizeof(m_currentUSBPath));

					foundValid = true;
				}
				else
				{
					SERVER_LOG_INFO("TrackerDeviceEnumerator") << "Skipping device (" << USBPath << ") - " << errorReason;
				}
			}
		}
	}

	return foundValid;
}

//-- private methods -----
static bool is_tracker_supported(
	USBDeviceEnumerator *enumerator, 
	CommonDeviceState::eDeviceType device_type_filter,
	CommonDeviceState::eDeviceType &out_device_type)
{
	USBDeviceFilter devInfo;
	bool bIsValidDevice = false;

	if (usb_device_enumerator_get_filter(enumerator, devInfo))
	{
		// See if the next filtered device is a camera that we care about
		for (int tracker_type_index = 0; tracker_type_index < MAX_CAMERA_TYPE_INDEX; ++tracker_type_index)
		{
			const USBDeviceFilter &supported_type = k_supported_tracker_infos[tracker_type_index];

			if (devInfo.product_id == supported_type.product_id &&
				devInfo.vendor_id == supported_type.vendor_id)
			{
				CommonDeviceState::eDeviceType device_type = 
					static_cast<CommonDeviceState::eDeviceType>(CommonDeviceState::TrackingCamera + tracker_type_index);

				if (device_type_filter == CommonDeviceState::INVALID_DEVICE_TYPE || // i.e. no filter
					device_type_filter == device_type)
				{
					out_device_type = device_type;
					bIsValidDevice = true;
					break;
				}
			}
		}
	}

	return bIsValidDevice;
}