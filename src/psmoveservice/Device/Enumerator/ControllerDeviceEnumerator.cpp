// -- includes -----
#include "ControllerDeviceEnumerator.h"
#include "ControllerHidDeviceEnumerator.h"
#include "ControllerUSBDeviceEnumerator.h"
#include "ControllerGamepadEnumerator.h"
#include "VirtualControllerEnumerator.h"
#include "assert.h"
#include "string.h"

#define ENUM_INDEX_VIRTUAL 0
#define ENUM_INDEX_HID 1
#define ENUM_INDEX_USB 2
#define ENUM_INDEX_GAMEPAD 3

// -- globals -----

// -- ControllerDeviceEnumerator -----
ControllerDeviceEnumerator::ControllerDeviceEnumerator(
	eAPIType _apiType)
	: DeviceEnumerator()
	, api_type(_apiType)
	, enumerators(nullptr)
	, enumerator_count(0)
	, enumerator_index(0)
{
	switch (_apiType)
	{
	case eAPIType::CommunicationType_HID:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerHidDeviceEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_USB:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerUSBDeviceEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerGamepadEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new VirtualControllerEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		enumerators = new DeviceEnumerator *[4];
		enumerators[ENUM_INDEX_VIRTUAL] = new VirtualControllerEnumerator;
		enumerators[ENUM_INDEX_HID] = new ControllerHidDeviceEnumerator;
		enumerators[ENUM_INDEX_USB] = new ControllerUSBDeviceEnumerator;
		enumerators[ENUM_INDEX_GAMEPAD] = new ControllerGamepadEnumerator;
		enumerator_count = 4;
		break;
	}

	if (is_valid())
	{
		m_deviceType= enumerators[enumerator_index]->get_device_type();
	}
	else
    {
        next();
    }
}

ControllerDeviceEnumerator::ControllerDeviceEnumerator(
	eAPIType _apiType,
	CommonDeviceState::eDeviceType deviceTypeFilter)
    : DeviceEnumerator(deviceTypeFilter)
	, api_type(_apiType)
	, enumerators(nullptr)
	, enumerator_count(0)
	, enumerator_index(0)
{
	switch (_apiType)
	{
	case eAPIType::CommunicationType_HID:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerHidDeviceEnumerator(deviceTypeFilter);
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_USB:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerUSBDeviceEnumerator(deviceTypeFilter);
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerGamepadEnumerator(deviceTypeFilter);
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new VirtualControllerEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		enumerators = new DeviceEnumerator *[4];
		enumerators[0] = new ControllerHidDeviceEnumerator(deviceTypeFilter);
		enumerators[1] = new ControllerUSBDeviceEnumerator(deviceTypeFilter);
		enumerators[2] = new ControllerGamepadEnumerator(deviceTypeFilter);
        enumerators[3] = new VirtualControllerEnumerator;
		enumerator_count = 4;
		break;
	}

	if (is_valid())
	{
		m_deviceType = enumerators[enumerator_index]->get_device_type();
	}
	else
	{
		next();
	}
}

ControllerDeviceEnumerator::~ControllerDeviceEnumerator()
{
	for (int index = 0; index < enumerator_count; ++index)
	{
		delete enumerators[index];
	}
	delete[] enumerators;
}

const char *ControllerDeviceEnumerator::get_path() const
{
    return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_path() : nullptr;
}

int ControllerDeviceEnumerator::get_vendor_id() const
{
	return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_vendor_id() : -1;
}

int ControllerDeviceEnumerator::get_product_id() const
{
	return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_product_id() : -1;
}

bool ControllerDeviceEnumerator::get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const
{
    bool success = false;

    if ((api_type == eAPIType::CommunicationType_HID) ||
		(api_type == eAPIType::CommunicationType_ALL && enumerator_index == ENUM_INDEX_HID))
    {
		ControllerHidDeviceEnumerator *hid_enumerator = static_cast<ControllerHidDeviceEnumerator *>(enumerators[enumerator_index]);

        success = hid_enumerator->get_serial_number(out_mb_serial, mb_buffer_size);
    }

    return success;
}

ControllerDeviceEnumerator::eAPIType ControllerDeviceEnumerator::get_api_type() const
{
	ControllerDeviceEnumerator::eAPIType result= ControllerDeviceEnumerator::CommunicationType_INVALID;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		result = (enumerator_index < enumerator_count) ? ControllerDeviceEnumerator::CommunicationType_HID : ControllerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_USB:
		result = (enumerator_index < enumerator_count) ? ControllerDeviceEnumerator::CommunicationType_USB : ControllerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		result = (enumerator_index < enumerator_count) ? ControllerDeviceEnumerator::CommunicationType_GAMEPAD : ControllerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		result = (enumerator_index < enumerator_count) ? ControllerDeviceEnumerator::CommunicationType_VIRTUAL : ControllerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			switch (enumerator_index)
			{
			case ENUM_INDEX_VIRTUAL:
				result = ControllerDeviceEnumerator::CommunicationType_VIRTUAL;
				break;
			case ENUM_INDEX_HID:
				result = ControllerDeviceEnumerator::CommunicationType_HID;
				break;
			case ENUM_INDEX_USB:
				result = ControllerDeviceEnumerator::CommunicationType_USB;
				break;
			case ENUM_INDEX_GAMEPAD:
				result = ControllerDeviceEnumerator::CommunicationType_GAMEPAD;
				break;
			default:
				result = ControllerDeviceEnumerator::CommunicationType_INVALID;
				break;
			}
		}
		else
		{
			result = ControllerDeviceEnumerator::CommunicationType_INVALID;
		}
		break;
	}

	return result;
}

const ControllerHidDeviceEnumerator *ControllerDeviceEnumerator::get_hid_controller_enumerator() const
{
	ControllerHidDeviceEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<ControllerHidDeviceEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_USB:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == ENUM_INDEX_HID) ? static_cast<ControllerHidDeviceEnumerator *>(enumerators[ENUM_INDEX_HID]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

const ControllerUSBDeviceEnumerator *ControllerDeviceEnumerator::get_usb_controller_enumerator() const
{
	ControllerUSBDeviceEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_USB:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<ControllerUSBDeviceEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == ENUM_INDEX_USB) ? static_cast<ControllerUSBDeviceEnumerator *>(enumerators[ENUM_INDEX_USB]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

const ControllerGamepadEnumerator *ControllerDeviceEnumerator::get_gamepad_controller_enumerator() const
{
	ControllerGamepadEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_USB:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<ControllerGamepadEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == ENUM_INDEX_GAMEPAD) ? static_cast<ControllerGamepadEnumerator *>(enumerators[ENUM_INDEX_GAMEPAD]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

const VirtualControllerEnumerator *ControllerDeviceEnumerator::get_virtual_controller_enumerator() const
{
	VirtualControllerEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_USB:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<VirtualControllerEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == ENUM_INDEX_VIRTUAL) ? static_cast<VirtualControllerEnumerator *>(enumerators[ENUM_INDEX_VIRTUAL]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

bool ControllerDeviceEnumerator::is_valid() const
{
    bool bIsValid = false;

	if (enumerator_index < enumerator_count)
	{
		bIsValid = enumerators[enumerator_index]->is_valid();
	}

    return bIsValid;
}

bool ControllerDeviceEnumerator::next()
{
    bool foundValid = false;

    while (!foundValid && enumerator_index < enumerator_count)
    {
		if (enumerators[enumerator_index]->is_valid())
		{
			enumerators[enumerator_index]->next();
			foundValid = enumerators[enumerator_index]->is_valid();
		}
		else
		{
			++enumerator_index;

			if (enumerator_index < enumerator_count)
			{
				foundValid = enumerators[enumerator_index]->is_valid();
			}
		}
    }

	if (foundValid)
	{
		m_deviceType = enumerators[enumerator_index]->get_device_type();
	}
	else
	{
		m_deviceType = CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT; // invalid
	}

    return foundValid;
}