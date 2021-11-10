// -- includes -----
#include "VirtualTrackerEnumerator.h"
#include "ServerUtility.h"
#include "assert.h"
#include "hidapi.h"
#include "string.h"

//-- Statics
int VirtualTrackerEnumerator::virtual_tracker_count = 0;

// -- VirtualControllerDeviceEnumerator -----
VirtualTrackerEnumerator::VirtualTrackerEnumerator()
    : DeviceEnumerator(CommonDeviceState::VirtualTracker)
{
	m_deviceType= CommonDeviceState::VirtualTracker;
    m_device_index= 0;

    m_current_device_identifier= "VirtualTracker_0";
    m_device_count= virtual_tracker_count;
}

const char *VirtualTrackerEnumerator::get_path() const
{
	return m_current_device_identifier.c_str();
}

int VirtualTrackerEnumerator::get_vendor_id() const
{
	return is_valid() ? 0x0000 : -1;
}

int VirtualTrackerEnumerator::get_product_id() const
{
	return is_valid() ? 0x0000 : -1;
}

bool VirtualTrackerEnumerator::is_valid() const
{
	return m_device_index < m_device_count;
}

bool VirtualTrackerEnumerator::next()
{
	bool foundValid = false;

	++m_device_index;
    if (m_device_index < m_device_count)
    {
        char device_path[32];
        ServerUtility::format_string(device_path, sizeof(device_path), "VirtualTracker_%d", m_device_index);

        m_current_device_identifier= device_path;
    }

	return foundValid;
}