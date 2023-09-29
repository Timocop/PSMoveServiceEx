#ifndef TRACKER_DEVICE_ENUMERATOR_H
#define TRACKER_DEVICE_ENUMERATOR_H

//-- includes -----
#include "DeviceEnumerator.h"
#include "USBApiInterface.h"
#include <deque>
#include <map>
#include <vector>

//-- definitions -----
class TrackerDeviceEnumerator : public DeviceEnumerator
{
public:
	enum eAPIType
	{
		CommunicationType_INVALID = -1,
		CommunicationType_HID,
		CommunicationType_VIRTUAL,
		CommunicationType_ALL
	};

	struct AnyDeviceEnumerator
	{
		struct USBDeviceEnumerator *m_usb_enumerator;
		DeviceEnumerator *enumerator;
	};

    TrackerDeviceEnumerator(eAPIType api_type);
	~TrackerDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
	int get_vendor_id() const override;
	int get_product_id() const override;
    const char *get_path() const override;
	inline int get_camera_index() const { return m_cameraIndex; }
	inline int get_camera_hid_index() const { return m_cameraHidIndex; }
	inline int get_camera_virt_index() const { return m_cameraVirtIndex; }
	const class USBDeviceEnumerator *get_hid_tracker_enumerator() const;
	const class VirtualTrackerEnumerator *get_virtual_tracker_enumerator() const;

protected: 
	bool testUSBEnumerator();

private:
	eAPIType api_type;
    char m_currentUSBPath[256];
	std::vector<AnyDeviceEnumerator> enumerators;
	int enumerator_count;
	int enumerator_index;
	int m_cameraIndex;
	int m_cameraHidIndex;
	int m_cameraVirtIndex;
};

#endif // TRACKER_DEVICE_ENUMERATOR_H
