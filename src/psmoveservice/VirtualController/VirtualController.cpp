//-- includes -----
#include "VirtualController.h"
#include "ControllerDeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include <vector>

#include "gamepad/Gamepad.h"

// -- public methods

// -- Virtual Controller Config
// Bump this version when you are making a breaking config change.
// Simply adding or removing a field is ok and doesn't require a version bump.
const int VirtualControllerConfig::CONFIG_VERSION= 1;

const boost::property_tree::ptree
VirtualControllerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);
    pt.put("version", VirtualControllerConfig::CONFIG_VERSION);

    pt.put("gamepad_index", gamepad_index);
	pt.put("psmove_emulation", psmove_emulation);
	pt.put("enable_optical_tracking", enable_optical_tracking);

    pt.put("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
    pt.put("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);

    pt.put("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

    pt.put("PositionFilter.FilterType", position_filter_type);
    pt.put("PositionFilter.MaxVelocity", max_velocity);

	pt.put("Offsets.Position.X", offset_position.x);
	pt.put("Offsets.Position.Y", offset_position.y);
	pt.put("Offsets.Position.Z", offset_position.z);
	pt.put("Offsets.LocalOrientation.X", offset_orientation.x);
	pt.put("Offsets.LocalOrientation.Y", offset_orientation.y);
	pt.put("Offsets.LocalOrientation.Z", offset_orientation.z);
	pt.put("Offsets.WorldOrientation.X", offset_world_orientation.x);
	pt.put("Offsets.WorldOrientation.Y", offset_world_orientation.y);
	pt.put("Offsets.WorldOrientation.Z", offset_world_orientation.z);
	pt.put("Offsets.Scale.X", offset_scale.x);
	pt.put("Offsets.Scale.Y", offset_scale.y);
	pt.put("Offsets.Scale.Z", offset_scale.z);

	pt.put("prediction_time", prediction_time);
	pt.put("ang_prediction_time", ang_prediction_time);
    pt.put("bulb_radius", bulb_radius);

	pt.put("hand", hand);
	pt.put("FilterSettings.LowPassOptical.Distance", filter_lowpassoptical_distance);
	pt.put("FilterSettings.LowPassOptical.Smoothing", filter_lowpassoptical_smoothing);

	pt.put("FilterSettings.VelocitySmoothingFactor", filter_velocity_smoothing_factor);

    writeTrackingColor(pt, tracking_color_id);

    return pt;
}

void
VirtualControllerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == VirtualControllerConfig::CONFIG_VERSION)
    {
        is_valid = pt.get<bool>("is_valid", false);

		gamepad_index = pt.get<int>("gamepad_index", gamepad_index);
		psmove_emulation = pt.get<bool>("psmove_emulation", psmove_emulation);
		enable_optical_tracking = pt.get<bool>("enable_optical_tracking", enable_optical_tracking);

		prediction_time = pt.get<float>("prediction_time", prediction_time);
		ang_prediction_time = pt.get<float>("ang_prediction_time", ang_prediction_time);

        position_variance_exp_fit_a = pt.get<float>("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
        position_variance_exp_fit_b = pt.get<float>("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);

        mean_update_time_delta = pt.get<float>("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

        position_filter_type = pt.get<std::string>("PositionFilter.FilterType", position_filter_type);
        max_velocity = pt.get<float>("PositionFilter.MaxVelocity", max_velocity);

		offset_position.x = pt.get<float>("Offsets.Position.X", offset_position.x);
		offset_position.y = pt.get<float>("Offsets.Position.Y", offset_position.y);
		offset_position.z = pt.get<float>("Offsets.Position.Z", offset_position.z);
		offset_orientation.x = pt.get<float>("Offsets.LocalOrientation.X", offset_orientation.x);
		offset_orientation.y = pt.get<float>("Offsets.LocalOrientation.Y", offset_orientation.y);
		offset_orientation.z = pt.get<float>("Offsets.LocalOrientation.Z", offset_orientation.z);
		offset_world_orientation.x = pt.get<float>("Offsets.WorldOrientation.X", offset_world_orientation.x);
		offset_world_orientation.y = pt.get<float>("Offsets.WorldOrientation.Y", offset_world_orientation.y);
		offset_world_orientation.z = pt.get<float>("Offsets.WorldOrientation.Z", offset_world_orientation.z);
		offset_scale.x = pt.get<float>("Offsets.Scale.X", offset_scale.x);
		offset_scale.y = pt.get<float>("Offsets.Scale.Y", offset_scale.y);
		offset_scale.z = pt.get<float>("Offsets.Scale.Z", offset_scale.z);

        // Read the tracking color
        tracking_color_id = static_cast<eCommonTrackingColorID>(readTrackingColor(pt));
        bulb_radius = pt.get<float>("bulb_radius", bulb_radius);

		hand = pt.get<std::string>("hand", hand);

		filter_lowpassoptical_distance = pt.get<float>("FilterSettings.LowPassOptical.Distance", filter_lowpassoptical_distance);
		filter_lowpassoptical_smoothing = pt.get<float>("FilterSettings.LowPassOptical.Smoothing", filter_lowpassoptical_smoothing);

		filter_velocity_smoothing_factor = pt.get<float>("FilterSettings.VelocitySmoothingFactor", filter_velocity_smoothing_factor);
    }
    else
    {
        SERVER_LOG_WARNING("VirtualControllerConfig") << 
            "Config version " << version << " does not match expected version " << 
            VirtualControllerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- button helper methods -----
inline void 
setButtonBit(unsigned int &buttons, unsigned int button_mask, bool is_pressed)
{
	if (is_pressed)
	{
		buttons|= button_mask;
	}
	else
	{
		buttons&= ~button_mask;
	}
}

inline enum CommonControllerState::ButtonState
getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask)
{
    return (enum CommonControllerState::ButtonState)((((lastButtons & buttonMask) > 0) << 1) + ((buttons & buttonMask)>0));
}

// -- PSMove Controller -----
VirtualController::VirtualController()
    : cfg()
    , NextPollSequenceNumber(0)
    , bIsOpen(false)
    , bIsTracking(false)
	, m_controllerListener(nullptr)
{
	memset(&ControllerState, 0, sizeof(VirtualControllerState));
}

VirtualController::~VirtualController()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~VirtualController") << "Controller deleted without calling close() first!";
    }
}

bool VirtualController::open()
{
    ControllerDeviceEnumerator enumerator(ControllerDeviceEnumerator::CommunicationType_VIRTUAL, CommonControllerState::VirtualController);
    bool success= false;

    if (enumerator.is_valid())
    {
        success= open(&enumerator);
    }

    return success;
}

bool VirtualController::open(
    const DeviceEnumerator *enumerator)
{
    const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);
    
    const char *cur_dev_path= pEnum->get_path();
    bool success= false;

    if (getIsOpen())
    {
        SERVER_LOG_WARNING("VirtualController::open") << "VirtualController(" << cur_dev_path << ") already open. Ignoring request.";
        success= true;
    }
    else
    {
        SERVER_LOG_INFO("VirtualController::open") << "Opening VirtualController(" << cur_dev_path << ").";

        device_identifier = cur_dev_path;
        bIsOpen= true;

        // Load the config file
        cfg = VirtualControllerConfig(pEnum->get_path());
        cfg.load();

        // Save it back out again in case any defaults changed
        cfg.save();

        // Reset the polling sequence counter
        NextPollSequenceNumber = 0;

        success = true;
    }

    return success;
}

void VirtualController::close()
{
    if (bIsOpen)
    {
        device_identifier= "";
        bIsOpen= true;
    }
    else
    {
        SERVER_LOG_INFO("VirtualController::close") << "VirtualController already closed. Ignoring request.";
    }
}

bool 
VirtualController::setHostBluetoothAddress(const std::string &new_host_bt_addr)
{
    SERVER_LOG_WARNING("VirtualController::setHostBluetoothAddress") << "VirtualController(" << device_identifier << ") Can't have host bluetooth address assigned.";

    return false;
}

bool
VirtualController::setTrackingColorID(const eCommonTrackingColorID tracking_color_id)
{
	bool bSuccess = false;

	if (getIsOpen())
	{
		cfg.tracking_color_id = tracking_color_id;
		cfg.save();
		bSuccess = true;
	}

	return bSuccess;
}

void VirtualController::setControllerListener(IControllerListener *listener)
{
	m_controllerListener = listener;
}

// Getters
bool 
VirtualController::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);
    
    bool matches = false;

    if (pEnum->get_device_type() == getDeviceType())
    {
        const char *enumerator_path = pEnum->get_path();
        const char *dev_path = device_identifier.c_str();

#ifdef _WIN32
        matches = _stricmp(dev_path, enumerator_path) == 0;
#else
        matches = strcmp(dev_path, enumerator_path) == 0;
#endif
    }

    return matches;
}

bool 
VirtualController::getIsBluetooth() const
{ 
    return false; 
}

bool
VirtualController::getIsReadyToPoll() const
{
    return getIsOpen();
}

std::string 
VirtualController::getUSBDevicePath() const
{
    return device_identifier;
}

int
VirtualController::getVendorID() const
{
	return 0x00;
}

int
VirtualController::getProductID() const
{
	return 0x00;
}

std::string 
VirtualController::getSerial() const
{
    return device_identifier;
}

std::string 
VirtualController::getAssignedHostBluetoothAddress() const
{
    return "00:00:00:00:00:00";
}

bool
VirtualController::getIsOpen() const
{
    return bIsOpen;
}

CommonDeviceState::eDeviceType
VirtualController::getDeviceType() const
{
    return CommonDeviceState::VirtualController;
}

IControllerInterface::ePollResult
VirtualController::poll()
{
    IControllerInterface::ePollResult result= IControllerInterface::_PollResultFailure;
      
    if (getIsOpen())
    {
        VirtualControllerState newState;

        // Device still in valid state
        result= IControllerInterface::_PollResultSuccessNewData;

        // Get the latest button data from the bound gamepad
        pollGamepad(newState);
                
		// Can't report the true battery state
		newState.Battery = CommonControllerState::Batt_MAX;

        // Increment the sequence for every new polling packet
        newState.PollSequenceNumber= NextPollSequenceNumber;
        ++NextPollSequenceNumber;

        // Cache the new controller state
        ControllerState= newState;

		// Send the sensor data for processing by filter
		if (m_controllerListener != nullptr)
		{
			m_controllerListener->notifySensorDataReceived(&newState);

		}
    }

    return result;
}

void
VirtualController::pollGamepad(VirtualControllerState &newState)
{
	assert(getIsOpen());
    bool bValidGamepad= false;

    newState.clear_gamepad_data();

    if (cfg.gamepad_index >= 0)
    {
	    const Gamepad_device * gamepad = Gamepad_deviceAtIndex(static_cast<unsigned int>(cfg.gamepad_index));

	    if (gamepad != nullptr)
	    {
		    unsigned int lastButtons = ControllerState.AllButtons;

            newState.vendorID= gamepad->vendorID;
            newState.productID= gamepad->productID;

            // Button states
            newState.numButtons= std::min(gamepad->numButtons, (unsigned int)MAX_VIRTUAL_CONTROLLER_BUTTONS);
            for (int buttonIndex = 0; buttonIndex < newState.numButtons; ++buttonIndex)
            {
                unsigned int button_mask= 1 << buttonIndex;

                // Set button bit
                setButtonBit(newState.AllButtons, button_mask, gamepad->buttonStates[buttonIndex]);

                // Button de-bounce
                newState.buttonStates[buttonIndex]= getButtonState(newState.AllButtons, lastButtons, button_mask);
            }
		    

		    // Analog axis states
            newState.numAxes= std::min(gamepad->numAxes, (unsigned int)MAX_VIRTUAL_CONTROLLER_AXES);
            for (int axisIndex = 0; axisIndex < newState.numAxes; ++axisIndex)
            {
                newState.axisStates[axisIndex] = static_cast<unsigned char>((gamepad->axisStates[axisIndex] + 1.f) * 127.f);
            }
	    }
    }
}

const CommonDeviceState * 
VirtualController::getState(
	int lookBack) const
{
    return &ControllerState;
}

const std::tuple<unsigned char, unsigned char, unsigned char> 
VirtualController::getColour() const
{
    return std::make_tuple(0, 0, 0);
}

void 
VirtualController::getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const
{
    outTrackingShape.shape_type= eCommonTrackingShapeType::Sphere;
    outTrackingShape.shape.sphere.radius_cm = getConfig()->bulb_radius;
}

bool
VirtualController::getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const
{
	bool bSuccess = false;

	if (getIsOpen())
	{
		out_tracking_color_id = cfg.tracking_color_id;
		bSuccess = true;
	}

	return bSuccess;
}

float VirtualController::getIdentityForwardDegrees() const
{
	// Controller model points down the -Z axis when it has the identity orientation
	return 270.f;
}

float VirtualController::getPredictionTime() const
{
	return getConfig()->prediction_time;
}

float VirtualController::getOrientationPredictionTime() const
{
	return getConfig()->ang_prediction_time;
}

bool VirtualController::getWasSystemButtonPressed() const
{
    return false;
}

long VirtualController::getMaxPollFailureCount() const
{
    return 1;
}