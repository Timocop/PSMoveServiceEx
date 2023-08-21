#ifndef VIRTUAL_CONTROLLER_H
#define VIRTUAL_CONTROLLER_H

#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "MathUtility.h"
#include "hidapi.h"
#include <string>
#include <array>
#include <chrono>

#define MAX_VIRTUAL_CONTROLLER_BUTTONS 32
#define MAX_VIRTUAL_CONTROLLER_AXES 32

class VirtualControllerConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    VirtualControllerConfig(const std::string &fnamebase = "VirtualControllerConfig")
        : PSMoveConfig(fnamebase)
		, is_valid(false)
		, version(CONFIG_VERSION)
        , gamepad_index(-1)
		, psmove_emulation(false)
		, enable_optical_tracking(true)
		, position_filter_type("LowPassOptical")
        , max_velocity(1.f)
		, mean_update_time_delta(0.008333f)
		, position_variance_exp_fit_a(0.0994158462f)
		, position_variance_exp_fit_b(-0.000567041978f)
		, prediction_time(0.f)
		, ang_prediction_time(0.f)
		, tracking_color_id(eCommonTrackingColorID::Blue)
        , bulb_radius(2.25f) // The radius of the psmove tracking bulb in cm
		, hand("Any")
		, filter_lowpassoptical_distance(10.f)
		, filter_lowpassoptical_smoothing(0.40f)
		, filter_velocity_smoothing_factor(0.25f)
    {
		offset_position.set(0.0, 0.0, 0.0);
		offset_orientation.set(0.0, 0.0, 0.0);
		offset_world_orientation.set(0.0, 0.0, 0.0);
		offset_scale.set(1.0, 1.0, 1.0);
    };

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    bool is_valid;
    long version;

    // The index of gamepad attached to the PC to poll buttons from
    int gamepad_index;
	bool psmove_emulation;

	// Enable or disable optical tracking.
	bool enable_optical_tracking;

	// The type of position filter to use
	std::string position_filter_type;

	// Maximum velocity for the controller physics (meters/second)
	float max_velocity;

	// The average time between updates in seconds
	float mean_update_time_delta;

	// The variance of the controller position as a function of pixel area
	float position_variance_exp_fit_a;
	float position_variance_exp_fit_b;

	inline float get_position_variance(float projection_area) const {
		return position_variance_exp_fit_a*exp(position_variance_exp_fit_b*projection_area);
	}

	// The offset added in post
	CommonDevicePosition offset_position;
	CommonDevicePosition offset_orientation;
	CommonDevicePosition offset_world_orientation;
	CommonDevicePosition offset_scale;

	float prediction_time;
	float ang_prediction_time;

	eCommonTrackingColorID tracking_color_id;
    float bulb_radius;

	// The assigned hand for this controller
	std::string hand;

	// Filter settings
	float filter_lowpassoptical_distance;
	float filter_lowpassoptical_smoothing;
	float filter_velocity_smoothing_factor;
};

struct VirtualControllerState : public CommonControllerState
{
    // USB device VID
	int vendorID;

    // USB device PID
	int productID;
	
    // Number of button elements belonging to the device
	int numAxes;
	
	// Number of button elements belonging to the device
	int numButtons;
	
	// Array[numAxes] of values representing the current state of each axis, in the range [0,255]
	unsigned char axisStates[MAX_VIRTUAL_CONTROLLER_AXES];
	
	// Array[numButtons] of values representing the current state of each button
	ButtonState buttonStates[MAX_VIRTUAL_CONTROLLER_BUTTONS];

    VirtualControllerState()
    {
        clear();
    }

    void clear()
    {
        CommonControllerState::clear();
		DeviceType = VirtualController;

	    clear_gamepad_data();
    }

    void clear_gamepad_data()
    {
        CommonControllerState::AllButtons = 0;

	    vendorID= 0;
	    productID= 0;
	
	    numAxes= 0;
	    numButtons= 0;
	
	    memset(axisStates, 0, sizeof(axisStates));
	    memset(buttonStates, 0, sizeof(buttonStates));
    }
};


class VirtualController : public IControllerInterface {
public:
    VirtualController();
    virtual ~VirtualController();

    // VirtualController
    bool open(); // Opens the first HID device for the controller
    
    // -- IDeviceInterface
    virtual bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    virtual bool open(const DeviceEnumerator *enumerator) override;
    virtual bool getIsOpen() const override;
    virtual bool getIsReadyToPoll() const override;
    virtual IDeviceInterface::ePollResult poll() override;
    virtual void close() override;
    virtual long getMaxPollFailureCount() const override;
    virtual CommonDeviceState::eDeviceType getDeviceType() const override;
    virtual const CommonDeviceState * getState(int lookBack = 0) const override;
    
    // -- IControllerInterface
    virtual bool setHostBluetoothAddress(const std::string &address) override;
	virtual bool setTrackingColorID(const eCommonTrackingColorID tracking_color_id) override;
	virtual void setControllerListener(IControllerListener *listener) override;
    virtual bool getIsBluetooth() const override;
    virtual std::string getUSBDevicePath() const override;
	virtual int getVendorID() const override;
	virtual int getProductID() const override;
    virtual std::string getAssignedHostBluetoothAddress() const override;
    virtual std::string getSerial() const override;
    virtual const std::tuple<unsigned char, unsigned char, unsigned char> getColour() const override;
    virtual void getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const override;
	virtual bool getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const override;
	virtual float getIdentityForwardDegrees() const override;
	virtual float getPredictionTime() const override;
	virtual float getOrientationPredictionTime() const override;
    virtual bool getWasSystemButtonPressed() const override;

    // -- Getters
    inline const VirtualControllerConfig *getConfig() const
    { return &cfg; }
    inline VirtualControllerConfig *getConfigMutable()
    { return &cfg; }
    static CommonDeviceState::eDeviceType getDeviceTypeStatic()
    { return CommonDeviceState::VirtualController; }
    
protected:
    void pollGamepad(VirtualControllerState &newState);

private:      
    // Constant while a controller is open
    VirtualControllerConfig cfg;
    std::string device_identifier;
    bool bIsOpen;

    // Controller State
    int NextPollSequenceNumber;
    VirtualControllerState ControllerState;

	bool bIsTracking;
	IControllerListener* m_controllerListener;
};
#endif // VIRTUAL_CONTROLLER_h