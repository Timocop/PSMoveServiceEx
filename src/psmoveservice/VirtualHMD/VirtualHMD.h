#ifndef VIRTUAL_HMD_H
#define VIRTUAL_HMD_H

#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "MathUtility.h"
#include <string>
#include <vector>
#include <deque>
#include <array>


class VirtualHMDConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    VirtualHMDConfig(const std::string &fnamebase = "VirtualHMDConfig")
        : PSMoveConfig(fnamebase)
		, is_valid(false)
		, version(CONFIG_VERSION)
		, position_filter_type("LowPassOptical")
        , max_velocity(1.f)
		, mean_update_time_delta(0.008333f)
		, position_variance_exp_fit_a(0.0994158462f)
		, position_variance_exp_fit_b(-0.000567041978f)
		, prediction_time(0.f)
		, ang_prediction_time(0.f)
		, tracking_color_id(eCommonTrackingColorID::Blue)
        , bulb_radius(2.25f) // The radius of the psmove tracking bulb in cm
		, filter_lowpassoptical_distance(1.f)
		, filter_lowpassoptical_smoothing(0.40f)
		, filter_velocity_smoothing_factor(0.25f)
		, filter_angular_smoothing_factor(0.25f)
		, filter_velocity_prediction_cutoff(1.0f)
		, filter_angular_prediction_cutoff(0.25f)
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

	// The type of position filter to use
	std::string position_filter_type;

	// The offset added in post
	CommonDevicePosition offset_position;
	CommonDevicePosition offset_orientation;
	CommonDevicePosition offset_world_orientation;
	CommonDevicePosition offset_scale;

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

	float prediction_time;
	float ang_prediction_time;

	eCommonTrackingColorID tracking_color_id;
    float bulb_radius;

	// Filter settings
	float filter_lowpassoptical_distance;
	float filter_lowpassoptical_smoothing;

	float filter_velocity_smoothing_factor;
	float filter_angular_smoothing_factor;
	float filter_velocity_prediction_cutoff;
	float filter_angular_prediction_cutoff;
};

struct VirtualHMDState : public CommonHMDState
{
    VirtualHMDState()
    {
        clear();
    }

    void clear()
    {
        CommonHMDState::clear();
		DeviceType = VirtualHMD;
    }
};

class VirtualHMD : public IHMDInterface 
{
public:
    VirtualHMD();
    virtual ~VirtualHMD();

    // VirtualHMD
    bool open(); // Opens the first virtualHMD listed in the HMD manager

    // -- IDeviceInterface
    bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    bool open(const DeviceEnumerator *enumerator) override;
    bool getIsOpen() const override;
    bool getIsReadyToPoll() const override;
    IDeviceInterface::ePollResult poll() override;
    void close() override;
    long getMaxPollFailureCount() const override;
    CommonDeviceState::eDeviceType getDeviceType() const override
    {
        return CommonDeviceState::VirtualHMD;
    }
    static CommonDeviceState::eDeviceType getDeviceTypeStatic()
    {
        return CommonDeviceState::VirtualHMD;
    }
    const CommonDeviceState * getState(int lookBack = 0) const override;

    // -- IHMDInterface
    std::string getUSBDevicePath() const override;
	void getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const override;
	bool setTrackingColorID(const eCommonTrackingColorID tracking_color_id) override;
	void setHmdListener(IHMDListener *listener) override;
	bool getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const override;
	float getPredictionTime() const override;
	float getOrientationPredictionTime() const override;

    // -- Getters
    inline const VirtualHMDConfig *getConfig() const
    {
        return &cfg;
    }
    inline VirtualHMDConfig *getConfigMutable()
    {
        return &cfg;
    }

    // -- Setters
	void setTrackingEnabled(bool bEnableTracking);

private:
    // Constant while the HMD is open
    VirtualHMDConfig cfg;
    std::string device_identifier;
    bool bIsOpen;

    // Read HMD State
    int NextPollSequenceNumber;

	bool bIsTracking;
	VirtualHMDState m_cachedState;
	IHMDListener* m_hmdListener;
};

#endif // VIRTUAL_HMD_H
