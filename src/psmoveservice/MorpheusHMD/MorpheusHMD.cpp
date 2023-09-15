//-- includes -----
#include "AtomicPrimitives.h"
#include "MorpheusHMD.h"
#include "DeviceInterface.h"
#include "DeviceManager.h"
#include "HMDDeviceEnumerator.h"
#include "HidHMDDeviceEnumerator.h"
#include "MathUtility.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "hidapi.h"
#include "libusb.h"
#include "WorkerThread.h"
#include <vector>
#include <cstdlib>
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

//-- constants -----
#define MORPHEUS_VENDOR_ID 0x054c
#define MORPHEUS_PRODUCT_ID 0x09af

#define MORPHEUS_CONFIGURATION_PSVR  1
#define MORPHEUS_ENDPOINT_IN 0x80

#define MORPHEUS_SENSOR_INTERFACE 4
#define MORPHEUS_COMMAND_INTERFACE 5

#define MORPHEUS_USB_INTERFACES_MASK_TO_CLAIM ( \
	(1 << MORPHEUS_COMMAND_INTERFACE) \
)

#define MORPHEUS_COMMAND_MAGIC 0xAA
#define MORPHEUS_COMMAND_MAX_PAYLOAD_LEN 60

#define MORPHEUS_HMD_STATE_BUFFER_MAX 4
#define METERS_TO_CENTIMETERS 100

enum eMorpheusRequestType
{
	Morpheus_Req_EnableTracking= 0x11,
	Morpheus_Req_TurnOffProcessorUnit = 0x13,
	Morpheus_Req_SetLEDBrightness = 0x15,
	Morpheus_Req_SetHeadsetPower= 0x17,
	Morpheus_Req_SetCinematicConfiguration= 0x21,
	Morpheus_Req_SetVRMode= 0x23,
};

enum eMorpheusLED
{
	_MorpheusLED_A = 1 << 0, // Front-Bottom-Right
	_MorpheusLED_B = 1 << 1, // Front-Bottom-Left
	_MorpheusLED_C = 1 << 2, // Front-Top-Right
	_MorpheusLED_D = 1 << 3, // Front-Top-Left
	_MorpheusLED_E = 1 << 4, // Front-Middle-Center
	_MorpheusLED_F = 1 << 5, // Front-Middle-Right
	_MorpheusLED_G = 1 << 6, // Front-Middle-Left
	_MorpheusLED_H = 1 << 7, // Rear-Right
	_MorpheusLED_I = 1 << 8, // Rear-Left

	_MorpheusLED_ALL = 0x1FF,
	_MorpheusLED_FRONT = _MorpheusLED_A|_MorpheusLED_B|_MorpheusLED_C|_MorpheusLED_D|_MorpheusLED_E|_MorpheusLED_F|_MorpheusLED_G,
	_MorpheusLED_BACK = _MorpheusLED_H|_MorpheusLED_I,
	_MorpheusLED_FONTTOPTRI = _MorpheusLED_C|_MorpheusLED_D|_MorpheusLED_E
};

// -- private definitions -----
class MorpheusUSBContext 
{
public:
	std::string device_identifier;

	// HIDApi state
    std::string sensor_device_path;
	hid_device *sensor_device_handle;
	
	// LibUSB state
	libusb_context *usb_context;
	libusb_device_handle *usb_device_handle;
	libusb_config_descriptor *usb_device_descriptor;
	unsigned int usb_claimed_interface_mask;

    MorpheusUSBContext()
    {
        Reset();
    }

    void Reset()
    {
		device_identifier = "";
        sensor_device_path = "";
		sensor_device_handle = nullptr;
		usb_context = nullptr;
		usb_device_handle = nullptr;
		usb_device_descriptor = nullptr;
		usb_claimed_interface_mask = 0;
    }
};

enum eMorpheusButton : unsigned char
{
	VolumePlus = 2,
	VolumeMinus = 4,
	MicrophoneMute = 8,
};

struct MorpheusRawSensorFrame
{
	unsigned char seq_frame[3];
	unsigned char seq_padding;

	unsigned char gyro_yaw[2];
	unsigned char gyro_pitch[2];
	unsigned char gyro_roll[2];
	unsigned char accel_x[2]; //It's the X value of the sensor, but it's mounted rotated on the headset
	unsigned char accel_y[2];
	unsigned char accel_z[2];
};

#pragma pack(1)
//See https://github.com/gusmanb/PSVRFramework/wiki/Sensor-report
struct MorpheusSensorData
{
	eMorpheusButton buttons;					// byte 0
	unsigned char unk0;							// byte 1
	unsigned char volume;                       // byte 2
	unsigned char unk1[5];                      // byte 3-7

	union
	{
		unsigned char asByte;
		struct
		{
			unsigned char hmdOnHead : 1;
			unsigned char displayIsOn : 1;
			unsigned char HDMIDisconnected : 1;
			unsigned char microphoneMuted : 1;
			unsigned char headphonesPresent : 1;
			unsigned char unk1 : 2;
			unsigned char timer : 1;
		};
	} headsetFlags;								// byte 8

	unsigned char unkFlags;     				// byte 9
	
	unsigned char unk2[6];						// byte 10-15

	MorpheusRawSensorFrame imu_frame_0;         // byte 16-31
	MorpheusRawSensorFrame imu_frame_1;         // byte 32-47

	unsigned char calibration_status;           // byte 48: 255 = boot, 0 - 3 calibrating ? 4 = calibrated, maybe a bit mask with sensor status ? (0 bad, 1 good) ?
	unsigned char sensors_ready;                // byte 49 
	unsigned char unk4[3];						// byte 50-52 
	unsigned char unk5;                         // byte 53: Voltage reference ? starts in 0 and suddenly jumps to 3
	unsigned char unk6;	                        // byte 54: Voltage value ? starts on 0, ranges very fast to 255, when switched from VR to Cinematic and back varies between 255 and 254 and sometimes oscillates between them
	unsigned char face_distance[2];             // byte 55-56: Infrared headset sensor, 0 to 1023, used to measure distance between the face / head and visor
	unsigned char unk7[6];                      // byte 57-62
	unsigned char sequence;                     // byte 63

    MorpheusSensorData()
    {
        Reset();
    }

    void Reset()
    {
        memset(this, 0, sizeof(MorpheusSensorData));
    }
};

struct MorpheusCommandHeader
{
	unsigned char request_id;
	unsigned char command_status;
	unsigned char magic;
	unsigned char length;
};

struct MorpheusCommand
{
	MorpheusCommandHeader header;
	unsigned char payload[MORPHEUS_COMMAND_MAX_PAYLOAD_LEN]; // variable sized payload depending on report id
};
#pragma pack()

class MorpheusHIDSensorProcessor : public WorkerThread
{
public:
	MorpheusHIDSensorProcessor(const MorpheusHMDConfig &cfg)
		: WorkerThread("MorpheusSensorProcessor")
		, m_cfg(cfg)
		, m_hidDevice(nullptr)
		, m_nextPollSequenceNumber(0)
	{
		memset(&m_previousRawHIDPacket, 0, sizeof(MorpheusSensorData));
		memset(&m_currentRawHIDPacket, 0, sizeof(MorpheusSensorData));
	}

	virtual ~MorpheusHIDSensorProcessor()
	{
	}

	void fetchLatestInputData(MorpheusHMDState &input_state)
	{
		m_currentState.fetchValue(input_state);
	}

	void start(class MorpheusUSBContext *USBContext, IHMDListener *hmd_listener)
	{
		if (!hasThreadStarted())
		{
			m_hidDevice = USBContext->sensor_device_handle;
			m_hmdListener = hmd_listener;
			WorkerThread::startThread();
		}
	}

	void stop()
	{
		WorkerThread::stopThread();
	}

protected:
	virtual bool doWork() override
	{
		bool bWorking = true;

		// Attempt to read the next sensor update packet from the HMD
		memcpy(&m_previousRawHIDPacket, &m_currentRawHIDPacket, sizeof(MorpheusSensorData));
		int res = hid_read_timeout(m_hidDevice, (unsigned char*)&m_currentRawHIDPacket, sizeof(MorpheusSensorData), 1000);

		if (res > 0)
		{
			// https://github.com/hrl7/node-psvr/blob/master/lib/psvr.js
			MorpheusHMDState newState;

			// Increment the sequence for every new polling packet
			newState.PollSequenceNumber = m_nextPollSequenceNumber;
			++m_nextPollSequenceNumber;

			// Processes the IMU data
			newState.parse_data_input(&m_cfg, &m_currentRawHIDPacket);

			// Store a copy of the parsed input date for functions
			// that want to query input state off of the worker thread
			m_currentState.storeValue(newState);

			if (m_hmdListener != nullptr)
			{
				m_hmdListener->notifySensorDataReceived(&newState);
			}
		}
		else if (res < 0)
		{
			char hidapi_err_mbs[256];
			bool valid_error_mesg =
				ServerUtility::convert_wcs_to_mbs(hid_error(m_hidDevice), hidapi_err_mbs, sizeof(hidapi_err_mbs));

			// Device no longer in valid state.
			if (valid_error_mesg)
			{
				SERVER_MT_LOG_ERROR("PSMoveSensorProcessor::doWork") << "HID ERROR: " << hidapi_err_mbs;
			}

			bWorking = false;
		}

		return bWorking;
	}

	// Multithreaded state
	const MorpheusHMDConfig m_cfg;
	hid_device *m_hidDevice;
	IHMDListener *m_hmdListener;

	// Worker thread state
	int m_nextPollSequenceNumber;
	MorpheusSensorData m_previousRawHIDPacket;
	MorpheusSensorData m_currentRawHIDPacket;

	AtomicObject<MorpheusHMDState> m_currentState;
};

// -- private methods
static bool morpheus_open_usb_device(MorpheusUSBContext *morpheus_context);
static void morpheus_close_usb_device(MorpheusUSBContext *morpheus_context);
static bool morpheus_enable_tracking(MorpheusUSBContext *morpheus_context);
static bool morpheus_set_headset_power(MorpheusUSBContext *morpheus_context, bool bIsOn);
static bool morpheus_set_led_brightness(MorpheusUSBContext *morpheus_context, unsigned short led_bitmask, unsigned char intensity);
static bool morpheus_turn_off_processor_unit(MorpheusUSBContext *morpheus_context);
static bool morpheus_set_vr_mode(MorpheusUSBContext *morpheus_context, bool bIsOn);
static bool morpheus_set_cinematic_configuration(
	MorpheusUSBContext *morpheus_context,
	unsigned char ScreenDistance, unsigned char ScreenSize, unsigned char Brightness, unsigned char MicVolume, bool UnknownVRSetting);
static bool morpheus_send_command(MorpheusUSBContext *morpheus_context, MorpheusCommand &command);

// -- public interface
// -- Morpheus HMD Config
const int MorpheusHMDConfig::CONFIG_VERSION = 2;

const boost::property_tree::ptree
MorpheusHMDConfig::config2ptree()
{
    boost::property_tree::ptree pt;

	pt.put("is_valid", is_valid);
	pt.put("version", MorpheusHMDConfig::CONFIG_VERSION);

	pt.put("Calibration.Accel.X.k", accelerometer_gain.i);
	pt.put("Calibration.Accel.Y.k", accelerometer_gain.j);
	pt.put("Calibration.Accel.Z.k", accelerometer_gain.k);
	pt.put("Calibration.Accel.X.b", raw_accelerometer_bias.i);
	pt.put("Calibration.Accel.Y.b", raw_accelerometer_bias.j);
	pt.put("Calibration.Accel.Z.b", raw_accelerometer_bias.k);
	pt.put("Calibration.Accel.Variance", raw_accelerometer_variance);
	pt.put("Calibration.Gyro.X.k", gyro_gain.i);
	pt.put("Calibration.Gyro.Y.k", gyro_gain.j);
	pt.put("Calibration.Gyro.Z.k", gyro_gain.k);
	pt.put("Calibration.Gyro.X.b", raw_gyro_bias.i);
	pt.put("Calibration.Gyro.Y.b", raw_gyro_bias.j);
	pt.put("Calibration.Gyro.Z.b", raw_gyro_bias.k);
	pt.put("Calibration.Gyro.Variance", raw_gyro_variance);
	pt.put("Calibration.Gyro.Drift", raw_gyro_drift);
	pt.put("Calibration.Identity.Gravity.X", identity_gravity_direction.i);
	pt.put("Calibration.Identity.Gravity.Y", identity_gravity_direction.j);
	pt.put("Calibration.Identity.Gravity.Z", identity_gravity_direction.k);

	pt.put("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
	pt.put("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);

	pt.put("Calibration.Orientation.Variance", orientation_variance);

	pt.put("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

	pt.put("OrientationFilter.FilterType", orientation_filter_type);

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
	pt.put("max_poll_failure_timeout_ms", max_poll_failure_timeout_ms);

	pt.put("FilterSettings.LowPassOptical.Distance", filter_lowpassoptical_distance);
	pt.put("FilterSettings.LowPassOptical.Smoothing", filter_lowpassoptical_smoothing);

	pt.put("FilterSettings.Madgwick.Beta", filter_madgwick_beta);
	pt.put("FilterSettings.Madgwick.Stabilization", filter_madgwick_stabilization);
	pt.put("FilterSettings.Madgwick.StabilizationMinBeta", filter_madgwick_stabilization_min_beta);
	pt.put("FilterSettings.Madgwick.StabilizationSmoothingFactor", filter_madgwick_stabilization_smoothing_factor);

	pt.put("FilterSettings.VelocitySmoothingFactor", filter_velocity_smoothing_factor);
	pt.put("FilterSettings.AngularSmoothingFactor", filter_angular_smoothing_factor);
	pt.put("FilterSettings.VelocityPredictionCutoff", filter_velocity_prediction_cutoff);
	pt.put("FilterSettings.AngularPredictionCutoff", filter_angular_prediction_cutoff);

	writeTrackingColor(pt, tracking_color_id);

    return pt;
}

void
MorpheusHMDConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == MorpheusHMDConfig::CONFIG_VERSION)
    {
		is_valid = pt.get<bool>("is_valid", false);

		prediction_time = pt.get<float>("prediction_time", prediction_time);
		ang_prediction_time = pt.get<float>("ang_prediction_time", ang_prediction_time);
		max_poll_failure_timeout_ms = pt.get<long>("max_poll_failure_timeout_ms", max_poll_failure_timeout_ms);

		// Use the current accelerometer values (constructor defaults) as the default values
		accelerometer_gain.i = pt.get<float>("Calibration.Accel.X.k", accelerometer_gain.i);
		accelerometer_gain.j = pt.get<float>("Calibration.Accel.Y.k", accelerometer_gain.j);
		accelerometer_gain.k = pt.get<float>("Calibration.Accel.Z.k", accelerometer_gain.k);
		raw_accelerometer_bias.i = pt.get<float>("Calibration.Accel.X.b", raw_accelerometer_bias.i);
		raw_accelerometer_bias.j = pt.get<float>("Calibration.Accel.Y.b", raw_accelerometer_bias.j);
		raw_accelerometer_bias.k = pt.get<float>("Calibration.Accel.Z.b", raw_accelerometer_bias.k);
		raw_accelerometer_variance = pt.get<float>("Calibration.Accel.Variance", raw_accelerometer_variance);

		// Use the current gyroscope values (constructor defaults) as the default values
		gyro_gain.i = pt.get<float>("Calibration.Gyro.X.k", gyro_gain.i);
		gyro_gain.j = pt.get<float>("Calibration.Gyro.Y.k", gyro_gain.j);
		gyro_gain.k = pt.get<float>("Calibration.Gyro.Z.k", gyro_gain.k);
		raw_gyro_bias.i = pt.get<float>("Calibration.Gyro.X.b", raw_gyro_bias.i);
		raw_gyro_bias.j = pt.get<float>("Calibration.Gyro.Y.b", raw_gyro_bias.j);
		raw_gyro_bias.k = pt.get<float>("Calibration.Gyro.Z.b", raw_gyro_bias.k);
		raw_gyro_variance = pt.get<float>("Calibration.Gyro.Variance", raw_gyro_variance);
		raw_gyro_drift = pt.get<float>("Calibration.Gyro.Drift", raw_gyro_drift);

		position_variance_exp_fit_a = pt.get<float>("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
		position_variance_exp_fit_b = pt.get<float>("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);

		orientation_variance = pt.get<float>("Calibration.Orientation.Variance", orientation_variance);

		mean_update_time_delta = pt.get<float>("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

		orientation_filter_type = pt.get<std::string>("OrientationFilter.FilterType", orientation_filter_type);

		position_filter_type = pt.get<std::string>("PositionFilter.FilterType", position_filter_type);
		max_velocity = pt.get<float>("PositionFilter.MaxVelocity", max_velocity);

		// Get the calibration direction for "down"
		identity_gravity_direction.i = pt.get<float>("Calibration.Identity.Gravity.X", identity_gravity_direction.i);
		identity_gravity_direction.j = pt.get<float>("Calibration.Identity.Gravity.Y", identity_gravity_direction.j);
		identity_gravity_direction.k = pt.get<float>("Calibration.Identity.Gravity.Z", identity_gravity_direction.k);

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

		filter_lowpassoptical_distance = pt.get<float>("FilterSettings.LowPassOptical.Distance", filter_lowpassoptical_distance);
		filter_lowpassoptical_smoothing = pt.get<float>("FilterSettings.LowPassOptical.Smoothing", filter_lowpassoptical_smoothing);

		filter_madgwick_beta = pt.get<float>("FilterSettings.Madgwick.Beta", filter_madgwick_beta);
		filter_madgwick_stabilization = pt.get<bool>("FilterSettings.Madgwick.Stabilization", filter_madgwick_stabilization);
		filter_madgwick_stabilization_min_beta = pt.get<float>("FilterSettings.Madgwick.StabilizationMinBeta", filter_madgwick_stabilization_min_beta);
		filter_madgwick_stabilization_smoothing_factor = pt.get<float>("FilterSettings.Madgwick.StabilizationSmoothingFactor", filter_madgwick_stabilization_smoothing_factor);

		filter_velocity_smoothing_factor = pt.get<float>("FilterSettings.VelocitySmoothingFactor", filter_velocity_smoothing_factor);
		filter_angular_smoothing_factor = pt.get<float>("FilterSettings.AngularSmoothingFactor", filter_angular_smoothing_factor);
		filter_velocity_prediction_cutoff = pt.get<float>("FilterSettings.VelocityPredictionCutoff", filter_velocity_prediction_cutoff);
		filter_angular_prediction_cutoff = pt.get<float>("FilterSettings.AngularPredictionCutoff", filter_angular_prediction_cutoff);
    }
    else
    {
        SERVER_LOG_WARNING("MorpheusHMDConfig") <<
            "Config version " << version << " does not match expected version " <<
            MorpheusHMDConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- Morpheus HMD Sensor Frame -----
void MorpheusHMDSensorFrame::parse_data_input(
	const MorpheusHMDConfig *config,
	const MorpheusRawSensorFrame *data_input)
{
	//short raw_seq = static_cast<short>((data_input->seq_frame[1] << 8) | data_input->seq_frame[0]);
	int raw_seq = static_cast<int>((data_input->seq_frame[2] << 16) | (data_input->seq_frame[1] << 8) | data_input->seq_frame[0]);

	// Piece together the 12-bit accelerometer data 
	// rotate data 90degrees about Z so that sensor Y is up, flip X and Z)
	// +X - goes out the left of the headset
	// +Y - goes out the top of the headset
	// +Z - goes out the back of the headset
	short raw_accelX = static_cast<short>(((data_input->accel_y[1] << 8) | data_input->accel_y[0])) >> 4;
	short raw_accelY = static_cast<short>(((data_input->accel_x[1] << 8) | data_input->accel_x[0])) >> 4;
	//- ?
	short raw_accelZ = -(static_cast<short>(((data_input->accel_z[1] << 8) | data_input->accel_z[0])) >> 4);

	// Piece together the 16-bit gyroscope data
	short raw_gyroYaw = static_cast<short>((data_input->gyro_yaw[1] << 8) | data_input->gyro_yaw[0]);
	short raw_gyroPitch = static_cast<short>((data_input->gyro_pitch[1] << 8) | data_input->gyro_pitch[0]);
	//- ?
	short raw_gyroRoll = -static_cast<short>((data_input->gyro_roll[1] << 8) | data_input->gyro_roll[0]);

	// Save the sequence number
	SequenceNumber = static_cast<int>(raw_seq);

	// Save the raw accelerometer values
	RawAccel.i = static_cast<int>(raw_accelX); //y
	RawAccel.j = static_cast<int>(raw_accelY); //x
	RawAccel.k = static_cast<int>(raw_accelZ); //-

											   // Save the raw gyro values
	RawGyro.i = static_cast<int>(raw_gyroPitch); //y
	RawGyro.j = static_cast<int>(raw_gyroYaw); //x
	RawGyro.k = static_cast<int>(raw_gyroRoll); //-

												// calibrated_acc= (raw_acc - acc_bias) * acc_gain
	CalibratedAccel.i = (static_cast<float>(raw_accelX) - config->raw_accelerometer_bias.i) * config->accelerometer_gain.i;
	CalibratedAccel.j = (static_cast<float>(raw_accelY) - config->raw_accelerometer_bias.j) * config->accelerometer_gain.j;
	CalibratedAccel.k = (static_cast<float>(raw_accelZ) - config->raw_accelerometer_bias.k) * config->accelerometer_gain.k;

	// calibrated_gyro= (raw_gyro - gyro_bias) * gyro_gain
	CalibratedGyro.i = (static_cast<float>(raw_gyroPitch) - config->raw_gyro_bias.i) * config->gyro_gain.i;
	CalibratedGyro.j = (static_cast<float>(raw_gyroYaw) - config->raw_gyro_bias.j) * config->gyro_gain.j;
	CalibratedGyro.k = (static_cast<float>(raw_gyroRoll) - config->raw_gyro_bias.k) * config->gyro_gain.k;
}

// -- Morpheus HMD State -----
void MorpheusHMDState::parse_data_input(
	const MorpheusHMDConfig *config, 
	const struct MorpheusSensorData *data_input)
{
	SensorFrames[0].parse_data_input(config, &data_input->imu_frame_0);
	SensorFrames[1].parse_data_input(config, &data_input->imu_frame_1);
}

// -- Morpheus HMD -----
MorpheusHMD::MorpheusHMD()
    : cfg()
    , USBContext(nullptr)
    , NextPollSequenceNumber(0)
	, bIsTracking(false)
	, m_HIDPacketProcessor(nullptr)
	, m_hmdListener(nullptr)
{
    USBContext = new MorpheusUSBContext;
}

MorpheusHMD::~MorpheusHMD()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~MorpheusHMD") << "HMD deleted without calling close() first!";
    }

	if (m_HIDPacketProcessor)
	{
		delete m_HIDPacketProcessor;
	}

    delete USBContext;
}

bool MorpheusHMD::open()
{
    HMDDeviceEnumerator enumerator(HMDDeviceEnumerator::CommunicationType_HID);
    bool success = false;

    if (enumerator.is_valid())
    {
        success = open(&enumerator);
    }

    return success;
}

bool MorpheusHMD::open(
    const DeviceEnumerator *enumerator)
{
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    const char *cur_dev_path = pEnum->get_path();
    bool success = false;

    if (getIsOpen())
    {
        SERVER_LOG_WARNING("MorpheusHMD::open") << "MorpheusHMD(" << cur_dev_path << ") already open. Ignoring request.";
        success = true;
    }
    else
    {
		SERVER_LOG_INFO("MorpheusHMD::open") << "Opening MorpheusHMD(" << cur_dev_path << ").";

		USBContext->device_identifier = cur_dev_path;

		// Open the sensor interface using HIDAPI
		USBContext->sensor_device_path = pEnum->get_hid_hmd_enumerator()->get_interface_path(MORPHEUS_SENSOR_INTERFACE);
		USBContext->sensor_device_handle = hid_open_path(USBContext->sensor_device_path.c_str());
		if (USBContext->sensor_device_handle != nullptr)
		{
			hid_set_nonblocking(USBContext->sensor_device_handle, 1);
		}

		// Open the command interface using libusb.
		// NOTE: Ideally we would use one usb library for both interfaces, but there are some complications.
		// A) The command interface uses the bulk transfer endpoint and HIDApi doesn't support that endpoint.
		// B) In Windows, libusb doesn't handle a high frequency of requests coming from two different threads well.
		// In this case, PS3EyeDriver is constantly sending bulk transfer requests in its own thread to get video frames.
		// If we started sending control transfer requests for the sensor data in the main thread at the same time
		// it can lead to a crash. It shouldn't, but this was a problem previously setting video feed properties
		// from the color config tool while a video feed was running.
		morpheus_open_usb_device(USBContext);

        if (getIsOpen())  // Controller was opened and has an index
        {
			SERVER_LOG_INFO("MorpheusHMD::open") << "Turning on MorpheusHMD power.";
			if (morpheus_set_headset_power(USBContext, true))
			{
				SERVER_LOG_INFO("MorpheusHMD::open") << "Turning on MorpheusHMD tracking.";
				if (morpheus_enable_tracking(USBContext))
				{
					//morpheus_enable_tracking() resets morpheus_set_led_brightness() LED settings?
					//Lets just wait a bit.
					std::this_thread::sleep_for(std::chrono::milliseconds(1500));
					
					morpheus_set_led_brightness(USBContext, _MorpheusLED_FRONT, 0);
					morpheus_set_led_brightness(USBContext, _MorpheusLED_BACK, 50);
				}
				else
				{
					SERVER_LOG_ERROR("MorpheusHMD::open") << "... failed!";
				}
			}
			else
			{
				SERVER_LOG_ERROR("MorpheusHMD::open") << "... failed!";
			}

			/*std::string identifier = "Morpheus_";

			char usb_port_path[128];
			if (getUSBPortPath(usb_port_path, sizeof(usb_port_path)))
			{
				identifier.append(usb_port_path);

				// Load the config file
				cfg = MorpheusHMDConfig(identifier);
				cfg.load();
			}
			else
			{
				SERVER_LOG_ERROR("MorpheusHMD::open") << "getUSBPortPath() failed!";

				// Load the config file
				cfg = MorpheusHMDConfig();
				cfg.load();
			}*/

			// Load the config file
			cfg = MorpheusHMDConfig();
			cfg.load();

			// Create the sensor processor thread
			m_HIDPacketProcessor = new MorpheusHIDSensorProcessor(cfg);
			m_HIDPacketProcessor->start(USBContext, m_hmdListener);

			// Always save the config back out in case some defaults changed
			cfg.save();

            // Reset the polling sequence counter
            NextPollSequenceNumber = 0;

			success = true;
        }
        else
        {
            SERVER_LOG_ERROR("MorpheusHMD::open") << "Failed to open MorpheusHMD(" << cur_dev_path << ")";
			close();
        }
    }

    return success;
}

void MorpheusHMD::close()
{
    if (USBContext->sensor_device_handle != nullptr || USBContext->usb_device_handle != nullptr)
    {
		if (m_HIDPacketProcessor != nullptr)
		{
			// halt the HID packet processing thread
			m_HIDPacketProcessor->stop();
			delete m_HIDPacketProcessor;
			m_HIDPacketProcessor = nullptr;
		}

		if (USBContext->sensor_device_handle != nullptr)
		{
			SERVER_LOG_INFO("MorpheusHMD::close") << "Closing MorpheusHMD sensor interface(" << USBContext->sensor_device_path << ")";
			hid_close(USBContext->sensor_device_handle);
		}

		if (USBContext->usb_device_handle != nullptr)
		{
			SERVER_LOG_INFO("MorpheusHMD::close") << "Closing MorpheusHMD command interface";
			morpheus_set_headset_power(USBContext, false);
			morpheus_close_usb_device(USBContext);
		}

        USBContext->Reset();
    }
    else
    {
        SERVER_LOG_INFO("MorpheusHMD::close") << "MorpheusHMD already closed. Ignoring request.";
    }
}

// Getters
bool
MorpheusHMD::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == getDeviceType())
    {
        const char *enumerator_path = pEnum->get_path();
        const char *dev_path = USBContext->device_identifier.c_str();

#ifdef _WIN32
        matches = _stricmp(dev_path, enumerator_path) == 0;
#else
        matches = strcmp(dev_path, enumerator_path) == 0;
#endif
    }

    return matches;
}

bool
MorpheusHMD::getIsReadyToPoll() const
{
    return (getIsOpen());
}

std::string
MorpheusHMD::getUSBDevicePath() const
{
    return USBContext->sensor_device_path;
}

bool
MorpheusHMD::getIsOpen() const
{
    return USBContext->sensor_device_handle != nullptr && USBContext->usb_device_handle != nullptr;
}

IControllerInterface::ePollResult
MorpheusHMD::poll()
{
	if (m_HIDPacketProcessor != nullptr && !m_HIDPacketProcessor->hasThreadEnded())
	{
		int LastRawSequence = m_cachedState.PollSequenceNumber;

		m_HIDPacketProcessor->fetchLatestInputData(m_cachedState);

		if (m_cachedState.PollSequenceNumber != LastRawSequence)
		{
			return IDeviceInterface::_PollResultSuccessNewData;
		}
		else
		{
			return IDeviceInterface::_PollResultSuccessNoData;
		}
	}
	else
	{
		return IDeviceInterface::_PollResultFailure;
	}
}

void
MorpheusHMD::getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const
{
	outTrackingShape.shape_type = eCommonTrackingShapeType::PointCloud;
	//###HipsterSloth TODO: These are just me eye balling the LED centers with a ruler
	// This should really be computed using the calibration tool
	outTrackingShape.shape.point_cloud.point[0] = { 0.00f, 0.00f, 0.00f }; // 0
	outTrackingShape.shape.point_cloud.point[1] = { 7.25f, 4.05f, 3.75f }; // 1
	outTrackingShape.shape.point_cloud.point[2] = { 9.05f, 0.00f, 9.65f }; // 2
	outTrackingShape.shape.point_cloud.point[3] = { 7.25f, -4.05f, 3.75f }; // 3
	outTrackingShape.shape.point_cloud.point[4] = { -7.25f, 4.05f, 3.75f }; // 4
	outTrackingShape.shape.point_cloud.point[5] = { -9.05f, 0.00f, 9.65f }; // 5
	outTrackingShape.shape.point_cloud.point[6] = { -7.25f, -4.05f, 3.75f }; // 6
	outTrackingShape.shape.point_cloud.point[7] = { 5.65f, -1.07f, 27.53f }; // 7
	outTrackingShape.shape.point_cloud.point[8] = { -5.65f, -1.07f, 27.53f }; // 8
	outTrackingShape.shape.point_cloud.point_count = 9;
}

bool 
MorpheusHMD::setTrackingColorID(const eCommonTrackingColorID tracking_color_id)
{
    return false;
}

void MorpheusHMD::setHmdListener(IHMDListener * listener)
{
	m_hmdListener = listener;
}

bool 
MorpheusHMD::getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const
{
	out_tracking_color_id = eCommonTrackingColorID::Blue;
	return true;
}

float
MorpheusHMD::getPredictionTime() const
{
	return getConfig()->prediction_time;
}

float
MorpheusHMD::getOrientationPredictionTime() const
{
	return getConfig()->ang_prediction_time;
}

const CommonDeviceState *
MorpheusHMD::getState(
    int lookBack) const
{
	return &m_cachedState;
}

long MorpheusHMD::getMaxPollFailureCount() const
{
    return cfg.max_poll_failure_timeout_ms;
}

void MorpheusHMD::setTrackingEnabled(bool bEnable)
{
	if (USBContext->usb_device_handle != nullptr)
	{
		if (!bIsTracking && bEnable)
		{
			morpheus_set_led_brightness(USBContext, _MorpheusLED_ALL, 0);
			morpheus_set_led_brightness(USBContext, _MorpheusLED_FONTTOPTRI, 50);
			bIsTracking = true;
		}
		else if (bIsTracking && !bEnable)
		{
			morpheus_set_led_brightness(USBContext, _MorpheusLED_FRONT, 0);
			morpheus_set_led_brightness(USBContext, _MorpheusLED_BACK, 50);
			bIsTracking = false;
		}
	}
}

#define MAX_USB_DEVICE_PORT_PATH 7

bool MorpheusHMD::getUSBPortPath(char *out_identifier, size_t max_identifier_length) const
{
	bool success = false;

	if (getIsOpen())
	{
		uint8_t port_numbers[MAX_USB_DEVICE_PORT_PATH];

		memset(out_identifier, 0, max_identifier_length);

		memset(port_numbers, 0, sizeof(port_numbers));

		libusb_device *device = libusb_get_device(USBContext->usb_device_handle);

		int port_count = libusb_get_port_numbers(device, port_numbers, MAX_USB_DEVICE_PORT_PATH);
		int bus_id = libusb_get_bus_number(device);

		snprintf(out_identifier, max_identifier_length, "b%d", bus_id);
		if (port_count > 0)
		{
			success = true;

			for (int port_index = 0; port_index < port_count; ++port_index)
			{
				uint8_t port_number = port_numbers[port_index];
				char port_string[8];

				snprintf(port_string, sizeof(port_string), (port_index == 0) ? "_p%d" : ".%d", port_number);
				port_string[sizeof(port_string) - 1] = '0';

				if (strlen(out_identifier) + strlen(port_string) + 1 <= max_identifier_length)
				{
					std::strcat(out_identifier, port_string);
				}
				else
				{
					success = false;
					break;
				}
			}
		}
	}

	return success;
}

//-- private morpheus commands ---
static bool morpheus_open_usb_device(
	MorpheusUSBContext *morpheus_context)
{
	bool bSuccess = true;
	if (libusb_init(&morpheus_context->usb_context) == LIBUSB_SUCCESS)
	{
		libusb_set_debug(morpheus_context->usb_context, 3);
	}
	else
	{
		SERVER_LOG_ERROR("morpeus_open_usb_device") << "libusb context initialization failed!";
		bSuccess = false;
	}

	if (bSuccess)
	{
		morpheus_context->usb_device_handle = 
			libusb_open_device_with_vid_pid(
				morpheus_context->usb_context,
				MORPHEUS_VENDOR_ID, MORPHEUS_PRODUCT_ID);

		if (morpheus_context->usb_device_handle == nullptr)
		{
			SERVER_LOG_ERROR("morpeus_open_usb_device") << "Morpheus USB device not found!";
			bSuccess = false;
		}
	}

	if (bSuccess)
	{
		libusb_device *device = libusb_get_device(morpheus_context->usb_device_handle);
		int result = libusb_get_config_descriptor_by_value(
			device, 
			MORPHEUS_CONFIGURATION_PSVR, 
			&morpheus_context->usb_device_descriptor);

		if (result != LIBUSB_SUCCESS) 
		{
			SERVER_LOG_ERROR("morpeus_open_usb_device") << "Failed to retrieve Morpheus usb config descriptor";
			bSuccess = false;
		}
	}

	for (int interface_index = 0; 
		 bSuccess && interface_index < morpheus_context->usb_device_descriptor->bNumInterfaces; 
		 interface_index++) 
	{
		int mask = 1 << interface_index;

		if (MORPHEUS_USB_INTERFACES_MASK_TO_CLAIM & mask) 
		{
			int result = 0;

			#ifndef _WIN32
			result = libusb_kernel_driver_active(morpheus_context->usb_device_handle, interface_index);
			if (result < 0) 
			{
				SERVER_LOG_ERROR("morpeus_open_usb_device") << "USB Interface #"<< interface_index <<" driver status failed";
				bSuccess = false;
			}

			if (bSuccess && result == 1)
			{
				SERVER_LOG_ERROR("morpeus_open_usb_device") << "Detach kernel driver on interface #" << interface_index;

				result = libusb_detach_kernel_driver(morpheus_context->usb_device_handle, interface_index);
				if (result != LIBUSB_SUCCESS) 
				{
					SERVER_LOG_ERROR("morpeus_open_usb_device") << "Interface #" << interface_index << " detach failed";
					bSuccess = false;
				}
			}
			#endif //_WIN32

			result = libusb_claim_interface(morpheus_context->usb_device_handle, interface_index);
			if (result == LIBUSB_SUCCESS)
			{
				morpheus_context->usb_claimed_interface_mask |= mask;
			}
			else
			{
				SERVER_LOG_ERROR("morpeus_open_usb_device") << "Interface #" << interface_index << " claim failed";
				bSuccess = false;
			}
		}
	}

	if (!bSuccess)
	{
		morpheus_close_usb_device(morpheus_context);
	}

	return bSuccess;
}

static void morpheus_close_usb_device(
	MorpheusUSBContext *morpheus_context)
{
	for (int interface_index = 0; morpheus_context->usb_claimed_interface_mask != 0; ++interface_index) 
	{
		int interface_mask = 1 << interface_index;

		if ((morpheus_context->usb_claimed_interface_mask & interface_mask) != 0)
		{
			libusb_release_interface(morpheus_context->usb_device_handle, interface_index);
			morpheus_context->usb_claimed_interface_mask &= ~interface_mask;
		}
	}

	if (morpheus_context->usb_device_descriptor != nullptr)
	{
		libusb_free_config_descriptor(morpheus_context->usb_device_descriptor);
		morpheus_context->usb_device_descriptor = nullptr;
	}

	if (morpheus_context->usb_device_handle != nullptr)
	{
		libusb_close(morpheus_context->usb_device_handle);
		morpheus_context->usb_device_handle = nullptr;
	}

	if (morpheus_context->usb_context != nullptr)
	{
		libusb_exit(morpheus_context->usb_context);
		morpheus_context->usb_context = nullptr;
	}
}

static bool morpheus_enable_tracking(
	MorpheusUSBContext *morpheus_context)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_EnableTracking;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 8;
	((int*)command.payload)[0] = 0xFFFFFF00; // Magic numbers!  Turns on the VR mode and the blue lights on the front
	((int*)command.payload)[1] = 0x00000000;

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_set_headset_power(
	MorpheusUSBContext *morpheus_context,
	bool bIsOn)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetHeadsetPower;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 4;
	((int*)command.payload)[0] = bIsOn ? 0x00000001 : 0x00000000;

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_set_led_brightness(
	MorpheusUSBContext *morpheus_context,
	unsigned short led_bitmask,
	unsigned char intensity)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetLEDBrightness;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 16;
	((unsigned short*)command.payload)[0] = led_bitmask;

	unsigned short mask = led_bitmask;
	for (int led_index = 0; led_index < 9; ++led_index)
	{
		command.payload[2 + led_index] = ((mask & 0x001) > 0) ? intensity : 0;
		mask = mask >> 1;
	}

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_turn_off_processor_unit(
	MorpheusUSBContext *morpheus_context)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_TurnOffProcessorUnit;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 4;
	((int*)command.payload)[0] = 0x00000001;

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_set_vr_mode(
	MorpheusUSBContext *morpheus_context,
	bool bIsOn)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetVRMode;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 4;
	((int*)command.payload)[0] = bIsOn ? 0x00000001 : 0x00000000;

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_set_cinematic_configuration(
	MorpheusUSBContext *morpheus_context,
	unsigned char ScreenDistance, 
	unsigned char ScreenSize, 
	unsigned char Brightness, 
	unsigned char MicVolume, 
	bool UnknownVRSetting)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetCinematicConfiguration;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 16;
	command.payload[1] = ScreenSize;
	command.payload[2] = ScreenDistance;
	command.payload[10] = Brightness;
	command.payload[11] = MicVolume;
	command.payload[14] = UnknownVRSetting ? 0 : 1;

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_send_command(
	MorpheusUSBContext *morpheus_context,
	MorpheusCommand &command)
{
	if (morpheus_context->usb_device_handle != nullptr)
	{
		const size_t command_length = static_cast<size_t>(command.header.length) + sizeof(command.header);
		const int endpointAddress =
			(morpheus_context->usb_device_descriptor->interface[MORPHEUS_COMMAND_INTERFACE]
				.altsetting[0]
				.endpoint[0]
				.bEndpointAddress) & ~MORPHEUS_ENDPOINT_IN;

		int transferredByteCount = 0;
		int result =
			libusb_bulk_transfer(
				morpheus_context->usb_device_handle,
				endpointAddress,
				(unsigned char *)&command,
				command_length,
				&transferredByteCount,
				0);


		// $TODO Workaround for swallowing bulk request.
		std::this_thread::sleep_for(std::chrono::milliseconds(30));

		return result == LIBUSB_SUCCESS && transferredByteCount == command_length;
	}
	else
	{
		return false;
	}
}
