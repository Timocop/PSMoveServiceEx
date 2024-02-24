#include "PSEyeVideoCapture.h"
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <chrono>
#ifdef HAVE_PS3EYE
#include "ps3eye.h"
#endif
#ifdef HAVE_CLEYE
#include "CLEyeMulticam.h"
#include "PlatformDeviceAPIWin32.h"
const uint16_t VENDOR_ID = 0x1415;
const uint16_t PRODUCT_ID = 0x2000;
const char *CLEYE_DRIVER_PROVIDER_NAME = "Code Laboratories, Inc.";
const char *CL_DRIVER_REG_PATH = "Software\\PS3EyeCamera\\Settings";  // [HKCU]
#endif

#ifdef WIN32
#include <windows.h> 
#include <stdio.h> 
#include <tchar.h>
#include <strsafe.h>
#include <fstream>
#endif

enum
{
#ifdef HAVE_CLEYE
    PSEYE_CAP_CLMULTI   = 2100,
    PSEYE_CAP_CLEYE     = 2200,
#endif
#ifdef HAVE_PS3EYE
    PSEYE_CAP_PS3EYE    = 2300
#endif
};

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

/**
 cv::IVideoCapture is a convenient abstract base for custom capture devices.
 Unfortunately, it does not have a public interface, so we redefine it here.
 https://github.com/Itseez/opencv/blob/09e6c82190b558e74e2e6a53df09844665443d6d/modules/videoio/src/precomp.hpp#L164-L176
*/
class cv::IVideoCapture
{
public:
    virtual ~IVideoCapture() {}
    virtual double getProperty(int) const { return 0; }
    virtual bool setProperty(int, double) { return false; }
    virtual bool grabFrame() = 0;
    virtual bool retrieveFrame(int, cv::OutputArray) = 0;
    virtual bool isOpened() const = 0;
    virtual int getCaptureDomain() { return CAP_ANY; } // Return the type of the capture object: CAP_VFW, etc...
};

/*
-- Camera-specific implementations of cv::IVideoCapture --
 Examples
 
 HAVE_DSHOW:
 https://github.com/Itseez/opencv/blob/ddf82d0b154873510802ef75c53e628cd7b2cb13/modules/videoio/src/cap_dshow.hpp#L23
 https://github.com/Itseez/opencv/blob/ddf82d0b154873510802ef75c53e628cd7b2cb13/modules/videoio/src/cap_dshow.cpp#L3141
 
 WINRT_VIDEO:
 https://github.com/Itseez/opencv/blob/ddf82d0b154873510802ef75c53e628cd7b2cb13/modules/videoio/src/cap_winrt_capture.hpp#L45
 https://github.com/Itseez/opencv/blob/ddf82d0b154873510802ef75c53e628cd7b2cb13/modules/videoio/src/cap_winrt_capture.cpp#L121
*/

#ifdef HAVE_CLEYE
/// Implementation of cv::IVideoCapture when using CLEyeMulticam.dll
/**
Either uses the DLL that comes with PSMoveService and the user has their camera activated.
Or the user has the CL Eye Platform SDK developer binaries installed and they delete
the DLL that comes with PSMoveService.
*/
class PSEYECaptureCAM_CLMULTI : public cv::IVideoCapture
{
public:
    PSEYECaptureCAM_CLMULTI(int _index)
        : m_index(-1), m_width(-1), m_height(-1),
        m_frame(NULL), m_frame4ch(NULL)
    {
		m_isValid = open(_index);
    }

    ~PSEYECaptureCAM_CLMULTI()
    {
        close();
    }

    double getProperty(int property_id) const
    {
        int _width, _height;
        switch (property_id)
        {
        case CV_CAP_PROP_BRIGHTNESS:
            return (double)(CLEyeGetCameraParameter(m_eye, CLEYE_LENSBRIGHTNESS)); // [-500, 500]
        case CV_CAP_PROP_CONTRAST:
            return false;
        case CV_CAP_PROP_EXPOSURE:
            // [0, 511] -> [0, 255]
            return double(CLEyeGetCameraParameter(m_eye, CLEYE_EXPOSURE))/2.0;
        case CV_CAP_PROP_FPS:
            return (double)(60);
        case CV_CAP_PROP_FRAME_HEIGHT:
            CLEyeCameraGetFrameDimensions(m_eye, _width, _height);
            return (double)(_height);
        case CV_CAP_PROP_FRAME_WIDTH:
            CLEyeCameraGetFrameDimensions(m_eye, _width, _height);
            return (double)(_width);
        case CV_CAP_PROP_GAIN:
            // [0, 79] -> [0, 255]
            return double(CLEyeGetCameraParameter(m_eye, CLEYE_GAIN)) * (256.0/80.0);
        case CV_CAP_PROP_HUE:
            return 0;
        case CV_CAP_PROP_SHARPNESS:
            return 0;
        }
        return 0;
    }

    bool setProperty(int property_id, double value)
    {
        int val;
        if (!m_eye)
        {
            return false;
        }
        switch (property_id)
        {
        case CV_CAP_PROP_BRIGHTNESS:
            // [-500, 500]
            CLEyeSetCameraParameter(m_eye, CLEYE_LENSBRIGHTNESS, (int)value);
        case CV_CAP_PROP_CONTRAST:
            return false;
        case CV_CAP_PROP_EXPOSURE:
            CLEyeSetCameraParameter(m_eye, CLEYE_AUTO_EXPOSURE, value <= 0);
            if (value > 0)
            {
                //[0, 255] -> [0, 511]
                val = (int)(value * 2.0);
                CLEyeSetCameraParameter(m_eye, CLEYE_EXPOSURE, val);
            }
        case CV_CAP_PROP_FPS:
            return false; //TODO: Modifying FPS probably requires resetting the camera
        case CV_CAP_PROP_FRAME_HEIGHT:
            return false; //TODO: Modifying frame size probably requires resetting the camera
        case CV_CAP_PROP_FRAME_WIDTH:
            return false; //TODO: Modifying frame size probably requires resetting the camera
        case CV_CAP_PROP_GAIN:
            CLEyeSetCameraParameter(m_eye, CLEYE_AUTO_GAIN, value <= 0);
            if (value > 0)
            {
                //[0, 255] -> [0, 79]
                val = (int)ceil(value * 80.0 / 256.0);
                CLEyeSetCameraParameter(m_eye, CLEYE_GAIN, val);
            }
        case CV_CAP_PROP_HUE:
            return false;
        case CV_CAP_PROP_SHARPNESS:
            return false; // TODO: Using OpenCV interface, sharpness appears to work
        }
        return true;
    }

    bool grabFrame()
    {
        cvGetRawData(m_frame4ch, &pCapBuffer, 0, 0);
        return true;
    }

    bool retrieveFrame(int channel, cv::OutputArray outArray)
    {
        CLEyeCameraGetFrame(m_eye, pCapBuffer, 33);
        const int from_to[] = { 0, 0, 1, 1, 2, 2 };
        const CvArr** src = (const CvArr**)&m_frame4ch;
        CvArr** dst = (CvArr**)&m_frame;
        cvMixChannels(src, 1, dst, 1, from_to, 3);

        if (m_frame->origin == IPL_ORIGIN_TL)
            cv::cvarrToMat(m_frame).copyTo(outArray);
        else
        {
            cv::Mat temp = cv::cvarrToMat(m_frame);
            flip(temp, outArray, 0);
        }

        return true;
    }

    int getCaptureDomain() { return PSEYE_CAP_CLMULTI; }

    bool isOpened() const
    {
        return (m_isValid && m_index != -1);
    }

    std::string getUniqueIndentifier() const
    {
        std::string identifier= "cleye_";

        if (isOpened())
        {
            GUID guid = CLEyeGetCameraUUID(m_index);
            char guid_string[128];

            snprintf(guid_string, sizeof(guid_string), "cleye_%x-%x-%x-%x%x%x%x%x%x%x%x",
                guid.Data1, guid.Data2, guid.Data3,
                guid.Data4[0], guid.Data4[1], guid.Data4[2], guid.Data4[3],
                guid.Data4[4], guid.Data4[5], guid.Data4[6], guid.Data4[7]);
            identifier.append(guid_string);
        }

        return identifier;
    }

protected:
    
    bool open(int _index) {
        close();
        int cams = CLEyeGetCameraCount();
        std::cout << "CLEyeGetCameraCount() found " << cams << " devices." << std::endl;
        if (_index < cams)
        {
            std::cout << "Attempting to open camera " << _index << " of " << cams << "." << std::endl;
            GUID guid = CLEyeGetCameraUUID(_index);
            m_eye = CLEyeCreateCamera(guid, CLEYE_COLOR_PROCESSED, CLEYE_VGA, 75);
            CLEyeCameraGetFrameDimensions(m_eye, m_width, m_height);
            
            m_frame4ch = cvCreateImage(cvSize(m_width, m_height), IPL_DEPTH_8U, 4);
            m_frame = cvCreateImage(cvSize(m_width, m_height), IPL_DEPTH_8U, 3);
            
            CLEyeCameraStart(m_eye);
            CLEyeSetCameraParameter(m_eye, CLEYE_AUTO_EXPOSURE, false);
            CLEyeSetCameraParameter(m_eye, CLEYE_AUTO_GAIN, false);
            m_index = _index;
        }
        return isOpened();
    }
    
    void close()
    {
        if (isOpened())
        {
            CLEyeCameraStop(m_eye);
            CLEyeDestroyCamera(m_eye);
        }
        cvReleaseImage(&m_frame);
        cvReleaseImage(&m_frame4ch);
        m_index = -1;
		m_isValid = false;
    }
    
    
    int m_index, m_width, m_height;
    PBYTE pCapBuffer;
    IplImage* m_frame;
    IplImage* m_frame4ch;
    CLEyeCameraInstance m_eye;
	bool m_isValid;
};

// We don't need an implementation for CL EYE Driver because
// it uses the native DShow IVideoCapture device.
#endif

#ifdef HAVE_PS3EYE
/// Implementation of PS3EyeCapture when using PS3EYEDriver
class PSEYECaptureCAM_PS3EYE : public cv::IVideoCapture
{
public:
    PSEYECaptureCAM_PS3EYE(int _index)
    : m_index(-1), m_width(-1), m_height(-1), m_widthStep(-1),
		m_frameAvailable(false), m_waitFrame(false), m_maxFailPoll(0),
		m_size(-1), m_MatBayer(0, 0, CV_8UC1)
    {
		m_lastPollDataTimestamp = std::chrono::high_resolution_clock::now();

        //CoInitialize(NULL);
		m_isValid = open(_index);
    }

    ~PSEYECaptureCAM_PS3EYE()
    {
        close();
    }

    double getProperty(int property_id) const
    {
        switch (property_id)
        {
        case CV_CAP_PROP_BRIGHTNESS:
            return (double)(eye->getBrightness());
        case CV_CAP_PROP_CONTRAST:
            return (double)(eye->getContrast());
        case CV_CAP_PROP_EXPOSURE:
            // Default 120
            return (double)(eye->getExposure());
        case CV_CAP_PROP_FPS:
            return (double)(eye->getFrameRate());
        case CV_CAP_PROP_FRAME_HEIGHT:
            return (double)(eye->getHeight());
        case CV_CAP_PROP_FRAME_WIDTH:
            return (double)(eye->getWidth());
        case CV_CAP_PROP_GAIN:
            // [0, 63] -> [0, 255]
            return (double)(eye->getGain())*256.0/64.0;
        case CV_CAP_PROP_HUE:
            return (double)(eye->getHue());
        case CV_CAP_PROP_SHARPNESS:
            // [0, 63] -> [0, 255]
            return (double)(eye->getSharpness())*256.0 / 64.0;
		case CV_CAP_PROP_FRAMEAVAILABLE:
			return (bool)m_frameAvailable;
		case CV_CAP_PROP_WAITFRAME:
			return (bool)m_waitFrame;
		case CV_CAP_PROP_MAXFAILPOLL:
			return (int)m_maxFailPoll;
        }
        return 0;
    }

    bool setProperty(int property_id, double value)
    {
        if (!eye)
        {
            return false;
        }

		int val;
		bool refresh = true;

        switch (property_id)
        {
        case CV_CAP_PROP_BRIGHTNESS:
            // [0, 255] [20]
            eye->setBrightness((int)round(value));
			break;
        case CV_CAP_PROP_CONTRAST:
            // [0, 255] [37]
            eye->setContrast((int)round(value));
			break;
        case CV_CAP_PROP_EXPOSURE:
            // [0, 255] [120]
            eye->setExposure((int)round(value));
			break;
        case CV_CAP_PROP_FPS:
			// [15, 20, 30, 40 50, 60, 75]
			eye->stop();
			if (!eye->setFrameRate((int)round(value))) return false;
			eye->start();
			break;
        case CV_CAP_PROP_FRAME_HEIGHT:
			eye->stop();
			if (!eye->setHeight((int)round(value))) return false;
			eye->start();
			break;
            //return false; //TODO: Modifying frame size probably requires resetting the camera
        case CV_CAP_PROP_FRAME_WIDTH:
			eye->stop();
			if (!eye->setWidth((int)round(value))) return false;
			eye->start();
			break;
            //return false;
        case CV_CAP_PROP_GAIN:
            // [0, 255] -> [0, 63] [20]
            val = (int)(value * 64.0 / 256.0);
            eye->setGain(val);
			break;
        case CV_CAP_PROP_HUE:
            // [0, 255] [143]
            eye->setHue((int)round(value));
			break;
        case CV_CAP_PROP_SHARPNESS:
            // [0, 255] -> [0, 63] [0]
            val = (int)(value * 64.0 / 256.0);
            eye->setSharpness((int)round(value));
			break;
		case CV_CAP_PROP_FRAMEAVAILABLE:
			m_frameAvailable = (bool)value;
			refresh = false;
			break;
		case CV_CAP_PROP_WAITFRAME:
			m_waitFrame = (bool)value;
			refresh = false;
			break;
		case CV_CAP_PROP_MAXFAILPOLL:
			m_maxFailPoll = (int)value;
			refresh = false;
			break;
        }
        
		if (refresh)
		{
			refreshDimensions();
		}
        
        return true;
    }

    bool grabFrame()
    {
		std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> timeSinceLastPoll = now - m_lastPollDataTimestamp;

		if (!eye->isStreaming())
		{
			if (m_maxFailPoll > 0 && timeSinceLastPoll.count() > m_maxFailPoll)
			{
				capFrame.setTo(cv::Scalar(0, 0, 0));
				cv::putText(
					capFrame,
					"Tracker timed out.",
					cv::Point(128, capFrame.rows / 2),
					cv::FONT_HERSHEY_DUPLEX,
					1.0,
					CvScalar(255, 255, 255),
					2
				);
				m_frameAvailable = true;
				return true;
			}
			else 
			{
				return false;
			}
		}

		if (!m_waitFrame && m_frameAvailable)
			return true;

		bool success = eye->getFrame(m_MatBayer.data, m_waitFrame);
		if (!success)
		{
			if (m_maxFailPoll > 0 && timeSinceLastPoll.count() > m_maxFailPoll)
			{
				capFrame.setTo(cv::Scalar(0, 0, 0));
				cv::putText(
					capFrame,
					"Tracker timed out.",
					cv::Point(128, capFrame.rows / 2),
					cv::FONT_HERSHEY_DUPLEX,
					1.0,
					CvScalar(255, 255, 255),
					2
				);
				m_frameAvailable = true;
				return true;
			}
			else
			{
				return false;
			}
		}

		cv::cvtColor(m_MatBayer, capFrame, CV_BayerGB2BGR);
		m_frameAvailable = true;
		m_lastPollDataTimestamp = now;
		return true;
    }

    bool retrieveFrame(int outputType, cv::OutputArray outArray)
    {
		if (!m_waitFrame && !m_frameAvailable)
			return false;
		
		capFrame.copyTo(outArray);
        return true;
    }

    int getCaptureDomain() {
        return PSEYE_CAP_PS3EYE;
    }
    
    bool isOpened() const
    {
        return (m_isValid && m_index != -1);
    }

    std::string getUniqueIndentifier() const
    {
        std::string identifier = "ps3eye_";

        if (isOpened())
        {
            char usb_port_path[128];

            if (eye->getUSBPortPath(usb_port_path, sizeof(usb_port_path)))
            {
                identifier.append(usb_port_path);
            }
        }

        return identifier;
    }
protected:
    
    bool open(int _index)
    {
		capFrame = cv::Mat(480, 640, CV_8UC3, CvScalar(0, 0, 0));

        // Enumerate libusb devices
        std::vector<ps3eye::PS3EYECam::PS3EYERef> devices = ps3eye::PS3EYECam::getDevices();
		std::cout << "ps3eye::PS3EYECam::getDevices() found " << devices.size() << " devices." << std::endl;
        
        if (devices.size() > (unsigned int)_index) {
            
            eye = devices[_index];

            if (eye && eye->init(640, 480, 15, ps3eye::PS3EYECam::EOutputFormat::Bayer))
            {
                // Change any default settings here
                
                eye->start();
                
                eye->setAutogain(false);
                eye->setAutoWhiteBalance(false);

				eye->setFlip(true, false);
                
                m_index = _index;
                refreshDimensions();
                
                return true;
            }
        }
        return false;
    }
    
    void close()
    {
		std::cout << "ps3eye::PS3EYECam() index " << m_index << " closed." << std::endl;

        // eye will close itself when going out of scope.
        m_index = -1;
		m_isValid = false;
    }
    
    void refreshDimensions()
    {
        m_width = eye->getWidth();
        m_widthStep = eye->getRowBytes(); // just width * 1 byte per pixel.
        m_height = eye->getHeight();
        m_size = m_widthStep * m_height;
        m_MatBayer.create(cv::Size(m_width, m_height), CV_8UC1);
    }

    int m_index, m_width, m_height, m_widthStep;
	bool m_frameAvailable, m_waitFrame;
	int m_maxFailPoll;
	bool m_isValid;

    size_t m_size;
    cv::Mat m_MatBayer;
	cv::Mat capFrame;
    ps3eye::PS3EYECam::PS3EYERef eye;
	std::chrono::time_point<std::chrono::high_resolution_clock> m_lastPollDataTimestamp;
};

#endif

#ifdef WIN32

// $TODO This duplicated code is massive cancer. Merge SD and HD together.
#define VRIT_BUFF_SD_SIZE (480 * 640 * 3)
#define VRIT_BUFF_HD_SIZE (1080 * 1920 * 3)

/// Implementation of PS3EyeCapture using named pipes
class PSEYECaptureCAM_VIRTUAL : public cv::IVideoCapture
{
public:
	PSEYECaptureCAM_VIRTUAL(int _index) :
		m_index(-1), m_frameAvailable(false), m_pipeConnected(false),
		capturePipeSD(INVALID_HANDLE_VALUE), capturePipeHD(INVALID_HANDLE_VALUE),
		m_frameHeight(480), m_frameWidth(640)
	{
		//CoInitialize(NULL);
		m_isValid = open(_index);
	}

	~PSEYECaptureCAM_VIRTUAL()
	{
		close();
	}

	// Currently there is no way to get/set properties on virtual trackers.
	double getProperty(int property_id) const
	{
		switch (property_id)
		{
		case CV_CAP_PROP_BRIGHTNESS:
			return 0;
		case CV_CAP_PROP_CONTRAST:
			return 0;
		case CV_CAP_PROP_EXPOSURE:
			return 0;
		case CV_CAP_PROP_FPS:
			return 30;
		case CV_CAP_PROP_FRAME_HEIGHT:
			return (double)m_frameHeight;
		case CV_CAP_PROP_FRAME_WIDTH:
			return (double)m_frameWidth;
		case CV_CAP_PROP_GAIN:
			return 0;
		case CV_CAP_PROP_HUE:
			return 0;
		case CV_CAP_PROP_SHARPNESS:
			return 0;
		case CV_CAP_PROP_FORMAT:
			return cv::CAP_MODE_RGB;
		case CV_CAP_PROP_FRAMEAVAILABLE:
			return (bool)m_frameAvailable;
		}
		return 0;
	}

	// Currently there is no way to get/set properties on virtual trackers.
	bool setProperty(int property_id, double value)
	{
		switch (property_id)
		{
		case CV_CAP_PROP_FRAME_HEIGHT:
			m_frameHeight = (int)round(value);

			if (m_frameHeight == 0 || m_frameHeight == 480)
			{
				m_frameWidth = 640;
				m_frameHeight = 480;
			}
			else 
			{
				m_frameWidth = 1920;
				m_frameHeight = 1080;
			}

			m_frameAvailable = false;
			refreshPipe();
			return true;
		case CV_CAP_PROP_FRAME_WIDTH:
			m_frameWidth = (int)round(value);

			if (m_frameWidth == 0 || m_frameWidth == 640)
			{
				m_frameWidth = 640;
				m_frameHeight = 480;
			}
			else 
			{
				m_frameWidth = 1920;
				m_frameHeight = 1080;
			}

			m_frameAvailable = false;
			refreshPipe();
			return true;
		case CV_CAP_PROP_FRAMEAVAILABLE:
			m_frameAvailable = (bool)value;
		}

		return false;
	}

	bool grabFrame()
	{
		if (capFrameSD.rows == m_frameHeight && capFrameSD.cols == m_frameWidth)
		{
			return grabFrameSD();
		}

		if (capFrameHD.rows == m_frameHeight && capFrameHD.cols == m_frameWidth)
		{
			return grabFrameHD();
		}
	}

	bool grabFrameSD()
	{
		if (capturePipeSD == INVALID_HANDLE_VALUE)
		{
			capFrameSD.setTo(cv::Scalar(0, 0, 0));
			cv::putText(
				capFrameSD,
				"Virtual tracker not connected.",
				cv::Point(64, capFrameSD.rows / 2),
				cv::FONT_HERSHEY_DUPLEX,
				1.0,
				CvScalar(255, 255, 255),
				2
			);

			m_frameAvailable = true;
			return true;
		}

		BOOL connected = ConnectNamedPipe(capturePipeSD, NULL);
		if (connected)
			std::cout << "ps3eye::VIRTUAL() ConnectNamedPipe success, index " << m_index << std::endl;

		if (!connected)
			connected = (GetLastError() == ERROR_PIPE_CONNECTED);

		if (!connected)
		{
			if (GetLastError() != ERROR_PIPE_LISTENING) {
				std::cout << "ps3eye::VIRTUAL() ConnectNamedPipe failed, index " << m_index << ", GLE=" << GetLastError() << "." << std::endl;

				DisconnectNamedPipe(capturePipeSD);
			}
		}

		m_pipeConnected = (connected == TRUE);

		if (!connected)
		{
			capFrameSD.setTo(cv::Scalar(0, 0, 0));
			cv::putText(
				capFrameSD,
				"Virtual tracker not connected.",
				cv::Point(64, capFrameSD.rows / 2),
				cv::FONT_HERSHEY_DUPLEX,
				1.0,
				CvScalar(255, 255, 255),
				2
			);

			m_frameAvailable = true;
			return true;
		}

		DWORD dwRead;
		BOOL success = ReadFile(capturePipeSD, pipeBufferSD, sizeof(pipeBufferSD), &dwRead, NULL);
		if (!success)
		{
			bool bDisplayText = true;

			switch (GetLastError())
			{
			case ERROR_BROKEN_PIPE:
			{
				std::cout << "ps3eye::VIRTUAL() client disconnected, index " << m_index << "." << std::endl;
				break;
			}
			case ERROR_PIPE_LISTENING:
			{
				break;
			}
			case ERROR_NO_DATA:
			{
				// Everything ok, we are just waiting for new data.
				bDisplayText = false;
				break;
			}
			default:
			{
				std::cout << "ps3eye::VIRTUAL() ReadFile failed, index " << m_index << ", GLE=" << GetLastError() << "." << std::endl;
				break;
			}
			}

			if (bDisplayText)
			{
				capFrameSD.setTo(cv::Scalar(0, 0, 0));
				cv::putText(
					capFrameSD,
					"Virtual tracker not connected.",
					cv::Point(64, capFrameSD.rows / 2),
					cv::FONT_HERSHEY_DUPLEX,
					1.0,
					CvScalar(255, 255, 255),
					2
				);

				m_frameAvailable = true;
				return true;
			}

			return true;
		}

		if (m_frameAvailable)
			return true;

		memcpy((uchar*)capFrameSD.data, pipeBufferSD, sizeof(pipeBufferSD));

		m_frameAvailable = true;
		return true;
	}

	bool grabFrameHD()
	{
		if (capturePipeHD == INVALID_HANDLE_VALUE)
		{
			capFrameHD.setTo(cv::Scalar(0, 0, 0));
			cv::putText(
				capFrameHD,
				"Virtual tracker not connected.",
				cv::Point(256, capFrameHD.rows / 2),
				cv::FONT_HERSHEY_DUPLEX,
				3.0,
				CvScalar(255, 255, 255),
				5
			);

			m_frameAvailable = true;
			return true;
		}

		BOOL connected = ConnectNamedPipe(capturePipeHD, NULL);
		if (connected)
			std::cout << "ps3eye::VIRTUAL() ConnectNamedPipe success, index " << m_index << std::endl;

		if (!connected)
			connected = (GetLastError() == ERROR_PIPE_CONNECTED);

		if (!connected)
		{
			if (GetLastError() != ERROR_PIPE_LISTENING) {
				std::cout << "ps3eye::VIRTUAL() ConnectNamedPipe failed, index " << m_index << ", GLE=" << GetLastError() << "." << std::endl;

				DisconnectNamedPipe(capturePipeHD);
			}
		}

		m_pipeConnected = (connected == TRUE);

		if (!connected)
		{
			capFrameHD.setTo(cv::Scalar(0, 0, 0));
			cv::putText(
				capFrameHD,
				"Virtual tracker not connected.",
				cv::Point(256, capFrameHD.rows / 2),
				cv::FONT_HERSHEY_DUPLEX,
				3.0,
				CvScalar(255, 255, 255),
				5
			);

			m_frameAvailable = true;
			return true;
		}

		DWORD dwRead;
		BOOL success = ReadFile(capturePipeHD, pipeBufferHD, sizeof(pipeBufferHD), &dwRead, NULL);
		if (!success)
		{
			bool bDisplayText = true;

			switch (GetLastError())
			{
			case ERROR_BROKEN_PIPE:
			{
				std::cout << "ps3eye::VIRTUAL() client disconnected, index " << m_index << "." << std::endl;
				break;
			}
			case ERROR_PIPE_LISTENING:
			{
				break;
			}
			case ERROR_NO_DATA:
			{
				// Everything ok, we are just waiting for new data.
				bDisplayText = false;
				break;
			}
			default:
			{
				std::cout << "ps3eye::VIRTUAL() ReadFile failed, index " << m_index << ", GLE=" << GetLastError() << "." << std::endl;
				break;
			}
			}

			if (bDisplayText)
			{
				capFrameHD.setTo(cv::Scalar(0, 0, 0));
				cv::putText(
					capFrameHD,
					"Virtual tracker not connected.",
					cv::Point(256, capFrameHD.rows / 2),
					cv::FONT_HERSHEY_DUPLEX,
					3.0,
					CvScalar(255, 255, 255),
					5
				);

				m_frameAvailable = true;
				return true;
			}

			return true;
		}

		if (m_frameAvailable)
			return true;

		memcpy((uchar*)capFrameHD.data, pipeBufferHD, sizeof(pipeBufferHD));

		m_frameAvailable = true;
		return true;
	}

	bool retrieveFrame(int outputType, cv::OutputArray outArray)
	{
		if (capFrameSD.rows == m_frameHeight && capFrameSD.cols == m_frameWidth)
		{
			return retrieveFrameSD(outputType, outArray);
		}

		if (capFrameHD.rows == m_frameHeight && capFrameHD.cols == m_frameWidth)
		{
			return retrieveFrameHD(outputType, outArray);
		}

		return false;
	}

	bool retrieveFrameSD(int outputType, cv::OutputArray outArray)
	{
		if (!m_frameAvailable)
			return false;

		// ###Externet We need to flip the input image because PSEyes do so too.
		// If we dont do this the Y axis will be flipped on pose calibration.
		// EDIT:
		// Let the virtual tracker manager control the flip. Not all cameras have flipped horizontal streams.
		//cv::Mat flipped;
		//cv::flip(capFrameSD, flipped, 1);

		capFrameSD.copyTo(outArray);
		return true;
	}

	bool retrieveFrameHD(int outputType, cv::OutputArray outArray)
	{
		if (!m_frameAvailable)
			return false;

		// ###Externet We need to flip the input image because PSEyes do so too.
		// If we dont do this the Y axis will be flipped on pose calibration.
		// EDIT:
		// Let the virtual tracker manager control the flip. Not all cameras have flipped horizontal streams.
		//cv::Mat flipped;
		//cv::flip(capFrameSD, flipped, 1);

		capFrameHD.copyTo(outArray);
		return true;
	}

	int getCaptureDomain() {
		return CV_CAP_IMAGES;
	}

	bool isOpened() const
	{
		return (m_isValid && m_index != -1);
	}

	bool isConnected() const
	{
		return m_pipeConnected;
	}

	std::string getUniqueIndentifier() const
	{
		std::string identifier = "virtual_";

		char indexStr[20];
		identifier.append(itoa(m_index, indexStr, 10));

		return identifier;
	}

protected:

	bool open(int _index)
	{
		m_index = _index;
		capFrameSD = cv::Mat(480, 640, CV_8UC3, CvScalar(0, 0, 0));
		capFrameHD = cv::Mat(1080, 1920, CV_8UC3, CvScalar(0, 0, 0));

		std::cout << "ps3eye::VIRTUAL() index " << m_index << " open." << std::endl;

		if (refreshPipe())
		{
			// Try to connect to pipe and check for connection.
			grabFrame();
			return true;
		}
		else
		{
			m_index = -1;
			return false;
		}
	}

	void close()
	{
		std::cout << "ps3eye::VIRTUAL() index " << m_index << " closed." << std::endl;

		if (capturePipeSD != INVALID_HANDLE_VALUE)
		{
			DisconnectNamedPipe(capturePipeSD);
			CloseHandle(capturePipeSD);

			capturePipeSD = INVALID_HANDLE_VALUE;
		}

		if (capturePipeHD != INVALID_HANDLE_VALUE)
		{
			DisconnectNamedPipe(capturePipeHD);
			CloseHandle(capturePipeHD);

			capturePipeHD = INVALID_HANDLE_VALUE;
		}

		m_index = -1;
		m_isValid = false;
	}

	bool refreshPipe() 
	{
		if (capturePipeSD != INVALID_HANDLE_VALUE)
		{
			DisconnectNamedPipe(capturePipeSD);
			CloseHandle(capturePipeSD);

			capturePipeSD = INVALID_HANDLE_VALUE;
		}

		if (capturePipeHD != INVALID_HANDLE_VALUE)
		{
			DisconnectNamedPipe(capturePipeHD);
			CloseHandle(capturePipeHD);

			capturePipeHD = INVALID_HANDLE_VALUE;
		}

		std::string pipeNameSD = "\\\\.\\pipe\\PSMoveSerivceEx\\VirtPSeyeStream0_";
		std::string pipeNameHD = "\\\\.\\pipe\\PSMoveSerivceEx\\VirtPSeyeStream1_";

		char indexStr[20];
		pipeNameSD.append(itoa(m_index, indexStr, 10));
		pipeNameHD.append(itoa(m_index, indexStr, 10));

		// 480p
		if (capFrameSD.rows == m_frameHeight && capFrameSD.cols == m_frameWidth)
		{
			capturePipeSD = CreateNamedPipe(
				pipeNameSD.c_str(),
				PIPE_ACCESS_INBOUND,
				PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_NOWAIT,
				1,
				sizeof(pipeBufferSD),
				sizeof(pipeBufferSD),
				NMPWAIT_USE_DEFAULT_WAIT,
				NULL
			);

			if (capturePipeSD != INVALID_HANDLE_VALUE)
			{
				std::cout << "ps3eye::VIRTUAL() " << pipeNameSD.c_str() << " pipe created." << std::endl;
				return true;
			}
			else
			{
				std::cout << "ps3eye::VIRTUAL() " << pipeNameSD.c_str() << " pipe failed!, GLE=" << GetLastError() << std::endl;
				return false;
			}
		}

		// 1080p
		if (capFrameHD.rows == m_frameHeight && capFrameHD.cols == m_frameWidth)
		{
			capturePipeHD = CreateNamedPipe(
				pipeNameHD.c_str(),
				PIPE_ACCESS_INBOUND,
				PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_NOWAIT,
				1,
				sizeof(pipeBufferHD),
				sizeof(pipeBufferHD),
				NMPWAIT_USE_DEFAULT_WAIT,
				NULL
			);

			if (capturePipeHD != INVALID_HANDLE_VALUE)
			{
				std::cout << "ps3eye::VIRTUAL() " << pipeNameSD.c_str() << " pipe created." << std::endl;
				return true;
			}
			else
			{
				std::cout << "ps3eye::VIRTUAL() " << pipeNameSD.c_str() << " pipe failed!, GLE=" << GetLastError() << std::endl;
				return false;
			}
		}

		return false;
	}

	int m_index;
	bool m_frameAvailable;
	bool m_pipeConnected;
	bool m_isValid;
	int m_frameHeight;
	int m_frameWidth;

	HANDLE capturePipeSD;
	HANDLE capturePipeHD;
	char pipeBufferSD[VRIT_BUFF_SD_SIZE];
	char pipeBufferHD[VRIT_BUFF_HD_SIZE];
	cv::Mat capFrameSD;
	cv::Mat capFrameHD;
};

static bool usingCLEyeDriver()
{
    bool cleyedriver_found = false;
    // Check if CLEYE_DRIVER
#ifdef HAVE_CLEYE
    PlatformDeviceAPIWin32 platformAPI;
    char provider_name[128];

    if (platformAPI.get_device_property(
			DeviceClass::DeviceClass_Camera,
			VENDOR_ID,
			PRODUCT_ID,
			k_reg_property_provider_name,
			provider_name,
			sizeof(provider_name)))
    {
        if (strcmp(provider_name, CLEYE_DRIVER_PROVIDER_NAME) == 0)
        {
            cleyedriver_found = true;
        }
    }
#endif
    return cleyedriver_found;
}
#endif


/*
-- Definitions for PSEyeVideoCapture --
*/
bool PSEyeVideoCapture::open(int index)
{
	if (isOpened())
	{
		release();
	}

	if (m_iVideoCaptureType == eVideoCaptureType::CaptureType_HID || m_iVideoCaptureType == eVideoCaptureType::CaptureType_ALL)
	{
		// Try to open a PS3EYE-specific camera capture
		// Only works for CLEYE_MULTICAM and PS3EYEDRIVER
		icap = pseyeVideoCapture_create(index);
		if (!icap.empty() && icap->isOpened())
		{
			return true;
		}

		// Keep track of the camera index. Necessary for CLEyeDriver only.
		// non -1 m_index is used as a CL Eye Driver check elsewhere.
		if (usingCLEyeDriver())
		{
			m_api_index = index;
			std::cout << "CL Eye Driver being used with native DShow. Setting m_index to " << m_api_index << std::endl;

			if (!isOpened())
			{
				std::cout << "Attempting cv::VideoCapture::open(index) for CLEye DShow camera." << std::endl;

				return cv::VideoCapture::open(index);
			}
		}

		// PS3EYE-specific camera capture if available, else use base class open()

		//###HipsterSloth $TODO
		// Disabling the OpenCV camera open fallback.
		// We don't officially support anything but the PS3Eye camera at the moment
		// and it's currently confusing debugging other peoples camera issues with 
		// this code path in place (random web cams getting opened)
		//if (!isOpened())
		//{
		//    std::cout << "Attempting cv::VideoCapture::open(index)" << std::endl;
		//    return cv::VideoCapture::open(index);
		//}
	}

	if (m_iVideoCaptureType == eVideoCaptureType::CaptureType_VIRTUAL || m_iVideoCaptureType == eVideoCaptureType::CaptureType_ALL)
	{
#ifdef WIN32
		icap = cv::makePtr<PSEYECaptureCAM_VIRTUAL>(index);
		m_indentifier = icap.dynamicCast<PSEYECaptureCAM_VIRTUAL>()->getUniqueIndentifier();
		m_api_index = index;

		if (!icap.empty())
		{
			return true;
		}

		return false;
#else
		return isOpened();
#endif
	}

	return isOpened();
}

bool PSEyeVideoCapture::set(int propId, double value)
{
#ifdef HAVE_CLEYE
    if (m_index != -1)
    {
        bool param_set = false;
        int val;
        HKEY hKey;
        DWORD l = sizeof(DWORD);
        int err = RegOpenKeyExA(HKEY_CURRENT_USER, CL_DRIVER_REG_PATH, 0, KEY_ALL_ACCESS, &hKey);
        if (err != ERROR_SUCCESS) {
            printf("Error: %d Unable to open reg-key:  [HKCU]\\%s!\n", err, CL_DRIVER_REG_PATH);
            printf("CL-Eye Test must be run at least once for each Windows user.\n");
            return false;
        }

        switch (propId)
        {
        case CV_CAP_PROP_EXPOSURE:
            val = (value == 0);
            RegSetValueExA(hKey, "AutoAEC", 0, REG_DWORD, (CONST BYTE*)&val, l);
            val = (int)(value * 2) % 511;
            RegSetValueExA(hKey, "Exposure", 0, REG_DWORD, (CONST BYTE*)&val, l);
            param_set = true;
            break;
        case CV_CAP_PROP_GAIN:
            val = (value == 0);
            RegSetValueExA(hKey, "AutoAGC", 0, REG_DWORD, (CONST BYTE*)&val, l);
            val = (int)ceil(value * 79/256) % 79;
            RegSetValueExA(hKey, "Gain", 0, REG_DWORD, (CONST BYTE*)&val, l);
            param_set = true;
            break;
        default:
            param_set = cv::VideoCapture::set(propId, value);
        }

        // restart the camera capture
        if (param_set && icap) {
            std::cout << "Parameter changed via registry. Resetting capture device." << std::endl;
            cv::VideoCapture::open(m_index);
        }

        return param_set;
    }
#endif
    return cv::VideoCapture::set(propId, value);

}

double PSEyeVideoCapture::get(int propId) const
{
#ifdef HAVE_CLEYE
    if (m_index != -1)
    {
        HKEY hKey;
        DWORD l = sizeof(DWORD);
        DWORD resultA = 0;
        DWORD resultB = 0;

        int err = RegOpenKeyExA(HKEY_CURRENT_USER, CL_DRIVER_REG_PATH, 0, KEY_ALL_ACCESS, &hKey);
        if (err != ERROR_SUCCESS) {
            printf("Error: %d Unable to open reg-key:  [HKCU]\\%s!\n", err, CL_DRIVER_REG_PATH);
            printf("CL-Eye Test must be run at least once for each Windows user.\n");
            return 0;
        }

        switch (propId)
        {
        case CV_CAP_PROP_EXPOSURE:
            RegQueryValueExA(hKey, "AutoAEC", NULL, NULL, (LPBYTE)&resultA, &l);
            RegQueryValueExA(hKey, "Exposure", NULL, NULL, (LPBYTE)&resultB, &l);
            return (resultA == 1) ? 0 : ((double)(resultB))/2.0;
        case CV_CAP_PROP_GAIN:
            RegQueryValueExA(hKey, "AutoAGC", NULL, NULL, (LPBYTE)&resultA, &l);
            RegQueryValueExA(hKey, "Gain", NULL, NULL, (LPBYTE)&resultB, &l);
            return (resultA == 1) ? 0 : ((double)(resultB))*(256.0/79.0);
        default:
            return cv::VideoCapture::get(propId);
        }
    }
#endif
    return cv::VideoCapture::get(propId);
}

std::string PSEyeVideoCapture::getUniqueIndentifier() const
{
    return m_indentifier;
}

int PSEyeVideoCapture::getIndex() const
{
	return m_asign_index;
}

int PSEyeVideoCapture::getApiIndex() const
{
	return m_api_index;
}

bool PSEyeVideoCapture::isOpened() const
{
	return m_valid && cv::VideoCapture::isOpened();
}

cv::Ptr<cv::IVideoCapture> PSEyeVideoCapture::pseyeVideoCapture_create(int index)
{
    // https://github.com/Itseez/opencv/blob/09e6c82190b558e74e2e6a53df09844665443d6d/modules/videoio/src/cap.cpp#L432
    int  domains[] =
    {
#ifdef HAVE_CLEYE
        PSEYE_CAP_CLMULTI,
        PSEYE_CAP_CLEYE,
#endif
#ifdef HAVE_PS3EYE
        PSEYE_CAP_PS3EYE,
#endif
        -1, -1
    };
    
    // interpret preferred interface (0 = autodetect)
    int pref = (index / 100) * 100;
    if (pref)
    {
        domains[0]=pref;
        index %= 100;
        domains[1]=-1;
    }
    // try every possibly installed camera API
    for (int i = 0; domains[i] >= 0; i++)
    {
#if defined(HAVE_CLEYE) || defined(HAVE_PS3EYE) || (0)
        cv::Ptr<cv::IVideoCapture> capture;
        switch (domains[i])
        {
#ifdef HAVE_CLEYE
            case PSEYE_CAP_CLMULTI:
                {
                    capture = cv::makePtr<PSEYECaptureCAM_CLMULTI>(index);
                    m_indentifier = capture.dynamicCast<PSEYECaptureCAM_CLMULTI>()->getUniqueIndentifier();
					m_index = index;
                }
                break;

            case PSEYE_CAP_CLEYE:
                // We want to use the native OpenCV icap in this case.
                // So we will return empty capture here.
                if (usingCLEyeDriver())
                {
                    std::cout << "CL Eye Driver detected." << std::endl;
                    capture = cv::Ptr<cv::IVideoCapture>();

                    m_indentifier = "opencv_";
                    m_indentifier.append(std::to_string(index));
					m_index = index;

                    return capture;
                }
                break;
#endif
#ifdef HAVE_PS3EYE
            case PSEYE_CAP_PS3EYE:
                {
                    capture = cv::makePtr<PSEYECaptureCAM_PS3EYE>(index);
                    m_indentifier = capture.dynamicCast<PSEYECaptureCAM_PS3EYE>()->getUniqueIndentifier();
					m_api_index = index;
                }
                break;
#endif
        }
        if (capture && capture->isOpened())
        {
            return capture;
        }
#endif
    }
    
    // failed to open a camera
    return cv::Ptr<cv::IVideoCapture>();
}