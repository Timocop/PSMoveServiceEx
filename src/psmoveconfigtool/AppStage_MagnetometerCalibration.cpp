//-- inludes -----
#include "AppStage_MagnetometerCalibration.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "PSMoveClient_CAPI.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathGLM.h"
#include "MathUtility.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "AssetManager.h"

#include "SDL_keycode.h"

#include <imgui.h>

#include <algorithm>

//-- statics ----
const char *AppStage_MagnetometerCalibration::APP_STAGE_NAME= "MagnetometerCalibration";

//-- constants -----
static const int k_max_bounds_magnetometer_samples = 600;
static const double k_stabilize_wait_time_ms= 3000.f;
static const int k_max_identity_magnetometer_samples= 100;
static const int k_min_sample_distance= 50;
static const int k_min_sample_distance_sq= k_min_sample_distance*k_min_sample_distance;

enum eEllipseFitMethod
{
    _ellipse_fit_method_least_squares,
    _ellipse_fit_method_box,
};

//-- private methods -----
struct MagnetometerBoundsStatistics
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PSMVector3i magnetometerIntSamples[k_max_bounds_magnetometer_samples];
    Eigen::Vector3f magnetometerEigenSamples[k_max_bounds_magnetometer_samples];
    int sampleCount;
    int samplePercentage;
	int sampleQuality;
	bool sampleSkip;

    PSMVector3i minSampleExtent;
    PSMVector3i maxSampleExtent;

    EigenFitEllipsoid sampleFitEllipsoid;
    int ellipseFitMethod;

	MagnetometerBoundsStatistics()
		: sampleCount(0)
		, samplePercentage(0)
		, sampleQuality(0)
		, sampleSkip(false)
		, minSampleExtent()
		, maxSampleExtent()
		, ellipseFitMethod(_ellipse_fit_method_box)
	{
		clear();
	}

	void setCompleted()
	{
		sampleSkip = true;
	}

	bool getIsComplete() const 
	{
		return sampleCount >= k_max_bounds_magnetometer_samples || sampleSkip;
	}

	int getQuality() const
	{
		if (sampleCount < 3)
			return 0;

		const int size = 6;

		PSMVector3f forward[size];
		forward[0] = eigen_vector3f_to_psm_vector3f(Eigen::Vector3f::UnitX());
		forward[1] = eigen_vector3f_to_psm_vector3f(Eigen::Vector3f::UnitY());
		forward[2] = eigen_vector3f_to_psm_vector3f(Eigen::Vector3f::UnitZ());
		forward[3] = eigen_vector3f_to_psm_vector3f(-Eigen::Vector3f::UnitX());
		forward[4] = eigen_vector3f_to_psm_vector3f(-Eigen::Vector3f::UnitY());
		forward[5] = eigen_vector3f_to_psm_vector3f(-Eigen::Vector3f::UnitZ());

		float maxAng[size];
		for (int i = 0; i < size; ++i)
			maxAng[i] = 180.f;

		PSMVector3f boundsMax = PSM_Vector3iCastToFloat(&maxSampleExtent);
		PSMVector3f boundsMin = PSM_Vector3iCastToFloat(&minSampleExtent);
		PSMVector3f boundsCenter = PSM_Vector3fAdd(&boundsMax, &boundsMin);
		boundsCenter = PSM_Vector3fScale(&boundsCenter, 0.5f);

		for (int i = 0; i < sampleCount; ++i)
		{
			PSMVector3f sample = eigen_vector3f_to_psm_vector3f(magnetometerEigenSamples[i]);
			PSMVector3f vecCent = PSM_Vector3fSubtract(&sample, &boundsCenter);

			for (int j = 0; j < size; ++j)
			{
				float ang =
					acosf(PSM_Vector3fDot(&forward[j], &vecCent)
						/ (PSM_Vector3fLength(&forward[j])
							* PSM_Vector3fLength(&vecCent))) * k_radians_to_degreees;

				if (maxAng[j] > ang)
					maxAng[j] = ang;
			}
		}

		const float scale = 45.f;
		
		// Clamp to the worest angle sample
		float quality = 0.f;
		for (int i = 0; i < size; ++i)
			quality = fmaxf(quality, maxAng[i] / scale);

		const float quality_offset = 0.1f;

		quality = clampf(quality - quality_offset, 0.0f, 1.0f);

		return (int)((1.f - (quality)) * 100.f);
	}

	void clear()
	{
		sampleCount= 0;
		samplePercentage= 0;
		sampleQuality = 0;
		sampleSkip = false;

		minSampleExtent= *k_psm_int_vector3_zero;
		maxSampleExtent= *k_psm_int_vector3_zero;

		sampleFitEllipsoid.clear();
	}

	bool addSample(const PSMVector3i &sample)
	{
		bool bSuccess= sampleCount < k_max_bounds_magnetometer_samples;

		if (bSuccess)
		{
			// Grow the measurement extents bounding box
			expandMagnetometerBounds(sample);

			if (sampleCount > 0) 
			{
				// Make sure this sample isn't too close to another sample
				const PSMVector3i diff = PSM_Vector3iSubtract(&sample, &magnetometerIntSamples[sampleCount - 1]);
				const int distanceSquared = PSM_Vector3iLengthSquared(&diff);

				if (distanceSquared < k_min_sample_distance_sq)
				{
					bSuccess = false;
				}
			}
		}

		if (bSuccess)
		{
            // Store the new sample
            magnetometerIntSamples[sampleCount]= sample;
            magnetometerEigenSamples[sampleCount] = psm_vector3i_to_eigen_vector3(sample);
            ++sampleCount;

            // Compute a best fit ellipsoid for the sample points
            switch (ellipseFitMethod)
            {
            case _ellipse_fit_method_least_squares:
                eigen_alignment_fit_least_squares_axis_aligned_ellipsoid(
                    magnetometerEigenSamples, sampleCount, sampleFitEllipsoid);
                break;
            case _ellipse_fit_method_box:
                eigen_alignment_fit_bounding_box_ellipsoid(
                    magnetometerEigenSamples, sampleCount, sampleFitEllipsoid);
                break;
            }

            // Update the extents progress based on min extent size
            int minRange = computeMagnetometerCalibrationMinRange();
            if (minRange > 0)
            {
                samplePercentage = 
					std::min((100 * sampleCount) / k_max_bounds_magnetometer_samples, 100);

				sampleQuality = getQuality();
            }
		}

		return bSuccess;
	}

private:
	void expandMagnetometerBounds(const PSMVector3i &sample)
	{
        if (sampleCount > 1)
        {
		    minSampleExtent= PSM_Vector3iMin(&minSampleExtent, &sample);
		    maxSampleExtent= PSM_Vector3iMax(&maxSampleExtent, &sample);
        }
        else
        {
		    minSampleExtent= sample;
		    maxSampleExtent= sample;
        }
	}

	int computeMagnetometerCalibrationMinRange()
	{
		PSMVector3i extents= PSM_Vector3iSubtract(&maxSampleExtent, &minSampleExtent);

		return PSM_Vector3iMinValue(&extents);
	}

	int computeMagnetometerCalibrationMaxRange()
	{
		PSMVector3i extents= PSM_Vector3iSubtract(&maxSampleExtent, &minSampleExtent);

		return PSM_Vector3iMaxValue(&extents);
	}
};

struct MagnetometerIdentityStatistics
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Samples of the magnetometer in the space
    Eigen::Vector3f magnetometerEllipsoidSamples[k_max_identity_magnetometer_samples];
    int sampleCount;

	Eigen::Vector3f magnetometerIdentity;
	float magnetometerVariance;

	MagnetometerIdentityStatistics()
	{
		clear();
	}

	bool getIsComplete() const 
	{
		return sampleCount >= k_max_identity_magnetometer_samples;
	}

	void clear()
	{
		sampleCount= 0;
		magnetometerIdentity= Eigen::Vector3f::Zero();
		magnetometerVariance= 0.f;
	}

	void addSample(const Eigen::Vector3f &ellipsoid_sample)
	{
		if (sampleCount < k_max_identity_magnetometer_samples)
		{
            magnetometerEllipsoidSamples[sampleCount] = ellipsoid_sample;
			++sampleCount;

			// If we just added the last sample, compute the statistics
			if (sampleCount >= k_max_identity_magnetometer_samples)
			{
				Eigen::Vector3f meanVector;
				Eigen::Vector3f varianceVector;

				// Compute the mean and variance samples
				eigen_vector3f_compute_mean_and_variance(
					magnetometerEllipsoidSamples,
					k_max_identity_magnetometer_samples,
					&magnetometerIdentity,
					&varianceVector);
				
				// The mean vector should be close to the unit length
				// but normalize it just in case 
				eigen_vector3f_normalize_with_default(magnetometerIdentity, Eigen::Vector3f(0.f, 1.f, 0.f));

				// Only the the max component of the variance
				magnetometerVariance= varianceVector.maxCoeff();
			}
		}
	}
};

static void write_calibration_parameter(const Eigen::Vector3f &in_vector, PSMoveProtocol::FloatVector *out_vector);

//-- public methods -----
AppStage_MagnetometerCalibration::AppStage_MagnetometerCalibration(App *app) 
    : AppStage(app)
    , m_bBypassCalibration(false)
    , m_menuState(AppStage_MagnetometerCalibration::inactive)
	, m_playspaceYawOffset(0.f)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
    , m_lastRawMagnetometer()
    , m_lastCalibratedAccelerometer()
    , m_boundsStatistics(new MagnetometerBoundsStatistics)
	, m_identityStatistics(new MagnetometerIdentityStatistics)
    , m_stableStartTime()
    , m_bIsStable(false)
	, m_bForceControllerStable(false)
    , m_identityPoseMVectorSum()
    , m_identityPoseSampleCount(0)
{ 
}

AppStage_MagnetometerCalibration::~AppStage_MagnetometerCalibration()
{
	if (m_boundsStatistics != nullptr)
	{
		delete m_boundsStatistics;
		m_boundsStatistics = nullptr;
	}

	if (m_identityStatistics != nullptr)
	{
		delete m_identityStatistics;
		m_identityStatistics = nullptr;
	}
}

void AppStage_MagnetometerCalibration::enter()
{
    const AppStage_ControllerSettings *controllerSettings= 
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo=
        controllerSettings->getSelectedControllerInfo();
	
    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
	m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);

    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
	PSM_AllocateControllerListener(controllerInfo->ControllerID);
	m_controllerView= PSM_GetController(controllerInfo->ControllerID);
	

    m_lastRawMagnetometer= *k_psm_int_vector3_zero;
    m_lastCalibratedAccelerometer= *k_psm_float_vector3_zero;

	m_boundsStatistics->clear();
	m_identityStatistics->clear();

	m_stableStartTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
    m_bIsStable= false;
	m_bForceControllerStable= false;

    assert(!m_isControllerStreamActive);
    m_lastControllerSeqNum= -1;

	PSMRequestID request_id;
	PSM_StartControllerDataStreamAsync(m_controllerView->ControllerID, PSMStreamFlags_includeRawSensorData | PSMStreamFlags_includeCalibratedSensorData, &request_id);
	PSM_RegisterCallback(request_id, &AppStage_MagnetometerCalibration::handle_acquire_controller, this);

	request_playspace_info();
}

void AppStage_MagnetometerCalibration::exit()
{
    assert(m_controllerView != nullptr);
    PSM_FreeControllerListener(m_controllerView->ControllerID);
    m_controllerView= nullptr;
    m_menuState= eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_MagnetometerCalibration::update()
{
    bool bControllerDataUpdatedThisFrame= false;

    if (m_isControllerStreamActive && m_controllerView->OutputSequenceNum != m_lastControllerSeqNum)
    {
        const PSMPSMoveRawSensorData &rawSensorData= m_controllerView->ControllerState.PSMoveState.RawSensorData;
        const PSMPSMoveCalibratedSensorData &calibratedSensorData = m_controllerView->ControllerState.PSMoveState.CalibratedSensorData;

        m_lastRawMagnetometer = rawSensorData.Magnetometer;
        m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
        m_lastControllerSeqNum = m_controllerView->OutputSequenceNum;
        bControllerDataUpdatedThisFrame= true;
    }

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingPlayspaceRequest:
		{
		} break;
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_controllerView->ControllerState.PSMoveState.bHasValidHardwareCalibration)
                {
					m_controllerView->ControllerState.PSMoveState.bPoseResetButtonEnabled= true;

					m_boundsStatistics->clear();
                    
                    if (m_bBypassCalibration)
                    {
                        m_app->getOrbitCamera()->resetOrientation();
						m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);
                        m_menuState= AppStage_MagnetometerCalibration::complete;
                    }
                    else
                    {
                        m_menuState= AppStage_MagnetometerCalibration::measureBExtents;
                    }
                }
                else
                {
                    m_menuState= AppStage_MagnetometerCalibration::failedBadCalibration;
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::failedBadCalibration:
        {
        } break;
    case eCalibrationMenuState::measureBExtents:
        {
            if (bControllerDataUpdatedThisFrame && !m_boundsStatistics->getIsComplete())
            {
				if (m_boundsStatistics->addSample(m_lastRawMagnetometer))
				{
					if (m_boundsStatistics->samplePercentage >= 100)
					{
						PSM_SetControllerLEDOverrideColor(m_controllerView->ControllerID, 0, 255, 0);
					}
					else if (m_boundsStatistics->samplePercentage >= 50)
					{
						PSM_SetControllerLEDOverrideColor(m_controllerView->ControllerID, 255, 255, 0);
					}
					else
					{
						PSM_SetControllerLEDOverrideColor(m_controllerView->ControllerID, 255, 0, 0);
					}
                }
            }
        } break;
    case eCalibrationMenuState::waitForGravityAlignment:
        {
			bool bIsStable= false;
			bool bCanBeStable= PSM_GetIsControllerStable(m_controllerView->ControllerID, &bIsStable) == PSMResult_Success;

            if ((bCanBeStable && bIsStable) || m_bForceControllerStable)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

                if (m_bIsStable || m_bForceControllerStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
    
                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
                        m_identityStatistics->clear();
                        m_menuState= AppStage_MagnetometerCalibration::measureBDirection;
                    }
                }
                else
                {
                    m_bIsStable= true;
                    m_stableStartTime= now;
                }
            }
            else
            {
                if (m_bIsStable)
                {
                    m_bIsStable= false;
                }
            }
        } break;
    case eCalibrationMenuState::measureBDirection:
        {
			bool bIsStable= false;
			bool bCanBeStable= PSM_GetIsControllerStable(m_controllerView->ControllerID, &bIsStable) == PSMResult_Success;

            if ((bCanBeStable && bIsStable) || m_bForceControllerStable)
            {
				// Add sample when there is new data
                if (bControllerDataUpdatedThisFrame)
                {
                    // Project the magnetometer sample into the space of the ellipsoid
                    Eigen::Vector3f ellipsoid_sample =
                        eigen_alignment_project_point_on_ellipsoid_basis(
                            psm_vector3f_to_eigen_vector3(PSM_Vector3iCastToFloat(&m_lastRawMagnetometer)),
                            m_boundsStatistics->sampleFitEllipsoid);                   

					// Add the normalized sample to the 
					m_identityStatistics->addSample(ellipsoid_sample);

                    if (m_identityStatistics->getIsComplete())
                    {
						const EigenFitEllipsoid &ellipsoid= m_boundsStatistics->sampleFitEllipsoid;
						const Eigen::Vector3f &magnetometerIdentity= m_identityStatistics->magnetometerIdentity;
						const float magnetometerVariance= m_identityStatistics->magnetometerVariance;

                        // Tell the psmove service about the new magnetometer settings
                        RequestPtr request(new PSMoveProtocol::Request());
                        request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_MAGNETOMETER_CALIBRATION);

                        PSMoveProtocol::Request_RequestSetControllerMagnetometerCalibration *calibration=
                                request->mutable_set_controller_magnetometer_calibration_request();

                        calibration->set_controller_id(m_controllerView->ControllerID);

                        write_calibration_parameter(ellipsoid.center, calibration->mutable_ellipse_center());
                        write_calibration_parameter(ellipsoid.extents, calibration->mutable_ellipse_extents());
                        write_calibration_parameter(ellipsoid.basis.col(0), calibration->mutable_ellipse_basis_x());
                        write_calibration_parameter(ellipsoid.basis.col(1), calibration->mutable_ellipse_basis_y());
                        write_calibration_parameter(ellipsoid.basis.col(2), calibration->mutable_ellipse_basis_z());
                        calibration->set_ellipse_fit_error(ellipsoid.error);
						calibration->set_magnetometer_variance(magnetometerVariance);

                        write_calibration_parameter(magnetometerIdentity, calibration->mutable_magnetometer_identity());

						PSMRequestID request_id;
						PSM_SendOpaqueRequest(&request, &request_id);
						PSM_RegisterCallback(request_id, AppStage_MagnetometerCalibration::handle_set_magnetometer_calibration, this);

                        // Wait for the response
                        m_menuState= AppStage_MagnetometerCalibration::waitForSetCalibrationResponse;
                    }
                }
            }
            else
            {
                m_bIsStable= false;
                m_menuState= AppStage_MagnetometerCalibration::waitForGravityAlignment;
            }
        } break;
	case eCalibrationMenuState::skipBDirection:
	{
		const EigenFitEllipsoid &ellipsoid = m_boundsStatistics->sampleFitEllipsoid;

		// Tell the psmove service about the new magnetometer settings
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_MAGNETOMETER_CALIBRATION_BASIC);

		PSMoveProtocol::Request_RequestSetControllerMagnetometerCalibrationBasic *calibration =
			request->mutable_set_controller_magnetometer_calibration_basic_request();

		calibration->set_controller_id(m_controllerView->ControllerID);

		write_calibration_parameter(ellipsoid.center, calibration->mutable_ellipse_center());
		write_calibration_parameter(ellipsoid.extents, calibration->mutable_ellipse_extents());
		write_calibration_parameter(ellipsoid.basis.col(0), calibration->mutable_ellipse_basis_x());
		write_calibration_parameter(ellipsoid.basis.col(1), calibration->mutable_ellipse_basis_y());
		write_calibration_parameter(ellipsoid.basis.col(2), calibration->mutable_ellipse_basis_z());
		calibration->set_ellipse_fit_error(ellipsoid.error);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_MagnetometerCalibration::handle_set_magnetometer_calibration, this);

		// Wait for the response
		m_menuState = AppStage_MagnetometerCalibration::waitForSetCalibrationResponse;
	} break;
	case eCalibrationMenuState::waitForSetCalibrationResponse:
	{
	} break;
    case eCalibrationMenuState::failedSetCalibration:
        {
        } break;
    case eCalibrationMenuState::complete:
        {
        } break;
    case eCalibrationMenuState::pendingExit:
        {
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_MagnetometerCalibration::render()
{
    const float k_modelScale = 18.f;
    glm::mat4 scaleAndRotateModelX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(k_modelScale, k_modelScale, k_modelScale)),
            90.f, glm::vec3(1.f, 0.f, 0.f));
    
	const EigenFitEllipsoid &sampleFitEllipsoid= m_boundsStatistics->sampleFitEllipsoid;	
    const PSMVector3i &minSampleExtent= m_boundsStatistics->minSampleExtent;
    const PSMVector3i &maxSampleExtent= m_boundsStatistics->maxSampleExtent;

	PSMVector3i rawSampleDiff= PSM_Vector3iSubtract(&maxSampleExtent, &minSampleExtent);
    PSMVector3i rawSampleExtents = PSM_Vector3iUnsafeScalarDivide(&rawSampleDiff, 2);

    glm::vec3 boxMin = psm_vector3f_to_glm_vec3(PSM_Vector3iCastToFloat(&minSampleExtent));
    glm::vec3 boxMax = psm_vector3f_to_glm_vec3(PSM_Vector3iCastToFloat(&maxSampleExtent));
    glm::vec3 boxCenter = (boxMax + boxMin) * 0.5f;
    glm::vec3 boxExtents = (boxMax - boxMin) * 0.5f;

    glm::mat4 recenterMatrix = 
        glm::translate(glm::mat4(1.f), -eigen_vector3f_to_glm_vec3(sampleFitEllipsoid.center));

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingPlayspaceRequest:
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
        } break;
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::failedBadCalibration:
        {
        } break;
    case eCalibrationMenuState::measureBExtents:
        {
            // Draw the psmove model in the middle
            drawPSMoveModel(scaleAndRotateModelX90, glm::vec3(1.f, 1.f, 1.f));

            // Draw the sample point cloud around the origin
            drawPointCloud(
                recenterMatrix,
                glm::vec3(1.f, 1.f, 1.f),
                reinterpret_cast<float *>(&m_boundsStatistics->magnetometerEigenSamples[0]),
                m_boundsStatistics->sampleCount);

            // Draw the sample bounding box
            // Label the min and max corners with the min and max magnetometer readings
            drawTransformedBox(recenterMatrix, boxMin, boxMax, glm::vec3(1.f, 1.f, 1.f));
            drawTextAtWorldPosition(recenterMatrix, boxMin, "%d,%d,%d",
                                    minSampleExtent.x, minSampleExtent.y, minSampleExtent.z);
            drawTextAtWorldPosition(recenterMatrix, boxMax, "%d,%d,%d",
                                    maxSampleExtent.x, maxSampleExtent.y, maxSampleExtent.z);

            // Draw and label the extent axes
            drawTransformedAxes(glm::mat4(1.f), boxExtents.x, boxExtents.y, boxExtents.z);
            drawTextAtWorldPosition(glm::mat4(1.f), glm::vec3(boxExtents.x, 0.f, 0.f), "%d", rawSampleExtents.x);
            drawTextAtWorldPosition(glm::mat4(1.f), glm::vec3(0.f, boxExtents.y, 0.f), "%d", rawSampleExtents.y);
            drawTextAtWorldPosition(glm::mat4(1.f), glm::vec3(0.f, 0.f, boxExtents.z), "%d", rawSampleExtents.z);

            // Draw the best fit ellipsoid
            {
                glm::mat3 basis = eigen_matrix3f_to_glm_mat3(sampleFitEllipsoid.basis);
                glm::vec3 center = eigen_vector3f_to_glm_vec3(sampleFitEllipsoid.center);
                glm::vec3 extents = eigen_vector3f_to_glm_vec3(sampleFitEllipsoid.extents);

                drawEllipsoid(
                    recenterMatrix,
                    glm::vec3(0.f, 0.4f, 1.f),
                    basis, center, extents);
                drawTextAtWorldPosition(
                    recenterMatrix,
                    center - basis[0]*extents.x,
                    "E:%.1f", sampleFitEllipsoid.error);
            }

            // Draw the current magnetometer direction
            {
                glm::vec3 m_start= boxCenter;
                glm::vec3 m_end= psm_vector3f_to_glm_vec3(PSM_Vector3iCastToFloat(&m_lastRawMagnetometer));

                drawArrow(recenterMatrix, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(recenterMatrix, m_end, "M");
            }
        } break;
    case eCalibrationMenuState::waitForGravityAlignment:
        {
            drawPSMoveModel(scaleAndRotateModelX90, glm::vec3(1.f, 1.f, 1.f));

            // Draw the current direction of gravity
            {
                const float renderScale = 200.f;
                glm::mat4 renderScaleMatrix = 
                    glm::scale(glm::mat4(1.f), glm::vec3(renderScale, renderScale, renderScale));
                glm::vec3 g= psm_vector3f_to_glm_vec3(m_lastCalibratedAccelerometer);

                drawArrow(
                    renderScaleMatrix,
                    glm::vec3(), g, 
                    0.1f, 
                    glm::vec3(0.f, 1.f, 0.f));
                drawTextAtWorldPosition(renderScaleMatrix, g, "G");
            }
        } break;
    case eCalibrationMenuState::measureBDirection:
        {
            drawPSMoveModel(scaleAndRotateModelX90, glm::vec3(1.f, 1.f, 1.f));

            // Draw the current magnetometer direction
            {
                glm::vec3 m_start = boxCenter;
                glm::vec3 m_end = psm_vector3f_to_glm_vec3(PSM_Vector3iCastToFloat(&m_lastRawMagnetometer));

                drawArrow(recenterMatrix, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(recenterMatrix, m_end, "M");
            }

        } break;
	case eCalibrationMenuState::skipBDirection:
		{
		} break;
    case eCalibrationMenuState::waitForSetCalibrationResponse:
        {
        } break;
    case eCalibrationMenuState::failedSetCalibration:
        {
        } break;
    case eCalibrationMenuState::complete:
        {
            // Get the orientation of the controller in world space (OpenGL Coordinate System)            
			PSMQuatf orientation;
			
			if (PSM_GetControllerOrientation(m_controllerView->ControllerID, &orientation) == PSMResult_Success)
			{
				// Compensate for playspace offsets
				const Eigen::Quaternionf offset_yaw = eigen_quaternion_angle_axis(-m_playspaceYawOffset * k_degrees_to_radians, Eigen::Vector3f::UnitY());
				const Eigen::Quaternionf eigen_quat = offset_yaw.inverse() * psm_quatf_to_eigen_quaternionf(orientation);
				orientation = PSM_QuatfCreate(eigen_quat.w(), eigen_quat.x(), eigen_quat.y(), eigen_quat.z());

				glm::quat q= psm_quatf_to_glm_quat(orientation);
				glm::mat4 worldSpaceOrientation= glm::mat4_cast(q);
				glm::mat4 worldTransform = glm::scale(worldSpaceOrientation, glm::vec3(k_modelScale, k_modelScale, k_modelScale));

				drawTransformedAxes(glm::scale(glm::mat4(1.f), glm::vec3(k_modelScale, k_modelScale, k_modelScale)), 20.f);

				drawPSMoveModel(worldTransform, glm::vec3(1.f, 1.f, 1.f));
				drawTransformedAxes(worldTransform, 20.f);
			}
        } break;
    case eCalibrationMenuState::pendingExit:
        {
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_MagnetometerCalibration::renderUI()
{
	static float waitCount;

    const float k_panel_width= 500;
    const char *k_window_title= "Magnetometer Calibration";
    const ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingPlayspaceRequest:
	{
		ImGui::SetNextWindowPosCenter();
		ImGui::Begin(k_window_title, nullptr, window_flags);

		waitCount += 0.025f;
		switch ((int)floorf(waitCount))
		{
		case 0:
			ImGui::Image(AssetManager::getInstance()->getIconUpdate()->getImTextureId(), ImVec2(32, 32));
			break;
		case 1:
			ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
			break;
		default:
			ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
			waitCount = 0;
			break;
		}

		ImGui::SameLine();
		ImGui::Text("Waiting for server response...");

		ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
		ImGui::End();
	} break;
	case eCalibrationMenuState::waitingForStreamStartResponse:
	{
		ImGui::SetNextWindowPosCenter();
		ImGui::Begin(k_window_title, nullptr, window_flags);

		waitCount += 0.025f;
		switch ((int)floorf(waitCount))
		{
		case 0:
			ImGui::Image(AssetManager::getInstance()->getIconUpdate()->getImTextureId(), ImVec2(32, 32));
			break;
		case 1:
			ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
			break;
		default:
			ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
			waitCount = 0;
			break;
		}

		ImGui::SameLine();
		ImGui::Text("Waiting for controller stream to start...");

		ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
		ImGui::End();
	} break;
    case eCalibrationMenuState::failedStreamStart:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::Text("Failed to start controller stream!");

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }
            
            ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconLeft(), "Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }
			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::failedBadCalibration:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::TextWrapped(
                "Controller hardware calibration error detected! Please try unpairing and repairing the controller.");

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconLeft(), "Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::measureBExtents:
        {
            {
                ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
                ImGui::Begin(k_window_title, nullptr, window_flags);

                if (!m_boundsStatistics->getIsComplete())
                {
					waitCount += 0.025f;
					switch ((int)floorf(waitCount))
					{
					case 0:
						ImGui::Image(AssetManager::getInstance()->getIconUpdate()->getImTextureId(), ImVec2(32, 32));
						break;
					case 1:
						ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
						break;
					default:
						ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
						waitCount = 0;
						break;
					}

					ImGui::SameLine();
                    ImGui::TextWrapped(
                        "Rotate the controller in all directions to calibrate the magnetometer.", m_controllerView->ControllerID);
                }
                else
                {
					ImGui::Image(AssetManager::getInstance()->getIconCheck()->getImTextureId(), ImVec2(32, 32));
					ImGui::SameLine();
                    ImGui::TextWrapped(
                        "Magnetometer sampling completed!", m_controllerView->ControllerID);
                }

				if (!m_boundsStatistics->getIsComplete())
                {
					ImGui::Separator();
					ImGui::Text("Progress");
					ImGui::SameLine(100.f);
					ImGui::ProgressBar(static_cast<float>(m_boundsStatistics->samplePercentage) / 100.f, ImVec2(-1, 0));
					ImGui::Text("Quality");
					ImGui::SameLine(100.f);
					ImGui::ProgressBar(static_cast<float>(m_boundsStatistics->sampleQuality) / 100.f, ImVec2(-1, 0), " ");

					ImGui::Separator();
                }
                else
                {
					ImGui::Separator();

					ImGui::PushTextWrapPos();
					ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), AssetManager::k_imcolor_orange());
					ImGui::SameLine();
					ImGui::TextColored(AssetManager::k_imcolor_orange(),
						"The default orientation must be calibrated at least once for the magnetometer to function. If you have already done this, click 'Skip'."
					);
					ImGui::PopTextWrapPos();

					ImGui::Spacing();
					ImGui::Separator();
					ImGui::Spacing();

					if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconTarget(), "Calibrate Default Orientation"))
					{
						PSM_SetControllerLEDOverrideColor(m_controllerView->ControllerID, 0, 0, 0);
						m_menuState = waitForGravityAlignment;
					}
					
					ImGui::SameLine();
					if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconRight(), "Skip"))
					{
						PSM_SetControllerLEDOverrideColor(m_controllerView->ControllerID, 0, 0, 0);
						m_menuState = skipBDirection;
					}

					ImGui::SameLine();
                }

                if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconBan(), "Cancel"))
                {
                    request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
                }

				ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
                ImGui::End();
            }

			// $TODO: least squares fit method does not work with non-uniform samples. Use box fit.
            /*{
                ImGui::SetNextWindowPos(ImVec2(10.f, 450.f));
                ImGui::Begin("Ellipse Fitting Mode", nullptr, window_flags);

                if (ImGui::RadioButton("Least Squares Fit", &m_boundsStatistics->ellipseFitMethod, _ellipse_fit_method_least_squares))
                {
                    // Re-fit using min bounds
                    eigen_alignment_fit_least_squares_axis_aligned_ellipsoid(
                        m_boundsStatistics->magnetometerEigenSamples, 
						m_boundsStatistics->sampleCount, 
						m_boundsStatistics->sampleFitEllipsoid);
                }

                if (ImGui::RadioButton("Bounds Fit", &m_boundsStatistics->ellipseFitMethod, _ellipse_fit_method_box))
                {
                    // Refit to a box
                    eigen_alignment_fit_bounding_box_ellipsoid(
                        m_boundsStatistics->magnetometerEigenSamples, 
						m_boundsStatistics->sampleCount, 
						m_boundsStatistics->sampleFitEllipsoid);
                }

				ImGui::SetWindowSize(ImVec2(170, 0));
                ImGui::End();
            }*/
        } break;
    case eCalibrationMenuState::waitForGravityAlignment:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x/2.f - k_panel_width/2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::TextWrapped(
                "Place the controller on a level surface.\n"
				"This will set the default orientation of the controller.\n"
				"Measurement will begin once the controller is aligned with gravity and remains stable.");

            if (m_bIsStable || m_bForceControllerStable)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
                float fraction = static_cast<float>(stableDuration.count() / k_stabilize_wait_time_ms);

				ImGui::Separator();
                ImGui::ProgressBar(fraction, ImVec2(-1, 0), " ");
            }
            else
            {
				ImGui::Separator();
				ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), AssetManager::k_imcolor_orange());
				ImGui::SameLine();
				ImGui::TextColored(AssetManager::k_imcolor_orange(), "Controller destabilized! Waiting for stabilization...");
            }

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "Force Continue"))
            {
                m_bForceControllerStable= true;
            }
            
			ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconBan(), "Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::measureBDirection:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x/2.f - k_panel_width/2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			waitCount += 0.025f;
			switch ((int)floorf(waitCount))
			{
			case 0:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate()->getImTextureId(), ImVec2(32, 32));
				break;
			case 1:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				break;
			default:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				waitCount = 0;
				break;
			}

			ImGui::SameLine();
            ImGui::TextWrapped(
				"Place the controller on a level surface.\n"
				"This will set the default orientation of the controller.\n"
				"Measurement will begin once the controller is aligned with gravity and remains stable.");

			ImGui::Separator();
            ImGui::ProgressBar(static_cast<float>(m_identityStatistics->sampleCount) / static_cast<float>(k_max_identity_magnetometer_samples), ImVec2(-1, 0));

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconBan(), "Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
	case eCalibrationMenuState::skipBDirection:
		{
		} break;
	case eCalibrationMenuState::waitForSetCalibrationResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			waitCount += 0.025f;
			switch ((int)floorf(waitCount))
			{
			case 0:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate()->getImTextureId(), ImVec2(32, 32));
				break;
			case 1:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				break;
			default:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				waitCount = 0;
				break;
			}

			ImGui::SameLine();
            ImGui::Text("Sending final calibration to server...");

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::failedSetCalibration:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::Text("Failed to set calibration!");

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconLeft(), "Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::complete:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::Begin("Test Orientation", nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Calibration of Controller ID #%d", m_controllerView->ControllerID);
            }
            else
            {
				ImGui::Image(AssetManager::getInstance()->getIconCheck()->getImTextureId(), ImVec2(32, 32));
				ImGui::SameLine();
                ImGui::Text("Calibration of Controller ID #%d complete!", m_controllerView->ControllerID);
            }

			PSMQuatf orientation;			
			if (PSM_GetControllerOrientation(m_controllerView->ControllerID, &orientation) == PSMResult_Success)
			{
				// Compensate for playspace offsets
				const Eigen::Quaternionf offset_yaw = eigen_quaternion_angle_axis(-m_playspaceYawOffset * k_degrees_to_radians, Eigen::Vector3f::UnitY());
				const Eigen::Quaternionf eigen_quat = offset_yaw.inverse() * psm_quatf_to_eigen_quaternionf(orientation);
				const Eigen::EulerAnglesf euler_angles = eigen_quaternionf_to_euler_angles(eigen_quat);
				orientation = PSM_QuatfCreate(eigen_quat.w(), eigen_quat.x(), eigen_quat.y(), eigen_quat.z());

				ImGui::Text("Attitude: %.2f, Heading: %.2f, Bank: %.2f",
					euler_angles.get_attitude_degrees(), euler_angles.get_heading_degrees(), euler_angles.get_bank_degrees());
			}
			
			ImGui::Separator();
			ImGui::PushTextWrapPos();
			ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(32, 32), ImVec2(0,0), ImVec2(1,1), AssetManager::k_imcolor_blue());
			ImGui::SameLine();
			ImGui::TextColored(AssetManager::k_imcolor_blue(),
				"Hold the SELECT button with controller pointed forward to recenter the controller");
			ImGui::PopTextWrapPos();

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconLeft(), "Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::pendingExit:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			waitCount += 0.025f;
			switch ((int)floorf(waitCount))
			{
			case 0:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate()->getImTextureId(), ImVec2(32, 32));
				break;
			case 1:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				break;
			default:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				waitCount = 0;
				break;
			}

			ImGui::SameLine();
            ImGui::Text("Waiting for controller stream to stop...");

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
void AppStage_MagnetometerCalibration::request_playspace_info()
{
	if (m_menuState != AppStage_MagnetometerCalibration::pendingPlayspaceRequest)
	{
		m_menuState = AppStage_MagnetometerCalibration::pendingPlayspaceRequest;

		// Tell the psmove service that we we want a list of HMDs connected to this machine
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_GET_PLAYSPACE_OFFSETS);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_MagnetometerCalibration::handle_playspace_info_response, this);
	}
}

void AppStage_MagnetometerCalibration::handle_playspace_info_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_MagnetometerCalibration *thisPtr = static_cast<AppStage_MagnetometerCalibration *>(userdata);

	const PSMResult ResultCode = response_message->result_code;
	const PSMResponseHandle response_handle = response_message->opaque_response_handle;

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

		thisPtr->m_playspaceYawOffset = response->result_get_playspace_offsets().playspace_orientation_yaw();

		thisPtr->m_menuState = AppStage_MagnetometerCalibration::waitingForStreamStartResponse;
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_MagnetometerCalibration::failedStreamStart;
	} break;
	}
}

void AppStage_MagnetometerCalibration::handle_acquire_controller(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_MagnetometerCalibration *thisPtr= reinterpret_cast<AppStage_MagnetometerCalibration *>(userdata);

    if (response->result_code == PSMResult_Success)
    {
        thisPtr->m_isControllerStreamActive= true;
        thisPtr->m_lastControllerSeqNum= -1;
        // Wait for the first controller packet to show up...
    }
    else
    {
        thisPtr->m_menuState= AppStage_MagnetometerCalibration::failedStreamStart;
    }
}

void AppStage_MagnetometerCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
    if (m_isControllerStreamActive)
    {
		PSM_SetControllerLEDOverrideColor(m_controllerView->ControllerID, 0, 0, 0);

		PSMRequestID requestId;
		PSM_StopControllerDataStreamAsync(m_controllerView->ControllerID, &requestId);
		PSM_EatResponse(requestId);

		m_isControllerStreamActive = false;
		m_app->setAppStage(app_stage_name);
    }
    else
    {
        m_app->setAppStage(app_stage_name);
    }
}

void AppStage_MagnetometerCalibration::handle_set_magnetometer_calibration(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_MagnetometerCalibration *thisPtr= reinterpret_cast<AppStage_MagnetometerCalibration *>(userdata);

    if (response->result_code == PSMResult_Success)
    {
        thisPtr->m_app->getOrbitCamera()->resetOrientation();
		thisPtr->m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);
        thisPtr->m_menuState= AppStage_MagnetometerCalibration::complete;
    }
    else
    {
        thisPtr->m_menuState= AppStage_MagnetometerCalibration::failedSetCalibration;
    }
}

//-- private methods -----
static void
write_calibration_parameter(
    const Eigen::Vector3f &in_vector,
    PSMoveProtocol::FloatVector *out_vector)
{
    out_vector->set_i(in_vector.x());
    out_vector->set_j(in_vector.y());
    out_vector->set_k(in_vector.z());
}
