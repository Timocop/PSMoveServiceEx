//-- inludes -----
#include "AppStage_PairController.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "AssetManager.h"

#include "SDL_keycode.h"

#include <imgui.h>
#include <sstream>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_PairController::APP_STAGE_NAME= "PairController";

//-- constants -----

//-- public methods -----
AppStage_PairController::AppStage_PairController(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_PairController::inactive)
    , m_pendingBluetoothOpControllerIndex(-1)
    , m_pair_steps_completed(0)
    , m_pair_steps_total(0)
	, m_controllerID(-1)
	, m_controllerType(PSMControllerType::PSMController_None)
{ }

void AppStage_PairController::enter()
{
    m_app->setCameraType(_cameraFixed);
	m_app->getFixedCamera()->resetOrientation();

    // These should have been set already before we entered
    assert(m_menuState != AppStage_PairController::inactive);
    assert(m_pendingBluetoothOpControllerIndex != -1);
}

void AppStage_PairController::exit()
{
    m_menuState= AppStage_PairController::inactive;
    m_pendingBluetoothOpControllerIndex= -1;
}

void AppStage_PairController::update()
{
}
    
void AppStage_PairController::renderUI()
{
	static float waitCount;

    const char *k_window_title= "Controller Pairing";
    const ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eControllerMenuState::pendingControllerUnpairRequest:
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
            ImGui::Text("Waiting for controller to unpair...");

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconBan(), "Cancel"))
            {
                assert(m_pendingBluetoothOpControllerIndex != -1);
                request_cancel_bluetooth_operation(m_pendingBluetoothOpControllerIndex);
            }

			ImGui::SetWindowSize(ImVec2(300, 0));
            ImGui::End();
        } break;
    case eControllerMenuState::failedControllerUnpairRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::Text("Failed to unpair controller!");

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

			ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconLeft(), "Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(300, 0));
            ImGui::End();
        } break;
    case eControllerMenuState::pendingControllerPairRequest:
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

            // Show progress
            if (m_pair_steps_total > 0)
            {
                const float fraction= clampf01(static_cast<float>(m_pair_steps_completed) / static_cast<float>(m_pair_steps_total));

				switch (m_pair_steps_completed)
				{
				case ePairingStatus::start:
				{
					ImGui::Text("Starting pairing process...");
					break;
				}
				case ePairingStatus::setupBluetoothRadio:
				{
					ImGui::Text("Setting up bluetooth radio...");
					break;
				}
				case ePairingStatus::removeDevice:
				{
					ImGui::Text("Removing existing device...");
					break;
				}
				case ePairingStatus::deviceScan:
				{
					ImGui::Text("Searching for device...");
					break;
				}
				case ePairingStatus::authenticateDevice:
				{
					ImGui::Text("Authenticate device...");
					break;
				}
				case ePairingStatus::attemptConnection:
				{
					ImGui::Text("Attempt connection to device...");
					break;
				}
				case ePairingStatus::patchRegistry:
				{
					ImGui::Text("Patching registry...");
					break;
				}
				case ePairingStatus::verifyConnection:
				{
					ImGui::Text("Verifying connection to device...");
					break;
				}
				case ePairingStatus::success:
				{
					ImGui::Text("Success!");
					break;
				}
				case ePairingStatus::failed:
				{
					ImGui::Text("Failed!");
					break;
				}
				}

                ImGui::ProgressBar(fraction, ImVec2(-1, 0), " ");

				ImGui::Separator();

				ImGui::PushTextWrapPos();
				ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0,0), ImVec2(1,1), AssetManager::getInstance()->k_imcolor_orange());
				ImGui::SameLine();

				if (m_controllerType == PSMController_DualShock4)
				{
					ImGui::TextColored(AssetManager::getInstance()->k_imcolor_orange(),
						"Unplug the DualShock4 form USB.\n"
						"Then press and HOLD the DualShock4's PS and SHARE buttons.\n"
						"After a moment, the lightbar will start flashing rapidly.\n"
						"Repeat this until the controller completes pairing");
				}
				else
				{
					ImGui::TextColored(AssetManager::getInstance()->k_imcolor_orange(),
						"Unplug the PSMove from USB.\n"
						"Then press the PSMove's PS button.\n"
						"The red status LED will start blinking.\n"
						"Whenever it goes fully off, press the PS button again.\n"
						"Repeat this until the status LED finally remains lit.");
				}
				ImGui::PopTextWrapPos();
            }
            else
            {
                ImGui::Text("Waiting for controller to pair...");
            }

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconBan(), "Cancel"))
            {
                assert(m_pendingBluetoothOpControllerIndex != -1);
                request_cancel_bluetooth_operation(m_pendingBluetoothOpControllerIndex);
            }

			ImGui::SetWindowSize(ImVec2(400, 0));
            ImGui::End();
        } break;
    case eControllerMenuState::failedControllerPairRequest:
        {
			ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::Text("Failed to pair controller!");

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

			ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconLeft(), "Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(300, 0));
            ImGui::End();
        } break;

    case eControllerMenuState::pendingCancelBluetoothRequest:
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
            ImGui::Text("Canceling bluetooth operation...");

			ImGui::SetWindowSize(ImVec2(300, 0));
            ImGui::End();
        } break;

    case eControllerMenuState::failedCancelBluetoothRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::Text("Failed to cancel bluetooth operation (already completed?)");

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

			ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconLeft(), "Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(300, 0));
            ImGui::End();
        } break;

    default:
        assert(0 && "unreachable");
    }
}

bool AppStage_PairController::onClientAPIEvent(
    PSMEventMessage::eEventType event, 
    PSMEventDataHandle opaque_event_handle)
{
    bool bHandled= false;

    switch(event)
    {
    case PSMEventMessage::PSMEvent_opaqueServiceEvent:
        {
            const PSMoveProtocol::Response *event= GET_PSMOVEPROTOCOL_EVENT(opaque_event_handle);

            switch(event->type())
            {
            case PSMoveProtocol::Response_ResponseType_UNPAIR_REQUEST_COMPLETED:
                {
                    bHandled= true;
                    handle_controller_unpair_end_event(event);
                } break;
            case PSMoveProtocol::Response_ResponseType_PAIR_REQUEST_COMPLETED:
                {
                    bHandled= true;
                    handle_controller_pair_end_event(event);
                } break;
            case PSMoveProtocol::Response_ResponseType_BLUETOOTH_REQUEST_PROGRESS:
                {
                    bHandled= true;
                    handle_bluetooth_request_progress_event(event);
                } break;
            }
        } break;
    }

    return bHandled;
}

void AppStage_PairController::request_controller_unpair(
    int controllerID,
	PSMControllerType controllerType)
{
    if (m_menuState != AppStage_PairController::pendingControllerUnpairRequest && 
        m_menuState != AppStage_PairController::pendingControllerPairRequest)
    {
		m_controllerID= controllerID;
		m_controllerType= controllerType;

        m_menuState= AppStage_PairController::pendingControllerUnpairRequest;
        m_pendingBluetoothOpControllerIndex= controllerID;

        // Tell the psmove service that we want to unpair the given controller
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_UNPAIR_CONTROLLER);
        request->mutable_unpair_controller()->set_controller_id(controllerID);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_PairController::handle_controller_unpair_start_response, this);
    }
}

void AppStage_PairController::handle_controller_unpair_start_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_PairController *thisPtr= static_cast<AppStage_PairController *>(userdata);

    switch(response->result_code)
    {
        case PSMResult_Success:
        {
            thisPtr->m_menuState= AppStage_PairController::pendingControllerUnpairRequest;
        } break;

        case PSMResult_Error:
        case PSMResult_Canceled:
		case PSMResult_Timeout:
        { 
            thisPtr->m_menuState= AppStage_PairController::failedControllerUnpairRequest;
            thisPtr->m_pendingBluetoothOpControllerIndex= -1;
        } break;
    }
}

void AppStage_PairController::handle_controller_unpair_end_event(
    const PSMoveProtocol::Response *event)
{
    // No longer have a pending bluetooth operation
    m_pendingBluetoothOpControllerIndex= -1;

    switch(event->result_code())
    {
        case PSMoveProtocol::Response_ResultCode_RESULT_OK:
        {
            // Refresh the list of controllers now that we have confirmation the controller is unpaired
            m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
        } break;

        case PSMoveProtocol::Response_ResultCode_RESULT_ERROR:
        case PSMoveProtocol::Response_ResultCode_RESULT_CANCELED:
        { 
            this->m_menuState= AppStage_PairController::failedControllerUnpairRequest;
        } break;
    }
}

void AppStage_PairController::request_controller_pair(
    int controllerID,
	PSMControllerType controllerType)
{
    if (m_menuState != AppStage_PairController::pendingControllerUnpairRequest && 
        m_menuState != AppStage_PairController::pendingControllerPairRequest)
    {
		m_controllerID= controllerID;
		m_controllerType= controllerType;

        m_menuState= AppStage_PairController::pendingControllerPairRequest;
        m_pendingBluetoothOpControllerIndex= controllerID;
        m_pair_steps_completed = 0;
        m_pair_steps_total = 0;

        // Tell the psmove service that we want to pair the given controller
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_PAIR_CONTROLLER);
        request->mutable_pair_controller()->set_controller_id(controllerID);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_PairController::handle_controller_pair_start_response, this);
    }
}

void AppStage_PairController::handle_controller_pair_start_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_PairController *thisPtr= static_cast<AppStage_PairController *>(userdata);

    switch(response->result_code)
    {
        case PSMResult_Success:
        {
            thisPtr->m_menuState= AppStage_PairController::pendingControllerPairRequest;
        } break;

        case PSMResult_Error:
        case PSMResult_Canceled:
		case PSMResult_Timeout:
        { 
            thisPtr->m_menuState= AppStage_PairController::failedControllerPairRequest;
            thisPtr->m_pendingBluetoothOpControllerIndex= -1;
        } break;
    }
}

void AppStage_PairController::handle_controller_pair_end_event(
    const PSMoveProtocol::Response *event)
{
    // No longer have a pending bluetooth operation
    m_pendingBluetoothOpControllerIndex= -1;

    switch(event->result_code())
    {
        case PSMoveProtocol::Response_ResultCode_RESULT_OK:
        {
            // Refresh the list of controllers now that we have confirmation the controller is unpaired
            m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
        } break;

        case PSMoveProtocol::Response_ResultCode_RESULT_ERROR:
        case PSMoveProtocol::Response_ResultCode_RESULT_CANCELED:
        { 
            this->m_menuState= AppStage_PairController::failedControllerPairRequest;
        } break;
    }
}

void AppStage_PairController::handle_bluetooth_request_progress_event(
    const PSMoveProtocol::Response *event)
{
    const PSMoveProtocol::Response_ResultBluetoothRequestProgress &progress= 
        event->result_bluetooth_request_progress();

    if (m_pendingBluetoothOpControllerIndex == progress.controller_id())
    {
        m_pair_steps_completed = progress.steps_completed();
        m_pair_steps_total = progress.total_steps();
    }
}

void AppStage_PairController::request_cancel_bluetooth_operation(
    int controllerID)
{
    if (m_pendingBluetoothOpControllerIndex != -1 &&
        (m_menuState == AppStage_PairController::pendingControllerUnpairRequest ||
         m_menuState == AppStage_PairController::pendingControllerPairRequest))
    {
        m_menuState= AppStage_PairController::pendingCancelBluetoothRequest;
        m_pendingBluetoothOpControllerIndex= controllerID;

        // Tell the psmove service that we want to cancel the pending bluetooth request
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_CANCEL_BLUETOOTH_REQUEST);
        request->mutable_cancel_bluetooth_request()->set_controller_id(controllerID);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_PairController::handle_cancel_bluetooth_operation_response, this);
    }
}

void AppStage_PairController::handle_cancel_bluetooth_operation_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_PairController *thisPtr= static_cast<AppStage_PairController *>(userdata);

    if (thisPtr->m_menuState == AppStage_PairController::pendingCancelBluetoothRequest)
    {
        // No longer pending a bluetooth operation in any case
        thisPtr->m_pendingBluetoothOpControllerIndex= -1;

        switch(response->result_code)
        {
            case PSMResult_Success:
            case PSMResult_Canceled:
			case PSMResult_Timeout:
            {
                // Refresh the list of controllers now that we have confirmation the controller is unpaired
                thisPtr->m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
            } break;

            case PSMResult_Error:
            { 
                thisPtr->m_menuState= AppStage_PairController::failedCancelBluetoothRequest;
            } break;
        }
    }
}