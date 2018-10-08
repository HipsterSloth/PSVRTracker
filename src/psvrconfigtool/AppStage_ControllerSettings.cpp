//-- inludes -----
#include "AppStage_ControllerSettings.h"
#include "AppStage_AccelerometerCalibration.h"
//#include "AppStage_OpticalCalibration.h"
//#include "AppStage_GyroscopeCalibration.h"
//#include "AppStage_MagnetometerCalibration.h"
#include "AppStage_MainMenu.h"
//#include "AppStage_PairController.h"
//#include "AppStage_TestButtons.h"
//#include "AppStage_TestRumble.h"
#include "App.h"
#include "Camera.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
#include <sstream>

#ifdef _WIN32
#include <windows.h>  // Required for data types
#include <winuser.h>
#include <bthsdpdef.h>
#include <bluetoothapis.h>
#include <Dbt.h>
#include <guiddef.h>
#include <setupapi.h> // Device setup APIs
#include <assert.h>
#include <strsafe.h>
#include <winreg.h>
#include <Shellapi.h>
#include <TlHelp32.h>
#include <Psapi.h>
#endif // _WIN32

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

#ifdef _WIN32
class Win32AdminCheck {

public:
	BOOL psMoveServiceFound = FALSE;
	BOOL psMoveServiceAdminFound = FALSE;
	BOOL psMoveServiceElevated = FALSE;
	BOOL psMoveServiceAdminElevated = FALSE;
	DWORD psMoveServiceId = 0;
	DWORD psMoveServiceAdminEId = 0;

	BOOL IsElevated(HANDLE hProcess) {
		BOOL fRet = FALSE;
		HANDLE hToken = NULL;
		if (OpenProcessToken(hProcess, TOKEN_QUERY, &hToken)) {
			TOKEN_ELEVATION Elevation;
			DWORD cbSize = sizeof(TOKEN_ELEVATION);
			if (GetTokenInformation(hToken, TokenElevation, &Elevation, sizeof(Elevation), &cbSize)) {
				fRet = Elevation.TokenIsElevated;
			}
		}
		if (hToken) {
			CloseHandle(hToken);
		}
		return fRet;
	}

	void CheckProcesses() {

		psMoveServiceFound = FALSE;
		psMoveServiceAdminFound = FALSE;
		psMoveServiceElevated = FALSE;
		psMoveServiceAdminElevated = FALSE;
		psMoveServiceId = 0;
		psMoveServiceAdminEId = 0;

		
		PROCESSENTRY32 entry;
		entry.dwSize = sizeof(PROCESSENTRY32);

		HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, NULL);

		if (Process32First(snapshot, &entry) == TRUE)
		{
			while (Process32Next(snapshot, &entry) == TRUE)
			{
				if (stricmp(entry.szExeFile, "PSMoveService.exe") == 0)
				{
					psMoveServiceId = entry.th32ProcessID;
					HANDLE hProcess = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, entry.th32ProcessID);

					psMoveServiceFound = TRUE;
					psMoveServiceElevated = IsElevated(hProcess);

					CloseHandle(hProcess);
				}

				if (stricmp(entry.szExeFile, "PSMoveServiceAdmin.exe") == 0)
				{
					psMoveServiceAdminEId = entry.th32ProcessID;
					HANDLE hProcess = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, entry.th32ProcessID);

					psMoveServiceAdminFound = TRUE;
					psMoveServiceAdminElevated = IsElevated(hProcess);

					CloseHandle(hProcess);
				}

			}
		}

		CloseHandle(snapshot);
	}

	void RestartAdminMode() {
		if (psMoveServiceId != 0 && !psMoveServiceElevated) 
		{
			char moduleFileName[MAXCHAR];
			HANDLE hProcess = OpenProcess(PROCESS_ALL_ACCESS, FALSE, psMoveServiceId);
			if (hProcess != NULL) {

				GetModuleFileNameEx(hProcess, NULL, moduleFileName, MAXCHAR);

				TerminateProcess(hProcess, 0);
				CloseHandle(hProcess);

				char drive[5];
				char dir[MAXCHAR];
				_splitpath_s(moduleFileName, drive, sizeof(drive), dir, sizeof(dir), NULL, 0, NULL, 0);

				std::string adminDir = drive;
				adminDir = adminDir + dir;
				std::string adminPath = adminDir + "PSMoveServiceAdmin.exe";

				SHELLEXECUTEINFO ShExecInfo;
				ShExecInfo.cbSize = sizeof(SHELLEXECUTEINFO);
				ShExecInfo.fMask = NULL;
				ShExecInfo.hwnd = NULL;
				ShExecInfo.lpVerb = NULL;
				ShExecInfo.lpFile = adminPath.c_str();
				ShExecInfo.lpParameters = NULL;
				ShExecInfo.lpDirectory = NULL;
				ShExecInfo.nShow = SW_NORMAL;
				ShExecInfo.hInstApp = NULL;

				BOOL create = ShellExecuteEx(&ShExecInfo);
			}
		}
	}

	BOOL IsAnyElevated() {
		return psMoveServiceElevated || psMoveServiceAdminElevated;
	}

};

Win32AdminCheck adminCheck;
#endif

//-- statics ----
const char *AppStage_ControllerSettings::APP_STAGE_NAME= "ControllerSettings";

//-- constants -----
const int k_default_position_filter_index = 3; // LowPassExponential
const int k_default_psmove_orientation_filter_index = 3; // ComplementaryMARG
const int k_default_ds4_position_filter_index = 5; // PositionKalman
const int k_default_ds4_orientation_filter_index = 3; // OrientationKalman
const int k_default_ds4_gyro_gain_index = 4; // 2000deg/s

const char* k_controller_position_filter_names[] = { "PassThru", "LowPassOptical", "LowPassIMU", "LowPassExponential", "ComplimentaryOpticalIMU", "PositionKalman" };
const char* k_psmove_orientation_filter_names[] = { "PassThru", "MadgwickARG", "MadgwickMARG", "ComplementaryMARG", "ComplementaryOpticalARG", "OrientationKalman" };
const char* k_ds4_orientation_filter_names[] = { "PassThru", "MadgwickARG", "ComplementaryOpticalARG", "OrientationKalman" };
const char* k_ds4_gyro_gain_setting_labels[] = { "125deg/s", "250deg/s", "500deg/s", "1000deg/s", "2000deg/s", "custom"};

const float k_max_hmd_prediction_time = 0.15f; // About 150ms seems to be about the point where you start to get really bad over-prediction 

inline int find_string_entry(const char *string_entry, const char* string_list[], size_t list_size)
{
    int found_index = -1;
    for (size_t test_index = 0; test_index < list_size; ++test_index)
    {
        if (strncmp(string_entry, string_list[test_index], 32) == 0)
        {
            found_index = static_cast<int>(test_index);
            break;
        }
    }

    return found_index;
}

//-- public methods -----
AppStage_ControllerSettings::AppStage_ControllerSettings(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_ControllerSettings::inactive)
    , m_selectedControllerIndex(-1)
{ }

void AppStage_ControllerSettings::enter()
{
#ifdef _WIN32
	adminCheck.CheckProcesses();
#endif

    m_app->setCameraType(_cameraFixed);

    rebuild_controller_list();
}

void AppStage_ControllerSettings::exit()
{
    m_menuState= AppStage_ControllerSettings::inactive;
}

void AppStage_ControllerSettings::update()
{
}
    
void AppStage_ControllerSettings::render()
{
    glm::mat4 scale2RotateX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f)), 
            90.f, glm::vec3(1.f, 0.f, 0.f));    

    switch (m_menuState)
    {
    case eControllerMenuState::idle:
        {
            if (m_selectedControllerIndex >= 0 && m_selectedControllerIndex < m_controllerInfos.size())
            {
                const ControllerInfo &controllerInfo= m_controllerInfos[m_selectedControllerIndex];

                switch(controllerInfo.controller.controller_type)
                {
                    case PSVRController_Move:
                    case PSVRController_DualShock4:
                        {
                            const ControllerInfo &controllerInfo = m_controllerInfos[m_selectedControllerIndex];

                            // Display the tracking color being used for the controller
                            glm::vec3 bulb_color = glm::vec3(1.f, 1.f, 1.f);

                            switch (controllerInfo.controller.tracking_color_type)
                            {
                            case PSVRTrackingColorType_Magenta:
                                bulb_color = glm::vec3(1.f, 0.f, 1.f);
                                break;
                            case PSVRTrackingColorType_Cyan:
                                bulb_color = glm::vec3(0.f, 1.f, 1.f);
                                break;
                            case PSVRTrackingColorType_Yellow:
                                bulb_color = glm::vec3(1.f, 1.f, 0.f);
                                break;
                            case PSVRTrackingColorType_Red:
                                bulb_color = glm::vec3(1.f, 0.f, 0.f);
                                break;
                            case PSVRTrackingColorType_Green:
                                bulb_color = glm::vec3(0.f, 1.f, 0.f);
                                break;
                            case PSVRTrackingColorType_Blue:
                                bulb_color = glm::vec3(0.f, 0.f, 1.f);
                                break;
                            default:
                                break;
                            }

                            if (controllerInfo.controller.controller_type == PSVRController_Move)
                            {
                                drawPSMoveModel(scale2RotateX90, bulb_color);
                            }
                            else if (controllerInfo.controller.controller_type == PSVRController_DualShock4)
                            {
                                drawPSDualShock4Model(scale2RotateX90, bulb_color);
                            }
                        } break;
                    default:
                        assert(0 && "Unreachable");
                }        
            }
        } break;

    case eControllerMenuState::failedControllerListRequest:
        {
        } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ControllerSettings::renderUI()
{
    const char *k_window_title= "Controller Settings";
    const ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eControllerMenuState::idle:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(350, 490));
            ImGui::Begin("Controller Settings", nullptr, window_flags | ImGuiWindowFlags_MenuBar);

            if (ImGui::CollapsingHeader("Host Info", 0, true, true))
            {
                if (m_hostSerial.length() > 1 && m_hostSerial != "00:00:00:00:00:00")
                {
                    ImGui::Text("Host Serial: %s", m_hostSerial.c_str());
                }
                else
                {
                    ImGui::Text("No bluetooth adapter detected!");
                }
            }

            if (m_controllerInfos.size() > 0)
            {
                AppStage_ControllerSettings::ControllerInfo &info= m_controllerInfos[m_selectedControllerIndex];

                if (ImGui::CollapsingHeader("Controller Settings", 0, true, true))
                {
                    if (m_selectedControllerIndex > 0)
                    {
                        if (ImGui::Button("<##ControllerIndex"))
                        {
                            --m_selectedControllerIndex;
                        }
                        ImGui::SameLine();
                    }
                    ImGui::Text("Controller: %d", m_selectedControllerIndex);
                    if (m_selectedControllerIndex + 1 < static_cast<int>(m_controllerInfos.size()))
                    {
                        ImGui::SameLine();
                        if (ImGui::Button(">##ControllerIndex"))
                        {
                            ++m_selectedControllerIndex;
                        }
                    }

                    // Combo box selection for controller tracking color
                    if (info.controller.is_bluetooth)
                    {
                        int newTrackingColorType = info.controller.tracking_color_type;

                        if (ImGui::Combo("Tracking Color", &newTrackingColorType, "Magenta\0Cyan\0Yellow\0Red\0Green\0Blue\0\0"))
                        {
                            info.controller.tracking_color_type = static_cast<PSVRTrackingColorType>(newTrackingColorType);

							PSVR_SetControllerLEDTrackingColor(info.controller.controller_id, info.controller.tracking_color_type);

                            // Re-request the controller list since the tracking colors could changed for other controllers
                            rebuild_controller_list();
                        }
                    }

                    // Combo box selection for controller hand
                    if (info.controller.is_bluetooth)
                    {
                        int newHand = info.controller.controller_hand;

                        if (ImGui::Combo("Hand", &newHand, "Any\0Left\0Right\0\0"))
                        {
                            info.controller.controller_hand = static_cast<PSVRControllerHand>(newHand);

							PSVR_SetControllerHand(info.controller.controller_id, info.controller.controller_hand);
                        }
                    }

                    ImGui::BulletText("Controller ID: %d", info.controller.controller_id);

                    switch(info.controller.controller_type)
                    {
                        case PSVRController_Move:
                            {
                                ImGui::BulletText("Controller Type: PSMove");
                            } break;
                        case PSVRController_DualShock4:
                            {
                                ImGui::BulletText("Controller Type: PSDualShock4");
                            } break;
                        default:
                            assert(0 && "Unreachable");
                    }

                    ImGui::BulletText("Device Serial: %s", info.controller.controller_serial);
                    ImGui::BulletText("Assigned Host Serial: %s", info.controller.assigned_host_serial);

					if (info.controller.controller_type == PSVRController_Move)
					{
#ifdef _WIN32
						if (adminCheck.IsAnyElevated())
						{
#endif
							if (info.controller.is_bluetooth)
							{
								if (ImGui::Button("Unpair Controller"))
								{
									//###HipsterSloth $TODO $CONTROLLER_UI
									//m_app->getAppStage<AppStage_PairController>()->request_controller_unpair(info.ControllerID, info.controller.controller_type);
									//m_app->setAppStage(AppStage_PairController::APP_STAGE_NAME);
								}
							}
							else
							{
								if (ImGui::Button("Pair Controller"))
								{
									//###HipsterSloth $TODO $CONTROLLER_UI
									//m_app->getAppStage<AppStage_PairController>()->request_controller_pair(info.ControllerID, info.controller.controller_type);
									//m_app->setAppStage(AppStage_PairController::APP_STAGE_NAME);
								}
							}
#ifdef _WIN32
						}
						else 
						{
							if (ImGui::Button("Restart Service in Admin mode\nto Pair or Unpair Controller"))
							{
								adminCheck.RestartAdminMode();
							}
						}
#endif
					}

				    if (info.controller.is_bluetooth)
                    {
                        if (ImGui::CollapsingHeader("Filters", 0, true, false))
                        {
					        if (info.controller.controller_type == PSVRController_Move)
					        {
						        ImGui::PushItemWidth(195);
						        if (ImGui::Combo("Position Filter", &info.PositionFilterIndex, k_controller_position_filter_names, UI_ARRAYSIZE(k_controller_position_filter_names)))
						        {
							        info.PositionFilterName = k_controller_position_filter_names[info.PositionFilterIndex];
							        PSVR_SetControllerPositionFilter(info.controller.controller_id, info.PositionFilterName.c_str());
						        }
						        if (info.controller.controller_type == PSVRController_Move)
						        {
							        if (ImGui::Combo("Orientation Filter", &info.OrientationFilterIndex, k_psmove_orientation_filter_names, UI_ARRAYSIZE(k_psmove_orientation_filter_names)))
							        {
								        info.OrientationFilterName = k_psmove_orientation_filter_names[info.OrientationFilterIndex];
										PSVR_SetControllerOrientationFilter(info.controller.controller_id, info.OrientationFilterName.c_str());
							        }
						        }
						        if (ImGui::SliderFloat("Prediction Time", &info.controller.prediction_time, 0.f, k_max_hmd_prediction_time))
						        {
									PSVR_SetControllerPredictionTime(info.controller.controller_id, info.controller.prediction_time);
						        }
						        if (ImGui::Button("Reset Filter Defaults"))
						        {
							        info.PositionFilterIndex = k_default_position_filter_index;
							        info.PositionFilterName = k_controller_position_filter_names[k_default_position_filter_index];
									PSVR_SetControllerPositionFilter(info.controller.controller_id, info.PositionFilterName.c_str());

							        if (info.controller.controller_type == PSVRController_Move)
							        {
								        info.OrientationFilterIndex = k_default_psmove_orientation_filter_index;
								        info.OrientationFilterName = k_psmove_orientation_filter_names[k_default_psmove_orientation_filter_index];
										PSVR_SetControllerOrientationFilter(info.controller.controller_id, info.OrientationFilterName.c_str());
							        }
						        }
						        ImGui::PopItemWidth();
					        }
					        else if (info.controller.controller_type == PSVRController_DualShock4)
					        {
						        ImGui::PushItemWidth(195);
						        if (ImGui::Combo("Position Filter", &info.PositionFilterIndex, k_controller_position_filter_names, UI_ARRAYSIZE(k_controller_position_filter_names)))
						        {
							        info.PositionFilterName = k_controller_position_filter_names[info.PositionFilterIndex];
							        PSVR_SetControllerPositionFilter(info.controller.controller_id, info.PositionFilterName.c_str());
						        }
						        if (ImGui::Combo("Orientation Filter", &info.OrientationFilterIndex, k_ds4_orientation_filter_names, UI_ARRAYSIZE(k_ds4_orientation_filter_names)))
						        {
							        info.OrientationFilterName = k_ds4_orientation_filter_names[info.OrientationFilterIndex];
							        PSVR_SetControllerOrientationFilter(info.controller.controller_id, info.OrientationFilterName.c_str());
						        }
						        if (ImGui::Combo("Gyro Gain", &info.GyroGainIndex, k_ds4_gyro_gain_setting_labels, UI_ARRAYSIZE(k_ds4_gyro_gain_setting_labels)))
						        {
							        info.GyroGainSetting = k_ds4_gyro_gain_setting_labels[info.GyroGainIndex];
							        PSVR_SetControllerGyroscopeCalibration(info.controller.controller_id, -1.f, -1.f, info.GyroGainSetting.c_str());
						        }
						        if (ImGui::SliderFloat("Prediction Time", &info.controller.prediction_time, 0.f, k_max_hmd_prediction_time))
						        {
									PSVR_SetControllerPredictionTime(info.controller.controller_id, info.controller.prediction_time);
						        }
						        if (ImGui::Button("Reset Filter Defaults"))
						        {
							        info.PositionFilterIndex = k_default_ds4_position_filter_index;
							        info.OrientationFilterIndex = k_default_ds4_orientation_filter_index;
							        info.GyroGainIndex = k_default_ds4_gyro_gain_index;
							        info.PositionFilterName = k_controller_position_filter_names[k_default_ds4_position_filter_index];
							        info.OrientationFilterName = k_ds4_orientation_filter_names[k_default_ds4_orientation_filter_index];
							        info.GyroGainSetting = k_ds4_gyro_gain_setting_labels[k_default_ds4_gyro_gain_index];
									PSVR_SetControllerPositionFilter(info.controller.controller_id, info.PositionFilterName.c_str());
									PSVR_SetControllerOrientationFilter(info.controller.controller_id, info.OrientationFilterName.c_str());
									PSVR_SetControllerGyroscopeCalibration(info.controller.controller_id, -1.f, -1.f, info.GyroGainSetting.c_str());
						        }
						        ImGui::PopItemWidth();
					        }
                        }
				    }
                }

				if (info.controller.controller_type == PSVRController_Move && 
					info.controller.is_bluetooth)
                {
					if (ImGui::CollapsingHeader("Controller Calibration", 0, true))
					{
						if (info.controller.has_magnetometer)
						{
							ImGui::TextDisabled("Calibrate Magnetometer");
							//if (ImGui::Button("Calibrate Magnetometer"))
							{
								//###HipsterSloth $TODO $CONTROLLER_UI
								//m_app->getAppStage<AppStage_MagnetometerCalibration>()->setBypassCalibrationFlag(false);
								//m_app->setAppStage(AppStage_MagnetometerCalibration::APP_STAGE_NAME);
							}
						}
						else
						{
							ImGui::TextDisabled("Magnetometer Disabled");
						}

						ImGui::TextDisabled("Calibrate Gyroscope");
						//if (ImGui::Button("Calibrate Gyroscope"))
						{
							//###HipsterSloth $TODO $CONTROLLER_UI
							//m_app->getAppStage<AppStage_GyroscopeCalibration>()->setBypassCalibrationFlag(false);
							//m_app->setAppStage(AppStage_GyroscopeCalibration::APP_STAGE_NAME);
						}

						ImGui::TextDisabled("Calibrate Optical Noise");
						//if (ImGui::Button("Calibrate Optical Noise"))
						{
							//###HipsterSloth $TODO $CONTROLLER_UI
						    //m_app->getAppStage<AppStage_OpticalCalibration>()->setBypassCalibrationFlag(false);
						    //m_app->setAppStage(AppStage_OpticalCalibration::APP_STAGE_NAME);
						}
					}
				}

				if (info.controller.is_bluetooth)
				{
					if (ImGui::CollapsingHeader("Controller Tests", 0, true))
					{
						if (info.controller.controller_type == PSVRController_Move)
						{
							ImGui::TextDisabled("Test Orientation");
							//if (ImGui::Button("Test Orientation"))
							{
								//###HipsterSloth $TODO $CONTROLLER_UI
								//m_app->getAppStage<AppStage_MagnetometerCalibration>()->setBypassCalibrationFlag(true);
								//m_app->setAppStage(AppStage_MagnetometerCalibration::APP_STAGE_NAME);
							}
						}

						if (info.controller.controller_type == PSVRController_DualShock4)
						{
							ImGui::TextDisabled("Calibrate Optical Noise");
							//if (ImGui::Button("Calibrate Optical Noise"))
							{
								//###HipsterSloth $TODO $CONTROLLER_UI
							    //m_app->getAppStage<AppStage_OpticalCalibration>()->setBypassCalibrationFlag(false);
							    //m_app->setAppStage(AppStage_OpticalCalibration::APP_STAGE_NAME);
							}

							if (info.controller.controller_type == PSVRController_DualShock4)
							{
								ImGui::TextDisabled("Test Orientation");
								//if (ImGui::Button("Test Orientation"))
								{
									//###HipsterSloth $TODO $CONTROLLER_UI
									//m_app->getAppStage<AppStage_GyroscopeCalibration>()->setBypassCalibrationFlag(true);
									//m_app->setAppStage(AppStage_GyroscopeCalibration::APP_STAGE_NAME);
								}
							}
						}

						if (info.controller.controller_type == PSVRController_Move || 
							info.controller.controller_type == PSVRController_DualShock4)
						{
							if (ImGui::Button("Test Accelerometer"))
							{
								m_app->getAppStage<AppStage_AccelerometerCalibration>()->setBypassCalibrationFlag(true);
								m_app->setAppStage(AppStage_AccelerometerCalibration::APP_STAGE_NAME);
							}

							ImGui::TextDisabled("Test Rumble");
							//if (ImGui::Button("Test Rumble"))
							{
								//###HipsterSloth $TODO $CONTROLLER_UI
								//m_app->setAppStage(AppStage_TestRumble::APP_STAGE_NAME);
							}
						}

						ImGui::TextDisabled("Test Buttons");
						//if (ImGui::Button("Test Buttons"))
						{
							//###HipsterSloth $TODO $CONTROLLER_UI
							//m_app->setAppStage(AppStage_TestButtons::APP_STAGE_NAME);
						}
					}
				}
            }
            else
            {
                if (ImGui::CollapsingHeader("Controller Settings"))
                {
                    ImGui::Text("No connected usable controllers");
                }
            }

            ImGui::Spacing();
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::Spacing();

			if (ImGui::Button("Return to Main Menu"))
			{
				m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
			}

            ImGui::End();
        } break;

    case eControllerMenuState::failedControllerListRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(300, 150));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed to get controller list!");

            if (ImGui::Button("Retry"))
            {
                rebuild_controller_list();
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    default:
        assert(0 && "unreachable");
    }
}

bool AppStage_ControllerSettings::onClientAPIEvent(
    PSVREventType event_type)
{
    bool bHandled= false;

    switch(event_type)
    {
    case PSVREvent_controllerListUpdated:
        {
            bHandled= true;
            rebuild_controller_list();
        } break;
    }

    return bHandled;
}

void AppStage_ControllerSettings::rebuild_controller_list()
{
    m_controllerInfos.clear();

    // Do get controllers connected bia USB in this menu since we need the info for pairing/unpairing
	PSVRControllerList controller_list;
	if (PSVR_GetControllerList(true, &controller_list) != PSVRResult_Success)
	{
		m_menuState= AppStage_ControllerSettings::failedControllerListRequest;
		return;
	}

    int oldSelectedControllerIndex= m_selectedControllerIndex;

    m_hostSerial = controller_list.host_serial;
    m_selectedControllerIndex= -1;
    m_controllerInfos.clear();

    for (int controller_index= 0; controller_index < controller_list.count; ++controller_index)
    {
        AppStage_ControllerSettings::ControllerInfo info;

		info.controller= controller_list.controllers[controller_index];

        if (info.controller.controller_type == PSVRController_Move)
        {
            info.OrientationFilterIndex =
                find_string_entry(
                    info.controller.position_filter,
                    k_psmove_orientation_filter_names,
                    UI_ARRAYSIZE(k_psmove_orientation_filter_names));
            if (info.OrientationFilterIndex == -1)
            {
                info.OrientationFilterName = k_psmove_orientation_filter_names[0];
                info.OrientationFilterIndex = 0;
            }
        }
        else if (info.controller.controller_type == PSVRController_DualShock4)
        {
            info.OrientationFilterIndex =
                find_string_entry(
                    info.controller.orientation_filter,
                    k_ds4_orientation_filter_names,
                    UI_ARRAYSIZE(k_ds4_orientation_filter_names));
            if (info.OrientationFilterIndex == -1)
            {
                info.OrientationFilterName = k_ds4_orientation_filter_names[0];
                info.OrientationFilterIndex = 0;
            }
        }
        else
        {
            info.OrientationFilterName = "";
            info.OrientationFilterIndex = -1;
        }

        info.PositionFilterIndex =
            find_string_entry(
                info.controller.position_filter,
                k_controller_position_filter_names,
                UI_ARRAYSIZE(k_controller_position_filter_names));
        if (info.PositionFilterIndex == -1)
        {
            info.PositionFilterName = k_controller_position_filter_names[0];
            info.PositionFilterIndex = 0;
        }

        if (info.controller.controller_type == PSVRController_DualShock4)
        {
            info.GyroGainIndex =
                find_string_entry(
                    info.controller.gyro_gain_setting,
                    k_ds4_gyro_gain_setting_labels,
                    UI_ARRAYSIZE(k_ds4_gyro_gain_setting_labels));
            if (info.GyroGainIndex == -1)
            {
                info.GyroGainSetting = k_ds4_gyro_gain_setting_labels[0];
                info.GyroGainIndex = 0;
            }
        }
        else
        {
            info.GyroGainSetting = "";
            info.GyroGainIndex = -1;
        }

        m_controllerInfos.push_back(info);
    }

	// Remove any USB controller entries that have a corresponding bluetooth controller entry
	// This means the controller is already paired via bluetooth
	auto iter = m_controllerInfos.begin();
	while (iter != m_controllerInfos.end())
	{
		bool bRemoveUSBControllerEntry= false;

		ControllerInfo &usb_psmove_info= *iter;
		if (!usb_psmove_info.controller.is_bluetooth)
		{
			for (const ControllerInfo &bluetooth_psmove_info : m_controllerInfos)
			{
				if (bluetooth_psmove_info.controller.is_bluetooth &&
					strncmp(bluetooth_psmove_info.controller.controller_serial, 
						usb_psmove_info.controller.controller_serial,
						PSVRSERVICE_CONTROLLER_SERIAL_LEN) == 0)
				{
					bRemoveUSBControllerEntry= true;
					break;
				}
			}
		}

		if (bRemoveUSBControllerEntry)
		{
			iter= m_controllerInfos.erase(iter);
		}
		else
		{
			++iter;
		}
	}

	// Find the index of the first non bluetooth PSMove controller, if any
	int first_usb_psmove_index= -1;
	int list_index= 0;
	for (auto it= m_controllerInfos.begin(); it != m_controllerInfos.end(); ++it)
	{
		if (!it->controller.is_bluetooth && it->controller.controller_type == PSVRController_Move)
		{
			first_usb_psmove_index= list_index;
			break;
		}

		++list_index;
	}

	// Determine which controller should be selected
	if (first_usb_psmove_index != -1)
	{
		m_selectedControllerIndex= first_usb_psmove_index;
	}
	else
	{
		if (oldSelectedControllerIndex != -1)
		{
			int controllerCount= static_cast<int>(m_controllerInfos.size());

			// Maintain the same position in the list if possible
			if (controllerCount > 0)
			{
				m_selectedControllerIndex= 
					(oldSelectedControllerIndex < controllerCount) ? oldSelectedControllerIndex : controllerCount-1;
			}
			else
			{
				m_selectedControllerIndex= -1;
			}
		}
		else
		{
			m_selectedControllerIndex= (m_controllerInfos.size() > 0) ? 0 : -1;
		}
	}

    m_menuState= AppStage_ControllerSettings::idle;
}