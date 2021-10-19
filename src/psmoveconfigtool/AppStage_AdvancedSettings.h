#ifndef APP_STAGE_ADVANCED_SETTINGS_H
#define APP_STAGE_ADVANCED_SETTINGS_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_AdvancedSettings : public AppStage
{
public:    
	AppStage_AdvancedSettings(class App *app);

    virtual bool init(int argc, char** argv) override;
    virtual void enter() override;
    virtual void exit() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    enum eMainMenuState
    {
        inactive,
        idle,
    };
    eMainMenuState m_menuState;
};

#endif // APP_STAGE_ADVANCED_SETTINGS_H