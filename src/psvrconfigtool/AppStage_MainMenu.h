#ifndef APP_STAGE_INTRO_SCREEN_H
#define APP_STAGE_INTRO_SCREEN_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_MainMenu : public AppStage
{
public:    
    AppStage_MainMenu(class App *app);

    virtual bool init(int argc, char** argv) override;
    virtual void enter() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;
};

#endif // APP_STAGE_INTRO_SCREEN_H