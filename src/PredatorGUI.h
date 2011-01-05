#ifndef PREDATORTRACKERGUI_H
#define PREDATORTRACKERGUI_H

#ifdef _WIN32
#include <windows.h>
#endif

/* Include necessary MADTraC headers */
#include "MT_Core.h"
#include "MT_GUI.h"
#include "MT_Tracking.h"
#include "MT_Robot.h"

#include "PredatorRobot.h"
#include "PredatorTracker.h"
#include "PredatorServer.h"

/**********************************************************************
 * GUI Frame Class
 *********************************************************************/

class PredatorFrame : public MT_RobotFrameBase
{
protected:
    PredatorTracker* m_pPredatorTracker;
    MT_Server* m_pServer;

	double m_dGotoDist;
	double m_dGotoMaxSpeed;
	double m_dGotoTurningGain;

    int m_iNToTrack;
	int m_iGrabbedTrackedObj;

    bool m_bControlActive;
	bool m_bGotoActive;
	double m_dGotoX;
	double m_dGotoY;
    double m_dPolarizationThresh;

public:
    PredatorFrame(wxFrame* parent,
                         wxWindowID id = wxID_ANY,
                         const wxString& title = wxT("Tracker View"), 
                         const wxPoint& pos = wxDefaultPosition, 
                         const wxSize& size = wxSize(640,480),     
                         long style = MT_FIXED_SIZE_FRAME);

    virtual ~PredatorFrame(){if(m_pServer) delete m_pServer;};

   /* menu callbacks */

    void initTracker();
    void initUserData();

	void doUserControl();
	void doUserGLDrawing();

    MT_RobotBase* getNewRobot(const char* config, const char* name);

    void handleCommandLineArguments(int argc, wxChar** argv);
	void updateRobotStatesFromTracker();

	bool doKeyboardCallback(wxKeyEvent &event);
	bool doMouseCallback(wxMouseEvent& event, double viewport_x, double viewport_y);

	void onMenuAssign(wxCommandEvent& event);

};


/**********************************************************************
 * GUI App Class
 *********************************************************************/

class PredatorApp
: public MT_AppBase
{
public:
    MT_FrameWithInit* createMainFrame()
    {
        return new PredatorFrame(NULL);
    };
};


#endif /* PREDATORTRACKERGUI_H */
