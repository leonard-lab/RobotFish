#ifndef PREDATOR_SERVER_H
#define PREDATOR_SERVER_H

/* Include necessary MADTraC headers */
#include "MT_Core.h"
#include "MT_GUI.h"
#include "MT_Tracking.h"

#include "PredatorTracker.h"

#ifdef WITH_SERVER
/**********************************************************************
 * Server Module Class (Optional)
 *********************************************************************/

class MT_SM_Predator : public MT_ServerModule
{
private:
    PredatorTracker* m_pPredatorTracker;

protected:
    bool handleMessage(MT_Server::t_msg msg_code, wxSocketBase* sock);
    MT_Server::t_msg_def* getMessageDefs();

    enum
    {
        msg_GetBlobInfo = 0,
        msg_Sentinel
    } msg_index;

public:
    MT_SM_Predator()
        : m_pPredatorTracker(NULL), MT_ServerModule("Predator"){};
    MT_SM_Predator(MT_Server* pServer, PredatorTracker* pTracker)
        : m_pPredatorTracker(pTracker),
          MT_ServerModule(pServer, "Predator"){};

    void sendBlobInfo(wxSocketBase* sock);

    void getBlobInfo(wxSocketBase* sock);

};
#endif /* WITH_SERVER */    

#endif /* PREDATOR_SERVER_H */
