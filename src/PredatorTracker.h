#ifndef PREDATORTRACKER_H
#define PREDATORTRACKER_H

#include "MT_Core.h"
#include "MT_Tracking.h"

#include "PredatorRobot.h"

const double DEFAULT_SIGMA_POSITION = 4.0; /* pixels */
const double DEFAULT_SIGMA_HEADING = 0.26; /* rad ~= 15 deg */
const double DEFAULT_SIGMA_SPEED = 1.0; /* pixels/frame */
const double DEFAULT_SIGMA_POSITION_MEAS = 1.0; /* pixels */
const double DEFAULT_SIGMA_HEADING_MEAS = 0.087; /* rad ~= 5 deg */

typedef std::vector<double> t_p_history;

class PredatorTracker : public MT_TrackerBase
{
private:
    /* frames */
    IplImage* m_pGSFrame;      /* Grayscale version of current frame */
    IplImage* m_pDiffFrame;    /* Background subtracted frame */
    IplImage* m_pThreshFrame;  /* Thresholded frame */
    IplImage* m_pThreshFrameCopy;

    /* blobber parameters */
    unsigned int m_iBlobValThresh;
	unsigned int m_iBlobAreaThreshLow;
	unsigned int m_iBlobAreaThreshHigh;
    
    CvRect m_SearchArea;
	unsigned int m_iSearchAreaPadding;

    /* only used to add to XDF */
    int m_iStartFrame;
    int m_iStopFrame;

    MT_GSThresholder* m_pGSThresholder;
    GYBlobber* m_pGYBlobber;
    YABlobber* m_pYABlobber;
    std::vector<MT_UKF_struct*> m_vpUKF;
    CvMat* m_pQ;
    CvMat* m_pR;
    CvMat* m_px0;
    CvMat* m_pz;
    double m_dSigmaPosition;
    double m_dSigmaHeading;
    double m_dSigmaSpeed;
    double m_dSigmaPositionMeas;
    double m_dSigmaHeadingMeas;
    
    double m_dPrevSigmaPosition;
    double m_dPrevSigmaHeading;
    double m_dPrevSigmaSpeed;
    double m_dPrevSigmaPositionMeas;
    double m_dPrevSigmaHeadingMeas;

    bool m_bShowBlobs;
    bool m_bShowTracking;

    /* operational variables */
    double m_dDt;
    int m_iFrameCounter;
    int m_iNObj;

    std::vector<double> m_vdBlobs_X;
    std::vector<double> m_vdBlobs_Y; 
    std::vector<double> m_vdBlobs_Area; 
    std::vector<double> m_vdBlobs_Orientation;
    std::vector<double> m_vdBlobs_MajorAxis;
    std::vector<double> m_vdBlobs_MinorAxis;
    std::vector<double> m_vdBlobs_Speed;

    int m_iMinFishPerimeter;
    int m_iMinFishArea;
    int m_iMaxFishPerimeter;
    int m_iMaxFishArea;
    int m_iNFishFound;
    
    std::vector<double> m_vdFish_X;
	std::vector<double> m_vdFish_Y;
	std::vector<double> m_vdFish_Orientation;
	std::vector<double> m_vdFish_Area;
    double m_dTotalFishArea;
    double m_dFishPolarization;
    double m_dFishCOMx;
    double m_dFishCOMy;
    double m_dFishXX;
    double m_dFishXY;
    double m_dFishYY;

    std::vector<double> m_vdTracked_X;
    std::vector<double> m_vdTracked_Y;
    std::vector<double> m_vdTracked_Heading;
    std::vector<double> m_vdTracked_Speed;

    std::vector<t_p_history> m_vdHistories_X;
    std::vector<t_p_history> m_vdHistories_Y;

	unsigned int m_iFrameWidth;
    unsigned int m_iFrameHeight;
    
public:
    /* constructor */
    PredatorTracker(IplImage* ProtoFrame, unsigned int n_obj);
    /* destructor - note we need the virtual destructor */
    virtual ~PredatorTracker();
    
    /* Initialization */
    void doInit(IplImage* ProtoFrame);

    /* Memory allocation / deallocation */
    void createFrames();
    void releaseFrames();

    void setDiffThresh(int thresh){m_iBlobValThresh = thresh;};
    void setStartStopFrames(int start_frame, int stop_frame)
    {m_iStartFrame = start_frame; m_iStopFrame = stop_frame;};

    void doTrain(IplImage* frame);

    void initDataFile();
    void writeData();

    /* Main tracking functions */
    void doTracking(IplImage* frame);

    void doGLDrawing(int flags);

	std::vector<double> getPredatorState(unsigned int i);
	double getPredatorX(unsigned int i){if(i >= m_vdTracked_X.size()){return 0;} else {return m_vdTracked_X[i];}};
	double getPredatorY(unsigned int i){if(i >= m_vdTracked_Y.size()){return 0;} else {return m_iFrameHeight - m_vdTracked_Y[i];}};

    void getFishInfo(int* nfish, double* xcom, double* ycom, double* polarization);

};

#endif /* PREDATORTRACKER_H */
