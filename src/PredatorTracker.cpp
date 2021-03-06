#include "PredatorTracker.h"

/* default parameter values */
const unsigned int DEFAULT_BG_THRESH = 60;
const double DEFAULT_MIN_BLOB_PERIMETER = 10;   
const double DEFAULT_MIN_BLOB_AREA = 10;        
const double DEFAULT_MAX_BLOB_PERIMETER = 1000; 
const double DEFAULT_MAX_BLOB_AREA = 1000;
const unsigned int DEFAULT_SEARCH_AREA_PADDING = 100;

/* default color to draw blobs */
const MT_Color DEFAULT_BLOB_COLOR = MT_Red;

/* number of past positions to use when estimating orientation */
const unsigned int N_hist = 5;

/* This is the state mapping used by the Unscented Kalman Filter
 * (UKF).  I.e.
 * x(t+1) = f(x(t), u(t) (control input), v(t) (process noise)
 *
 * This particular function is based on a model where heading and
 * speed remain constant unless they are changed by "random" noise.
 * In reality the fish are speeding up, slowing down, and turning as a
 * result of behavioral decisions, but without knowing what those
 * decisions were, the best we can do is model it as noise and guess
 * at the standard deviations.  In this model we don't use u_k, which
 * is fine in terms of the UKF.
 *
 * The k subscript is for time - e.g. x_k = x(kT) where T is the
 * sampling time
 */
static void fish_dynamics(const CvMat* x_k,
                          const CvMat* u_k,
                          const CvMat* v_k,
                          CvMat* x_kplus1)
{
    /* Assuming that the time-step is one frame rather than e.g.
     * 33 msec - I take the actual time step into account in
     * analysis. */
    double dT = 1.0;

    /* cvGetReal2D is useful to get an element of a 2D matrix
     * cvGetReal2D(x_k, i, j) gets the (i,k)^th element
     * Note that x_k is a 4x1 matrix here */
    double x = cvGetReal2D(x_k, 0, 0);
    double y = cvGetReal2D(x_k, 1, 0);
    double hdg = cvGetReal2D(x_k, 2, 0); /* heading [rad] */
    double spd = cvGetReal2D(x_k, 3, 0); /* speed */

    /* position(t + 1) = position(t) + dT*velocity */
    x += dT*spd*cos(hdg);
    y += dT*spd*sin(hdg);

    /* works just like cvGetReal2D */
    cvSetReal2D(x_kplus1, 0, 0, x);
    cvSetReal2D(x_kplus1, 1, 0, y);
    cvSetReal2D(x_kplus1, 2, 0, hdg);
    cvSetReal2D(x_kplus1, 3, 0, fabs(spd));

    /* this allows v_k to be a NULL pointer, in which case
     * this step is skipped */
    if(v_k)
    {
        /* simple additive noise: x(t+1) <- x(t+1) + noise */
        cvAdd(x_kplus1, v_k, x_kplus1);
    }
    
}

/* This is the measurement mapping used by the UKF, i.e.
 * z(t) = h(x(t), n(t))
 *
 * In this case, z is a vector with (x,y) position and heading and
 * noise is additive. */
static void fish_measurement(const CvMat* x_k,
                             const CvMat* n_k,
                             CvMat* z_k)
{
    cvSetReal2D(z_k, 0, 0, cvGetReal2D(x_k, 0, 0));
    cvSetReal2D(z_k, 1, 0, cvGetReal2D(x_k, 1, 0));
    cvSetReal2D(z_k, 2, 0, cvGetReal2D(x_k, 2, 0));

    /* as above, skip this step when n_k is null */
    if(n_k)
    {
        cvAdd(z_k, n_k, z_k);
    }
}

/* Using this function below to constrain the state estimate.  The
 * constraint is applied after the UKF correction step.
 *
 * Constraints applied:
 *   0 <= x <= frame (image) width
 *   0 <= y <= frame height
 *   0 <= speed <= 100 (px/frame)
 *   If x, y, heading, or speed are NaN, then
 *      a) if the corresponding estimate is valid, that number is used
 *      b) if the estimate is also NaN, x, y, and heading are set to
 *          0, speed is set to 0.1 
 */
static void constrain_state(CvMat* x_k,
                            CvMat* X_p,
                            IplImage* frame)
{
    double x = cvGetReal2D(x_k, 0, 0);
    double y = cvGetReal2D(x_k, 1, 0);
    double hdg = cvGetReal2D(x_k, 2, 0);
    double spd = cvGetReal2D(x_k, 3, 0);

    double x_p = cvGetReal2D(X_p, 0, 0);
    double y_p = cvGetReal2D(X_p, 1, 0);
    double hdg_p = cvGetReal2D(X_p, 2, 0);
    double spd_p = cvGetReal2D(X_p, 3, 0);

    /* MT_CLAMP(x, a, b) =
     *    x, if a <= x <= b,
     *    a, if x < a,
     *    b, if x > b
     */
    x = MT_CLAMP(x, 0, frame->width);
    y = MT_CLAMP(y, 0, frame->height);
    spd = MT_CLAMP(spd, 0, 100);

    /* MT_isnan(x) returns true if x is NaN */
    if(MT_isnan(x))
    {
        if(!MT_isnan(x_p))
        {
            x = x_p;
        }
        else
        {
            x = 0;
        }
    }
    if(MT_isnan(y))
    {
        if(!MT_isnan(y_p))
        {
            y = y_p;
        }
        else
        {
            y = 0;
        }
    }
    if(MT_isnan(hdg))
    {
        if(!MT_isnan(hdg_p))
        {
            hdg = hdg_p;
        }
        else
        {
            hdg = 0;
        }
    }
    if(MT_isnan(spd))
    {
        if(!MT_isnan(spd_p))
        {
            spd = spd_p;
        }
        else
        {
            spd = 0.1;            
        }
    }
    
    cvSetReal2D(x_k, 0, 0, x);
    cvSetReal2D(x_k, 1, 0, y);
    cvSetReal2D(x_k, 2, 0, hdg);
    cvSetReal2D(x_k, 3, 0, fabs(spd));
}

/* helper function.  Basic FIFO buffer with N_hist entries
 *
 * Given new values for x and y, do
 * if size of X < N_hist,
 *    X = [existing values.... x]
 * else (have N_hist values)
 *    drop the first value, shift all of the other values forward
 *     (i.e. X[k] = X[k+1]), and set the last value to x
 *
 * The same is done for Y. */
void rollHistories(std::vector<double>* X,
                   std::vector<double>* Y,
                   double x,
                   double y)
{
    if(X->size() < N_hist)
    {
        X->push_back(x);
        Y->push_back(y);
    }
    else
    {
        for(unsigned int i = 0; i < X->size()-1; i++)
        {
            X->at(i) = X->at(i+1);
            Y->at(i) = Y->at(i+1);
        }
        X->at(X->size()-1) = x;
        Y->at(Y->size()-1) = y;
    }
}

/* helper function.  Returns false if any element of M is either NaN
 * or larger in magnitude than max_val */
bool CvMatIsOk(const CvMat* M, double max_val = 1e10)
{
    double v;
    for(unsigned int i = 0; i < M->rows; i++)
    {
        for(unsigned int j = 0; j < M->cols; j++)
        {
            v = cvGetReal2D(M, i, j);
            if(MT_isnan(v) || fabs(v) > max_val)
            {
                return false;
            }
        }
    }
    return true;
}


/**********************************************************************
 * Tracker Class
 *********************************************************************/

/* Constructor - this gets called when we create a new instance of
 * this class.  It should initialize all of the member variables and
 * do whatever memory allocation is necessary. */
PredatorTracker::PredatorTracker(IplImage* ProtoFrame, unsigned int n_obj)
    : MT_TrackerBase(ProtoFrame),
      m_iBlobValThresh(DEFAULT_BG_THRESH),
	  m_iBlobAreaThreshLow(DEFAULT_MIN_BLOB_AREA),
	  m_iBlobAreaThreshHigh(DEFAULT_MAX_BLOB_AREA),
	  m_iSearchAreaPadding(DEFAULT_SEARCH_AREA_PADDING),
      m_iStartFrame(-1),
      m_iStopFrame(-1),
      m_pGSThresholder(            NULL                        ),
      m_pGYBlobber(NULL),
      m_pYABlobber(NULL),
      m_vpUKF(n_obj, NULL),
      m_dSigmaPosition(DEFAULT_SIGMA_POSITION),
      m_dSigmaHeading(DEFAULT_SIGMA_HEADING),
      m_dSigmaSpeed(DEFAULT_SIGMA_SPEED),
      m_dSigmaPositionMeas(DEFAULT_SIGMA_POSITION_MEAS),
      m_dSigmaHeadingMeas(DEFAULT_SIGMA_HEADING_MEAS),
      m_dPrevSigmaPosition(0),
      m_dPrevSigmaHeading(0),
      m_dPrevSigmaSpeed(0),
      m_dPrevSigmaPositionMeas(0),
      m_dPrevSigmaHeadingMeas(0),
      m_iMinFishPerimeter(10),
      m_iMinFishArea(10),
      m_iMaxFishPerimeter(100),
      m_iMaxFishArea(100),
      m_bShowBlobs(true),
      m_bShowTracking(true),
      m_dDt(0),
      m_iFrameCounter(0),
      m_iNObj(n_obj),
      m_iFrameHeight(0),
	  m_iFrameWidth(0)
{
    doInit(ProtoFrame);
}

/* Destructor - basically the opposite of the destructor, gets called
 * whenever an object is deleted or goes out of scope.  It should
 * de-allocate any memory that was allocated */
PredatorTracker::~PredatorTracker()
{
    if(m_pGSThresholder)
    {
        delete m_pGSThresholder;
    }

    if(m_pGYBlobber)
    {
        delete m_pGYBlobber;
    }
    if(m_pYABlobber)
    {
        delete m_pYABlobber;
    }

    /* cvReleaseMat deallocates matrices given to it */
    cvReleaseMat(&m_px0);
    cvReleaseMat(&m_pz);
    cvReleaseMat(&m_pQ);
    cvReleaseMat(&m_pR);

    /* MT_UKFFree frees up memory used by the UKF */
    for(unsigned int i = 0; i < m_iNObj; i++)
    {
        MT_UKFFree(&m_vpUKF[i]);
    }
}

/* this is the main initialization function and gets called by the
 * constructor.  It does memory allocation, etc. */
void PredatorTracker::doInit(IplImage* ProtoFrame)
{

	m_SearchArea = cvRect(0,0,0,0);

    /* Not using the built-in tracked objects functions - setting
     * this pointer to N ULL will ensure that the appropriate code is
     * disabled  */
    m_pTrackedObjects = NULL;

    /* It's always a good idea to initialize pointers to NULL so that
       other pieces of code can use e.g. if(p) to check for allocation */
    m_pGSFrame = NULL;
    m_pDiffFrame = NULL;
    m_pThreshFrame = NULL;
    m_pThreshFrameCopy = NULL;

    /* grab the frame height */
    m_iFrameHeight = ProtoFrame->height;
	m_iFrameWidth = ProtoFrame->width;

    /* Call the base class's doInit method.
     * This initializes variables to safe values and
     * calls doTrain to set the background frame,
     * frame sizes, and calls createFrames(). */
    MT_TrackerBase::doInit(ProtoFrame);

    /* resize all of our vectors. note that the std::vector object
       deallocates memory on its own, so we won't have to do that later */
    m_vdBlobs_X.resize(m_iNObj);
    m_vdBlobs_Y.resize(m_iNObj);          
    m_vdBlobs_Area.resize(m_iNObj);
    m_vdBlobs_Orientation.resize(m_iNObj);
    m_vdBlobs_MajorAxis.resize(m_iNObj);  
    m_vdBlobs_MinorAxis.resize(m_iNObj);  
    m_vdBlobs_Speed.resize(m_iNObj);
    m_viMatchAssignments.resize(m_iNObj);
    m_vdTracked_X.resize(m_iNObj);
    m_vdTracked_Y.resize(m_iNObj);
    m_vdTracked_Heading.resize(m_iNObj);
    m_vdTracked_Speed.resize(m_iNObj);

    m_vdFish_X.resize(0);
    m_vdFish_Y.resize(0);
    m_vdFish_Orientation.resize(0);
    m_vdFish_Area.resize(0);

    m_vdHistories_X.resize(m_iNObj);
    m_vdHistories_Y.resize(m_iNObj);
    /* there is an X and Y history for each object.  They need to be
       initially zero in length so that we can fill them up with real
       data as it becomes available. */
    for(unsigned int i = 0; i < m_iNObj; i++)
    {
        m_vdHistories_X[i].resize(0);
        m_vdHistories_Y[i].resize(0);        
    }


    /* sets up the frames that are available in the "view" menu */
    m_pTrackerFrameGroup = new MT_TrackerFrameGroup();
    m_pTrackerFrameGroup->pushFrame(&m_pDiffFrame,      "Diff Frame");
    m_pTrackerFrameGroup->pushFrame(&m_pThreshFrame,    "Threshold Frame");

    /* Data group and Data report setup.
     *
     * This is how the tracker class interacts with the GUI.  When the
     * tracker is initialized, the GUI pulls the contents of
     * m_vDataGroups and automatically creates dialog boxes enabling
     * the user to view/edit these values.
     *
     * An MT_DataGroup is a list of parameters and can be read/write -
     * e.g. minimum blob size, or drawing color.
     * An MT_DataReport is a list of vectors of numbers and is
     * read-only.  e.g. a report of the found blobs positions.
     */
    
    /* set up the parameter groups for parameter modification, etc. */
    /* first group is for blob tracking parameters */
    MT_DataGroup* dg_blob = new MT_DataGroup("Blob Tracking Parameters");
    dg_blob->AddUInt("Difference Threshold", /* parameter name */
                     &m_iBlobValThresh,      /* pointer to variable */
                     MT_DATA_READWRITE,      /* read-only or not */
                     0,                      /* minimum value */
                     255);                   /* maximum value */
    dg_blob->AddUInt("Min Blob Area",
                    &m_iBlobAreaThreshLow,
                    MT_DATA_READWRITE,
                    0);
    dg_blob->AddUInt("Max Blob Area",
                    &m_iBlobAreaThreshHigh,
                    MT_DATA_READWRITE,
                    0);
	dg_blob->AddUInt("Search Area Padding",
		             &m_iSearchAreaPadding,
					 MT_DATA_READWRITE,
					 0);
    dg_blob->AddDouble("Position Disturbance Sigma",
                       &m_dSigmaPosition,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Heading Disturbance Sigma",
                       &m_dSigmaHeading,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Speed Disturbance Sigma",
                       &m_dSigmaSpeed,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Position Measurement Sigma",
                       &m_dSigmaPositionMeas,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Heading Measurement Sigma",
                       &m_dSigmaHeadingMeas,
                       MT_DATA_READWRITE,
                       0);

    MT_DataGroup* dg_fish = new MT_DataGroup("Fish Blobbing Parameters");
    dg_fish->AddInt("Min Perimeter",
                    &m_iMinFishPerimeter,
                    MT_DATA_READWRITE,
                    0);
    dg_fish->AddInt("Min Area",
                    &m_iMinFishArea,
                    MT_DATA_READWRITE,
                    0);
    dg_fish->AddInt("Max Perimeter",
                    &m_iMaxFishPerimeter,
                    MT_DATA_READWRITE,
                    0);
    dg_fish->AddInt("Max Area",
                    &m_iMaxFishArea,
                    MT_DATA_READWRITE,
                    0);

    MT_DataGroup* dg_draw = new MT_DataGroup("Drawing Options");
    dg_draw->AddBool("Show blob arrows", &m_bShowBlobs);
    dg_draw->AddBool("Show tracking arrows", &m_bShowTracking);
    
    /* now stuff the parameter groups into m_vDataGroups, which
     * MT_TrackerBase will automagically report to the GUI */
    m_vDataGroups.resize(0);
    m_vDataGroups.push_back(dg_blob);
    m_vDataGroups.push_back(dg_fish);
    m_vDataGroups.push_back(dg_draw);
    
    MT_DataReport* dr_tracked = new MT_DataReport("Tracked data");
    dr_tracked->AddDouble("X", &m_vdTracked_X);
    dr_tracked->AddDouble("Y", &m_vdTracked_Y);
    dr_tracked->AddDouble("Hdg", &m_vdTracked_Heading);
    dr_tracked->AddDouble("Spd", &m_vdTracked_Speed);

	MT_DataReport* dr_blobs = new MT_DataReport("Blob Info");
	dr_blobs->AddDouble("X", &m_vdBlobs_X);
	dr_blobs->AddDouble("Y", &m_vdBlobs_Y);
	dr_blobs->AddDouble("Area", &m_vdBlobs_Area);

    m_vDataReports.resize(0);
    m_vDataReports.push_back(dr_blobs);
	m_vDataReports.push_back(dr_tracked);

}

void PredatorTracker::doTrain(IplImage* frame)
{
	m_iFrameWidth = frame->width;
	m_iFrameHeight = frame->height;
	CvSize fsize = cvSize(m_iFrameWidth, m_iFrameHeight);

	MT_TrackerBase::doTrain(frame);

	if(m_pGSThresholder)
	{
		delete m_pGSThresholder;
	}

	m_pGSThresholder = new MT_GSThresholder(BG_frame);
    /* The thresholder manages these frames, but by grabbing pointers
       to them we can pass them to the GUI. */
    m_pGSFrame = m_pGSThresholder->getGSFrame();
    m_pDiffFrame = m_pGSThresholder->getDiffFrame();
    m_pThreshFrame = m_pGSThresholder->getThreshFrame();

    m_pThreshFrameCopy = cvCloneImage(m_pThreshFrame);

}

/* This gets called by MT_TrackerBase::doInit.  I use it here more to
 * allocate other bits of memory - a lot of this could actually go
 * into doInit, but there's no real harm */
void PredatorTracker::createFrames()
{
    /* this makes sure that the BG_frame is created */
    MT_TrackerBase::createFrames();

	//m_pGSThresholder = NULL;
    /* Create the Thresholder and Blobber objects */
    //m_pGSThresholder = new MT_GSThresholder(BG_frame);
    m_pGYBlobber = new GYBlobber(m_iNObj);

    m_pYABlobber = new YABlobber();
    
    /* Initialize the Hungarian Matcher */
    m_HungarianMatcher.doInit(m_iNObj);

    /* Initialize some of the matrices used for tracking */
    m_pQ = cvCreateMat(4, 4, CV_64FC1);
    m_pR = cvCreateMat(3, 3, CV_64FC1);
    cvSetIdentity(m_pQ);
    cvSetIdentity(m_pR);
    m_px0 = cvCreateMat(4, 1, CV_64FC1);
    m_pz = cvCreateMat(3, 1, CV_64FC1);

    /* Create the UKF objects */
    m_vpUKF.resize(m_iNObj);
    for(unsigned int i = 0; i < m_iNObj; i++)
    {
        m_vpUKF[i] = MT_UKFInit(4, 3, 0.1); /* 4 states, 3
                                        * measurements, 0.1 is a parameter */
    }

} /* endof createFrames */

/* this gets called during the destructor */
void PredatorTracker::releaseFrames()
{

    if(m_pThreshFrameCopy)
    {
        cvReleaseImage(&m_pThreshFrameCopy);
    }
    
    /* Nothing really to do here.  BG_Frame is managed by the base
     * class and the GS, Diff, and Thresh frames are managed by
     * m_pGSThresholder */
}

/* Initialize a data file for output.  Gets called from the GUI or
 * command line */
void PredatorTracker::initDataFile()
{
    /* An XDF can store parameters in XML format and manages data
       output to other files that are keyed in the master XML file.

       These can then be read by e.g. MATLAB */
    char v[10];
    sprintf(v, "%d", m_iStartFrame);
    m_XDF.writeParameterToXML("Starting Frame", v);
    sprintf(v, "%d", m_iStopFrame);
    m_XDF.writeParameterToXML("Stopping Frame", v);

    m_XDF.addDataStream("Blob X", "blob_x.dat");
    m_XDF.addDataStream("Blob Y", "blob_y.dat");
    m_XDF.addDataStream("Blob Area", "blob_area.dat");
    m_XDF.addDataStream("Blob Orientation", "blob_orientation.dat");
    m_XDF.addDataStream("Blob Major Axis", "blob_major.dat");
    m_XDF.addDataStream("Blob Minor Axis", "blob_minor.dat");

    m_XDF.addDataStream("Tracked X", "tracked_x.dat");
    m_XDF.addDataStream("Tracked Y", "tracked_y.dat");    
    m_XDF.addDataStream("Tracked Heading", "tracked_heading.dat");    
    m_XDF.addDataStream("Tracked Speed", "tracked_speed.dat");
    
    MT_TrackerBase::initDataFile();
}

/* a helper function to write data to a file */
void PredatorTracker::writeData()
{
    /* the XDF object handles writing data to the right files - all we
       have to do is pass the data as vectors */
    m_XDF.writeData("Blob X"           , m_vdBlobs_X); 
    m_XDF.writeData("Blob Y"           , m_vdBlobs_Y); 
    m_XDF.writeData("Blob Area"        , m_vdBlobs_Area); 
    m_XDF.writeData("Blob Orientation" , m_vdBlobs_Orientation); 
    m_XDF.writeData("Blob Major Axis"  , m_vdBlobs_MajorAxis); 
    m_XDF.writeData("Blob Minor Axis"  , m_vdBlobs_MinorAxis); 

    m_XDF.writeData("Tracked X"        , m_vdTracked_X); 
    m_XDF.writeData("Tracked Y"        , m_vdTracked_Y); 
    m_XDF.writeData("Tracked Heading"  , m_vdTracked_Heading); 
    m_XDF.writeData("Tracked Speed"    , m_vdTracked_Speed); 
}

/* Main tracking function - gets called by MT_TrackerFrameBase every
 * time step when the application is not paused. */
void PredatorTracker::doTracking(IplImage* frame)
{
    /* time-keeping, if necessary
     * NOTE this is not necessary for keeping track of frame rate */
    static double t_prev = MT_getTimeSec();
    double t_now = MT_getTimeSec();
    m_dDt = t_now - t_prev;
    t_prev = t_now;

    /* keeping track of the frame number, if necessary */
    m_iFrameCounter++;

    /* This checks every time step to see if the UKF parameters have
       changed and modifies the UKF structures accordingly.  This will
       also get called the first time through b/c the "Prev" values get
       set to zero initially.   There may be a more efficient way to do
       this, but because the values need to be embedded into the CvMat
       objects I'm not sure how else to do it. */ 
    if(
        m_dSigmaPosition != m_dPrevSigmaPosition ||
        m_dSigmaHeading != m_dPrevSigmaHeading ||
        m_dSigmaSpeed != m_dPrevSigmaSpeed ||
        m_dSigmaPositionMeas != m_dPrevSigmaPositionMeas ||
        m_dSigmaHeadingMeas != m_dPrevSigmaHeadingMeas
        )
    {
        /* these are the diagonal entries of the "Q" matrix, which
           represents the variances of the process noise.  They're
           modeled here as being independent and uncorrellated. */
        cvSetReal2D(m_pQ, 0, 0, m_dSigmaPosition*m_dSigmaPosition);
        cvSetReal2D(m_pQ, 1, 1, m_dSigmaPosition*m_dSigmaPosition);
        cvSetReal2D(m_pQ, 2, 2, m_dSigmaHeading*m_dSigmaHeading);
        cvSetReal2D(m_pQ, 3, 3, m_dSigmaSpeed*m_dSigmaSpeed);        

        /* these are the diagonal entries of the "R matrix, also
           assumed to be uncorrellated. */
        cvSetReal2D(m_pR, 0, 0, m_dSigmaPositionMeas*m_dSigmaPositionMeas);
        cvSetReal2D(m_pR, 1, 1, m_dSigmaPositionMeas*m_dSigmaPositionMeas);
        cvSetReal2D(m_pR, 2, 2, m_dSigmaHeadingMeas*m_dSigmaHeadingMeas);

        /* this step actually copies the Q and R matrices to the UKF
           and makes sure that it's internals are properly initialized -
           it's set up to handle the fact that the sizes of these
           matrices could have changed. */
        for(unsigned int i = 0; i < m_iNObj; i++)
        {
            MT_UKFCopyQR(m_vpUKF[i], m_pQ, m_pR);
        }
    }

    /* Grayscale Thresholding
     *    - Takes as input the current frame, a threshold value, and
     *        optionally a mask image.
     *    - Has a background frame (gotten from initialization, but
     *        can be reset at any time), usually obtained beforehand
     *        by averaging a fixed number (say, 100) frames from the video
     *        source.
     *    - Calculates a difference frame D.  D is the current frame
     *        minus the background frame with all negative entries set to
     *        zero.  Therefore anything in the current frame that is
     *        darker than the background frame (after both are converted
     *        to grayscale) shows up in the difference frame.  If the
     *        ROI / mask frame is present, difference frame is set to zero
     *        wherever the mask frame is zero.
     *    - Thresholds the difference frame to obtain the threshold
     *        frame, T.  T is 255 wherever D > the threshold value.
     *
     * The operation returns an MT_SparseBinaryImage, which is
     * basically a list of pixels that are nonzero.  We don't
     * end up using the sparse image - instead we use the full
     * threshold image, which is pointed to by m_pThreshFrame.
     */       
    MT_SparseBinaryImage b = m_pGSThresholder->threshToBinary(frame,
                                                              m_iBlobValThresh,
                                                              ROI_frame,
															  MT_THRESH_DARKER);

    /* make sure these pointers are updated */
    m_pGSFrame = m_pGSThresholder->getGSFrame();
    m_pDiffFrame = m_pGSThresholder->getDiffFrame();
    m_pThreshFrame = m_pGSThresholder->getThreshFrame();



    /* Blobbing with George Young's Mixture-of-Gaussians blobber
     *
     * In a nutshell, this looks for a fixed number of ellipsoidal
     * objects in the thresholded frame, then returns a bunch of
     * objects containing information about what it found -
     * positions, orientations, areas, etc. 
     * 
     */
	m_pGYBlobber->m_iBlob_area_thresh_low = m_iBlobAreaThreshLow;
	m_pGYBlobber->m_iBlob_area_thresh_high = m_iBlobAreaThreshHigh;
	m_pGYBlobber->setSearchArea(m_SearchArea);
    std::vector<GYBlob> blobs = m_pGYBlobber->findBlobs(m_pThreshFrame);

    cvCopy(m_pThreshFrame, m_pThreshFrameCopy);
    std::vector<YABlob> fish = m_pYABlobber->FindBlobs(m_pThreshFrameCopy,
                                           m_iMinFishPerimeter,
										   m_iMinFishArea,
										   m_iMaxFishPerimeter,
										   m_iMaxFishArea);

    m_iNFishFound = 0;
    m_dFishCOMx = m_dFishCOMy = m_dFishPolarization = m_dTotalFishArea = 0;
    
    m_vdFish_X.resize(0);  m_vdFish_Y.resize(0);
    m_vdFish_Orientation.resize(0);  m_vdFish_Area.resize(0);
    
    double x, y;
    const double dist_thresh = 25.0;
    double pc, ps;
    pc = ps = 0;

    for(unsigned int i = 0; i < fish.size(); i++)
    {
        x = fish[i].COMx;
        y = fish[i].COMy;
        bool isa_robot = false;

        for(unsigned int j = 0; j < blobs.size(); j++)
        {
            double xr = blobs[j].m_dXCentre;
            double yr = blobs[j].m_dYCentre;
            
            if(fabs(x - xr) < dist_thresh &&
               fabs(y - yr) < dist_thresh)
            {
                isa_robot = true;
                break;
            }
        }
        if(isa_robot)
        {
            continue;
        }
        
        m_vdFish_X.push_back(x);
        m_vdFish_Y.push_back(y);
        m_vdFish_Orientation.push_back(MT_DEG2RAD*fish[i].orientation);
        m_vdFish_Area.push_back(fish[i].area);

        m_dFishCOMx += x;
        m_dFishCOMy += y;
        m_dTotalFishArea += fish[i].area;

        pc += cos(fish[i].orientation);
        ps += sin(fish[i].orientation);
        
        m_iNFishFound++;
    }

    m_dFishXX = 0;
    m_dFishYY = 0;
    m_dFishXY = 0;
    
    if(m_iNFishFound > 0)
    {
        double avg_fac = 1.0/((double) m_iNFishFound);
        m_dFishCOMx *= avg_fac;
        m_dFishCOMy *= avg_fac;
        m_dFishPolarization = sqrt(pc*pc + ps*ps)*avg_fac;
        for(unsigned int i = 0; i < m_iNFishFound; i++)
        {
            double dx = m_vdFish_X[i] - m_dFishCOMx;
            double dy = m_vdFish_Y[i] - m_dFishCOMy;
            m_dFishXX += dx*dx;
            m_dFishYY += dy*dy;
            m_dFishXY += dx*dy;
        }
        m_dFishXX *= avg_fac;
        m_dFishXY *= avg_fac;
        m_dFishYY *= avg_fac;
    }

    /* Matching step.
     *
     * After the blobbing step, we have N position measurements that
     * we need to match up with N previously estimated positions.
     * I.e. We need to assign measurement[j] to object[i], but we
     * don't know which j goes with which i.  This is the job of the
     * matcher; we load it up with a matrix where the [j,i] position
     * is the squared distance between measurement[j] and object[i].
     * The matcher figures out the optimal assignment between j's and
     * i's to minimize the total sum of squared distances.  It's
     * called a Hungarian matcher because it uses something called the
     * "Hungarian Algorithm" to solve the optimization problem.
     *
     * Note that the matrix need not hold squared distances.  It could
     * hold anything representing the "cost" of matching
     * measurement[j] with object[i].
     * 
     */
    if(m_iFrameCounter > 1 && m_iNObj > 1)  /* if there's only one object, */
    {                                       /* then this is unnecessary    */
        double dx, dy;
        for(unsigned int i = 0; i < m_iNObj; i++)
        {
            for(unsigned int j = 0; j < m_iNObj; j++)
            {
                dx = (m_vdBlobs_X[j] - blobs[i].m_dXCentre);
                dy = (m_vdBlobs_Y[j] - blobs[i].m_dYCentre);
                m_HungarianMatcher.setValue(j, i, dx*dx+dy*dy);
            }
        }
        m_HungarianMatcher.doMatch(&m_viMatchAssignments);
    }
    else
    {
        /* on the first frame, there's no history to compare to, so
           we'll just keep the order given by the blobber, i.e. i = j. */
        m_viMatchAssignments.resize(m_iNObj);
        for(unsigned int i = 0; i < m_iNObj; i++)
        {
            m_viMatchAssignments[i] = i;
        }
    }

    /* Tracking / UKF / State Estimation
     *
     * Now that we've got the mapping of which measurement goes with
     * which object, we need to feed the measurements into the UKF in
     * order to obtain a state estimate.
     *
     * This is a loop over each object we're tracking. 
     */
    unsigned int j;
	MT_BoundingBox object_limits;
    for(unsigned int i = 0; i< m_iNObj; i++)
    {
        /* The index of the optimal measurement as determined by the
           matching algorithm. */
        j = m_viMatchAssignments[i];

        /* Copy the raw blob data to the output vectors -
         *   It wasn't strictly necessary to use the matching for
         *   this, but it keeps the trajectories aligned when we look
         *   at the raw data. */
        m_vdBlobs_X[i] = blobs[j].m_dXCentre;
        m_vdBlobs_Y[i] = blobs[j].m_dYCentre;
        m_vdBlobs_Area[i] = blobs[j].m_dArea;
        m_vdBlobs_Orientation[i] = blobs[j].m_dOrientation;
        m_vdBlobs_MajorAxis[i] = blobs[j].m_dMajorAxis;
        m_vdBlobs_MinorAxis[i] = blobs[j].m_dMinorAxis;
        m_vdBlobs_Speed[i] = 0;  /* not getting speed from the blobs */

        /* update the position histories - see definition of
           rollHistories */
        rollHistories(&m_vdHistories_X[i],
                      &m_vdHistories_Y[i],
                      m_vdBlobs_X[i],
                      m_vdBlobs_Y[i]);

        /* we could throw out a measurement and use the blob
           state as an estimate for various reasons.  On the first
           frame we want to set the initial state, so we flag the
           measurement as invalid */
        bool valid_meas = m_iFrameCounter > 1;

        /* if any state is NaN, reset the UKF
         * This shouldn't happen anymore, but it's a decent safety
         * check.  It could probably be omitted if we want to
         * optimize for speed... */
        if(m_iFrameCounter > 1 &&
           (!CvMatIsOk(m_vpUKF[i]->x) ||
            !CvMatIsOk(m_vpUKF[i]->P)))
        {
            MT_UKFFree(&(m_vpUKF[i]));
            m_vpUKF[i] = MT_UKFInit(4, 3, 0.1);
            MT_UKFCopyQR(m_vpUKF[i], m_pQ, m_pR);
            valid_meas = false;
        }

        /* if we're going to accept this measurement */
        if(valid_meas)
        {
            /* UKF prediction step, note we use function pointers to
               the fish_dynamics and fish_measurement functions defined
               above.  The final parameter would be for the control input
               vector, which we don't use here so we pass a NULL pointer */
            MT_UKFPredict(m_vpUKF[i],
                          &fish_dynamics,
                          &fish_measurement,
                          NULL);

            /* Angle Madness
             *
             * Getting the orientation angle is one of the trickiest
             * parts of tracking.  There's a couple reasons for this.
             *
             * 1. Measurements of orientation are
             *        highly prone to pointing in the opposite
             *        direction (confusing the head for the tail).
             *
             * 2. Orientation measurements will be fixed in a range of
             *        [-\pi, \pi] or [0, 2\pi], etc, which creates
             *        discontinuities in the measurement.  That is,
             *        suppose your object is moving to the left in the
             *        image plane, so its heading is hovering right
             *        around \pi.  The measurement could go, in one
             *        time step, from \pi - 0.001 to -\pi + 0.001.
             *        This is a bad thing.
             *
             * Here's how we handle these problems.
             *
             * 1.   A) By default, we trust the orientation measurement.
             *            It uses an algorithm that should be able to
             *            distinguish the head from the tail.
             *      B) We keep a history of past positions.  If the
             *            object has moved far enough in the last few
             *            time steps, we use the use the displacement
             *            vector as a cue.  Otherwise we use the last
             *            known orientation estimate as a cue.
             *      C) The orientation measurement is compared to the
             *            cue.  If the two are similar (see below),
             *            the measurement is used as-is.  If they are
             *            not similar, 180 degrees is subtracted from
             *            the measurement.
             *
             * 2.  A) A shortest-arc difference between the
             *            measurement and last known orientation is
             *            calculated.
             *     B) Due to the way we calculate
             *            the orientation measurement, this difference
             *            is most likely less than 90 degrees in
             *            magnitude.
             *     C) The actual measurement given
             *            to the UKF is the current orientation
             *            estimate plus the calculated difference.
             * 
             */
            double th;
            bool a = false;
			double th_meas;
            if(m_vdHistories_X[i].size() == N_hist)
            {
                double dx = m_vdHistories_X[i].at(N_hist-1) -
                    m_vdHistories_X[i].at(0);
                double dy = m_vdHistories_Y[i].at(N_hist-1) -
                    m_vdHistories_Y[i].at(0);

                double min_pix_move = 2.0;

                /* if the object has moved far enough */
                if(dx*dx + dy*dy > min_pix_move*min_pix_move)
                {
                    /* negative sign accounts for screen coordinates */
                    th = atan2(-dy,dx);
                    a = true;
                }
            }

            /* if we don't have enough history or didn't move far
               enough, use the previous orientation */
            if(!a)
            {
                th = cvGetReal2D(m_vpUKF[i]->x, 2, 0);
            }

            /* this is how we determine if the two angles are "close"
               - the magnitude of the order parameter, which is
               equivalent to the length of the average phasor, i.e.
               pth = 0.5*|e^{i*th} + e^{i*orientation measurement}|
            */
            double pth = fabs(
                cos(0.5*(th - MT_DEG2RAD*m_vdBlobs_Orientation[i]))
                );

            /* sqrt(2)/2 is what we'd get if the angles differ by 90
               degrees */
            if(pth < 0.7)  /* a little less than sqrt(2)/2 */
            {
                /* looks like the orientation is opposite of what it
                   should be, so subtract 180 degrees. */
                th_meas = m_vdBlobs_Orientation[i] - 180.0;
            }
			else
			{
				th_meas = m_vdBlobs_Orientation[i];
			}

            /* taking asin(sin(angle)) ensures that |angle| < pi,
             * i.e. we get the shortest-arc distance  */
            double dth = asin(sin(MT_DEG2RAD*th_meas
                                    - cvGetReal2D(m_vpUKF[i]->x, 2, 0)));

            /* finally, set the measurement vector z */
            cvSetReal2D(m_pz, 0, 0, m_vdBlobs_X[i]);
            cvSetReal2D(m_pz, 1, 0, m_vdBlobs_Y[i]);
            cvSetReal2D(m_pz, 2, 0, cvGetReal2D(m_vpUKF[i]->x, 2, 0) + dth);
            MT_UKFSetMeasurement(m_vpUKF[i], m_pz);

            /* then do the UKF correction step, which accounts for the
               measurement */
            MT_UKFCorrect(m_vpUKF[i]);

            /* then constrain the state if necessary - see function
             * definition above */
            constrain_state(m_vpUKF[i]->x, m_vpUKF[i]->x1, frame);            

        }
        else  
        {
           /* measurement was not valid -> set the state.
             this happens on the first time step as well; this is
             where the initial conditions of the UKF are set. */
            cvSetReal2D(m_px0, 0, 0, m_vdBlobs_X[i]);
            cvSetReal2D(m_px0, 1, 0, m_vdBlobs_Y[i]);
            cvSetReal2D(m_px0, 2, 0, MT_DEG2RAD*m_vdBlobs_Orientation[i]);
            cvSetReal2D(m_px0, 3, 0, 0.1);
            MT_UKFSetState(m_vpUKF[i], m_px0);
        }

        /* grab the state estimate and store it in variables that will
           make it convenient to save it to a file. */
        CvMat* x = m_vpUKF[i]->x;

        m_vdTracked_X[i] = cvGetReal2D(x, 0, 0);
        m_vdTracked_Y[i] = cvGetReal2D(x, 1, 0);
        m_vdTracked_Heading[i] = cvGetReal2D(x, 2, 0);
        m_vdTracked_Speed[i] = cvGetReal2D(x, 3, 0);

	    object_limits.ShowX(m_vdTracked_X[i]);
		object_limits.ShowY(m_vdTracked_Y[i]);

        /* If we wanted the predicted state, this would be how to get
           it */
        /* CvMat* xp = m_vpUKF[i]->x1; */
    }

	if(object_limits.xmin == object_limits.xmax == 0 ||
		object_limits.ymin == object_limits.ymax == 0)
	{
		object_limits.ShowX(0);
		object_limits.ShowY(0);
		object_limits.ShowX(m_iFrameWidth);
		object_limits.ShowY(m_iFrameHeight);
	}

	double cx = 0.5*(object_limits.xmin + object_limits.xmax);
	double cy = 0.5*(object_limits.ymin + object_limits.ymax);
	double w = object_limits.xmax - object_limits.xmin + m_iSearchAreaPadding;
	double h = object_limits.ymax - object_limits.ymin + m_iSearchAreaPadding;
	m_SearchArea = cvRect(MT_CLAMP(cx - 0.5*w, 0, m_iFrameWidth),
		                  MT_CLAMP(cy - 0.5*h, 0, m_iFrameHeight),
						  MT_CLAMP(w, 0, m_iFrameWidth),
						  MT_CLAMP(h, 0, m_iFrameHeight));

    /* write data to file */
    writeData();

}

std::vector<double> PredatorTracker::getPredatorState(unsigned int i)
{
	std::vector<double> r;
	r.resize(0);

	if(i >= m_iNObj || i >= m_vdTracked_X.size())
	{
		fprintf(stderr, "PredatorTracker Error:  State request out of range; returning empty state.\n");
		return r;
	}

	r.resize(PREDATOR_STATE_SIZE);
	r[PREDATOR_STATE_X] = m_vdTracked_X[i];
	r[PREDATOR_STATE_Y] = m_vdTracked_Y[i];
	r[PREDATOR_STATE_HEADING] = m_vdTracked_Heading[i];
	r[PREDATOR_STATE_SPEED] = m_vdTracked_Speed[i];

	return r;
}

void PredatorTracker::getFishInfo(int* nfish,
                                  double* xcom,
                                  double* ycom,
                                  double* polarization)
{
    *nfish = m_iNFishFound;
    *xcom = m_dFishCOMx;
    *ycom = m_dFishCOMy;
    *polarization = m_dFishPolarization;
}


/* Drawing function - gets called by the GUI
 * All of the drawing is done with OpenGL */
void PredatorTracker::doGLDrawing(int flags)
{
    /* MT_R3 is a 3-vector class, used here for convenience */
    MT_R3 blobcenter;

    /* m_bShowBlobs is a boolean in the "Drawing Options" data group.
       If the user selects "Drawing Options" from the "Tracking" menu,
       a dialog will pop up with a check box labeled "Show blob arrows",
       and its state will be linked (via a pointer) to the value of this
       variable */
    if(m_bShowBlobs)
    {
        for (unsigned int i = 0; i < m_vdBlobs_X.size(); i++)
        {

            /* note that we have to subtract y from the frame_height
               to account for the window coordinates originating from the
               bottom left but the image coordinates originating from
               the top left */
            blobcenter.setx(m_vdBlobs_X[i]);
            blobcenter.sety(m_iFrameHeight - m_vdBlobs_Y[i]);
            blobcenter.setz( 0 );

            /* draws an arrow using OpenGL */
            MT_DrawArrow(blobcenter,  // center of the base of the arrow
                         20.0,        // arrow length (pixels)
                         MT_DEG2RAD*m_vdBlobs_Orientation[i], // arrow angle
                         MT_Primaries[(i+1) % MT_NPrimaries], // color
                         1.0 // fixes the arrow width
                );    
        }
    }

    /* essentially the same as above, but with the tracked positions
       instead of the blob positions */
    if(m_bShowTracking)
    {
        for (unsigned int i = 0; i < m_vdTracked_X.size(); i++)
        {

            blobcenter.setx(m_vdTracked_X[i]);
            blobcenter.sety(m_iFrameHeight - m_vdTracked_Y[i]);
            blobcenter.setz( 0 );

            MT_DrawArrow(blobcenter,
                         25.0, 
                         m_vdTracked_Heading[i],
                         MT_Primaries[(i+1) % MT_NPrimaries],
                         1.0 
                );    
        }
    }

	MT_DrawRectangle(m_SearchArea.x, m_iFrameHeight - m_SearchArea.y - m_SearchArea.height, m_SearchArea.width, m_SearchArea.height);

    if(m_iNFishFound)
    {
        for(unsigned int i = 0; i < m_iNFishFound; i++)
        {
            blobcenter.setx(m_vdFish_X[i]);
            blobcenter.sety(m_iFrameHeight - m_vdFish_Y[i]);
            blobcenter.setz(0);

            MT_DrawArrow(blobcenter,
                         8.0, 
                         m_vdFish_Orientation[i],
                         MT_Blue,
                         1.0 
                );

        }

        MT_DrawCircle(m_dFishCOMx, m_iFrameHeight - m_dFishCOMy, MT_Orange, 8.0);
        blobcenter.setx(m_dFishCOMx);
        blobcenter.sety(m_iFrameHeight - m_dFishCOMy);
        blobcenter.setz(0);

        if(m_iNFishFound > 2)
        {

            double o = -0.5*atan(2.0*m_dFishXY/fabs(m_dFishXX - m_dFishYY));
            double a = 0.5*(m_dFishXX + m_dFishYY);
            double b = 0.5*sqrt(fabs(4.0*m_dFishXY*m_dFishXY
                                     + ((m_dFishXX - m_dFishYY)
                                        *(m_dFishXX - m_dFishYY))));

            if(!(MT_isnan(o) || MT_isnan(a) || MT_isnan(b)))
            {
                MT_DrawEllipse(blobcenter,
                               2.0*sqrt(fabs(a + b)),
                               2.0*sqrt(fabs(a - b)),
                               o,
                               MT_Orange);
            }
        }
    }

}

