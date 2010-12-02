#include "PredatorRobot.h"

/* Predator doesn't use any handshaking on the COM port */
const bool PREDATOR_HANDSHAKING = false;

/* "Dummy" Constructor -- used for debugging only */
Predator::Predator()
    : MT_RobotBase("Anonymous"),
      m_COMPort("stderr", PREDATOR_HANDSHAKING),
      m_sPort("stderr"),
      m_bIsConnected(false)
{
    doCommonInit();
}

/* Normal ctor -- give a COM port and a display name, e.g. "Predator 1"
 * - every robot should have a unique name */
Predator::Predator(const char* onComPort, const char* name)
    : MT_RobotBase(onComPort, name),  /* base class init needs com
                                         port and name */
      /* com port initialization with the right handshaking and baud
         rate set up */
      m_COMPort(onComPort, PREDATOR_HANDSHAKING, MT_Baud4800),
      m_sPort(onComPort),    /* string name of the com port */
      m_bIsConnected(false)  /* initialize as not connected (will get
                              * set on calling IsConnected */
{
    /* common Predator initialization */
    doCommonInit();
}

/* Predator initialization */
Predator::~Predator()
{

	SafeStop();

}

void Predator::SafeStop()
{
	if(IsConnected())
	{
		double t0 = MT_getTimeSec();
		SendSpeed(0);
		while(MT_getTimeSec() - t0 < 0.1){};
		SendVerticalSpeed(0);
		while(MT_getTimeSec() - t0 < 0.2){};
		SendTurn(0);
	}
}

void Predator::doCommonInit()
{
    /* set default parameter values - these will get loaded from XML
     * if available */
    m_dMaxSpeed = PREDATOR_MAX_SPEED;
    m_dMaxVertSpeed = PREDATOR_MAX_VERT_SPEED;
    m_dMaxTurn = PREDATOR_MAX_TURN;
    m_dSpeedDeadBand = PREDATOR_DEFAULT_SPEED_DEADBAND;
    m_dTurnDeadBand = PREDATOR_DEFAULT_TURN_DEADBAND;
    m_dVertSpeed = 0;

    /* Adding these as an MT_DataGroup enables GUI adjustment and
     * persistence via XML.  The XML file is keyed on the robot name,
     * so the robot name needs to be unique */
	m_vdState.resize(PREDATOR_STATE_SIZE, 0.0),
	m_vdControls.resize(PREDATOR_CONTROL_SIZE, 0.0),

    m_pParameters = new MT_DataGroup(std::string(m_sName));
    m_pParameters->AddDouble("Max Speed",
                             &m_dMaxSpeed,
                             MT_DATA_READWRITE,
                             0,
                             PREDATOR_MAX_SPEED);
    m_pParameters->AddDouble("Vertical Speed",
                             &m_dVertSpeed,
                             MT_DATA_READWRITE,
                             0,
                             PREDATOR_MAX_VERT_SPEED);
    m_pParameters->AddDouble("Max Vertical Speed",
                             &m_dMaxVertSpeed,
                             MT_DATA_READWRITE,
                             0,
                             PREDATOR_MAX_VERT_SPEED);
    m_pParameters->AddDouble("Max Turn",
                             &m_dMaxTurn,
                             MT_DATA_READWRITE,
                             0,
                             PREDATOR_MAX_TURN);
    m_pParameters->AddDouble("Speed Deadband",
                             &m_dSpeedDeadBand,
                             MT_DATA_READWRITE,
                             0,
                             1.0);
    m_pParameters->AddDouble("Turning Deadband",
                             &m_dTurnDeadBand,
                             MT_DATA_READWRITE,
                             0,
                             1.0);

}

/* function to construct a vertical speed command.  speed is a signed
 * double number */
void Predator::Update(std::vector<double> state)
{
	SetState(state);
}

void Predator::SetState(std::vector<double> state)
{
	if(state.size() != PREDATOR_STATE_SIZE)
	{
		return;
	}

	m_vdState = state;
}

std::vector<double> Predator::GetState()
{
	return m_vdState;
}

double Predator::GetX() const
{
	return m_vdState[PREDATOR_STATE_X];
}

double Predator::GetY() const
{
	return m_vdState[PREDATOR_STATE_Y];
}

double Predator::GetTheta() const
{
	return m_vdState[PREDATOR_STATE_ORIENTATION];
}

void Predator::SetControl(std::vector<double> u)
{
	if(u.size() != PREDATOR_CONTROL_SIZE)
	{
		return;
	}

	m_vdControls = u;
}

std::vector<double> Predator::GetControl()
{
	return m_vdControls;
}

void Predator::SendVerticalSpeed(double speed)
{
    /* The speed command starts with a % and is followed by 3 digits
       and terminated by an exclamation point */
    char cmd[] = "%123!";
    unsigned int spd_cmd;
    char d = 0;

    /* the first digit is 1 if speed is negative */
    if(speed < 0)
    {
        d = 1;
    }

    /* make sure the speed is within an exceptable range */
    spd_cmd = MT_CLAMP(fabs(speed), 0, PREDATOR_MAX_VERT_SPEED);
    /* makes sure the first digit is 1 if the speed was negative */
    spd_cmd += 100*d;

    /* %% printf's a single %, %03d printfs a zero-padded 3-digit
       integer */
    sprintf(cmd, "%%%03d!", spd_cmd);
    SendCommand(cmd);
    
}

/* function to send a signed double number as the speed.  see
 * SendVerticalSpeed */
void Predator::SendSpeed(double speed)
{
    char cmd[] = "$123!";
    unsigned int spd_cmd;
    char d = 0;

    if(speed < 0)
    {
        d = 1;
    }
    
    spd_cmd = MT_CLAMP(fabs(speed), 0, PREDATOR_MAX_SPEED);
    spd_cmd += 100*d;
    
    sprintf(cmd, "$%03d!", spd_cmd);
    SendCommand(cmd);
    
}

/* function to send a signed double number as a turning servo command */
void Predator::SendTurn(double turn)
{
    char cmd[] = "#123!";
    unsigned int servo_cmd;

    /* assuming that halfway between the minimum and maximum servo
       positions is the neutral (straight forward) position */
    double half_speed = 0.5*((double) (PREDATOR_SERVO_MIN + PREDATOR_SERVO_MAX));

    servo_cmd = MT_CLAMP(half_speed + turn,
                         PREDATOR_SERVO_MIN,
                         PREDATOR_SERVO_MAX);
    
    sprintf(cmd, "#%03d!", servo_cmd);
    SendCommand(cmd);
    
}

/* function to send a command to a Predator via the COM port.  Makes
 * sure that the final two bytes of the command are an exclamation
 * mark '!' followed by a line feed (10).  Assumes that the line feed
 * is not already present (TODO: should check for this) */
void Predator::SendCommand(const char* command)
{
    char cmd[PREDATOR_MAX_COMMAND_LENGTH];
    
    /* make sure the command ends with a ! */
    if(*(command + strlen(command)-1) == '!')
    {
        sprintf(cmd, "%s%c", command, PREDATOR_LINEFEED);
    }
    else
    {
        sprintf(cmd, "%s!%c", command, PREDATOR_LINEFEED);
    }

    /* uncomment to see output in console */
	//printf("Sending %s\n", cmd);
    
    m_COMPort.SendCommand(cmd);
}

/* function to check if the COM port is connected */
unsigned char Predator::IsConnected() const
{
    m_bIsConnected = m_COMPort.IsConnected();
    return m_bIsConnected;
}

/* function to return some info about this robot */
const char* Predator::getInfo() const
{
    std::string info("Predator Robot:  ");
    info += m_sName;
    info += " on COM port ";
    info += m_sPort;
    return info.c_str();
}

/* function to handle joystick control */
void Predator::JoyStickControl(std::vector<double> js_axes,
                         unsigned int js_buttons)
{
    double speed = 0;
    double vert = 0;
    double turn = 0;

    static unsigned int which_cmd = 0;

    double x = -js_axes[0];
    double y = -js_axes[1];
    double w = js_axes[2];
    double z = -js_axes[3];

    speed = MT_DeadBandAndScale(y, m_dSpeedDeadBand, m_dMaxSpeed);
    turn = MT_DeadBandAndScale(x, m_dTurnDeadBand, m_dMaxTurn);

#ifndef _WIN32
    vert = MT_DeadBandAndScale(z, m_dSpeedDeadBand, m_dMaxVertSpeed);
#else
	if(js_buttons & PREDATOR_UP_BUTTON)
	{
		vert = m_dVertSpeed;
	}
	if(js_buttons & PREDATOR_DOWN_BUTTON)
	{
		vert = -m_dVertSpeed;
	}
#endif

    switch(which_cmd)
    {
    case 0:
        SendSpeed(speed);
        break;
    case 1:
        SendVerticalSpeed(vert);
        break;
    case 2:
        SendTurn(turn);
        break;
    }
    if(++which_cmd == 3)
    {
        which_cmd = 0;
    }
}

void Predator::Control()
{
	static unsigned int which_cmd = 0;

	switch(which_cmd)
	{
	case 0:
		SendSpeed(m_vdControls[PREDATOR_CONTROL_FWD_SPEED]);
		break;
	case 1:
		SendVerticalSpeed(m_vdControls[PREDATOR_CONTROL_VERT_SPEED]);
		break;
	case 2:
		SendTurn(m_vdControls[PREDATOR_CONTROL_STEERING]);
		break;
	}
	if(++which_cmd == 3)
	{
		which_cmd = 0;
	}
}
