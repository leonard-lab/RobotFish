#ifndef PREDATORROBOT_H
#define PREDATORROBOT_H

#include "MT_Robot.h"

const double PREDATOR_MAX_SPEED = 45;
const double PREDATOR_MAX_VERT_SPEED = 45;
const double PREDATOR_MAX_TURN = 68.5;
const double PREDATOR_DEFAULT_SPEED_DEADBAND = 0.05;
const double PREDATOR_DEFAULT_TURN_DEADBAND = 0.05;

const unsigned int PREDATOR_SERVO_MIN = 0;
const unsigned int PREDATOR_SERVO_MAX = 137;

const unsigned int PREDATOR_MAX_COMMAND_LENGTH = 30;


const unsigned char PREDATOR_LINEFEED = 10;

/* UP / DOWN buttons (windows only) */
const unsigned int PREDATOR_UP_BUTTON = 2;
const unsigned int PREDATOR_DOWN_BUTTON = 1;

enum
{
	PREDATOR_STATE_X = 0,
	PREDATOR_STATE_Y,
	PREDATOR_STATE_HEADING,
	PREDATOR_STATE_SPEED,
	PREDATOR_STATE_ORIENTATION,

	PREDATOR_STATE_SIZE /* must always be the last one */
};

enum
{
	PREDATOR_CONTROL_FWD_SPEED = 0,
	PREDATOR_CONTROL_STEERING,
	PREDATOR_CONTROL_VERT_SPEED,

	PREDATOR_CONTROL_SIZE /* must always be the last one */
};

class Predator : public MT_RobotBase
{
public:
    Predator();
    Predator(const char* onComPort, const char* name);

    virtual ~Predator();

    void JoyStickControl(std::vector<double> js_axes,
                         unsigned int js_buttons);

    const char* getInfo() const;

    void SendCommand(const char* command);

	void Update(std::vector<double> state);
	double GetX() const;
	double GetY() const;
	double GetTheta() const;
	void SetState(std::vector<double> state);
	void SetControl(std::vector<double> u);
	std::vector<double> GetState();
	std::vector<double> GetControl();

	void Control();
	void SafeStop();

    unsigned char IsConnected() const;

protected:
    MT_ComIO m_COMPort;
    std::string m_sPort;

	std::vector<double> m_vdState;
	std::vector<double> m_vdControls;
    
    void doCommonInit();

    void SendVerticalSpeed(double speed);
    void SendSpeed(double speed);
    void SendTurn(double turn);

private:
    mutable bool m_bIsConnected;

    double m_dMaxSpeed;
    double m_dMaxVertSpeed;
    double m_dMaxTurn;
    double m_dSpeedDeadBand;
    double m_dTurnDeadBand;

    double m_dVertSpeed;
    
};

#endif // PREDATORROBOT_H
