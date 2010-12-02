#include "PredatorRobot.h"

/* "Dummy" Constructor -- used for debugging only */
Predator::Predator()
    : MT_SteeredRobot()
{
    doCommonInit();
}

/* Normal ctor -- give a COM port and a display name, e.g. "Predator 1"
 * - every robot should have a unique name */
Predator::Predator(const char* onComPort, const char* name)
    : MT_SteeredRobot(onComPort, name)
{
    /* common Predator initialization */
    doCommonInit();
}

/* Predator initialization */
Predator::~Predator()
{
	SafeStop();
}

void Predator::doCommonInit()
{

    /* Adding these as an MT_DataGroup enables GUI adjustment and
     * persistence via XML.  The XML file is keyed on the robot name,
     * so the robot name needs to be unique */
	m_vdState.resize(PREDATOR_STATE_SIZE, 0.0);
	m_vdControls.resize(PREDATOR_CONTROL_SIZE, 0.0);

}

/* function to construct a vertical speed command.  speed is a signed
 * double number */
void Predator::Update(std::vector<double> state)
{
    MT_SteeredRobot::Update(state);
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
	return m_vdState[PREDATOR_STATE_HEADING];
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

void Predator::Control()
{
    SetSpeedOmega(m_vdControls[PREDATOR_CONTROL_SPEED],
                  m_vdControls[PREDATOR_CONTROL_STEERING]);
}
