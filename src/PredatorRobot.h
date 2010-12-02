#ifndef PREDATORROBOT_H
#define PREDATORROBOT_H

#include "MT_Robot.h"

enum
{
	PREDATOR_STATE_X = 0,
	PREDATOR_STATE_Y,
	PREDATOR_STATE_HEADING,
	PREDATOR_STATE_SPEED,

	PREDATOR_STATE_SIZE /* must always be the last one */
};

enum
{
	PREDATOR_CONTROL_SPEED = 0,
	PREDATOR_CONTROL_STEERING,

	PREDATOR_CONTROL_SIZE /* must always be the last one */
};

class Predator : public MT_SteeredRobot
{
public:
    Predator();
    Predator(const char* onComPort, const char* name);

    virtual ~Predator();

	void Update(std::vector<double> state);
	double GetX() const;
	double GetY() const;
	double GetTheta() const;
	void SetState(std::vector<double> state);
	void SetControl(std::vector<double> u);
	std::vector<double> GetState();
	std::vector<double> GetControl();

	void Control();

protected:
	std::vector<double> m_vdState;
	std::vector<double> m_vdControls;
    
    void doCommonInit();

private:
    
};

#endif // PREDATORROBOT_H
