#pragma once
#include "Maths.h"
#include "AeroData_1.h"
#include "State.h"
#include "Input.h"
#include "Vec3.h"
#include "Engine.h"

class Electricsystem
{
public:
	Electricsystem(State& state, Input& input, Engine& engine);

	//Set value functions
	void setbatteryswitch(double value);
	void setgeneratorstate_1(double value);
	void setgeneratorstate_2(double value);
	void setgeneratorstate_APU(double value);
	void setenginerpm_1(double value);
	void setenginerpm_2(double value);
	void setAPUrpm(double value);

	//Returning functions
	inline double getPP1_Bus() const;
	inline double getPP2_Bus() const;
	inline double getPP3_Bus() const;
	inline double getPP4_Bus() const;
	inline double getPP5_Bus() const;
	inline double getPP6_Bus() const;

	inline double getXP1_Bus() const;
	inline double getXP2_Bus() const;

	//Misc functions
	void update(double dt);

	void generator_controller();
	void bus_controller();

	virtual void zeroinit();
	virtual void coldinit();
	virtual void hotinit();

private:
	State& m_state;
	Input& m_input;
	Engine& m_engine;

	//Battery variables
	double m_battery_switch = 0.0;
	double m_battery_state = 0.0;
	double m_batter_voltage = 0.0;

	//-------------------------------------
	//Generator variables

	double m_generator_1_voltage = 0.0;
	double m_generator_2_voltage = 0.0;

	double m_generator_1_state = 0.0;
	double m_generator_2_state = 0.0;

	double m_generator_1_RPM = 0.0;
	double m_generator_2_RPM = 0.0;

	//-------------------------------------

	double m_generator_APU_voltage = 0.0;
	double m_generator_APU_state = 0.0;
	double m_generator_APU_RPM = 0.0;

	//-------------------------------------

	double m_generator_1DC_state = 0.0;
	double m_generator_2DC_state = 0.0;

	double m_generator_1DC_RPM = 0.0;
	double m_generator_2DC_RPM = 0.0;

	double m_generator_1DC_voltage = 0.0;
	double m_generator_2DC_voltage = 0.0;

	//-------------------------------------

	double m_pp1_bus_state = 0.0;
	double m_pp2_bus_state = 0.0;
	double m_pp3_bus_state = 0.0;
	double m_pp4_bus_state = 0.0;
	double m_pp5_bus_state = 0.0;
	double m_pp6_bus_state = 0.0;

	double m_xp1_bus_state = 0.0;
	double m_xp2_bus_state = 0.0;

	//-------------------------------------

	double m_dt = 0.0;
};
//---------------------------------------------
double Electricsystem::getPP1_Bus() const
{
	return m_pp1_bus_state;
}
double Electricsystem::getPP2_Bus() const
{
	return m_pp2_bus_state;
}
double Electricsystem::getPP3_Bus() const
{
	return m_pp3_bus_state;
}
double Electricsystem::getPP4_Bus() const
{
	return m_pp4_bus_state;
}
double Electricsystem::getPP5_Bus() const
{
	return m_pp5_bus_state;
}
double Electricsystem::getPP6_Bus() const
{
	return m_pp6_bus_state;
}
//---------------------------------------------
double Electricsystem::getXP1_Bus() const
{
	return m_xp1_bus_state;
}
double Electricsystem::getXP2_Bus() const
{
	return m_xp2_bus_state;
}