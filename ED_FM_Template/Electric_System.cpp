#include "Electric_System.h"

Electricsystem::Electricsystem
(
	State& state,
	Input& input,
	Engine& engine

) :
	m_state(state),
	m_input(input),
	m_engine(engine)

{
	//Un used
}
//Initialising functions

void Electricsystem::zeroinit()
{
	//Battery variables
	m_battery_switch = 0.0;
	m_battery_state = 0.0;
	m_batter_voltage = 0.0;

	//-------------------------------------
	//Generator variables

	m_generator_1_voltage = 0.0;
	m_generator_2_voltage = 0.0;

	m_generator_1_state = 0.0;
	m_generator_2_state = 0.0;

	m_generator_1_RPM = 0.0;
	m_generator_2_RPM = 0.0;

	//-------------------------------------

	m_generator_APU_voltage = 0.0;
	m_generator_APU_state = 0.0;
	m_generator_APU_RPM = 0.0;

	//-------------------------------------

	m_generator_1DC_state = 0.0;
	m_generator_2DC_state = 0.0;

	m_generator_1DC_RPM = 0.0;
	m_generator_2DC_RPM = 0.0;

	m_generator_1DC_voltage = 0.0;
	m_generator_2DC_voltage = 0.0;

	//-------------------------------------

	m_pp1_bus_state = 0.0;
	m_pp2_bus_state = 0.0;
	m_pp3_bus_state = 0.0;
	m_pp4_bus_state = 0.0;
	m_pp5_bus_state = 0.0;
	m_pp6_bus_state = 0.0;

	m_xp1_bus_state = 0.0;
	m_xp2_bus_state = 0.0; //There is also an XP3 bus however its just a branch of the XP1

	//-------------------------------------

	m_dt = 0.0;
}
void Electricsystem::coldinit()
{
	zeroinit();
}
void Electricsystem::hotinit()
{
	zeroinit();
	m_battery_switch = 1.0;
	m_battery_state = 1.0;
	m_pp1_bus_state = 1.0;
	m_pp2_bus_state = 1.0;
	m_pp3_bus_state = 1.0;
	m_pp4_bus_state = 1.0;
	m_pp5_bus_state = 1.0;
	m_pp6_bus_state = 1.0;

	m_xp1_bus_state = 1.0;
	m_xp2_bus_state = 1.0;
}
 
//Battery switch
void Electricsystem::setbatteryswitch(double value)
{
	m_battery_switch = clamp(value, 0, 1);
}
//Generator states
void Electricsystem::setgeneratorstate_1(double value)
{
	m_generator_1_state = value;
}
void Electricsystem::setgeneratorstate_2(double value)
{
	m_generator_2_state = value;
}
void Electricsystem::setgeneratorstate_APU(double value)
{
	m_generator_APU_state = value;
}
//Generator RPMS
void Electricsystem::setenginerpm_1(double value)
{
	m_generator_1_RPM = value;
	m_generator_1DC_RPM = value;
}
void Electricsystem::setenginerpm_2(double value)
{
	m_generator_2_RPM = value;
	m_generator_2DC_RPM = value;
}
void Electricsystem::setAPUrpm(double value)
{
	m_generator_APU_RPM = value;
}

//-------------------------------------

//Used to update generator states
void Electricsystem::generator_controller()
{
	//Generator 1 state control
	if (m_generator_1_RPM > 50)
	{
		m_generator_1_state = 1.0;
	}
	else if (m_generator_1_RPM < 50 && m_generator_1_state == 1.0)
	{
		m_generator_1_state = 0.0;
	}
	else
	{
		m_generator_1_state = 0.0;
	}

	//Generator 2 state control
	if (m_generator_2_RPM > 50)
	{
		m_generator_2_state = 1.0;
	}
	else if (m_generator_2_RPM < 50 && m_generator_2_state == 1.0)
	{
		m_generator_2_state = 0.0;
	}
	else
	{
		m_generator_2_state = 0.0;
	}

	//Generator APU state control
	if (m_generator_APU_RPM > 50)
	{
		m_generator_APU_state = 1.0;
	}
	else if (m_generator_APU_RPM < 50 && m_generator_APU_state == 1.0)
	{
		m_generator_APU_state = 0.0;
	}
	else
	{
		m_generator_APU_state = 0.0;
	}

	//Generator 1 DC state control
	if (m_generator_1DC_RPM > 50)
	{
		m_generator_1DC_state = 1.0;
	}
	else if (m_generator_1DC_RPM < 50 && m_generator_1DC_state == 1.0)
	{
		m_generator_1DC_state = 0.0;
	}
	else
	{
		m_generator_1DC_state = 0.0;
	}

	//Generator 2 DC state control
	if (m_generator_2DC_RPM > 50)
	{
		m_generator_2DC_state = 1.0;
	}
	else if (m_generator_2DC_RPM < 50 && m_generator_2DC_state == 1.0)
	{
		m_generator_2DC_state = 0.0;
	}
	else
	{
		m_generator_2DC_state = 0.0;
	}
}
//Used to update bus states
void Electricsystem::bus_controller()
{ 
	// XP1 and PP1 Bus control
	if (m_generator_1_state == 1.0 || m_generator_APU_state == 1.0)
	{
		m_xp1_bus_state = 1.0;
		m_pp1_bus_state = 1.0;
	}
	else
	{
		m_xp1_bus_state = 0.0;
		m_pp1_bus_state = 0.0;
	}

	// XP2 and PP2 Bus control
	if (m_generator_2_state == 1.0)
	{
		m_xp2_bus_state = 1.0;
		m_pp2_bus_state = 1.0;
	}
	else
	{
		m_xp2_bus_state = 0.0;
		m_pp2_bus_state = 0.0;
	}

	if (m_battery_state == 1.0 || m_generator_1DC_state == 1.0)
	{
		m_pp3_bus_state = 1.0;
	}
	else
	{
		m_pp3_bus_state = 0.0;
	}

	if (m_battery_state == 1.0)
	{
		m_pp5_bus_state = 1.0;
		m_pp6_bus_state = 1.0;
	}
	else
	{
		m_pp5_bus_state = 0.0;
		m_pp6_bus_state = 0.0;
	}
}

void Electricsystem::update(double dt)
{
	m_dt = dt;
	m_battery_switch = m_battery_state;
	setenginerpm_1(m_engine.getRPM());
	setenginerpm_2(m_engine.getRPM2());
	setAPUrpm(100); //Temporarly set APU to 100 RPM

	generator_controller();
	bus_controller();
}