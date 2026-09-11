#include "Maths.h" // For std::clamp
#include "FCS.h"

Flight_Control_System::Flight_Control_System
(
	State& state, 
	Input& input, 
	Airframe& airframe
): 
	m_state(state),
	m_input(input), 
	m_airframe(airframe)
{

}

void Flight_Control_System::zeroInit()
{
	pitch_cmd_filtered = 0.0;
	pitchcmd = 0.0; // Initialize pitch command
	nosewheel_angle = 0.0;
	new_canard_anims = 0.0;
	canard_position = 0.0;
	current_aoa = 0.0;
	current_g = 0.0;


	pitch_error_prior = 0.0;
	pitch_error = 0.0;
	pitch_derivative_prior = 0.0;
	pitch_pid_result = 0.0;
	pitch_integral_prior = 0; // or small value
	pitch_error_prior = pitch_error;
	pitch_meassurement_prior = pitch_rate;
	//                         P  I  D
	pitchController.initialize(0.1, 0.1, 0.2, -1.0, 1.0);
	//                        P  I  D
	rollController.initialize(3, 0, 0, -1.0, 1.0);
	//                        P  I  D
	yawController.initialize(1, 1, 1, -1.0, 1.0);


}
void Flight_Control_System::coldInit()
{
	zeroInit();
}
void Flight_Control_System::hotInit()
{
    zeroInit();
}
void Flight_Control_System::airborneInit()
{
	zeroInit();
}

//Revised FBW System 
void Flight_Control_System::limit_pitch()
{
	//Assign pitchcmd to filtered to make life easier
	double g_input = (-pitchcmd * 4) + 1;
	pitchController.update(g_input,current_g,m_dt);
	// New logic down here
	printf("Pitch command: %f \n", pitchcmd);
	pitch_cmd_filtered = pitchController.getOutputPID();
	printf("Pitch Output: %f \n", pitch_cmd_filtered);
	printf("Current G: %f \n", current_g);
	printf("Target G: %f \n", g_input);
}

void Flight_Control_System::limit_yaw()
{
	// Should be fine untouched?
	yaw_cmd_filtered = yawcmd;
	if (landing_FCS_mode == 1.0)
	{
		yaw_cmd_filtered *= 0.05;
	}
	else if (refueling_FCS_mode == 1.0)
	{
		yaw_cmd_filtered *= 0.25;
	}
	else
	{
		yaw_cmd_filtered *= 0.5;
	}

	if (current_aoa > (10 * DEG_TO_RAD))
	{
		double scale_factor = (10 * DEG_TO_RAD) / current_aoa;
		yaw_cmd_filtered *= scale_factor;
	}
}

//void Flight_Control_System::low_speed_recovery()
//{
//	For FBW Completion
//}

void Flight_Control_System::limit_roll()
{
	//Default filtered to cmd
	roll_cmd_filtered = rollcmd;
}

void Flight_Control_System::limiter_mode()
{
	/*
	Needs rewriting completely

	landing_FCS_mode = 1.0; //Old version for reference
	supersonic_FCS_mode = 0.0;
	subsonic_FCS_mode = 0.0;
	refueling_FCS_mode = 0.0;
	landing_limit();

	Old FBW limits reference:
	max_AoA = 30 * DEG_TO_RAD;
	max_g = 4.45; // 7.25 G
	max_neg_g = -1;

	max_current_pitch_rate = 15 * DEG_TO_RAD;
	min_current_pitch_rate = -2 * DEG_TO_RAD;
	*/
}

void Flight_Control_System::subsonic_limit()
{
	//Hard limit
	limited_roll_rate = 200.0 * DEG_TO_RAD;
}

void Flight_Control_System::landing_limit()
{
	//Hard limit
	limited_roll_rate = 80.0 * DEG_TO_RAD;
}

void Flight_Control_System::supersonic_limit()
{
	//Hard limit
	limited_roll_rate = 200.0 * DEG_TO_RAD;
}

void Flight_Control_System::refueling_limit()
{
	//Hard limit
	limited_roll_rate = 80.0 * DEG_TO_RAD;
}

void Flight_Control_System::autoDriveCanardPosition()
{
	/*
	
	Old reference code:

	double transition_speed = m_dt / 10;
	canard_position = new_canard_anims;
	// Move the canards to assist between 8 and 5 degrees
	if (canard_position < 1 && m_state.m_aoa >(5 * DEG_TO_RAD))
	{
		new_canard_anims += transition_speed;
	}
	else if (canard_position >= 1 && m_state.m_aoa > (5 * DEG_TO_RAD))
	{
		new_canard_anims = 1;
	}
	else if (canard_position >= 1 && m_state.m_aoa < (5 * DEG_TO_RAD))
	{
		new_canard_anims = transition_speed - new_canard_anims;
	}
	else if (canard_position <= 0 && m_state.m_aoa < (5 * DEG_TO_RAD))
	{
		new_canard_anims = 0;
	}
	*/
}
//----------------------------------------------------

void Flight_Control_System::update(double dt)
{
	pitch_rate = m_state.m_omega.z;
	roll_rate = m_state.m_omega.x;
	current_g = m_state.getNY();
	nosewheel_angle = m_airframe.getGearNPosition();
	pitchcmd = m_input.getPitch();
	rollcmd = m_input.getRoll();
	yawcmd = m_input.getYaw();
	//throttlecmd_1 = m_input.getThrottle();
	//throttlecmd_2 = m_input.getThrottle2();
	current_aoa = m_state.m_aoa;
	airspeed = m_state.m_mach; 
	//wing_stall = m_flight_model.getWingstall();
	limiter_mode();
	limit_roll();
	limit_yaw();
	limit_pitch();
	//low_speed_recovery();
	autoDriveCanardPosition();
    m_dt = dt;

}


