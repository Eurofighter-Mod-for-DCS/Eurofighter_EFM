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

void Flight_Control_System::limit_pitch()
{
	pitch_cmd_filtered = pitchcmd;
	double scale_factor = max_g / current_g;
	if (scale_factor < 1)
	{
		pitch_cmd_filtered *= limit(scale_factor, 0.0, 1.0);
	}
	scale_factor = max_AoA / limit(current_aoa, 0.0, 100.0);
	if (scale_factor < 1)
	{
		pitch_cmd_filtered *= limit(scale_factor, 0.0, 1.0);
	}
	pitch_cmd_filtered = limit(pitch_cmd_filtered, -1.0, 1.0);
	printf("Pitch_cmd_filtered = %f", pitch_cmd_filtered);
}

void Flight_Control_System::limit_yaw()
{
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
//	if (airspeed <= 0.209977)
//	{
//		throttle_cmd_filtered_1 = 1.0;
//		throttle_cmd_filtered_2 = 1.0;
//	}
//	else
//	{
//		throttle_cmd_filtered_1 = throttlecmd_1;
//		throttle_cmd_filtered_2 = throttlecmd_2;
//	}
//}

void Flight_Control_System::limit_roll()
{
	roll_cmd_filtered = rollcmd;
	//REALLY TEMPORARY
	if (landing_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= 0.5;
	}
	else if (subsonic_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= 1;
	}
	else if (supersonic_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= 0.25;
	}
	else if (refueling_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= 0.5;
	}
	roll_cmd_filtered = limit(roll_cmd_filtered, -1.0, 1.0);
}

void Flight_Control_System::limiter_mode()
{
	if (m_airframe.getRefuelingDoor() < 0.3)
	{
		if (nosewheel_angle > 0.1)
		{
			landing_FCS_mode = 1.0; //used for canard animations based on FCS mode
			supersonic_FCS_mode = 0.0;
			subsonic_FCS_mode = 0.0;
			refueling_FCS_mode = 0.0;
			landing_limit();
		}
		else if (m_state.m_mach > 0.98)
		{
			landing_FCS_mode = 0.0;
			supersonic_FCS_mode = 1.0;
			subsonic_FCS_mode = 0.0;
			refueling_FCS_mode = 0.0;
			supersonic_limit();
		}
		else
		{
			landing_FCS_mode = 0.0; 
			supersonic_FCS_mode = 0.0;
			subsonic_FCS_mode = 1.0;
			refueling_FCS_mode = 0.0;
			subsonic_limit();
		}
	}
	else
	{
		landing_FCS_mode = 0.0;
		supersonic_FCS_mode = 0.0;
		subsonic_FCS_mode = 0.0;
		refueling_FCS_mode = 1.0;
		refueling_limit();
	}
}

void Flight_Control_System::subsonic_limit()
{
	//Set max limits for this mode
	max_AoA = 30 * DEG_TO_RAD;
	limited_roll_rate = 200.0 * DEG_TO_RAD;
	max_g = 4.45; // 7.25 G
	max_neg_g = -1;
}

void Flight_Control_System::landing_limit()
{
	//Set max limits for this mode
	max_AoA = 15 * DEG_TO_RAD;
	limited_roll_rate = 80.0 * DEG_TO_RAD;
	max_g = 4/1.62921348; // 4 G
	max_neg_g = -0;
}

void Flight_Control_System::supersonic_limit()
{
	//Set max limits for this mode
	max_AoA = 15 * DEG_TO_RAD;
	limited_roll_rate = 200.0 * DEG_TO_RAD;
	max_g = 9.0/ 1.62921348; // 9 G
	max_neg_g = -1;
}

void Flight_Control_System::refueling_limit()
{
	//Set max limits for this mode
	max_AoA = 15 * DEG_TO_RAD;
	limited_roll_rate = 80.0 * DEG_TO_RAD;
	max_g = 2.0/1.62921348; //2 G
	max_neg_g = -0.0;
}

void Flight_Control_System::autoDriveCanardPosition()
{
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
}

void Flight_Control_System::PID_controller(double target, double meassurement, double kp, double ki, double kd, double tau, double bias)
{
	//--------------PID------------------------
	error = target - meassurement;

	double proportional = kp * error;

	double integral = integral_prior + 0.5 * ki * (error + error_prior);

	double derivative = 2 * kd * (meassurement - meassurement_prior) + 2 * (tau - m_dt) * derivative_prior / 2 * (tau + m_dt);

	double value_out = proportional + integral + derivative + bias;

	meassurement_prior = meassurement;
	error_prior = error;
	integral_prior = integral;
	derivative_prior = derivative;
	//----------------END OF PID--------------------
	PID_value_out = value_out;
}

void Flight_Control_System::update(double dt)
{
	pitch_rate = m_state.m_omega.z;
	roll_rate = m_state.m_omega.x; //convert to degrees for ease of use
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


