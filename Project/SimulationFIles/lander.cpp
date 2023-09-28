// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

//#include "lander.h"
#include <cmath>
#include "lander.h"

// variable for case 8 to specify the elliptical orbit whose one extreme 
// is the radius of the circular orbit and the next one is new_perigee
double new_perigee = 2 * MARS_RADIUS;

void autopilot (double kh_, double kp_, double thresh_hold =0.5)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{       

   
  // INSERT YOUR CODE HERE
    if (safe_to_deploy_parachute() && altitude < 10000) {
        parachute_status = parachute_status_t::DEPLOYED;
    }
    else parachute_status = parachute_status_t::NOT_DEPLOYED;

    
    //double kh = 0.05, kp = 0.978, power, thresh_delta = 0.5;
    
    double thresh_delta = thresh_hold;
    double power;
    double kp = kp_;
    double kh = kh_;

    velocity = (position - prev_position) / delta_t;
    
    double static error = -(velocity * position.norm() + kh * altitude + 0.5); 
    error = - ( velocity * position.norm() + kh * altitude + 0.5);
    power = kp * error;
   

    if (power < - thresh_delta) 
    {
        throttle = 0;
    }
    else if(power < 1 - thresh_delta) 
    {
       
        throttle = thresh_delta + power;
    }
    else {
       
        throttle = 1;
    }

    if (climb_speed >= 0 ) {
        throttle = 0;
    }
}

void autopilot_orbital_injection() {

    
    
    stabilized_attitude = true;    
    double apogee_velocity = sqrt(GRAVITY * MARS_MASS * (2 / apogee - 2 / (perigee + apogee)));
    velocity = (position - prev_position) / delta_t;
    
    double kh = 0.0008;
    double error = kh * (apogee - position.abs()) - velocity * position.norm() ;

      

    if (  stage_7 == 0 && position.abs() < apogee   ) 
    {   
    
        double power = error;
        if (power > 1) throttle = 1;
        else if (power > 0) throttle = power;
        else throttle = 0;
        up = position.norm();

        if ((position.abs() > 0.9 * apogee ) && (velocity.y > -0.6))
        {      
            stage_7 += 1;
            up = position.norm();
        }
    
    }
    


    if (stage_7 == 1)
    {
        double apogee_velocity = sqrt(GRAVITY * MARS_MASS * (2 / position.abs() - 2 / (perigee + position.abs())));
        vector3d er = position.norm();
        vector3d et = vector3d(position.y, -position.x, 0);
        et = et.norm();
        double mass_lander = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY;
        double power_er = (GRAVITY * MARS_MASS * mass_lander / position.abs2()) - (mass_lander * pow((velocity * et), 2) / position.abs());
        double power_et = sqrt(pow(MAX_THRUST, 2) - pow(power_er, 2));
        up = power_er * er + power_et * et;
        up = up.norm();
        throttle = 1;

        if (velocity * et > apogee_velocity) {
            stage_7 = 2;
            throttle = 0;
        }

    }

    if (stage_7 == 2) 
    {
        up = position.norm();
        throttle = 0;

    }

    if (DeOrbit && stage_7 == 2) stage_7 = 3;
    
    
    if (stage_7 == 3)
    {
        // Since the delta step is not small enough to detect when velocity is perpendicular to position vector - i.e apogee/perigee
        // we check when the angle between them changes the sign of its cosine function

        static bool prev_dot = false, current_dot = false;
        prev_dot = current_dot;
        current_dot = velocity * position > 0 ? true : false;
        if ((prev_dot != current_dot) && DeOrbit_init == false )
        {
            DeOrbit_init = true;
        }

        if (DeOrbit_init == true){
            vector3d er = position.norm();
            vector3d et = velocity.norm() ;

            double mass_lander = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY;
            double power_er = mass_lander * velocity.abs2() / position.abs() - GRAVITY * MARS_MASS * mass_lander / position.abs2();
            double power_et = sqrt(pow(MAX_THRUST, 2) - pow(power_er, 2));
            up = -(power_er * er + power_et * et);
            up = up.norm();
            throttle = 1;


            if (velocity.abs() < 1) {
            
                stage_7 = 4;                
            }
        }
    }

    
    if (stage_7 == 4)
    {
       
        if (safe_to_deploy_parachute() && altitude < EXOSPHERE) {
            parachute_status = parachute_status_t::DEPLOYED;
        }
        else parachute_status = parachute_status_t::NOT_DEPLOYED;


        //double kh = 0.05, kp = 0.978, power, thresh_delta = 0.5;

        double thresh_delta = 0.5;
        double power;
        double kp = 0.978;
        double kh = 0.05;

        
        double static error = -(velocity * position.norm() + kh * altitude + 0.5);
        error = -(velocity * position.norm() + kh * altitude + 0.5);
        power = kp * error;


        if (power < -thresh_delta) {

            throttle = 0;
        }
        else if (power < 1 - thresh_delta) {
            throttle = thresh_delta + power;
        }
        else {
            throttle = 1;
        }

        up = position.norm();
    }
}

void autopilot_hoffman_transfer(void) 
{
    velocity = (position - prev_position) / delta_t;
    double new_apogee_velocity = sqrt((GRAVITY * MARS_MASS * (2 / apogee - 2 / (apogee + new_perigee))));

    up = position.norm();

    
    if (Hoffman)
    {
        
        if (stage_8 == 0)
        {
            
            vector3d er = position.norm();
            vector3d et = velocity.norm();

            double mass_lander = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY;
            double power_er = mass_lander * velocity.abs2() / position.abs() - GRAVITY * MARS_MASS * mass_lander / position.abs2();
            double power_et = sqrt(pow(MAX_THRUST, 2) - pow(power_er, 2));

            if (velocity.abs() < new_apogee_velocity)
            {
                up = -(power_er * er) + (power_et * et);
                up = up.norm();
                throttle = 1;
            }
            if ( velocity.abs() > new_apogee_velocity)
            {
                stage_8 = 1;
            }
        
        }

        if (stage_8 == 1) {
        
            up = position.norm();
            throttle = 0;

        }

    }


}

void numerical_dynamics (void)
{
    // INSERT YOUR CODE HERE
    if (infinite_fuel) fuel = 1;
    vector3d thrust_force = thrust_wrt_world();
    vector3d gravitational_acc =  position.norm() * ( - MARS_MASS * UNIVERSAL_GRAVITATION_CONSTANT / position.abs2());
    double atm_density = atmospheric_density(position);
    vector3d drag;
    
    if (parachute_status == parachute_status_t::DEPLOYED)
    {
        vector3d chute_drag = - 0.5 * atm_density * DRAG_COEF_CHUTE * 5 * pow( 2 * LANDER_SIZE, 2 ) * velocity.abs2() * velocity.norm() ;
        vector3d lander_drag = - 0.5 * atm_density * DRAG_COEF_LANDER *  3.14 * pow( 2 * LANDER_SIZE, 2) * velocity.abs2() * velocity.norm();
        drag = chute_drag + lander_drag; 
    }
    else {
        
        drag =  -0.5 * atm_density * DRAG_COEF_LANDER * 3.14 * pow(2 * LANDER_SIZE, 2) * velocity.abs2() * velocity.norm() ;
    }
    
    double mass_lander = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY * fuel ;
    //double mass_lander = UNLOADED_LANDER_MASS ;
    vector3d acceleration =  gravitational_acc + (drag + thrust_force) / mass_lander ;

   
    //Verlet Integration 
    if (time_0) {
        time_0 = false;
        prev_position = position;
        position = position + velocity * delta_t;
    }
    else {
        vector3d new_position = 2 * position - prev_position + pow(delta_t, 2) * acceleration;
        prev_position = position;
        position = new_position;    
    }

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
    if (autopilot_enabled && scenario != 8) {
        
        switch (autopilot_no) 
        {
            double kp, kh, thresh_hold;
            case 0 : // min desc velocity
                //double kh = 0.05, kp = 0.978, power, thresh_delta = 0.5;
                kh = 0.03853176;
                kp = 0.60199072;
                thresh_hold = 0.80496405;
                autopilot(kh, kp, thresh_hold);
                break;
            case 1 :// min desc time
                kh = 0.16502185;
                kp = 1.53807146;
                thresh_hold = 0.5;
                autopilot(kh, kp, thresh_hold);
                break;
            case 2 :// min fuel usage

                 kh = 0.16502185;
                 kp = 1.33807146;
                 thresh_hold = 0.307;
                autopilot(kh, kp, thresh_hold);
                break;
            case 3 :
                autopilot_orbital_injection();
                break;
                      
        }
    }
    else if (scenario == 8) {

        autopilot_hoffman_transfer();
        stabilized_attitude = true;

       // cout << acceleration * position.norm() << endl;
    }

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
  //cout << up.norm() << thrust_force.norm() << endl;


}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "geostationary orbit";
  scenario_description[7] = "orbital injection with any specified apogee and perigee - with deorbiting and landing";
  scenario_description[8] = "Hoffman like transfer from a circular orbit to elliptical orbit";
  scenario_description[9] = "descent from 3386000km";

  time_0 = true;

  // variables for case 8
  double req_velocity, radius;

  switch (scenario) {

  case 0:
   
    // a circular equatorial orbit
      position = vector3d(1.2 * MARS_RADIUS, 0.0, 0.0);
      velocity = vector3d(0.0, -3247.087385863725, 0.0);
      orientation = vector3d(0.0, 90.0, 0.0);
      delta_t = 0.1;
      parachute_status = NOT_DEPLOYED;
      stabilized_attitude = false;
      autopilot_enabled = false;
      break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    position = vector3d(0.0, -(MARS_RADIUS + 170000), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
      // geostationary orbit
      position = vector3d(0.0, 0.0, (MARS_RADIUS + 17039000));
      velocity = vector3d(-1448.261, 0, 0);
      orientation = vector3d(0.0, 90.0, 0.0);
      delta_t = 0.1;
      parachute_status = NOT_DEPLOYED;
      stabilized_attitude = false;
      autopilot_enabled = false;
    break;

        

  case 7:      

      position = vector3d(0.0, -( MARS_RADIUS + LANDER_SIZE/2 + 1), 0.0);
     // position = vector3d(0.0, -( 1.2 * MARS_RADIUS ), 0.0);
      velocity = vector3d(0.0, 0.0, 0.0);
      orientation = vector3d(0.0, 0.0,0.0);
      delta_t = 0.1;
      parachute_status = NOT_DEPLOYED;
      stabilized_attitude = true;
      autopilot_enabled = true;
      perigee = MARS_RADIUS * 9;
      apogee = MARS_RADIUS * 1.2;
      infinite_fuel = true;
      autopilot_no = 3;
      
      break;

  case 8:

      radius = 1.25 * MARS_RADIUS;
      req_velocity = sqrt((GRAVITY * MARS_MASS  / radius ));      
      position = vector3d(0.0, - radius, 0.0);
      velocity = vector3d(req_velocity, 0.0, 0.0);
      orientation = vector3d(0.0, 90.0, 0.0);
      delta_t = 0.1;
      parachute_status = NOT_DEPLOYED;
      stabilized_attitude = true;
      autopilot_enabled = true;
      infinite_fuel = true;
      autopilot_no = 3;
      apogee = radius;
      perigee = radius;

    break;

  case 9:

      
      position = vector3d(0.0, -(2*MARS_RADIUS), 0.0);
      velocity = vector3d(0.0, 0.0, 0.0);
      orientation = vector3d(0.0, 0.0, 90.0);
      delta_t = 0.1;
      parachute_status = NOT_DEPLOYED;
      stabilized_attitude = true;
      autopilot_enabled = false;
      break;


    break;

  }
}
