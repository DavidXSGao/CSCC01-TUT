
/*
  Standard C libraries
*/
#include <math.h>
#include <string.h>

#include "Lander_Control.h"

//global data
char working_thruster[1024];
char direction[1024];
int angle;
int comp_angle;
int near_pad;

const int sample_size = 100000;
double velx_sample[sample_size] = {0.0};
double vely_sample[sample_size] = {0.0};
double posx_sample[sample_size] = {0.0};
double posy_sample[sample_size] = {0.0};
double angle_sample[sample_size] = {0.0};


double prev_velx = 0.0;
double prev_vely = 0.0;
double prev_posx = 0.0;
double prev_posy = 0.0;
double prev_angle = 0.0;
double prev_sonar1 = 0.0;
double prev_sonar9 = 0.0;
double prev_sonar18 = 0.0;
double prev_sonar27 = 0.0;

double capsule_time = 0.01249687962;
double delta = 0.0;
double prev_delta = 0.0;

double velx_avg = 0.0;
double vely_avg = 0.0;
double posx_avg = 0.0;
double posy_avg = 0.0;
double angle_avg = 0.0;

double curr_velx = 0.0;
double curr_vely = 0.0;
double curr_posx = 0.0;
double curr_posy = 0.0;
double curr_angle = 0.0;
// 4 main sonar directions
double curr_sonar1 = 0.0;
double curr_sonar9 = 0.0;
double curr_sonar18 = 0.0;
double curr_sonar27 = 0.0;

int set_curr = 0;
int set_prev = 0;

int velx_broke = 0;
int vely_broke = 0;
int posx_broke = 0;
int posy_broke = 0;
int angle_broke = 0;
int sonar_broke = 0;


int ignore_sonar = 0;
int use_method = 1;


//del this
#include <iostream>
using namespace std;

void left_thruster_left (double VXlim, double VYlim);
void left_thruster_right (double VXlim, double VYlim);
void left_thruster_collision ();

void right_thruster_left (double VXlim, double VYlim);
void right_thruster_right (double VXlim, double VYlim);
void right_thruster_collision ();

void main_thruster_left (double VXlim, double VYlim);
void main_thruster_right (double VXlim, double VYlim);
void main_thruster_collision ();

void straight_up();
void regular_move();


void
Lander_Control (void)
{
    /*
       This is the main control function for the lander. It attempts
       to bring the ship to the location of the landing platform
       keeping landing parameters within the acceptable limits.

       How it works:

       - First, if the lander is rotated away from zero-degree angle,
       rotate lander back onto zero degrees.
       - Determine the horizontal distance between the lander and
       the platform, fire horizontal thrusters appropriately
       to change the horizontal velocity so as to decrease this
       distance
       - Determine the vertical distance to landing platform, and
       allow the lander to descend while keeping the vertical
       speed within acceptable bounds. Make sure that the lander
       will not hit the ground before it is over the platform!

       As noted above, this function assumes everything is working
       fine.
     */


    // sampling sensor data
    int i = 0;
    while(i < sample_size) {
        velx_sample[i] = Velocity_X();
        vely_sample[i] = Velocity_Y();
        posx_sample[i] = Position_X();
        posy_sample[i] = Position_Y();
        angle_sample[i] = Angle();
        i++;
    }
    // calculate the average of each sensor data which will be the expected value
    double velx_sum = 0.0;
    double vely_sum = 0.0;
    double posx_sum = 0.0;
    double posy_sum = 0.0;
    double angle_sum = 0.0;

    for(int j = 0; j < sample_size; j++) {
        velx_sum += velx_sample[j];
        vely_sum += vely_sample[j];
        posx_sum += posx_sample[j];
        posy_sum += posy_sample[j];
        angle_sum += angle_sample[j];
    }

    velx_avg = velx_sum/sample_size;
    vely_avg = vely_sum/sample_size;
    posx_avg = posx_sum/sample_size;
    posy_avg = posy_sum/sample_size;
    angle_avg = angle_sum/sample_size;
    delta = PLAT_X - posy_avg;


    curr_sonar1 = SONAR_DIST[1];
    curr_sonar9 = SONAR_DIST[9];
    curr_sonar18 = SONAR_DIST[18];
    curr_sonar27 = SONAR_DIST[27];
    if (curr_sonar1 != -1 || curr_sonar9 != -1 || curr_sonar18  != -1  || curr_sonar27 != -1) {
        use_method = 0 ;

    }
    // if there is a previous value for each sensor, then we compare them to see if any of the sensor is failing
    if (set_prev == 1) {
        if(fabs(velx_avg - prev_velx) > 0.6) {
            velx_broke = 1;
        }
        if(fabs(vely_avg - prev_vely) > 0.6) {
            vely_broke = 1;
        }
        if(fabs(posx_avg - prev_posx) > 30.0) {
            posx_broke = 1;
        }
        if(fabs(posy_avg - prev_posy) > 20.0) {
            posy_broke = 1;
        }
        if(fabs(angle_avg - prev_angle) > 20.0) {
            angle_broke = 1;
        }
        if (use_method == 0) {
            if(fabs(curr_sonar1 - prev_sonar1) > 20.0 ||
                    fabs(curr_sonar9 - prev_sonar9) > 20.0||
                    fabs(curr_sonar18 - prev_sonar18) > 20.0||
                    fabs(curr_sonar27 - prev_sonar27) > 20.0) {
                // ignore failure the first time to prevent falsely positive
                ignore_sonar++;
            }

            if (curr_sonar1 == -1 && curr_sonar9 == -1
                    && curr_sonar18  == -1 && curr_sonar27  == -1
                    && prev_sonar1 == -1 && prev_sonar9 == -1
                    && prev_sonar18  == -1 && prev_sonar27  == -1)  {
                if (ignore_sonar < 25 ) {
                    ignore_sonar++;
                }
                else {
                    sonar_broke = 1;

                }
            }
        }

    }


    cout << "Here3 curr_sonar1 " << curr_sonar1 << " prev_sonar1 " << prev_sonar1 << "\n";
    cout << "Here3 curr_sonar9 " << curr_sonar9 << " prev_sonar9 " << prev_sonar9 << "\n";
    cout << "Here3 curr_sonar18 " << curr_sonar18 << " prev_sonar18 " << prev_sonar18 << "\n";
    cout << "Here3 curr_sonar27 " << curr_sonar27 << " prev_sonar27 " << prev_sonar27 << "\n";

    cout << "ignore_sonar : " << ignore_sonar << "\n";
    cout << "curr_posy : " << curr_posy << "\n";
    cout << "Here4 sonar_broke " << sonar_broke << "\n";

    // adjust the current sensor value if any of them is failing
    // if not, set to expected value
    if (posx_broke == 1) {
        curr_posx = curr_posx + (curr_velx/80);
    } else {
        curr_posx = posx_avg;
    }
    if (posy_broke == 1) {
        curr_posy = curr_posy - (curr_vely/78);
    } else {
        curr_posy = posy_avg;
    }
    if (velx_broke == 1) {
        curr_velx = (curr_posx - prev_posx)/capsule_time;
    } else {
        curr_velx = velx_avg;
    }
    if(vely_broke == 1) {
        curr_vely = (curr_posy - prev_posy)/capsule_time;
    } else {
        curr_vely = vely_avg;
    }

    if(angle_broke != 1) {
        curr_angle = angle_avg;
    }

    // use position y to check sonar
    // if sonar is not failing, then it should not return -1
    // when the lander is this close to land
    if (use_method == 1) {
        if (curr_posy > 550 && curr_sonar1 == -1 && curr_sonar9 == -1
                && curr_sonar18  == -1 && curr_sonar27  == -1)  {
            sonar_broke = 1;
        }
    }




    double VXlim;
    double VYlim;

    // Set velocity limits depending on distance to platform.
    // If the module is far from the platform allow it to
    // move faster, decrease speed limits as the module
    // approaches landing. You may need to be more conservative
    // with velocity limits when things fail.
    if (fabs (curr_posx - PLAT_X) > 200)
    {
        VXlim = 1.35;
        angle = 20;
        comp_angle = 10;
    } else if (fabs (curr_posx - PLAT_X) > 150)
    {
        VXlim = 5.5; //7 works ish
        angle = 15;
        comp_angle = 20;
    }
    else if (fabs (curr_posx - PLAT_X) > 100)
    {
        VXlim = 12; //7 works ish
        angle = 15;
        comp_angle = 20;
    }
    else if (fabs (curr_posx - PLAT_X) > 75)
    {
        VXlim = 10;
        angle = 10;
        comp_angle = 25;
    }
    else if (fabs (curr_posx - PLAT_X) > 50)
    {
        VXlim = 5;
        angle = 10;
        comp_angle = 25;
    }
    else if (fabs (curr_posx - PLAT_X) > 30)
    {
        VXlim = 3;
        angle = 10;
        comp_angle = 25;
    }
    else if (fabs (curr_posx - PLAT_X) > 15)
    {
        VXlim = 1;
        angle = 10;
        comp_angle = 25;
    }
    else if (fabs (curr_posx - PLAT_X) > 10)
    {
        VXlim = 0.8;
        angle = 10;
        comp_angle = 25;
    }
    else
    {
        VXlim = 0.4;
        angle = 10;
        comp_angle = 27;
    }


    // IMPORTANT NOTE: The code below assumes all components working
    // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
    // fail. More likely, you will need a set of case-based code
    // chunks, each of which works under particular failure conditions.

    // Check for rotation away from zero degrees - Rotate first,
    // use thrusters only when not rotating to avoid adding
    // velocity components along the rotation directions
    // Note that only the latest Rotate() command has any
    // effect, i.e. the rotation angle does not accumulate
    // for successive calls.


    // make function so that everything works with 1 thruster
    // set whatever thruster still work as that thruster

    if ((LT_OK != 1) && (RT_OK != 1))
    {
        strcpy (&working_thruster[0], "main");
    }
    else if (LT_OK != 1)
    {
        strcpy (&working_thruster[0], "right");
    }
    else if ((MT_OK != 1) || (RT_OK != 1))
    {
        strcpy (&working_thruster[0], "left");
    }



    if (PLAT_Y - curr_posy > 200)
        VYlim = -10;		//
    else if (PLAT_Y - curr_posy > 100)
        VYlim = -5;			// These are negative because they
    else
        VYlim = -2;			// limit descent velocity

    // Ensure we will be OVER the platform when we land
    if (fabs (PLAT_X - curr_posx) / fabs (curr_velx) >
            1.25 * fabs (PLAT_Y - curr_posy) / fabs (curr_vely))
        VYlim = 0;

    // if everything is ok or close to landing pad
    Left_Thruster (0.0);
    Right_Thruster (0.0);
    Main_Thruster (0.0);
    if (curr_angle > 1 && curr_angle < 359)
    {
        if (curr_angle >= 180) {
            Rotate (360 - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = 360 - curr_angle;
            }
        }
        else {
            Rotate (-curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = - curr_angle;
            }
        }
    }

    // Module is oriented properly, check for horizontal position
    // and set thrusters appropriately.
    if (curr_posx > PLAT_X)
    {
        // Lander is to the LEFT of the landing platform, use Right thrusters to move
        // lander to the left.
        if ((strcmp (working_thruster, "left") == 0)
                || (strcmp (working_thruster, "main") == 0)
                || (strcmp (working_thruster, "right") == 0))
        {
            strcpy (&direction[0], "left");
        }
        else
        {   // everything working

            Left_Thruster (0.0);	// Make sure we're not fighting ourselves here!
            if (curr_velx > (-VXlim))
                Right_Thruster ((VXlim + fmin (0, curr_velx)) / VXlim);
            else
            {
                // Efloat average (int n_args, ...);xceeded velocity limit, brake
                Right_Thruster (0.0);
                Left_Thruster (fabs (VXlim - curr_velx));
            }

        }
    }
    else if (curr_posx < PLAT_X)
    {
        // Lander is to the RIGHT of the landing platform, opposite from above
        if ((strcmp (working_thruster, "left") == 0)
                || (strcmp (working_thruster, "main") == 0)
                || (strcmp (working_thruster, "right") == 0))
        {
            strcpy (&direction[0], "right");
        }
        else
        {   // everything working

            Right_Thruster (0.0);
            if (curr_velx < VXlim)
                Left_Thruster ((VXlim - fmax (0, curr_velx)) / VXlim);
            else
            {
                Left_Thruster (0.0);
                Right_Thruster (fabs (VXlim - curr_velx));
            }
        }
    }
    else
    {
        strcpy (&direction[0], "done");
    }


    // case of sonar failure

    if (sonar_broke == 1)
    {

        if ((curr_posy) > 80 && fabs (curr_posx - PLAT_X) > 10)
        {
            // want curr_posy < 25, and  |curr_posx| < 35
            // go straight up until curr_posy < 25
            if (curr_posy > 80 ) {
                // to avoid crashing into hills in hard map
                straight_up();
            } else if (fabs (curr_posx - PLAT_X) > 35) {
                // run regular code to move toward landing plate

                if (curr_posx > PLAT_X)
                {
                    // if any thrusters are broken, call the move left function for the remaining thrusters

                    if ((strcmp (working_thruster, "left") == 0)
                            || (strcmp (working_thruster, "main") == 0)
                            || (strcmp (working_thruster, "right") == 0))
                    {
                        strcpy (&direction[0], "left");
                    }
                    else
                    {   // everything working

                        Left_Thruster (0.0);	// Make sure we're not fighting ourselves here!
                        if (curr_velx > (-VXlim))
                            Right_Thruster ((VXlim + fmin (0, curr_velx)) / VXlim);
                        else
                        {
                            // Efloat average (int n_args, ...);xceeded velocity limit, brake
                            Right_Thruster (0.0);
                            Left_Thruster (fabs (VXlim - curr_velx));
                        }

                    }
                }
                // Lander is to the RIGHT of the landing platform, opposite from above

                else if (curr_posx < PLAT_X)
                {
                    // if any thrusters are broken, call the move right function for the remaining thrusters

                    if ((strcmp (working_thruster, "left") == 0)
                            || (strcmp (working_thruster, "main") == 0)
                            || (strcmp (working_thruster, "right") == 0))
                    {
                        strcpy (&direction[0], "right");
                    }
                    else
                    {   // everything working

                        Right_Thruster (0.0);
                        if (curr_velx < VXlim)
                            Left_Thruster ((VXlim - fmax (0, curr_velx)) / VXlim);
                        else
                        {
                            Left_Thruster (0.0);
                            Right_Thruster (fabs (VXlim - curr_velx));
                        }
                    }
                }
                // if module is directly above landing pad call done function on remaining thrusters

                else
                {

                    strcpy (&direction[0], "done");
                }

                // check if near landing pad
                if ((PLAT_Y - curr_posy) < 25 && fabs (curr_posx - PLAT_X) < 35)
                {

                    Left_Thruster (0.0);
                    // orient the module for landing

                    if (curr_angle > 1 && curr_angle < 359)
                    {
                        if (curr_angle >= 180) {
                            Rotate (360 - curr_angle);
                            if (angle_broke == 1)
                            {
                                curr_angle = 360 - curr_angle;
                            }
                        }
                        else {
                            Rotate (-curr_angle);
                            if (angle_broke == 1)
                            {
                                curr_angle = - curr_angle;
                            }
                        }
                    }
                    // turn off thrusters

                    Left_Thruster (0.0);
                }
                else
                {
                    // Vertical adjustments. Basically, keep the module below the limit for
                    // vertical velocity and allow for continuous descent. We trust
                    // Safety_Override() to save us from crashing with the ground.

                    cout << "Here3 curr_vely" << curr_vely << " VYlim " << VYlim << "\n";
                    if ((strcmp (working_thruster, "left") == 0)
                            || (strcmp (working_thruster, "main") == 0)
                            || (strcmp (working_thruster, "right") == 0))
                    {
                        // if left thruster is working

                        if (strcmp (working_thruster, "left") == 0)
                        {   // check if falling too fast

                            if (curr_vely < -5.6)
                            {   // head straight up

                                Left_Thruster (0.0);
                                if ((curr_angle > 0 && curr_angle < 269)
                                        || (curr_angle > 271 && curr_angle < 360))
                                {
                                    Rotate (270 - curr_angle);
                                    if (angle_broke == 1)
                                    {
                                        curr_angle = 270 - curr_angle;
                                    }
                                }
                                Left_Thruster (2.0);
                            }
                            else
                            {   // call left function if heading left

                                Left_Thruster (0.0);
                                if (strcmp (direction, "left") == 0)
                                {
                                    left_thruster_left(VXlim, VYlim);
                                }                    // call right function if heading right

                                if (strcmp (direction, "right") == 0)
                                {
                                    left_thruster_right(VXlim, VYlim);
                                }
                                // head stright up if directly on top of landing pad

                                if (strcmp (direction, "done") == 0)
                                {
                                    if ((curr_angle > 0 && curr_angle < 269)
                                            || (curr_angle > 271 && curr_angle < 360))
                                    {
                                        Rotate (270 - curr_angle);
                                        if (angle_broke == 1)
                                        {
                                            curr_angle = 270 - curr_angle;
                                        }
                                    }
                                    Left_Thruster (1.0);
                                }
                            }
                        }
                        // same methods as above for right working thruster

                        else if (strcmp (working_thruster, "right") == 0)
                        {
                            if (curr_vely < -5.6)
                            {
                                // angle to head directly up is 180 degrees different from left thruster

                                if ((curr_angle > 0 && curr_angle < 89)
                                        || (curr_angle > 91 && curr_angle < 360))
                                {
                                    Rotate (90 - curr_angle);
                                    if (angle_broke == 1)
                                    {
                                        curr_angle = 90 - curr_angle;
                                    }
                                }
                                Right_Thruster (2.0);
                            }
                            else
                            {
                                Right_Thruster (0.0);
                                if (strcmp (direction, "left") == 0)
                                {
                                    right_thruster_left(VXlim, VYlim);
                                }
                                if (strcmp (direction, "right") == 0)
                                {   //northwest
                                    right_thruster_right(VXlim, VYlim);
                                }
                                if (strcmp (direction, "done") == 0)

                                {   //north
                                    if ((curr_angle > 0 && curr_angle < 89)
                                            || (curr_angle > 91 && curr_angle < 360))
                                    {
                                        Rotate (90 - curr_angle);
                                        if (angle_broke == 1)
                                        {
                                            curr_angle = 90 - curr_angle;
                                        }
                                    }
                                    Right_Thruster (1.0);
                                }
                            }
                        }
                        // same methods as above for main working thruster

                        else if (strcmp (working_thruster, "main") == 0)
                        {
                            if (curr_vely < -5.6)
                            {
                                // angle to head directly up is 90 degrees different from left and right thruster

                                if (curr_angle > 1 && curr_angle < 359)
                                {
                                    if (curr_angle >= 180) {
                                        Rotate (360 - curr_angle);
                                        if (angle_broke == 1)
                                        {
                                            curr_angle = 360 - curr_angle;
                                        }
                                    }
                                    else {
                                        Rotate (-curr_angle);
                                        if (angle_broke == 1)
                                        {
                                            curr_angle = - curr_angle;
                                        }
                                    }
                                }
                                Main_Thruster (2.0);
                            }
                            else
                            {
                                Main_Thruster (0.0);
                                if (strcmp (direction, "left") == 0)
                                {
                                    main_thruster_left(VXlim, VYlim);
                                }
                                if (strcmp (direction, "right") == 0)
                                {   //northwest
                                    main_thruster_right(VXlim, VYlim);
                                }
                                if (strcmp (direction, "done") == 0)

                                {   //north
                                    if (curr_angle > 1 && curr_angle < 359)
                                    {
                                        if (curr_angle >= 180) {
                                            Rotate (360 - curr_angle);
                                            if (angle_broke == 1)
                                            {
                                                curr_angle = 360 - curr_angle;
                                            }
                                        }
                                        else {
                                            Rotate (-curr_angle);
                                            if (angle_broke == 1)
                                            {
                                                curr_angle = - curr_angle;
                                            }
                                        }
                                    }
                                    Main_Thruster (1.0);
                                }
                            }
                        }
                    }
                    else
                    {   // everything's working
                        if (curr_vely < VYlim)
                            Main_Thruster (1.0);
                        else
                            Main_Thruster (0.0);
                    }
                }
            }
        }else{
            if (curr_posx > PLAT_X)
                            {
                                // if any thrusters are broken, call the move left function for the remaining thrusters

                                if ((strcmp (working_thruster, "left") == 0)
                                        || (strcmp (working_thruster, "main") == 0)
                                        || (strcmp (working_thruster, "right") == 0))
                                {
                                    strcpy (&direction[0], "left");
                                }
                                else
                                {   // everything working

                                    Left_Thruster (0.0);	// Make sure we're not fighting ourselves here!
                                    if (curr_velx > (-VXlim))
                                        Right_Thruster ((VXlim + fmin (0, curr_velx)) / VXlim);
                                    else
                                    {
                                        // Efloat average (int n_args, ...);xceeded velocity limit, brake
                                        Right_Thruster (0.0);
                                        Left_Thruster (fabs (VXlim - curr_velx));
                                    }

                                }
                            }
                            // Lander is to the RIGHT of the landing platform, opposite from above

                            else if (curr_posx < PLAT_X)
                            {
                                // if any thrusters are broken, call the move right function for the remaining thrusters

                                if ((strcmp (working_thruster, "left") == 0)
                                        || (strcmp (working_thruster, "main") == 0)
                                        || (strcmp (working_thruster, "right") == 0))
                                {
                                    strcpy (&direction[0], "right");
                                }
                                else
                                {   // everything working

                                    Right_Thruster (0.0);
                                    if (curr_velx < VXlim)
                                        Left_Thruster ((VXlim - fmax (0, curr_velx)) / VXlim);
                                    else
                                    {
                                        Left_Thruster (0.0);
                                        Right_Thruster (fabs (VXlim - curr_velx));
                                    }
                                }
                            }
                            // if module is directly above landing pad call done function on remaining thrusters

                            else
                            {

                                strcpy (&direction[0], "done");
                            }

                            // check if near landing pad
                            if ((PLAT_Y - curr_posy) < 25 && fabs (curr_posx - PLAT_X) < 35)
                            {

                                Left_Thruster (0.0);
                                // orient the module for landing

                                if (curr_angle > 1 && curr_angle < 359)
                                {
                                    if (curr_angle >= 180) {
                                        Rotate (360 - curr_angle);
                                        if (angle_broke == 1)
                                        {
                                            curr_angle = 360 - curr_angle;
                                        }
                                    }
                                    else {
                                        Rotate (-curr_angle);
                                        if (angle_broke == 1)
                                        {
                                            curr_angle = - curr_angle;
                                        }
                                    }
                                }
                                // turn off thrusters

                                Left_Thruster (0.0);
                            }
                            else
                            {
                                // Vertical adjustments. Basically, keep the module below the limit for
                                // vertical velocity and allow for continuous descent. We trust
                                // Safety_Override() to save us from crashing with the ground.

                                cout << "Here3 curr_vely" << curr_vely << " VYlim " << VYlim << "\n";
                                if ((strcmp (working_thruster, "left") == 0)
                                        || (strcmp (working_thruster, "main") == 0)
                                        || (strcmp (working_thruster, "right") == 0))
                                {
                                    // if left thruster is working

                                    if (strcmp (working_thruster, "left") == 0)
                                    {   // check if falling too fast

                                        if (curr_vely < -5.6)
                                        {   // head straight up

                                            Left_Thruster (0.0);
                                            if ((curr_angle > 0 && curr_angle < 269)
                                                    || (curr_angle > 271 && curr_angle < 360))
                                            {
                                                Rotate (270 - curr_angle);
                                                if (angle_broke == 1)
                                                {
                                                    curr_angle = 270 - curr_angle;
                                                }
                                            }
                                            Left_Thruster (2.0);
                                        }
                                        else
                                        {   // call left function if heading left

                                            Left_Thruster (0.0);
                                            if (strcmp (direction, "left") == 0)
                                            {
                                                left_thruster_left(VXlim, VYlim);
                                            }                    // call right function if heading right

                                            if (strcmp (direction, "right") == 0)
                                            {
                                                left_thruster_right(VXlim, VYlim);
                                            }
                                            // head stright up if directly on top of landing pad

                                            if (strcmp (direction, "done") == 0)
                                            {
                                                if ((curr_angle > 0 && curr_angle < 269)
                                                        || (curr_angle > 271 && curr_angle < 360))
                                                {
                                                    Rotate (270 - curr_angle);
                                                    if (angle_broke == 1)
                                                    {
                                                        curr_angle = 270 - curr_angle;
                                                    }
                                                }
                                                Left_Thruster (1.0);
                                            }
                                        }
                                    }
                                    // same methods as above for right working thruster

                                    else if (strcmp (working_thruster, "right") == 0)
                                    {
                                        if (curr_vely < -5.6)
                                        {
                                            // angle to head directly up is 180 degrees different from left thruster

                                            if ((curr_angle > 0 && curr_angle < 89)
                                                    || (curr_angle > 91 && curr_angle < 360))
                                            {
                                                Rotate (90 - curr_angle);
                                                if (angle_broke == 1)
                                                {
                                                    curr_angle = 90 - curr_angle;
                                                }
                                            }
                                            Right_Thruster (2.0);
                                        }
                                        else
                                        {
                                            Right_Thruster (0.0);
                                            if (strcmp (direction, "left") == 0)
                                            {
                                                right_thruster_left(VXlim, VYlim);
                                            }
                                            if (strcmp (direction, "right") == 0)
                                            {   //northwest
                                                right_thruster_right(VXlim, VYlim);
                                            }
                                            if (strcmp (direction, "done") == 0)

                                            {   //north
                                                if ((curr_angle > 0 && curr_angle < 89)
                                                        || (curr_angle > 91 && curr_angle < 360))
                                                {
                                                    Rotate (90 - curr_angle);
                                                    if (angle_broke == 1)
                                                    {
                                                        curr_angle = 90 - curr_angle;
                                                    }
                                                }
                                                Right_Thruster (1.0);
                                            }
                                        }
                                    }
                                    // same methods as above for main working thruster

                                    else if (strcmp (working_thruster, "main") == 0)
                                    {
                                        if (curr_vely < -5.6)
                                        {
                                            // angle to head directly up is 90 degrees different from left and right thruster

                                            if (curr_angle > 1 && curr_angle < 359)
                                            {
                                                if (curr_angle >= 180) {
                                                    Rotate (360 - curr_angle);
                                                    if (angle_broke == 1)
                                                    {
                                                        curr_angle = 360 - curr_angle;
                                                    }
                                                }
                                                else {
                                                    Rotate (-curr_angle);
                                                    if (angle_broke == 1)
                                                    {
                                                        curr_angle = - curr_angle;
                                                    }
                                                }
                                            }
                                            Main_Thruster (2.0);
                                        }
                                        else
                                        {
                                            Main_Thruster (0.0);
                                            if (strcmp (direction, "left") == 0)
                                            {
                                                main_thruster_left(VXlim, VYlim);
                                            }
                                            if (strcmp (direction, "right") == 0)
                                            {   //northwest
                                                main_thruster_right(VXlim, VYlim);
                                            }
                                            if (strcmp (direction, "done") == 0)

                                            {   //north
                                                if (curr_angle > 1 && curr_angle < 359)
                                                {
                                                    if (curr_angle >= 180) {
                                                        Rotate (360 - curr_angle);
                                                        if (angle_broke == 1)
                                                        {
                                                            curr_angle = 360 - curr_angle;
                                                        }
                                                    }
                                                    else {
                                                        Rotate (-curr_angle);
                                                        if (angle_broke == 1)
                                                        {
                                                            curr_angle = - curr_angle;
                                                        }
                                                    }
                                                }
                                                Main_Thruster (1.0);
                                            }
                                        }
                                    }
                                }
                                else
                                {   // everything's working
                                    if (curr_vely < VYlim)
                                        Main_Thruster (1.0);
                                    else
                                        Main_Thruster (0.0);
                                }
                            }
         }
    } else { // sonar isn't failing
        // check if near landing pad
        if ((PLAT_Y - curr_posy) < 25 && fabs (curr_posx - PLAT_X) < 35)
        {

            Left_Thruster (0.0);
            // orient the module for landing

            if (curr_angle > 1 && curr_angle < 359)
            {
                if (curr_angle >= 180) {
                    Rotate (360 - curr_angle);
                    if (angle_broke == 1)
                    {
                        curr_angle = 360 - curr_angle;
                    }
                }
                else {
                    Rotate (-curr_angle);
                    if (angle_broke == 1)
                    {
                        curr_angle = - curr_angle;
                    }
                }
            }
            // turn off thrusters

            Left_Thruster (0.0);
        }

        else
        {
            // Vertical adjustments. Basically, keep the module below the limit for
            // vertical velocity and allow for continuous descent. We trust
            // Safety_Override() to save us from crashing with the ground.
            if ((strcmp (working_thruster, "left") == 0)
                    || (strcmp (working_thruster, "main") == 0)
                    || (strcmp (working_thruster, "right") == 0))
            {
                // if left thruster is working

                if (strcmp (working_thruster, "left") == 0)
                {   // check if falling too fast

                    if (curr_vely < -5.6)
                    {   // head straight up

                        Left_Thruster (0.0);
                        if ((curr_angle > 0 && curr_angle < 269)
                                || (curr_angle > 271 && curr_angle < 360))
                        {
                            Rotate (270 - curr_angle);
                            if (angle_broke == 1)
                            {
                                curr_angle = 270 - curr_angle;
                            }
                        }
                        Left_Thruster (2.0);
                    }
                    else
                    {   // call left function if heading left

                        Left_Thruster (0.0);
                        if (strcmp (direction, "left") == 0)
                        {
                            left_thruster_left(VXlim, VYlim);
                        }                    // call right function if heading right

                        if (strcmp (direction, "right") == 0)
                        {
                            left_thruster_right(VXlim, VYlim);
                        }
                        // head stright up if directly on top of landing pad

                        if (strcmp (direction, "done") == 0)
                        {
                            if ((curr_angle > 0 && curr_angle < 269)
                                    || (curr_angle > 271 && curr_angle < 360))
                            {
                                Rotate (270 - curr_angle);
                                if (angle_broke == 1)
                                {
                                    curr_angle = 270 - curr_angle;
                                }
                            }
                            Left_Thruster (1.0);
                        }
                    }
                }
                // same methods as above for right working thruster

                else if (strcmp (working_thruster, "right") == 0)
                {
                    if (curr_vely < -5.6)
                    {
                        // angle to head directly up is 180 degrees different from left thruster

                        if ((curr_angle > 0 && curr_angle < 89)
                                || (curr_angle > 91 && curr_angle < 360))
                        {
                            Rotate (90 - curr_angle);
                            if (angle_broke == 1)
                            {
                                curr_angle = 90 - curr_angle;
                            }
                        }
                        Right_Thruster (2.0);
                    }
                    else
                    {
                        Right_Thruster (0.0);
                        if (strcmp (direction, "left") == 0)
                        {
                            right_thruster_left(VXlim, VYlim);
                        }
                        if (strcmp (direction, "right") == 0)
                        {   //northwest
                            right_thruster_right(VXlim, VYlim);
                        }
                        if (strcmp (direction, "done") == 0)

                        {   //north
                            if ((curr_angle > 0 && curr_angle < 89)
                                    || (curr_angle > 91 && curr_angle < 360))
                            {
                                Rotate (90 - curr_angle);
                                if (angle_broke == 1)
                                {
                                    curr_angle = 90 - curr_angle;
                                }
                            }
                            Right_Thruster (1.0);
                        }
                    }
                }
                // same methods as above for main working thruster

                else if (strcmp (working_thruster, "main") == 0)
                {
                    if (curr_vely < -5.6)
                    {
                        // angle to head directly up is 90 degrees different from left and right thruster

                        if (curr_angle > 1 && curr_angle < 359)
                        {
                            if (curr_angle >= 180) {
                                Rotate (360 - curr_angle);
                                if (angle_broke == 1)
                                {
                                    curr_angle = 360 - curr_angle;
                                }
                            }
                            else {
                                Rotate (-curr_angle);
                                if (angle_broke == 1)
                                {
                                    curr_angle = - curr_angle;
                                }
                            }
                        }
                        Main_Thruster (2.0);
                    }
                    else
                    {
                        Main_Thruster (0.0);
                        if (strcmp (direction, "left") == 0)
                        {
                            main_thruster_left(VXlim, VYlim);
                        }
                        if (strcmp (direction, "right") == 0)
                        {   //northwest
                            main_thruster_right(VXlim, VYlim);
                        }
                        if (strcmp (direction, "done") == 0)

                        {   //north
                            if (curr_angle > 1 && curr_angle < 359)
                            {
                                if (curr_angle >= 180) {
                                    Rotate (360 - curr_angle);
                                    if (angle_broke == 1)
                                    {
                                        curr_angle = 360 - curr_angle;
                                    }
                                }
                                else {
                                    Rotate (-curr_angle);
                                    if (angle_broke == 1)
                                    {
                                        curr_angle = - curr_angle;
                                    }
                                }
                            }
                            Main_Thruster (1.0);
                        }
                    }
                }
            }
            else
            {   // everything's working
                if (curr_vely < VYlim)
                    Main_Thruster (1.0);
                else
                    Main_Thruster (0.0);
            }
        }
    }
    // update the previous value for each sensor to current ones
    prev_velx = curr_velx;
    prev_vely = curr_vely;
    prev_posx = curr_posx;
    prev_posy = curr_posy;
    prev_angle = curr_angle;
    prev_sonar1 = curr_sonar1;
    prev_sonar9 = curr_sonar9;
    prev_sonar18 = curr_sonar18;
    prev_sonar27 = curr_sonar27;
    prev_delta = delta;

    set_prev = 1;
}






void
Safety_Override (void)
{
    /*
       This function is intended to keep the lander from
       crashing. It checks the sonar distance array,
       if the distance to nearby solid surfaces and
       uses thrusters to maintain a safe distance from
       the ground unless the ground happens to be the
       landing platform.

       Additionally, it enforces a maximum speed limit
       which when breached triggers an emergency brake
       operation.
     */

    /**************************************************
     TO DO: Modify this function so that it can do its
            work even if components or sensors
            fail
    **************************************************/

    /**************************************************
      How this works:
      Check the sonar readings, for each sonar
      reading that is below a minimum safety threshold
      AND in the general direction of motion AND
      not corresponding to the landing platform,
      carry out speed corrections using the thrusters
    **************************************************/


    double DistLimit;
    double Vmag;
    double dmin;
    // sampling to find the expected value (the average) for each sensor
    int i = 0;
    while(i < sample_size) {
        velx_sample[i] = Velocity_X();
        vely_sample[i] = Velocity_Y();
        posx_sample[i] = Position_X();
        posy_sample[i] = Position_Y();
        angle_sample[i] = Angle();
        i++;
    }
    double velx_sum = 0.0;
    double vely_sum = 0.0;
    double posx_sum = 0.0;
    double posy_sum = 0.0;
    double angle_sum = 0.0;

    for(int j = 0; j < sample_size; j++) {
        velx_sum += velx_sample[j];
        vely_sum += vely_sample[j];
        posx_sum += posx_sample[j];
        posy_sum += posy_sample[j];
        angle_sum += angle_sample[j];
    }

    velx_avg = velx_sum/sample_size;
    vely_avg = vely_sum/sample_size;
    posx_avg = posx_sum/sample_size;
    posy_avg = posy_sum/sample_size;
    angle_avg = angle_sum/sample_size;
    delta = PLAT_X - posy_avg;

    curr_sonar1 = SONAR_DIST[1];
    curr_sonar9 = SONAR_DIST[9];
    curr_sonar18 = SONAR_DIST[18];
    curr_sonar27 = SONAR_DIST[35];
    // initialize the current sensor value

    if (curr_sonar1 != -1 || curr_sonar9 != -1 || curr_sonar18  != -1  || curr_sonar27 != -1) {
        use_method = 0 ;

    }
    if (set_curr == 0) {
        curr_velx = velx_avg;
        curr_vely = vely_avg;
        curr_posx = posx_avg;
        curr_posy = posy_avg;
        curr_angle = angle_avg;
        set_curr = 1;
    }

    // check if any of the sensor is failing
    if (set_prev == 1) {
        if(fabs(velx_avg - prev_velx) > 0.6) {
            velx_broke = 1;
        }
        if(fabs(vely_avg - prev_vely) > 0.6) {
            vely_broke = 1;
        }
        if(fabs(posx_avg - prev_posx) > 30.0) {
            posx_broke = 1;
        }
        if(fabs(posy_avg - prev_posy) > 20.0) {
            posy_broke = 1;
        }
        if(fabs(angle_avg - prev_angle) > 20.0) {
            angle_broke = 1;
        }
        if (use_method == 0) {
            if(fabs(curr_sonar1 - prev_sonar1) > 20.0 ||
                    fabs(curr_sonar9 - prev_sonar9) > 20.0||
                    fabs(curr_sonar18 - prev_sonar18) > 20.0||
                    fabs(curr_sonar27 - prev_sonar27) > 20.0) {
                // ignore failure the first time to prevent falsely positive
                ignore_sonar++;
            }

            if (curr_sonar1 == -1 && curr_sonar9 == -1
                    && curr_sonar18  == -1 && curr_sonar27  == -1
                    && prev_sonar1 == -1 && prev_sonar9 == -1
                    && prev_sonar18  == -1 && prev_sonar27  == -1)  {
                if (ignore_sonar < 25 ) {
                    ignore_sonar++;
                }
                else {
                    sonar_broke = 1;

                }
            }
        }

    }


    cout << "Here3 curr_sonar1 " << curr_sonar1 << " prev_sonar1 " << prev_sonar1 << "\n";
    cout << "Here3 curr_sonar9 " << curr_sonar9 << " prev_sonar9 " << prev_sonar9 << "\n";
    cout << "Here3 curr_sonar18 " << curr_sonar18 << " prev_sonar18 " << prev_sonar18 << "\n";
    cout << "Here3 curr_sonar27 " << curr_sonar27 << " prev_sonar27 " << prev_sonar27 << "\n";

    cout << "ignore_sonar : " << ignore_sonar << "\n";
    cout << "curr_posy : " << curr_posy << "\n";
    cout << "Here4 sonar_broke " << sonar_broke << "\n";




    // determine how to get each sensor value depends on its condition
    if (posx_broke == 1) {
        curr_posx = curr_posx + (curr_velx/82);
    } else {
        curr_posx = posx_avg;
    }
    if (posy_broke == 1) {
        curr_posy = curr_posy - (curr_vely/82);
        //    cout << "curr_posy: " << curr_posy << "\n";
    } else {
        curr_posy = posy_avg;
    }
    if (velx_broke == 1) {
        curr_velx = (curr_posx - prev_posx)/capsule_time;
    } else {
        curr_velx = velx_avg;
    }
    if(vely_broke == 1) {
        curr_vely = (curr_posy - prev_posy)/capsule_time;
    } else {
        curr_vely = vely_avg;
    }
    if(angle_broke != 1) {
        curr_angle = angle_avg;
    }
    // use position y to check sonar
    // if sonar is not failing, then it should not return -1
    // when the lander is this close to land
    if (use_method == 1) {
        if (curr_posy > 550 && curr_sonar1 == -1 && curr_sonar9 == -1
                && curr_sonar18  == -1 && curr_sonar27  == -1)  {
            sonar_broke = 1;
        }
    }

    // Establish distance threshold based on lander
    // speed (we need more time to rectify direction
    // at high speed)
    Vmag = curr_velx * curr_velx;
    Vmag += curr_vely * curr_vely;


    DistLimit = fmax (75, Vmag);

    // If we're close to the landing platform, disable
    // safety override (close to the landing platform
    // the Control_Policy() should be trusted to
    // safely land the craft)
    if (fabs (PLAT_X - curr_posx) < 150
            && fabs (PLAT_Y - curr_posy) < 150)
        return;

    // Determine the closest surfaces in the direction
    // of motion. This is done by checking the sonar
    // array in the quadrant corresponding to the
    // ship's motion direction to find the entry
    // with the smallest registered distance

    // Horizontal direction.
    dmin = 1000000;
    if (curr_velx > 0)
    {


        for (int i = 5; i < 14; i++) {

            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin) {
                dmin = SONAR_DIST[i];

            }

        }
    }
    else
    {
        for (int i = 22; i < 32; i++) {

            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin) {
                dmin = SONAR_DIST[i];
            }

        }
    }
    // Determine whether we're too close for comfort. There is a reason
    // to have this distance limit modulated by horizontal speed...
    // what is it?
    if (dmin < DistLimit * fmax (.25, fmin (fabs (curr_velx) / 5.0, 1)))
    {   // Too close to a surface in the horizontal direction


        //sensor something close to the x direction
        if ((strcmp (working_thruster, "left") != 0)
                && (strcmp (working_thruster, "main") != 0)
                && (strcmp (working_thruster, "right") != 0))
        {
            if (curr_angle > 1 && curr_angle < 359)
            {
                if (curr_angle >= 180) {
                    Rotate (360 - curr_angle);
                    // keep track current angle
                    if (angle_broke == 1)
                    {
                        curr_angle = 360 - curr_angle;
                    }
                }
                else {
                    Rotate (-curr_angle);
                    if (angle_broke == 1)
                    {
                        curr_angle = - curr_angle;
                    }
                }
                return;
            }
        }
        if (strcmp (working_thruster, "left") == 0) {
            left_thruster_collision();
        } else if (strcmp (working_thruster, "right") == 0) {
            right_thruster_collision();
        } else if (strcmp (working_thruster, "main") == 0) {
            main_thruster_collision();
        }
        if ((strcmp (working_thruster, "left") != 0)
                && (strcmp (working_thruster, "main") != 0)
                && (strcmp (working_thruster, "right") != 0))
        {
            if (curr_velx > 0)
            {
                Right_Thruster (1.0);
                Left_Thruster (0.0);
            }
            else
            {
                Left_Thruster (1.0);
                Right_Thruster (0.0);
            }
        }
    }

    // Vertical direction
    dmin = 1000000;
    if (curr_vely > 5)	// Mind this! there is a reason for it...
    {
        for (int i = 0; i < 5; i++) {

            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin) {
                dmin = SONAR_DIST[i];
            }
        }
        for (int i = 32; i < 36; i++) {

            for (int j = 0; j < 36; j++) {
                cout << "Here2 : " << j <<" " << SONAR_DIST[j] << "\n";
            }
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin) {
                dmin = SONAR_DIST[i];
            }

        }
    }
    else
    {
        for (int i = 14; i < 22; i++) {

            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin) {
                dmin = SONAR_DIST[i];
            }
        }
    }
    if (dmin < DistLimit)		// Too close to a surface in the vertical direction
    {
        // sensor something close to the y direction
        // all of them are working
        if ((strcmp (working_thruster, "left") != 0)
                && (strcmp (working_thruster, "main") != 0)
                && (strcmp (working_thruster, "right") != 0))
        {
            if (curr_angle > 1 || curr_angle > 359)
            {
                if (curr_angle >= 180) {
                    Rotate (360 - curr_angle);
                    if (angle_broke == 1)
                    {
                        curr_angle = 360 - curr_angle;
                    }
                }
                else {
                    Rotate (-curr_angle);
                    if (angle_broke == 1)
                    {
                        curr_angle = - curr_angle;
                    }
                }
                return;
            }
        }
        straight_up();

    }
    prev_velx = curr_velx;
    prev_vely = curr_vely;
    prev_posx = curr_posx;
    prev_posy = curr_posy;
    prev_angle = curr_angle;
    prev_sonar1 = curr_sonar1;
    prev_sonar9 = curr_sonar9;
    prev_sonar18 = curr_sonar18;
    prev_sonar27 = curr_sonar27;
    prev_delta = delta;

    set_prev = 1;
}




void left_thruster_left (double VXlim, double VYlim)
{
    if (curr_velx > (-VXlim) && curr_vely < VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (224 + angle))
                || (curr_angle > (226 + angle) && curr_angle < 359))
        {
            Rotate ((225 + angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (225 + angle) - curr_angle;
            }
        }
        Left_Thruster ((VXlim -
                        fmax (0, curr_velx)) / VXlim);
    }
    else if (curr_vely >= VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (224 + angle))
                || (curr_angle > (226 + angle) && curr_angle < 359))
        {
            Rotate ((225 + angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (225 + angle) - curr_angle;
            }
        }
        Left_Thruster (0.0);
    }
    else
    {
        if ((curr_angle > 0 && curr_angle < (314 - angle))
                || (curr_angle > (316 - angle) && curr_angle < 359))
        {
            Rotate ((315 - angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (315 - angle) - curr_angle;
            }
        }
        Left_Thruster ((VXlim -
                        fmax (0, curr_velx)) / VXlim);
    }

}

void left_thruster_right (double VXlim, double VYlim)
{
    if (curr_velx < VXlim && curr_vely < VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (314 - angle))
                || (curr_angle > (316 - angle) && curr_angle < 359))
        {
            Rotate ((315 - angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (315 - angle) - curr_angle;
            }
        }
        Left_Thruster ((VXlim -
                        fmax (0, curr_velx)) / VXlim);
    }
    else if (curr_vely >= VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (314 - angle))
                || (curr_angle > (316 - angle) && curr_angle < 359))
        {
            Rotate ((315 - angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (315 - angle) - curr_angle;
            }
        }
        Left_Thruster (0.0);
    }
    else
    {
        if ((curr_angle > 0 && curr_angle < (224 + angle))
                || (curr_angle > (226 + angle) && curr_angle < 359))
        {
            Rotate ((225 + angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (255 + angle) - curr_angle;
            }
        }
        Left_Thruster ((VXlim -
                        fmax (0, curr_velx)) / VXlim);
    }
}

void left_thruster_collision (void)
{
    if (curr_velx > 0)
    {
        //Left_Thruster (0.0);
        if ((curr_angle > 0 && curr_angle < (224 + comp_angle))
                || (curr_angle > (226 + comp_angle) && curr_angle < 359))
        {
            Rotate ((225 + comp_angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (255 + angle) - curr_angle;
            }
        }
        Left_Thruster (0.50);
    } else {
        //Left_Thruster (0.0);
        if ((curr_angle > 0 && curr_angle < (314 - comp_angle))
                || (curr_angle > (316 - comp_angle) && curr_angle < 359))
        {
            Rotate ((315 - comp_angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (315 - angle) - curr_angle;
            }
        }
        Left_Thruster (0.50);
    }
}

void right_thruster_left (double VXlim, double VYlim) //45 135
{
    if (curr_velx > (-VXlim) && curr_vely < VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (44 + angle))
                || (curr_angle > (46 + angle) && curr_angle < 359))
        {
            Rotate ((45 + angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (45 + angle) - curr_angle;
            }
        }
        Right_Thruster ((VXlim -
                         fmax (0, curr_velx)) / VXlim);
    }
    else if (curr_vely >= VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (44 + angle))
                || (curr_angle > (46 + angle) && curr_angle < 359))
        {
            Rotate ((45 + angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (45 + angle) - curr_angle;
            }
        }
        Right_Thruster (0.0);
    }
    else
    {
        if ((curr_angle > 0 && curr_angle < (134 - angle))
                || (curr_angle > (136 - angle) && curr_angle < 359))
        {
            Rotate ((135 - angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (135 - angle) - curr_angle;
            }
        }
        Right_Thruster ((VXlim -
                         fmax (0, curr_velx)) / VXlim);
    }
}

void right_thruster_right (double VXlim, double VYlim) //135 45
{
    if (curr_velx < VXlim && curr_vely < VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (134 - angle))
                || (curr_angle > (136 - angle) && curr_angle < 359))
        {
            Rotate ((135 - angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (135 - angle) - curr_angle;
            }
        }

        Right_Thruster ((VXlim -
                         fmax (0, curr_velx)) / VXlim);
    }
    else if (curr_vely >= VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (134 - angle))
                || (curr_angle > (136 - angle) && curr_angle < 359))
        {
            Rotate ((135 - angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (135 - angle) - curr_angle;
            }
        }
        Right_Thruster (0.0);
    }
    else
    {
        if ((curr_angle > 0 && curr_angle < (44 + angle))
                || (curr_angle > (46 + angle) && curr_angle < 359))
        {
            Rotate ((45 + angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (45 + angle) - curr_angle;
            }
        }
        Right_Thruster ((VXlim -
                         fmax (0, curr_velx)) / VXlim);
    }
}

void right_thruster_collision (void) // 45 135
{
    if (curr_velx > 0)
    {
        if ((curr_angle > 0 && curr_angle < (44 + comp_angle))
                || (curr_angle > (46 + comp_angle) && curr_angle < 359))
        {
            Rotate ((45 + comp_angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (45 + comp_angle) - curr_angle;
            }
        }
        Right_Thruster (0.50);
    } else {
        if ((curr_angle > 0 && curr_angle < (134 - comp_angle))
                || (curr_angle > (136 - comp_angle) && curr_angle < 359))
        {
            Rotate ((135 - comp_angle) - curr_angle);
            if (angle_broke == 1)
            {
                curr_angle = (135 - comp_angle) - curr_angle;
            }
        }
        Right_Thruster (0.50);
    }
}

void main_thruster_left (double VXlim, double VYlim) //315  45
{
    if (curr_velx < VXlim && curr_vely < VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (314 + angle))
                || (curr_angle > (316 + angle) && curr_angle < 359))
        {
            if(curr_angle > 0 && curr_angle <270) {
                Rotate (-curr_angle - (45 - angle));
                if (angle_broke == 1)
                {
                    curr_angle = -curr_angle - (45 - angle);
                }
            } else {
                Rotate ((315 + angle) - curr_angle);
                if (angle_broke == 1)
                {
                    curr_angle = (315 + angle) - curr_angle;
                }


            }
        }
        Main_Thruster ((VXlim -
                        fmax (0, curr_velx)) / VXlim);
    }
    else if (curr_vely >= VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (314 + angle))
                || (curr_angle > (316 + angle) && curr_angle < 359))
        {
            if(curr_angle > 0 && curr_angle <270) {
                Rotate (-curr_angle - (45 - angle));
                if (angle_broke == 1)
                {
                    curr_angle = -curr_angle - (45 - angle);
                }
            }
            else {
                Rotate ((315 + angle) - curr_angle);
                if (angle_broke == 1)
                {
                    curr_angle = (315 + angle) - curr_angle;
                }

            }
        }
        Main_Thruster (0.0);
    }
    else
    {
        if ((curr_angle > 0 && curr_angle < (44 - angle))
                || (curr_angle > (46 - angle) && curr_angle < 359))
        {
            if (curr_angle > 270) {
                Rotate ((360 - curr_angle) + (45 - angle));
                if (angle_broke == 1)
                {
                    curr_angle = ((360 - curr_angle) + (45 - angle));
                }
            }
            else {
                Rotate ((45 - angle) - curr_angle);
                if (angle_broke == 1)
                {
                    curr_angle = ((45 - angle) - curr_angle);
                }

            }
        }
        Main_Thruster ((VXlim -
                        fmax (0, curr_velx)) / VXlim);
    }
}

void main_thruster_right (double VXlim, double VYlim) //45 315
{
    if (curr_velx < VXlim && curr_vely < VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (44 - angle))
                || (curr_angle > (46 - angle) && curr_angle < 359))
        {
            if (curr_angle > 270) {
                Rotate ((360 - curr_angle) + (45 - angle));
                if (angle_broke == 1)
                {
                    curr_angle = ((360 - curr_angle) + (45 - angle));
                }
            }
            else {
                Rotate ((45 - angle) - curr_angle);
                if (angle_broke == 1)
                {
                    curr_angle = ((45 - angle) - curr_angle);
                }
            }
        }
        Main_Thruster ((VXlim -
                        fmax (0, curr_velx)) / VXlim);
    }
    else if (curr_vely >= VYlim)
    {
        if ((curr_angle > 0 && curr_angle < (44 - angle))
                || (curr_angle > (46 - angle) && curr_angle < 359))
        {
            if (curr_angle > 270) {
                Rotate ((360 - curr_angle) + (45 - angle));
                if (angle_broke == 1)
                {
                    curr_angle = ((360 - curr_angle) + (45 - angle));
                }
            }
            else {
                Rotate ((45 - angle) - curr_angle);
                if (angle_broke == 1)
                {
                    curr_angle = ((45 - angle) - curr_angle);
                }
            }
        }
        Main_Thruster (0.0);
    }
    else
    {
        if ((curr_angle > 0 && curr_angle < (314 + angle))
                || (curr_angle > (316 + angle) && curr_angle < 359))
        {
            if(curr_angle > 0 && curr_angle <270) {
                Rotate (-curr_angle - (45 - angle));
                if (angle_broke == 1)
                {
                    curr_angle = -curr_angle - (45 - angle);
                }
            }
            else {
                Rotate ((315 + angle) - curr_angle);
                if (angle_broke == 1)
                {
                    curr_angle = (315 + angle) - curr_angle;
                }
            }
        }
        Main_Thruster ((VXlim -
                        fmax (0, curr_velx)) / VXlim);
    }
}

void main_thruster_collision (void) // 315 45
{
    if (curr_velx > 0)
    {
        if ((curr_angle > 0 && curr_angle < (314 + comp_angle))
                || (curr_angle > (316 + comp_angle) && curr_angle < 359))
        {
            if(curr_angle > 0 && curr_angle <270) {
                Rotate (-curr_angle - (45 - comp_angle));
                if (angle_broke == 1)
                {
                    curr_angle = -curr_angle - (45 - comp_angle);
                }
            }
            else {
                Rotate ((315 + comp_angle) - curr_angle);
                if (angle_broke == 1)
                {
                    curr_angle = (315 + comp_angle) - curr_angle;
                }
            }
        }
        Main_Thruster (0.50);
    } else {
        if ((curr_angle > 0 && curr_angle < (44 - comp_angle))
                || (curr_angle > (46 - comp_angle) && curr_angle < 359))
        {
            if (curr_angle > 270) {
                Rotate ((360 - curr_angle) + (45 - comp_angle));
                if (angle_broke == 1)
                {
                    curr_angle = (360 - curr_angle) + (45 - comp_angle);
                }
            }
            else {
                Rotate ((45 - comp_angle) - curr_angle);
                if (angle_broke == 1)
                {
                    curr_angle = (45 - comp_angle) - curr_angle;
                }
            }
        }
        Main_Thruster (0.50);
    }
}


void straight_up() {
    if (strcmp (working_thruster, "left") == 0) {
        if (curr_vely > 2.0)
        {
            Left_Thruster (0.0);
        } else {
            if ((curr_angle>0 &&curr_angle<269) || (curr_angle>271&&curr_angle<359)) {
                Rotate(270-curr_angle);
                if (angle_broke == 1)
                {
                    curr_angle = 270 - curr_angle;
                }
            }
            Left_Thruster (1.0);
        }
    } else if (strcmp (working_thruster, "right") == 0) {
        if (curr_vely > 2.0)
        {
            Right_Thruster (0.0);
        } else {
            if ((curr_angle>0 &&curr_angle<89) || (curr_angle>91&&curr_angle<359)) {
                Rotate(90-curr_angle);
                if (angle_broke == 1)
                {
                    curr_angle = 90 - curr_angle;
                }
            }
            Right_Thruster (1.0);
        }
    } else if (strcmp (working_thruster, "main") == 0) {
        if (curr_vely > 2.0)
        {
            Main_Thruster (0.0);
        } else {
            if (curr_angle>1&&curr_angle<359)
            {
                if (curr_angle>=180) {
                    Rotate(360-curr_angle);
                    if (angle_broke == 1)
                    {
                        curr_angle = - curr_angle;
                    }
                }
                else {
                    Rotate(-curr_angle);
                    if (angle_broke == 1)
                    {
                        curr_angle = - curr_angle;
                    }
                }
            }
            Main_Thruster (1.0);
        }
    } else {
        if (curr_vely > 2.0)
        {
            Main_Thruster (0.0);
        }
        else
        {
            Main_Thruster (1.0);
        }
    }
}





// 2 functions simultaneously runs, starts with Safety_Override

// Safety_Override:
// 1: checks if something near x direction
//      - rotates to the right direction
//      - turns on the thrusters in the opp direction to avoid collision
// 2: checks if something near y direction
//      - turn on thrusters unless going too fast

// Lander_Control:
// 1: checks x dist to platform
//      - adds a slower and slower speed limit as get closer
// 2: checks y dist to platform
//      - adds a slower and slower speed limit as get closer
// 3: move x in the right direction
//      - if speed past limit slow down
// 4: move y in the right direction
//      - if speed past limit slow down


/*//left down (bottom facing right)
   if ((curr_angle>0 &&curr_angle<269) || (curr_angle>271&&curr_angle<359)){
   Rotate(270-curr_angle);
   } */

/*//right down   (bottom facing left)
   if ((curr_angle>0 &&curr_angle<89) || (curr_angle>91&&curr_angle<359)){
   Rotate(90-curr_angle);
   } */

/*//upside down (left and right reversed)
   if ((curr_angle>0 &&curr_angle<179) || (curr_angle>181&&curr_angle<359)){
   Rotate(180-curr_angle);
   } */

/*//main down
   if (curr_angle>1&&curr_angle<359)
   {
   if (curr_angle>=180) Rotate(360-curr_angle);
   else Rotate(-curr_angle);
   } */

// left left 225
// left right 315
// right left 44
// right right 135
// main left 315
// main right 45
