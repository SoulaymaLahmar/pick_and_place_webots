#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 32
#define L1 0.61 // upper arm
#define L2 0.58 // forearm
#define L3 0.109  // wrist link

typedef struct {
  double x;
  double y;
  double z;
} Position;

Position wait = {0.05, -1, -0.3};           
Position depose = {0, 1, -0.6};

void inverse_kinematics(Position target, double *shoulder_pan, double *shoulder_lift, double *elbow, double *wrist1) {
  *shoulder_pan = atan2(target.y, target.x);
  double r = sqrt(target.x*target.x + target.y*target.y);
  double z = target.z;
  double D = (r*r + z*z - L1*L1 - L2*L2) / (2 * L1 * L2);
  if(D > 1.0) D = 1.0;
  if(D < -1.0) D = -1.0;
  *elbow = acos(D);
  *shoulder_lift = atan2(z, r) - atan2(L2 * sin(*elbow), L1 + L2 * cos(*elbow));
  *wrist1 = -(*shoulder_lift + *elbow);
}

void move_arm(WbDeviceTag pan, WbDeviceTag shoulder, WbDeviceTag elbow,
              WbDeviceTag wrist1, WbDeviceTag wrist2, WbDeviceTag wrist3,
              Position pos) {
  double sh_pan, sh_lift, elb, w1;
  inverse_kinematics(pos, &sh_pan, &sh_lift, &elb, &w1);

  wb_motor_set_position(pan,      sh_pan);
  wb_motor_set_position(shoulder, sh_lift);
  wb_motor_set_position(elbow,    elb);
  wb_motor_set_position(wrist1,   w1 - M_PI/2);
  wb_motor_set_position(wrist2,   -M_PI/2);
}

int main() {
  wb_robot_init();

  WbDeviceTag pan      = wb_robot_get_device("shoulder_pan_joint");
  WbDeviceTag shoulder = wb_robot_get_device("shoulder_lift_joint");
  WbDeviceTag elbow    = wb_robot_get_device("elbow_joint");
  WbDeviceTag wrist1   = wb_robot_get_device("wrist_1_joint");
  WbDeviceTag wrist2   = wb_robot_get_device("wrist_2_joint");
  WbDeviceTag wrist3   = wb_robot_get_device("wrist_3_joint");

  WbDeviceTag finger1  = wb_robot_get_device("ROBOTIQ 2F-140 Gripper::left finger joint");
  WbDeviceTag finger2  = wb_robot_get_device("ROBOTIQ 2F-140 Gripper::right finger joint");

  wb_motor_set_velocity(pan,      1.0);
  wb_motor_set_velocity(shoulder, 1.0);
  wb_motor_set_velocity(elbow,    1.0);
  wb_motor_set_velocity(wrist1,   1.0);
  wb_motor_set_velocity(wrist2,   1.0);
  wb_motor_set_velocity(wrist3,   1.0);
  wb_motor_set_velocity(finger1,  2.0);
  wb_motor_set_velocity(finger2,  2.0);
  
  for(int i=0;i<50;i++) wb_robot_step(TIME_STEP);
  
  WbDeviceTag receiver = wb_robot_get_device("cam_receiver");
  wb_receiver_enable(receiver, TIME_STEP);
  wb_receiver_set_channel(receiver, 1);

  enum State { WAIT, MOVING_TO_TARGET, PICKING, RETURNING } state = WAIT;

  double x_target, y_target, z_target, w_target;

  printf("ARM CONTROLLER: Ready and waiting...\n");

  while (wb_robot_step(TIME_STEP) != -1) {
    switch (state) {

      case WAIT:
        move_arm(pan, shoulder, elbow, wrist1, wrist2, wrist3, wait);

        if (wb_receiver_get_queue_length(receiver) > 0) {
          const char* data = (const char*) wb_receiver_get_data(receiver);
          if (sscanf(data, "%lf,%lf,%lf,%lf", &x_target, &y_target, &z_target, &w_target) == 4) {
            printf("========================================\n");
            printf("RECEIVED: x=%.3f, y=%.3f, z=%.3f, angle=%.3f\n", 
                   x_target, y_target, z_target, w_target);
            printf("========================================\n");
            state = MOVING_TO_TARGET;
          }
          wb_receiver_next_packet(receiver);
        }
        break;

      case MOVING_TO_TARGET:
        {
          Position target;
          target.x = x_target;
          target.y = y_target;
          target.z = -0.3;  // Approach height
          
          double wrist_orientation = w_target;
          
          printf("Waiting for object to reach position...\n");
          // Reduced from 420 to 200 steps (6.4 seconds)
          for(int i=0;i<200;i++) wb_robot_step(TIME_STEP);

          printf("Moving to target position: x=%.3f, y=%.3f, z=%.3f\n", 
                 target.x, target.y, target.z);
          move_arm(pan, shoulder, elbow, wrist1, wrist2, wrist3, target);
          for(int i=0;i<50;i++) wb_robot_step(TIME_STEP);

          printf("Adjusting wrist orientation: %.3f rad\n", wrist_orientation);
          wb_motor_set_position(wrist3, wrist_orientation);
          for(int i=0;i<50;i++) wb_robot_step(TIME_STEP);

          printf("Lowering to pick height: z=-0.08\n");
          target.z = -0.08;
          move_arm(pan, shoulder, elbow, wrist1, wrist2, wrist3, target);
          for(int i=0;i<100;i++) wb_robot_step(TIME_STEP);  // More time to reach pick height
       
          state = PICKING;
        }
        break;

      case PICKING:
        printf("Closing gripper...\n");
        for(int i=0;i<50;i++) wb_robot_step(TIME_STEP);

        wb_motor_set_position(finger1, 0.100);
        wb_motor_set_position(finger2, 0.100);
        
        for(int i=0;i<50;i++) wb_robot_step(TIME_STEP);  // Wait for gripper to close
        printf("Object picked!\n");
       
        state = RETURNING;
        break;

      case RETURNING:
        printf("Returning to deposit position...\n");
        for(int i=0;i<10;i++) wb_robot_step(TIME_STEP);

        move_arm(pan, shoulder, elbow, wrist1, wrist2, wrist3, depose);
        for(int i=0;i<100;i++) wb_robot_step(TIME_STEP);

        printf("Releasing object...\n");
        wb_motor_set_position(finger1, 0.0);
        wb_motor_set_position(finger2, 0.0);
        
        for(int i=0;i<50;i++) wb_robot_step(TIME_STEP);
        printf("Cycle complete! Returning to wait position.\n\n");
        
        state = WAIT;
        break;
    }
  }

  wb_robot_cleanup();
  return 0;
}