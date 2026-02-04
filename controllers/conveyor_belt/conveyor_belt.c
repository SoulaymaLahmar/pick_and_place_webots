#include <stdlib.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <assert.h>
#include <stdio.h>

int main(int argc, char *argv[]) {
  wb_robot_init();
  assert(argc == 3);
  
  const int timestep = (int)wb_robot_get_basic_time_step();
  
  double speed;
  sscanf(argv[1], "%lf", &speed);
  
  double timer;
  sscanf(argv[2], "%lf", &timer);
  
  WbDeviceTag belt_motor = wb_robot_get_device("belt_motor");
  wb_motor_set_position(belt_motor, INFINITY);
  wb_motor_set_velocity(belt_motor, speed);
  
  WbDeviceTag ds = wb_robot_get_device("DS");
  wb_distance_sensor_enable(ds, timestep);
  
  int object_detected = 0;
  int waiting_for_pickup = 0;
  int wait_counter = 0;
  
  int max_wait_steps = (int)(15000.0 / timestep);  // 15 seconds timeout
  int post_exit_delay = (int)(1000.0 / timestep);
  
  printf("Belt controller ready\n");
  
  while (wb_robot_step(timestep) != -1) {
    double distance = wb_distance_sensor_get_value(ds);
    double threshold = 900;
    
    if (!object_detected && !waiting_for_pickup && distance < threshold) {
      printf("Object detected - STOPPING BELT IMMEDIATELY\n");
      object_detected = 1;
      
      // Stop belt immediately
      wb_motor_set_velocity(belt_motor, 0.0);
      
      // Wait for object to settle (physics stabilization)
      int settle_time = (int)(500.0 / timestep);  // 0.5 seconds
      for (int i = 0; i < settle_time; i++) {
        wb_robot_step(timestep);
      }
      
      printf("Belt stopped - object should be stationary now\n");
      waiting_for_pickup = 1;
      wait_counter = 0;
    }
    
    if (waiting_for_pickup) {
      wait_counter++;
      
      if (distance >= threshold) {
        printf("Object removed - restarting belt\n");
        object_detected = 0;
        waiting_for_pickup = 0;
        
        for (int i = 0; i < post_exit_delay; i++) {
          wb_robot_step(timestep);
        }
        
        wb_motor_set_velocity(belt_motor, speed);
        wait_counter = 0;
      }
      else if (wait_counter >= max_wait_steps) {
        printf("Timeout - restarting belt\n");
        object_detected = 0;
        waiting_for_pickup = 0;
        wb_motor_set_velocity(belt_motor, speed);
        wait_counter = 0;
        
        for (int i = 0; i < post_exit_delay * 2; i++) {
          wb_robot_step(timestep);
        }
      }
    }
  }
  
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}