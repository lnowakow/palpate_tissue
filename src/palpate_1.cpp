//
// Created by dvrk-1804 on 2021-11-22.
//

#define PI 3.14159265
#define MAX_FORCE 10 // units in Newtons
#define PALPATION_INTERVAL 0.01 // units in meters
#define TOTAL_PALPATION_DISTANCE 0.1 // units in meters
#define PALPATE_SERVO_DISTANCE 0.001 // units in meters

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

enum states {
  start, down, next, end
};

// Function Prototypes
void psm1Callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
void optoCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
geometry_msgs::TransformStamped palpateTissue(geometry_msgs::TransformStamped currentTransform);
void nextPSMPosition(const ros::Publisher& move_command,
                     geometry_msgs::TransformStamped& startTransform);
float palpatedDistance(const geometry_msgs::TransformStamped& currentTransform,
                       const geometry_msgs::TransformStamped& initialTranslation);
geometry_msgs::TransformStamped calcNextTransform(const geometry_msgs::TransformStamped& currentTransform,
                                                  const Eigen::Matrix3Xd translation_vector);

// Global Variables (very bad practice, but this is simple program)
geometry_msgs::TransformStamped PSMTransformStamped;
geometry_msgs::Wrench Wrench;
bool PSM1Called = false, OptoCalled = false;

int main(int argc, char **argv) {

  // initiate ros
  ros::init(argc, argv, "palpate_1");

  // set up subs and pubs
  ros::NodeHandle nh;
  ros::Subscriber m_psm_measured_cp, opto_force;
  ros::Publisher psm_move_cp, psm_servo_cp;

  // initialize target topics
  m_psm_measured_cp = nh.subscribe("/PSM1/measured_cp", 10, psm1Callback);
  opto_force = nh.subscribe("OptoForceWrench", 1000, optoCallback);
  psm_move_cp = nh.advertise<geometry_msgs::TransformStamped>("/PSM1/move_cp", 10);
  psm_servo_cp = nh.advertise<geometry_msgs::TransformStamped>("/PSM1/servo_cp", 10);

  // get data rate from rosparam server to set loop rate
  int speed;
  if (!nh.getParam("/speed", speed)) {
    nh.param("speed", speed, 1000);
  }

  ros::Rate rate(speed);


  std::cout << "Waiting for first callbacks.\n";
  // Wait for first callback to prevent premature entry into program.
  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    // Add wrench when opto is needed
    if (PSM1Called) break;
  }

  // Grab initial position of PSM
  geometry_msgs::TransformStamped psm_initial_transform = PSMTransformStamped;
  // temp transform to store start and end of palpation cycle
  geometry_msgs::TransformStamped psm_start_palpation;
  // Set initial state
  states state = start;

  std::cout << "Got callbacks. Let's begin.\n";
  while(ros::ok() && state != end) {
    std::cout << "The current state is: " << state << std::endl;
    switch (state) {
      case start:
        std::cout << "In Start State.\n";
        psm_start_palpation.transform = psm_initial_transform.transform;
        state = down;
        break;
      case down:
        std::cout << "Beginning to palpate tissue.\n";
        psm_servo_cp.publish(palpateTissue(PSMTransformStamped));
        // Real daVinci
        /*
        if (Wrench.force.z > MAX_FORCE) {
          state = next;
        }
         */
        std::cout << "Palpated distance: " << palpatedDistance(PSMTransformStamped, psm_start_palpation) << std::endl;
        if (palpatedDistance(PSMTransformStamped, psm_start_palpation) >= 0.01) {
          std::cout << "step 1\n";
          state = next;
        }
        break;
      case next:
        std::cout << "Moving to next position.\n";
        nextPSMPosition(psm_move_cp, psm_start_palpation);
        std::cout << "Total tissue traversed: " << palpatedDistance(PSMTransformStamped, psm_initial_transform) << '\n';
        if (palpatedDistance(PSMTransformStamped, psm_initial_transform) > TOTAL_PALPATION_DISTANCE) {
          std::cout << "Ending palpation cycles.\n";
          state = end;
          break;
        }
        if (palpatedDistance(PSMTransformStamped, psm_start_palpation) >= PALPATION_INTERVAL) {
          std::cout << "New initial position.\n";
          state = down;
          break;
        }
      case end:
        std::cout << "Done Palpating.\n";
        break;
    }
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

void psm1Callback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  PSMTransformStamped.transform = msg->transform;
  PSM1Called = true;
}

void optoCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  Wrench = msg->wrench;
  OptoCalled = true;
}

geometry_msgs::TransformStamped palpateTissue(geometry_msgs::TransformStamped currentTransform) {
  geometry_msgs::TransformStamped next_transform;
  Eigen::Matrix3Xd translation_vector(3,1);
  translation_vector << 0, 0, -PALPATE_SERVO_DISTANCE;

  next_transform = calcNextTransform(currentTransform, translation_vector);

  return next_transform;
}

void nextPSMPosition(const ros::Publisher& move_command,
                     geometry_msgs::TransformStamped& startTransform) {

  geometry_msgs::TransformStamped next_transform; //
  Eigen::Matrix3Xd translation_vector(3,1);
  translation_vector << 0, PALPATION_INTERVAL, 0;

  // Go up to starting position
  std::cout << "step 2\n";
  move_command.publish(startTransform);
  ros::Duration(1).sleep();

  next_transform = calcNextTransform(startTransform, translation_vector);

  // Go to next palpation position
  std::cout << "step 3\n";
  move_command.publish(next_transform);
  ros::Duration(1).sleep();
  // new start position
  startTransform.transform = next_transform.transform;

}

float palpatedDistance(const geometry_msgs::TransformStamped& currentTransform,
                       const geometry_msgs::TransformStamped& initialTranslation) {

  float delta_x = currentTransform.transform.translation.x - initialTranslation.transform.translation.x;
  float delta_y = currentTransform.transform.translation.y - initialTranslation.transform.translation.y;
  float delta_z = currentTransform.transform.translation.z - initialTranslation.transform.translation.z;
  return std::sqrt(pow(delta_x,2) + pow(delta_y,2) + pow(delta_z,2));
}
geometry_msgs::TransformStamped calcNextTransform(const geometry_msgs::TransformStamped& currentTransform,
                                                  const Eigen::Matrix3Xd translation_vector) {

  geometry_msgs::TransformStamped next_transform;
  Eigen::Isometry3d delta_matrix;
  Eigen::Matrix3d orientation_matrix;
  Eigen::Matrix3d rotation_matrix;
  float rotX = -45;
  orientation_matrix << cos(rotX*PI/180), 0.0, sin(rotX*PI/180),\
                                        0.0, 1.0,                0.0,\
                       -sin(rotX*PI/180), 0.0, cos(rotX*PI/180);
  rotation_matrix << 0,0,-1,-1,0,0,0,1,0;

  // New palpation position
  next_transform.transform = currentTransform.transform;
  delta_matrix = tf2::transformToEigen(next_transform);
  delta_matrix.matrix().block(0,3,3,1) += orientation_matrix*translation_vector;
  next_transform = tf2::eigenToTransform(delta_matrix);

  return next_transform;
}

