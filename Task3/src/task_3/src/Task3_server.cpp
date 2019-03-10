#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <task_3/Task3Action.h>

#define max_radius 2500 // Radius around the center of the frame that Kalman center can lie

class Task3Action
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<task_3::Task3Action> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  task_3::Task3Feedback feedback_;
  task_3::Task3Result result_;

public:

  Task3Action(std::string name) :
    as_(nh_, name, boost::bind(&Task3Action::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~Task3Action(void)
  {
  }

  void executeCB(const task_3::Task3GoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.motion_sequence.clear();

    // publish info to the console for the user
    ROS_INFO("%s: Executing from initial position", action_name_.c_str());

    // start executing the action
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        return ;
      }
    
      // dist_x and dist_y are square of the distances in respective directions

      float dist_x = (goal->KF_center_x - goal->frame_center_x) * (goal->KF_center_x - goal->frame_center_x) ;
      float dist_y = (goal->KF_center_y - goal->frame_center_y) * (goal->KF_center_y - goal->frame_center_y) ;

      // If Kalman Filter is not detected

      if (goal->KF_center_x == 0 && goal->KF_center_y == 0) {
          feedback_.motion_sequence.push_back("Nothing Detected , Default Search Motion , Move Backwards");
          result_.motion = "Nothing Detected ,  Default Search Motion , Move Backwards" ;
      }

      // K filter center is close to the frame center

      else if (dist_x <= max_radius && dist_y <= max_radius) {
          feedback_.motion_sequence.push_back("Move Forward");
          result_.motion = "Move Forward" ;
      }

      // close in x direction but far in y direction

      else if (dist_x <= max_radius) {
        
        if (goal->KF_center_y > goal->frame_center_y) {    // Note : Y coordinate increases down the image
          feedback_.motion_sequence.push_back("Rotate Vertically Downwards");
          result_.motion = "Rotate Vertically Downwards" ;
        }
        
        else {
          feedback_.motion_sequence.push_back("Rotate Vertically Upwards");
          result_.motion = "Rotate Vertically Upwards" ;
        }
      }

      // close in y direction but far in x direction

      else if (dist_y <= max_radius) {
        
        if (goal->KF_center_x > goal->frame_center_x) {      // Note : X coordinate increases towards right of the image
          feedback_.motion_sequence.push_back("Rotate Right");
          result_.motion = "Rotate Right" ;
        }
        
        else {
          feedback_.motion_sequence.push_back("Rotate Left");
          result_.motion = "Rotate Left" ;
        }
      }

      // Far in both directions , so so first try to get closer in x direction

      else {
          if (goal->KF_center_x >= goal->frame_center_x ) {
            feedback_.motion_sequence.push_back("Rotate Right");
            result_.motion = "Rotate Right" ;
          }
          else {
            feedback_.motion_sequence.push_back("Rotate Left");
            result_.motion = "Rotate Left" ;
          }
      }

      // publish the feedback
      as_.publishFeedback(feedback_);
      

      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "task3");

  Task3Action task3("task3");
  ros::spin();

  return 0;
}
