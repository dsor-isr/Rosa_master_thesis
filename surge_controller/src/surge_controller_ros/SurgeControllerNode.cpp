/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "SurgeControllerNode.h"
#include "SurgeControllerAlgorithm.h"

// @.@ Constructor
SurgeControllerNode::SurgeControllerNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();

  double kp = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/kp");

  double ki = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/ki");

  double kd = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/kd", 0.0);

  double kff = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/kff");

  double kff_d = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/kff_d", 0.0);
  
  double kff_lin_drag = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/kff_lin_drag", 0.0);

  double kff_quad_drag = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/kff_quad_drag", 0.0);

  double max_err = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/max_err");
    
  double min_err = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/min_err");

  double max_out = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/max_out");

  double min_out = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/min_out");

  double max_ref = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/max_ref");
  
  double min_ref = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/min_ref");

  this->debug = FarolGimmicks::getParameters<bool>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/debug");

  double m = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/m");

  double Yu_dot = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/Yu_dot");

  max_ref_value_ = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/max_ref");

  min_ref_value_ = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/SurgeControllerNode/min_ref");

  ROS_WARN_STREAM("Kp: " << kp);
  ROS_WARN_STREAM("Ki: " << ki);
  ROS_WARN_STREAM("Kff: " << kff);

  this->surge_controller_ = std::make_unique<PID_Controller>(kp, ki, kd, kff, kff_d, kff_lin_drag, kff_quad_drag, max_err, max_out, min_err, min_out);
  
  this->surge_controller_->setAddedMass(m, Yu_dot);

  this->last_cmd_ = ros::Time::now();
  // initialize variables
  ref_value_ = 0.0;
  ref_time_ = ros::Time(0.0);
  timeout_ref_ = ros::Duration(1.5);
}

// @.@ Destructor
SurgeControllerNode::~SurgeControllerNode() {

  // +.+ shutdown publishers


  // +.+ shutdown subscribers


  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

// @.@ Member helper to load parameters from parameter server
void SurgeControllerNode::loadParams() {
  ROS_INFO("Load the SurgeControllerNode parameters");

  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);

}


// @.@ Member helper function to set up subscribers
void SurgeControllerNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for SurgeControllerNode");

  std::string state_topic = "/bluerov_heavy0/nav/filter/state";
  //std::string state_topic = "/bluerov_heavy0/gazebo/state";

  std::string surge_topic = "/bluerov_heavy0/ref/surge";
  
  this->state_sub_ = nh_.subscribe(state_topic, 1, &SurgeControllerNode::stateCallback, this);

  this->surge_sub_ = nh_.subscribe(surge_topic, 1, &SurgeControllerNode::surgeCallback, this);
  
}


// @.@ Member helper function to set up publishers
void SurgeControllerNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for SurgeControllerNode");
  std::string forces_torques_topic = "/bluerov_heavy0/force_bypass";

  std::string debug_topic = "debug/surge";
  
  this->forces_torques_pub_ = nh_.advertise<auv_msgs::BodyForceRequest>(forces_torques_topic, 1);

  debug_pub_ = nh_.advertise<farol_msgs::mPidDebug>(debug_topic, 1);
}


// @.@ Member helper function to set up services
void SurgeControllerNode::initializeServices() {
  ROS_INFO("Initializing Services for SurgeControllerNode");
  change_gains_srv_ = nh_.advertiseService("/surge_controller/change_inner_gains", &SurgeControllerNode::changeGainsService, this);
  change_ff_gains_srv_ = nh_.advertiseService("/surge_controller/change_ff_gains", &SurgeControllerNode::changeFFGainsService, this);
}


// @.@ Member helper function to set up the timer
void SurgeControllerNode::initializeTimer() {
  timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &SurgeControllerNode::timerIterCallback, this);
}


bool SurgeControllerNode::validRef() {
  if (this->surge_controller_ == NULL)
    return false;

  if ((ros::Time::now() - ref_time_) < timeout_ref_) {
    // reactivate the controller if needed
    if (this->surge_controller_->disable) {
      last_cmd_ = ros::Time::now();
      this->surge_controller_->reset();
      this->surge_controller_->disable = false;
    }
    return true;
  }
  this->surge_controller_->disable = true; // disable controller
  return false;
}

// @.@ Where the magic should happen.
void SurgeControllerNode::timerIterCallback(const ros::TimerEvent &event) {
  ros::Time tnow = ros::Time::now();

  double duration = (tnow - this->last_cmd_).toSec();
  double ref_value = this->surge_ref_;
  double error_p = this->surge_ref_ - this->surge_;
  double tau_u;
  
  if(!this->validRef()){
    if (debug) {
      debug_msg_.ref = 0.0;
      debug_msg_.ref_d = 0.0;
      debug_msg_.ref_d_filtered = 0.0;
      debug_msg_.state = this->surge_;
      debug_msg_.error = 0.0;
      debug_msg_.error_saturated = 0.0;
      debug_msg_.ffTerm = 0.0;
      debug_msg_.ffDTerm = 0.0;
      debug_msg_.ffDragTerm = 0.0;
      debug_msg_.pTerm = 0.0;
      debug_msg_.iTerm = 0.0;
      debug_msg_.dTerm = 0.0;
      debug_msg_.output = 0.0;

      debug_msg_.header.stamp = ros::Time::now();
      debug_msg_.controller = "surge";

      debug_pub_.publish(debug_msg_);
    }
  }
  else{
    tau_u = this->surge_controller_->computeCommand(error_p, ref_value, duration, this->sway_, this->yaw_rate_, this->debug);

    // update body force request structure
    auv_msgs::BodyForceRequest msg;
    msg.header.stamp = ros::Time::now();
    
    msg.wrench.force.x = tau_u;
    msg.wrench.force.y = 0.0;
    msg.wrench.force.z = 0.0;

    msg.wrench.torque.x = 0.0;
    msg.wrench.torque.y = 0.0;
    msg.wrench.torque.z = 0.0;

    msg.disable_axis = {0, 0, 0, 0, 0, 0};

    this->forces_torques_pub_.publish(msg);

    if (this->debug){
      debug_msg_ = this->surge_controller_->getDebugInfo();
      debug_pub_.publish(debug_msg_);
    }

    this->last_cmd_ = tnow;
  }
}


void SurgeControllerNode::stateCallback(const auv_msgs::NavigationStatus &msg) {
  // update state
  this->surge_ = msg.body_velocity.x;
  this->sway_ = msg.body_velocity.y;
  this->heave_ = msg.body_velocity.z;

  this->roll_rate_ = this->surge_controller_->DegreesToRadians(msg.orientation_rate.x);
  this->pitch_rate_ = this->surge_controller_->DegreesToRadians(msg.orientation_rate.y);
  this->yaw_rate_ = this->surge_controller_->DegreesToRadians(msg.orientation_rate.z);
}

void SurgeControllerNode::surgeCallback(const std_msgs::Float64 &msg){
  ref_time_ = ros::Time::now();
  if (max_ref_value_ == 0.0 && min_ref_value_ == 0)
    this->surge_ref_ = msg.data;
  else
    // saturate references
    this->surge_ref_ = fmin(fmax(msg.data, min_ref_value_), max_ref_value_);
}

bool SurgeControllerNode::changeGainsService(
    surge_controller::ChangeSurgeInnerGains::Request &req,
    surge_controller::ChangeSurgeInnerGains::Response &res) {

  bool control_changed{false};

  if (req.kp >= 0 && req.ki >= 0 && req.kd >= 0){
    control_changed = true;
    this->surge_controller_->setGains(req.kp, req.ki, req.kd);
  }

  if (!control_changed) {
    res.success = false;
    res.message += "Bad control name " + req.inner_type;
  } else {
    res.success = true;
    res.message += "New " + req.inner_type + " gains are" +
                   " kp: " + std::to_string(req.kp) +
                   " ki: " + std::to_string(req.ki) +
                   " kd: " + std::to_string(req.kd);
  }

  return true;
}

bool SurgeControllerNode::changeFFGainsService(
    surge_controller::ChangeSurgeFFGains::Request &req,
    surge_controller::ChangeSurgeFFGains::Response &res) {

  bool control_changed{false};

  if (req.kff >= 0 && req.kff_d >= 0 && req.kff_lin_drag >= 0 && req.kff_quad_drag >= 0){
    control_changed = true;
    this->surge_controller_->setFFGains(req.kff, req.kff_d, req.kff_lin_drag, req.kff_quad_drag);
  }

  if (!control_changed) {
    res.success = false;
    res.message += "Bad control name " + req.inner_type;
  } else {
    res.success = true;
    res.message += "New " + req.inner_type + " gains are" +
                   " kff: " + std::to_string(req.kff) +
                   " kff_d: " + std::to_string(req.kff_d) +
                   " kff_lin_drag: " + std::to_string(req.kff_lin_drag) +
                   " kff_quad_drag: " + std::to_string(req.kff_quad_drag);
  }

  return true;
}
/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "surge_controller_node"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type SurgeControllerNode");

  // +.+ instantiate an SurgeControllerNode class object and pass in pointers to nodehandle public and private for constructor to use
  SurgeControllerNode surge_controller(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
