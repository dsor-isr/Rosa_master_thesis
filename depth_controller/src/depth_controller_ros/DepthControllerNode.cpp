/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "DepthControllerNode.h"
#include "DepthControllerAlgorithm.h"

// @.@ Constructor
DepthControllerNode::DepthControllerNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();

  double kp = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/kp");

  double ki = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/ki");

  double kd = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/kd", 0.0);

  double kff = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/kff");

  double kff_d = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/kff_d", 0.0);
  
  double kff_lin_drag = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/kff_lin_drag", 0.0);

  double kff_quad_drag = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/kff_quad_drag", 0.0);

  double max_err = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/max_err");
    
  double min_err = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/min_err");

  double max_out = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/max_out");

  double min_out = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/min_out");

  double max_ref = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/max_ref");
  
  double min_ref = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/min_ref");

  this->debug = FarolGimmicks::getParameters<bool>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/debug");

  double m = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/m");

  double rho = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/rho");

  double g = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/g");

  double V = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/V");

  double Yu_dot = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/Yu_dot");


  double kff_lamb = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/kff_lamb");

  max_ref_value_ = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/max_ref");

  min_ref_value_ = FarolGimmicks::getParameters<double>(
    this->nh_, "/bluerov_heavy0/DepthControllerNode/min_ref");

  // double kp = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/kp");

  // double ki = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/ki");

  // double kd = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/kd", 0.0);

  // double kff = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/kff");

  // double kff_d = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/kff_d", 0.0);
  
  // double kff_lin_drag = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/kff_lin_drag", 0.0);

  // double kff_quad_drag = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/kff_quad_drag", 0.0);

  // double max_err = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/max_err");
    
  // double min_err = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/min_err");

  // double max_out = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/max_out");

  // double min_out = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/min_out");

  // double max_ref = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/max_ref");
  
  // double min_ref = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/min_ref");

  // this->debug = FarolGimmicks::getParameters<bool>(
  //   this->nh_, "/DepthControllerNode/debug");

  // double m = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/m");

  // double rho = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/rho");

  // double g = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/g");

  // double V = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/V");

  // double Yu_dot = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/Yu_dot");


  // double kff_lamb = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/kff_lamb");

  // max_ref_value_ = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/max_ref");

  // min_ref_value_ = FarolGimmicks::getParameters<double>(
  //   this->nh_, "/DepthControllerNode/min_ref");

  ROS_WARN_STREAM("Kp: " << kp);
  ROS_WARN_STREAM("Ki: " << ki);
  ROS_WARN_STREAM("Kff: " << kff);

  this->depth_controller_ = std::make_unique<PID_Controller>(kp, ki, kd, kff, kff_d, kff_lin_drag, kff_quad_drag, max_err, max_out, min_err, min_out, kff_lamb);
  
  this->depth_controller_->setBuyoancy(rho, g, V);

  this->last_cmd_ = ros::Time::now();
  // initialize variables
  ref_value_ = 0.0;
  ref_time_ = ros::Time(0.0);
  timeout_ref_ = ros::Duration(1.5);
  this->last_ref_ = -10000;
}

// @.@ Destructor
DepthControllerNode::~DepthControllerNode() {

  // +.+ shutdown publishers


  // +.+ shutdown subscribers


  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

// @.@ Member helper to load parameters from parameter server
void DepthControllerNode::loadParams() {
  ROS_INFO("Load the DepthControllerNode parameters");

  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);

}


// @.@ Member helper function to set up subscribers
void DepthControllerNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for DepthControllerNode");
  
  std::string state_topic = "/bluerov_heavy0/nav/filter/state";
  //std::string state_topic = "/bluerov_heavy0/gazebo/state";

  std::string depth_topic = "/bluerov_heavy0/ref/depth";
  
  this->state_sub_ = nh_.subscribe(state_topic, 1, &DepthControllerNode::stateCallback, this);

  this->depth_sub_ = nh_.subscribe(depth_topic, 1, &DepthControllerNode::depthCallback, this);

}


// @.@ Member helper function to set up publishers
void DepthControllerNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for DepthControllerNode");
  std::string forces_torques_topic = "/bluerov_heavy0/force_bypass";

  std::string debug_topic = "debug/depth";

  std::string integral_topic = "/integral_error";
  
  this->forces_torques_pub_ = nh_.advertise<auv_msgs::BodyForceRequest>(forces_torques_topic, 1);

  debug_pub_ = nh_.advertise<farol_msgs::mPidDebug>(debug_topic, 1);

  this->integral_error_pub_ = nh_.advertise<std_msgs::Float64>(integral_topic, 1);
}


// @.@ Member helper function to set up services
void DepthControllerNode::initializeServices() {
  ROS_INFO("Initializing Services for DepthControllerNode");
  change_gains_srv_ = nh_.advertiseService("/depth_controller/change_inner_gains", &DepthControllerNode::changeGainsService, this);
  change_ff_gains_srv_ = nh_.advertiseService("/depth_controller/change_ff_gains", &DepthControllerNode::changeFFGainsService, this);
  change_kff_lamb_srv_ = nh_.advertiseService("/depth_controller/change_kff_lamb", &DepthControllerNode::changeKff_lambService, this);
}


// @.@ Member helper function to set up the timer
void DepthControllerNode::initializeTimer() {
  timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &DepthControllerNode::timerIterCallback, this);
}

bool DepthControllerNode::validRef() {
  if (this->depth_controller_ == NULL)
    return false;

  if ((ros::Time::now() - ref_time_) < timeout_ref_) {
    // reactivate the controller if needed
    if (this->depth_controller_->disable) {
      last_cmd_ = ros::Time::now();
      this->depth_controller_->reset();
      this->depth_controller_->disable = false;
    }
    return true;
  }
  this->depth_controller_->disable = true; // disable controller
  return false;
}

// @.@ Where the magic should happen.
void DepthControllerNode::timerIterCallback(const ros::TimerEvent &event) {
  ros::Time tnow = ros::Time::now();

  double duration = (tnow - this->last_cmd_).toSec();
  double ref_value = this->depth_ref_;
  double error_p = this->depth_ref_ - this->depth_;
  double tau_w;
  
  if(!this->validRef()){
    if (debug) {
      debug_msg_.ref = 0.0;
      debug_msg_.ref_d = 0.0;
      debug_msg_.ref_d_filtered = 0.0;
      debug_msg_.state = this->depth_;
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
      debug_msg_.controller = "depth";

      debug_pub_.publish(debug_msg_);
    }
  }
  else{
    tau_w = this->depth_controller_->computeCommand(error_p, ref_value, duration, this->debug);

    // update body force request structure
    auv_msgs::BodyForceRequest msg;
    msg.header.stamp = ros::Time::now();
    
    
    msg.wrench.force.y = 0.0;
    msg.wrench.force.z = tau_w;

    msg.wrench.torque.x = 0.0;
    msg.wrench.torque.y = 0.0;
    msg.wrench.torque.z = 0.0;

    msg.disable_axis = {0, 0, 0, 0, 0, 0};

    this->forces_torques_pub_.publish(msg);

    if (this->debug){
      debug_msg_ = this->depth_controller_->getDebugInfo();
      debug_pub_.publish(debug_msg_);
    }

    this->last_cmd_ = tnow;
  }
  std_msgs::Float64 msg_int;
  msg_int.data = this->depth_controller_->getIntegralError();
  this->integral_error_pub_.publish(msg_int);
}

void DepthControllerNode::stateCallback(const auv_msgs::NavigationStatus &msg) {
  // update state
  this->surge_ = msg.body_velocity.x;
  this->sway_ = msg.body_velocity.y;
  this->heave_ = msg.body_velocity.z;

  this->roll_rate_ = this->depth_controller_->DegreesToRadians(msg.orientation_rate.x);
  this->pitch_rate_ = this->depth_controller_->DegreesToRadians(msg.orientation_rate.y);
  this->yaw_rate_ = this->depth_controller_->DegreesToRadians(msg.orientation_rate.z);

  this->depth_ = msg.position.depth;
}

void DepthControllerNode::depthCallback(const std_msgs::Float64 &msg){
  ref_time_ = ros::Time::now();
  if (max_ref_value_ == 0.0 && min_ref_value_ == 0)
    this->depth_ref_ = msg.data;
  else{
    // saturate references
    this->depth_ref_ = fmin(fmax(msg.data, min_ref_value_), max_ref_value_);
  }
  // if the reference changes reset pid
  if (this->last_ref_ != 10000){
    if (abs(this->last_ref_ - this->depth_ref_) > 2){
      this->depth_controller_->reset();
    }
  }
  this->last_ref_ = this->depth_ref_;
}

bool DepthControllerNode::changeGainsService(
    depth_controller::ChangeDepthInnerGains::Request &req,
    depth_controller::ChangeDepthInnerGains::Response &res) {

  bool control_changed{false};

  control_changed = true;
  this->depth_controller_->setGains(req.kp, req.ki, req.kd);


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

bool DepthControllerNode::changeFFGainsService(
    depth_controller::ChangeDepthFFGains::Request &req,
    depth_controller::ChangeDepthFFGains::Response &res) {

  bool control_changed{false};

  
  control_changed = true;
  this->depth_controller_->setFFGains(req.kff, req.kff_d, req.kff_lin_drag, req.kff_quad_drag);

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

bool DepthControllerNode::changeKff_lambService(
    depth_controller::ChangeKffLamb::Request &req,
    depth_controller::ChangeKffLamb::Response &res) {

  bool control_changed{false};

  this->depth_controller_->setKffLamb(req.kff_lamb);
  
  if (!control_changed) {
    res.success = false;
    res.message += "Bad control name Value";
  } else {
    res.success = true;
    res.message += "New gains are kff_lamb: " + std::to_string(req.kff_lamb);
  }

  return true;
}

/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "depth_controller_node"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type DepthControllerNode");

  // +.+ instantiate an DepthControllerNode class object and pass in pointers to nodehandle public and private for constructor to use
  DepthControllerNode depth_controller(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
