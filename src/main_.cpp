//// General includes
//#include <vector>
//#include <cstdlib>
//#include <ctime>
//#include <boost/thread.hpp>

//// OpenCV includes
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

// ROS includes
#include <ros/ros.h>
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/PoseArray.h> 
//#include <tf/transform_datatypes.h>

// Map class
class PathPlanner {

  // ROS handle, publishers and subscribers
  ros::NodeHandle _nh, _priv_nh;
  ros::Subscriber _odm_sub;
  ros::Publisher _goal_pub, _next_pub;

  // Map and planner varibles
  cv::Mat _img_map;                         // Original map image
  cv::Mat _eroded_map;                      // Eroded map image
  std::vector<std::vector<int> > _occ_map;  // Occupancy map
  std::vector<cv::Point3i> _traj;           // Full trajectory
  std::vector<cv::Point> _waypoints;        // List of waypoints
  
  // Robot and goal positions 
  cv::Point _goal;
  cv::Point _robot;
  float _alpha;        // Robot rotation
  int _distance;       // Total distance between goal and robot
  double _current_x, _current_y, _last_x, _last_y;
  
  // Occupancy states
  typedef enum {
    OCC_GOAL     =  0,
    OCC_DRIVABLE = -1,
    OCC_OBSTACLE = -2
  } OccupanceStates;


  /**
   * Private map and planner function
   */

  // Private thread variables
  boost::thread m_Thread;
  bool _searching;

  // Thread for running the planner
  void runPlanner() {
    this->OccupancyMap();
    if( this->goalReachable() ) {
      this->Search();
      if( this->robotReachable() ) {
	this->Trajectory();
      }
      else {
	ROS_WARN("[PathPlanner]: Could not get a trajectory between goal and robot position (robot is either inside a wall or too close to a wall).");
      }
    }
    else {
      ROS_WARN("[PathPlanner]: Could not reach the goal position (position is either inside a wall or too close to a wall).");
    }
    this->_searching = false;
  }

  // Thread-starting-function
  void Plan() {
    if( !this->_searching ) {
      this->_searching = true;
      m_Thread = boost::thread( &PathPlanner::runPlanner, this);
    }
  }

  // Planner search function
  bool Search() {
    int i, j, dist;
    std::vector<cv::Point3i> queue;

    // Wavefront search from goal to robot
    cv::Point3i p; 
    queue.push_back(cv::Point3i( this->_goal.y, this->_goal.x, OCC_GOAL));
    while( !queue.empty() ) {

      // Get next element from the list
      p = queue.front(); 
      queue.erase( queue.begin() );	

      // Have the planner reached the robot?
      if( p.x == this->_robot.y &&
	  p.y == this->_robot.x ) {
	this->_distance = p.z;
	ROS_INFO("Distance to goal: %d", this->_distance);
	
	// Clear the list and clean upp the map
	while( !queue.empty() ) {
	  p = queue.front(); 
	  queue.erase( queue.begin() );
	  this->_occ_map[p.x][p.y] = OCC_DRIVABLE;
	}
	return true;
      }
      else {
	for( int k = 0; k < 4; k++ ) {
	  i = p.x;
	  j = p.y;
	  dist = p.z + 1;
	  if( this->getGridValue( k, i, j) == OCC_DRIVABLE ) { // ...changes i, j -index as well.
	    this->_occ_map[i][j] = dist; 
	    queue.push_back(cv::Point3i( i, j, dist));
	  }
	}
      }
    }
    return false;
  }

  // Calculate the closest trajectory
  void Trajectory() {
    this->_traj.clear();  // ...clear all previous waypoints

    // Search for a new trajectory
    int i = this->_robot.y, j = this->_robot.x;
    int dist = this->getGridValue( -1, i, j);
    this->_traj.push_back(cv::Point3i( i, j, dist));
    while( this->_traj.back().z != OCC_GOAL ) {

      // Find closest point in neghbourhood
      int _i, _j, _dist = this->_traj.back().z;
      for( int k = 0; k < 4; k++) {
	i = this->_traj.back().x;
	j = this->_traj.back().y;
	dist = this->getGridValue( k, i, j); 
	if( dist >= 0 && dist <= _dist ) {
	  
	  // Add some randomness to avoid straight lines
	  if( dist == _dist ) {
	    if( (int)(rand() % 2) == 0 ) {
	      _i = i;
	      _j = j;
	    }
	  }
	  else {
	    _i = i;
	    _j = j;
	  }
	  _dist = dist;
	}
      }
      this->_traj.push_back(cv::Point3i( _i, _j, _dist));
    }

    // Set waypoints along the trajectory
    int counter = 0;
    this->_waypoints.clear();
    std::vector<cv::Point3i>::iterator ite;
    for( ite = this->_traj.begin(); ite != this->_traj.end(); ++ite) {
      if( counter % 60 == 0 ) {
	this->_waypoints.push_back(cv::Point(ite->y, ite->x));
      }
      counter++;
    }
    // Remove current position and adding goal position
    this->_waypoints.erase( this->_waypoints.begin() );
    this->_waypoints.push_back(this->_goal);
  }

  // Calculate the occupancy map
  void OccupancyMap() { 
    for( int i = 0; i < this->_eroded_map.rows; i++) {
      for( int j = 0; j < this->_eroded_map.cols; j++) {
	if( this->_eroded_map.at<uchar>( i, j) < 128 ) {
	  this->_occ_map[i][j] = OCC_OBSTACLE;
	}
	else {
	  this->_occ_map[i][j] = OCC_DRIVABLE;
	}
      }
    }
    if( this->_goal.x >= 0 && this->_goal.y >= 0 ) {
      this->_occ_map[this->_goal.y][this->_goal.x] = OCC_GOAL;
    }    
  }

  /**
   * Helper functions
   */
  int getGridValue( int idx, int &i, int &j) {
    if( idx == 0 ) i--;
    else if( idx == 1 ) i++;
    else if( idx == 2 ) j--;
    else if( idx == 3 ) j++;
    return this->_occ_map[i][j];
  }
  bool goalReachable() {
    int x = this->_goal.x, y = this->_goal.y;
    if( (x >= 0 && x <= this->_eroded_map.cols) &&
	(y >= 0 && y <= this->_eroded_map.rows) ) {
      if(this->_eroded_map.at<uchar>(y, x) > 128) {
	return true;
      }
    }
    return false;    
  }
  bool robotReachable() {
    int x = this->_robot.x, y = this->_robot.y;
    if( (x >= 0 && x <= this->_eroded_map.cols) &&
	(y >= 0 && y <= this->_eroded_map.rows) ) {
      if(this->_eroded_map.at<uchar>(y, x) > 128) {
	return true;
      }
    }
    return false;
  }

  // NEED TO READ MAP DIMENSION AND RESOLUTION AS PARAMETERS
  cv::Point convertMetricToPixel(double x, double y) {
    x = (x + 12.5) / 0.05;
    y = (12.5 - y) / 0.05;
    return cv::Point( (int)x, (int)y);
  }
  geometry_msgs::Pose convertPixelToMetric(cv::Point p) {
    geometry_msgs::Pose waypoint;
    waypoint.position.x = p.x * 0.05 - 12.5;
    waypoint.position.y = -(p.y * 0.05 - 12.5);
    return waypoint;
  }

  /**
   * Callback function for robot odometry
   */
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    // Robot pose (odometry)
    geometry_msgs::Pose robot_pose = msg->pose.pose;
    this->_current_x = robot_pose.position.x;
    this->_current_y = robot_pose.position.y;
    this->_alpha = tf::getYaw(robot_pose.orientation);

    // Convert and set new robot position
    this->_robot = this->convertMetricToPixel( this->_current_x, this->_current_y);
  }

//////////////////////////////////////////////////////////////
//
// Public functions 
//
/////////////////////////////////////////////////////////////
public:

  // Constructor
  PathPlanner(cv::Mat &map) : _img_map(map), _searching(false), _priv_nh("~") {

    // Initiate default goal and robot position 
    this->_robot = cv::Point( -1, -1);
    this->_goal = cv::Point( -1, -1);

    // ROS publishers and subscribers
    this->_odm_sub = this->_nh.subscribe( "/robot_0/base_pose_ground_truth",
					  10,
					  &PathPlanner::odometryCallback,
					  this);
    this->_next_pub = this->_nh.advertise<geometry_msgs::Pose>( "/planner/waypoints/next", 10);
    this->_goal_pub = this->_nh.advertise<geometry_msgs::Pose>( "/planner/waypoints/goal", 10);

    // Inititat the occupancy map
    for( int i = 0; i < map.rows; i++) {
      this->_occ_map.push_back(std::vector<int>( map.cols, -1));
    }

    // Initialize random seed
    srand( time(NULL) );

    // Read ROS params
    float dist = -1.0;
    if( !_priv_nh.getParam("wall_distance", dist) ) {
      this->_img_map.copyTo(this->_eroded_map);
    }
    else {

      // Erode the map (to avoid a path that is too close to the walls)
      int sz = (int)(dist/0.05); 
      cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE,
						  cv::Size( 2*sz + 1, 2*sz+1),
						  cv::Point( sz, sz) );
      cv::erode( this->_img_map, this->_eroded_map, kernel);
    }
  }


  /**
   * Draw functions
   */
  cv::Mat drawPathPlanner() { 
    cv::Mat img;
    cv::addWeighted( this->_img_map, 0.9, this->_eroded_map, 0.1, 0.0, img);
    cv::cvtColor( img, img, CV_GRAY2BGR);

    // Define colors
    cv::Vec3b orange = cv::Vec3b( 32, 84, 233);  // Orange
    cv::Vec3b white = cv::Vec3b( 255, 255, 255); // White
    cv::Scalar purple = cv::Scalar(80, 39, 94);  // Blue
    cv::Scalar gray = cv::Scalar::all(51);       // Dark gray

    // Draw wavefront
    if( !this->_searching ) {
      for( int i = 0; i < this->_img_map.rows; i++) {
	for( int j = 0; j < this->_img_map.cols; j++) {
	  if( this->_occ_map[i][j] > OCC_DRIVABLE ) {
	    float scalar = this->_occ_map[i][j] / (float)this->_distance;
	    cv::Vec3b scaled_color = cv::Vec3b( orange[0] + (uchar)((white[0] - orange[0]) * scalar),
						orange[1] + (uchar)((white[1] - orange[1]) * scalar),
					        orange[2] + (uchar)((white[2] - orange[2]) * scalar) );
	    img.at<cv::Vec3b>(i,j) = scaled_color; 
	  }
	}
      }
    }

    // Rescale the image (just for display purposes)
    float scalar = 2.0;
    cv::resize( img, img, cv::Size(), scalar, scalar, CV_INTER_LINEAR);

    // Draw trajectory
    for( int i = 0; i < this->_waypoints.size(); i++) {
      cv::circle( img, this->_waypoints[i] * scalar, 4, purple, -1);
      cv::circle( img, this->_waypoints[i] * scalar, 12, purple, 4);

      // Draw the goal point...
      if( this->_waypoints[i].x == this->_goal.x &&
	  this->_waypoints[i].y == this->_goal.y ) {
	cv::circle( img, this->_goal * scalar, 24, purple, 4);
	cv::Point p = this->_goal * scalar;
	cv::line( img, 
		  cv::Point(p.x - 20, p.y - 20),
		  cv::Point(p.x + 20, p.y + 20), 
		  purple, 4 );
	cv::line( img, 
		  cv::Point(p.x + 20, p.y - 20),
		  cv::Point(p.x - 20, p.y + 20), 
		  purple, 4 );
      }
      else if( i == 0 ) {  // ..draw the next point
	cv::circle( img, this->_waypoints[i] * scalar, 24, purple, 4);
      }
    }

    // Draw the robot
    if( (this->_robot.x >= 0 &&  this->_robot.x < this->_img_map.cols) && 
	(this->_robot.y >= 0 &&  this->_robot.y < this->_img_map.rows) ) {
      cv::circle( img, this->_robot * scalar, 16, gray, 4);
      cv::Point p( cos(this->_alpha) * 28, -sin(this->_alpha) * 28);
      cv::line( img, this->_robot * scalar, this->_robot * scalar + p, gray, 4);
    }

    // Rescale the image (just for display purposes)
    cv::resize( img, img, cv::Size(), 1.0 / scalar, 1.0 / scalar, CV_INTER_AREA);

    // Return resulting image
    return img;
  }

  /** 
   * Reset function - clear the map and the goal
   */
  void reset() {
    this->_goal = cv::Point( -1, -1);
    this->OccupancyMap();
    this->_waypoints.clear();
  }

  /**
   * Replan the path every now and then
   */
  void replan() {
    double diff_x = (this->_current_x - this->_last_x);
    double diff_y = (this->_current_y - this->_last_y);
    double dist = sqrt( diff_x * diff_x + diff_y * diff_y );

    if( !this->_searching ) { 
      if( dist >= 0.5 && this->_goal.x >= 0 && this->_goal.x >= 0 ) {
	this->_last_x = this->_current_x;
	this->_last_y = this->_current_y;
	this->Plan();
      }
    }

    // Publish waypoints
    if( !this->_waypoints.empty() ) {
      geometry_msgs::Pose next_point = this->convertPixelToMetric(this->_waypoints.front());
      geometry_msgs::Pose goal_point = this->convertPixelToMetric(this->_goal);
      this->_next_pub.publish(next_point);
      this->_goal_pub.publish(goal_point);
    }
  } 

  /** 
   * Callback and wrapper function for handle mouse click.
   * Used for setting the goal position.
   */
  static void clickHandler(int event, int x, int y, int flags, void *param) {
    PathPlanner *self = static_cast<PathPlanner*>(param);
    self->clickHandleWrapper( event, x, y, flags);
  }
  void clickHandleWrapper(int event, int x, int y, int flags) {
    if  ( event == cv::EVENT_LBUTTONDOWN ) {
      if( this->_eroded_map.at<uchar>(y, x) < 128 ) {
	ROS_WARN("[PathPlanner]: Could not set goal position (position is either inside a wall or too close to a wall).");
      }
      else {

	// Set new goal (and plan the path)
	this->_goal = cv::Point(x, y);  
	this->Plan();
      }
    }
  }

  /**
   * Access for the ROS node handle status function
   */
  bool ok() { return _nh.ok(); }
};

//////////////////////////////////////////////////////////////
//
// Main function
//
/////////////////////////////////////////////////////////////
int main(int argc, char *argv[])  {

  // Init the connection with the ROS system  
  ros::init(argc, argv, "intro_path_planner_node");

  // Read image 
  cv::Mat img;
  if( argc != 2 ) {
    ROS_ERROR("[PathPlanner]: No map image given.");
    return -1;
  }
  else {
    img = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    if( !img.data ) {
      ROS_ERROR("[PathPlanner]: Could not open map image.");
      return -1;
    }
  }

  // Create a class instance
  PathPlanner planner(img);

  // Initialize the GUI
  const char *window = "intro_path_planner";
  cv::namedWindow(window, CV_WINDOW_AUTOSIZE);

  // Set the callback function for mouse event
  cv::setMouseCallback( window, PathPlanner::clickHandler, &planner);

  // ROS main loop
  ros::Rate loop_rate(10);
  while (planner.ok()) {

    // Dsiplay frame on screen
    cv::imshow( window, planner.drawPathPlanner());

    // Check for key input
    char key = (char)cv::waitKey(30);
    if( key == 27 || !planner.ok() ) {
      break;
    }
    else if( key == 'r' || key == 'R' ) {
      ROS_INFO("[PathPlanner]: Goal and map cleared.");
      planner.reset();
    }
    else if( key == 'h' || key == 'H' ) {
      std::cout<<"Commands:"<<std::endl;
      std::cout<<"---------"<<std::endl;
      std::cout<<"Righ-click   Set new goal position."<<std::endl;      
      std::cout<<"H, h         Help menu."<<std::endl;
      std::cout<<"R, r         Reset goal position."<<std::endl;
      std::cout<<"ESC          Exit."<<std::endl;
      std::cout<<"---------"<<std::endl;
    }
    
    // Replan every 0.5m of movement
    planner.replan();

    // Check for incomming sensor messages
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return 0;
}
