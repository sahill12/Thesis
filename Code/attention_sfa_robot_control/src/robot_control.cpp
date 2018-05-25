#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_datatypes.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <attention_sfa_robot_control/Control_parameterConfig.h>

#include <attention_signal/Signal.h>
#include <attention_signal/SignalArray.h>
#include <cmath>
#include <ctime>
#include <time.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <map>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <fstream>

ros::Publisher velPub;
ros::Time lastUpdate;
ros:: Publisher arrayVisPub;


struct AttentionSignals
{
	attention_signal::Signal msg;
	double decay;
};

std::map<std::string, AttentionSignals> attentionSignals;
// configuration, parameters

double SOI				= 1.2f;		// sphere of influence of obstacle
double ROBOT_R			= 0.3f;		// radius of robot

double GAIN_GOAL		= 300.0f;		// gains
double GAIN_AVOID		= 130.0f;

double MAX_SPEED		= 0.5f;		// speed limits
double MAX_TURNRATE		= 1.0f;
// 0.261 for 15 degree angle 

bool RUN				= true;	// enable or disable robot motion for testing
double DISTANCE_LIMIT	= 1.0f;		// to limit the distance

//camera pixel parameters function

double imageW 	= 640.0;
double fov = 60 * (M_PI / 180.0);
double focal_length = tan(fov / 2.0) * imageW / 2.0;

inline double x_to_visual_angle(double x)
{	
	return atan2(x, focal_length);
}

// some helper functions
inline double dtor (double angle)
{
	return angle / 180.0 * M_PI;
}

inline double rtod (double angle)
{
	return angle / M_PI * 180;
}

double mod (double a, double b)
{
	double q;
	if (b > 0)
	{	q = floor (a / b);
	}
	else
	{	q = ceil (a / b);
	}
	return a - (b * q);
}

inline double normalize (double angle)
{
	return mod (angle + M_PI, 2 * M_PI) - M_PI;
}

inline void saturate (double& value, const double lowerLimit, const double upperLimit)
{
	if (value < lowerLimit)
	{
		value = lowerLimit;
	}
	if (value > upperLimit)
	{
		value = upperLimit;
	}
}

visualization_msgs::Marker createArrowMarker (int id, double x, double y, double r, double g, double b, std::string name)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "GETbot/base_link";
	marker.ns = "vector forces";
	marker.header.stamp = ros::Time();
	marker.id = id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.05;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	geometry_msgs::Point point;
	marker.points.push_back (point);
	point.x = x;
	point.y = y;
	point.z = 0.0;
	marker.points.push_back (point);

	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.text = name;

	return marker;
}


// callback for changing parameters
void reconfigureCallback (attention_sfa_robot_control::Control_parameterConfig& cfg, uint32_t level)
{

	SOI						= cfg.soi;
	ROBOT_R					= cfg.robot_r;

	GAIN_GOAL				= cfg.gain_goal;
	GAIN_AVOID				= cfg.gain_avoid;
	
	MAX_SPEED				= cfg.max_speed;
	MAX_TURNRATE			= dtor (cfg.max_turnrate);
	
	RUN						= cfg.run;
	DISTANCE_LIMIT			= cfg.distance_limit;
}

//  callback for laser data
void laserCallback (const sensor_msgs::LaserScanConstPtr& laser, const nav_msgs::OdometryConstPtr& odom)
{	
	lastUpdate = ros::Time::now ();
	visualization_msgs::MarkerArray markerArray;
		
	// read out laserscanner properties
	const double ANGLE_MIN			= laser->angle_min;
	const double RANGE_MIN			= laser->range_min;
	const double RANGE_MAX			= laser->range_max;
	const double ANGLE_INCREMENT	= laser->angle_increment;
	const double SCAN_POINT_COUNT	= laser->ranges.size ();

	double GoalX, GoalY;
	double vectorGoalX;
	double vectorGoalY;
	double vectorSumX = 0.0;
	double vectorSumY = 0.0;
	int markerCount = 0;

	// Goes through attention_signal map to extract signal information
	for (const auto &element : attentionSignals)

	{
		const auto& signals = element.second;
		markerCount++;
		ROS_DEBUG_THROTTLE(1, "Attentional Influences '%s': (Strength / decay) = (%f / %f), prediction = %d, foa = (%d, %d)",signals.msg.type.c_str(), signals.msg.strength, signals.decay, signals.msg.prediction, signals.msg.point.x, signals.msg.point.y);

		if (signals.msg.prediction == 1.0)
		{   
			GoalX 							= cos(-x_to_visual_angle(double(signals.msg.point.x) - (imageW / 2.0)));
			GoalY							= sin(-x_to_visual_angle(double(signals.msg.point.x) - (imageW / 2.0)));
	
			 //calculate Goal vector - direction to go in order to reach goal
			vectorGoalX 					= GoalX * GAIN_GOAL * signals.msg.strength * signals.decay;
			vectorGoalY 					= GoalY * GAIN_GOAL * signals.msg.strength * signals.decay;
			markerArray.markers.push_back (createArrowMarker (markerCount, GoalX * signals.decay, GoalY * signals.decay, signals.msg.red, signals.msg.green ,signals.msg.blue ,signals.msg.type));
			vectorSumX 						+= vectorGoalX;
			vectorSumY 						+= vectorGoalY;
			ROS_INFO("Received from: %s, Vector value X: %d, Vector value Y: %d",signals.msg.type.c_str(), vectorGoalX, vectorGoalY);
		}

	}

	// ================================
	// calculate avoid-obstacle vector,
	double vectorAvoidX = 0;
	double vectorAvoidY = 0;

	for (int i = 0; i < SCAN_POINT_COUNT; i++)
	{	double range = laser->ranges[i];
		double angle = ANGLE_MIN + i * ANGLE_INCREMENT;

		if (range < RANGE_MIN || range > RANGE_MAX)
		{	// invalid measurement
			continue;
		}

		// consider robot as circle with radius ROBOT_R and obstacles detected by laser scanner as points
		if ((range - ROBOT_R) > SOI)
		{	// obstacle outside sphere of influence
			continue;
		}

		// calculate magnitude of avoid vector
		double magnitude = (SOI - range + ROBOT_R)  / SOI;
		magnitude = magnitude < 0 ? 0 : magnitude;			// rounding side effects - assure valid range

		// compute x and y component of avoid-obstacle vector
		double x = cos (angle) *  -magnitude;
		double y = sin (angle) *  magnitude;

		vectorAvoidX += x;
		vectorAvoidY += y;
	}
	double length = std::hypot (vectorAvoidX, vectorAvoidY);
	if (length > 0.1)
	{
		vectorAvoidX = vectorAvoidX / length * GAIN_AVOID;
		vectorAvoidY = vectorAvoidY / length * GAIN_AVOID;
	}

	ROS_INFO ("1 - vectorAvoidX: %f vectorAvoidY: %f  norm: %f  direction: %f ", vectorAvoidX, vectorAvoidY, sqrt (vectorAvoidX * vectorAvoidX  + vectorAvoidY * vectorAvoidY),
			atan2 (vectorAvoidY, vectorAvoidX)/M_PI*180);

	length = std::hypot (vectorAvoidX, vectorAvoidY);
	length = length < 0.1 ? 1 : length;					// rounding length - to avoid 0 divided 0 case
	//obstacle marker
	markerArray.markers.push_back (createArrowMarker (7, vectorAvoidX / length, vectorAvoidY / length, 0.0, 0.0, 1.0,"laser_data"));

	// ================================
	// behavior fusion
	double sumx = vectorAvoidX + vectorSumX;
	double sumy = vectorAvoidY + vectorSumY;

	//sum marker
	length = std::hypot (sumx, sumy);
	length = length < 0.1 ? 1 : length;      			// rounding length - to avoid 0 divided 0 case
	markerArray.markers.push_back (createArrowMarker (8, sumx / length, sumy / length, 1.0, 1.0, 0.0, "Sum"));

	// desired direction after fusion
	double desiredDirection = atan2 (sumy, sumx);

	ROS_INFO ("\n 2 - sumx:         %f, sumy:         %f  norm: %f  direction: %f\n", sumx, sumy, sqrt (sumx * sumx  + sumy * sumy), desiredDirection / M_PI * 180);

	// publisher to publish markers in RVIZ

	arrayVisPub.publish(markerArray);
	// since all calculations were relative to robot position directionError == desiredDirection
	double directionError = desiredDirection;

	// user callbacks will be called from within the ros::spin() call.
	// ================================
	// calculate and publish velocities
	geometry_msgs::Twist velocities;


	// normalize direction error to range +/- PI
	directionError = normalize (directionError);

	if (fabs(directionError) > M_PI_2)
	{	// if |direction error| greater than 90° use maximum turnrate
		velocities.angular.z = MAX_TURNRATE;
		// if |direction error| > 90° just use a minimum speed
		velocities.linear.x = 0.0;
		if (directionError < 0)
		{	velocities.angular.z *= -1;
		}
	}
	else
	{	// use p-controller to minimize the error
		velocities.angular.z = directionError / M_PI_2 * MAX_TURNRATE;
		// only if the direction error is small use the maximum speed
		velocities.linear.x = (M_PI_2 - fabs(directionError)) / M_PI_2 * MAX_SPEED;
	}

	// checks if all decay values are zero
	double decaySum = 0.0;
	for (const auto& signals : attentionSignals)
	{
		decaySum += signals.second.decay;

	}
	if (decaySum < 0.0001)
	{	
		velocities.linear.x = 0.0;
	}

	ROS_INFO ("3 - speed:        %f, turnrate:     %f\n", velocities.linear.x, velocities.angular.z / M_PI * 180);

	if (RUN) velPub.publish (velocities);
	printf ("\n");
}

void attentionSignalCallback (const attention_signal::SignalArrayConstPtr& SignalArrayMsg)
{
	for (const auto& signalMsg : SignalArrayMsg->signals)
	{
		attentionSignals[signalMsg.type].msg	= signalMsg;
		attentionSignals[signalMsg.type].decay	= 1.0;
	}
}



void timerCallback1(const ros::TimerEvent&)
{
	for (auto& element : attentionSignals)
	{
		auto& decay = element.second.decay;
		decay -= 0.01;
		saturate (decay, 0.0, 1.0);
	}
}

int main(int argc, char* argv[])
{
	ros::init (argc, argv, "robot_control");
	ros::NodeHandle nh;

	ROS_INFO ("Starting node ...\n");
	velPub = nh.advertise<geometry_msgs::Twist>("/GETbot/cmd_vel", 1);

	//Publisher to publish visualization messages in rviz
	arrayVisPub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 1 );
	// dynamic reconfigure
	dynamic_reconfigure::Server<attention_sfa_robot_control::Control_parameterConfig> srv;
	dynamic_reconfigure::Server<attention_sfa_robot_control::Control_parameterConfig>::CallbackType f = boost::bind (reconfigureCallback, _1, _2);
	srv.setCallback (f);

	//synchronous subscriber for laser and odometry data
	message_filters::Subscriber<sensor_msgs::LaserScan>	laserSub (nh, "/GETbot/laser_front/scan", 10);
	message_filters::Subscriber<nav_msgs::Odometry>		odomSub (nh, "/GETbot/odom", 10);
	
	//Subscriber for classifier nodes
	ros::Subscriber classifierSub = nh.subscribe("Attention_Signal",10, attentionSignalCallback);
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback1);
	
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> MyPolicy;
	MyPolicy policy (100);
	ros::Duration myDuration (0.05);
	policy.setMaxIntervalDuration (myDuration);
	message_filters::Synchronizer<MyPolicy> sync ((const MyPolicy) policy, laserSub, odomSub);
	sync.registerCallback (boost::bind(&laserCallback, _1, _2));

	//user callbacks will be called from within the ros::spin() call.
	//ros::spin() will not return until the node has been shutdown
	ros::Rate r (2);
	lastUpdate = ros::Time::now ();
	ros::Duration timeout (0.5);
	while (ros::ok ())
	{
		ros::Duration diff = ros::Time::now () - lastUpdate;
//		if (diff > timeout)
//		{
//			velPub.publish (geometry_msgs::Twist());
//		}
		ros::spinOnce();
	}

	
	return 0;

}
