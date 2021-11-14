#include <ros/ros.h>

#include <std_msgs/String.h>

#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include <san_feature_extractor/peopleData.h>
#include <san_feature_extractor/sanData.h>
#include <san_feature_extractor/TrajectoryPoint.h>
#include <san_feature_extractor/Trajectory.h>
#include <san_feature_extractor/Features.h>
#include <san_feature_extractor/newMarkerMsg.h>

#include <hallway/hallwayMsg.h>

#include <san_nodes/Appscore.h>
#include <san_nodes/Classify.h>

#include <tf/transform_listener.h>

#include <string.h>
#include <math.h>
#include <fstream>
#include <ctime>


using namespace std;

struct personDetails{
  	string frame_id;
  	string  personId;
  	//float distanceFromPR2;
  	float distance_travelled;
  	double posx, posy;
  	double t0, t1, speed, xdistance, ydistance;
}personData; 

string peoplename[100];
int personFound = 0, hallwayFound = 0;

ros::ServiceClient clientClassify;
ros::ServiceClient clientAppscore;

vector<string> SANfeatures;

int flag = 0, robotFound = 0, testflag = 0;
float personInitialX = 0, personInitialY = 0, currentRobotPositionX = 0; 
float odom_0_OriginX = 0, odom_0_OriginY = 0, odom_1_OriginX = 0, odom_1_OriginY = 0;
float m = 0 ,c = 0, width = 0;
float poseX = 0, poseY = 0;
float intialPoseX = 0 , intialPoseY = 0, goalPoseX = 0 , goalPoseY = 0; 

ofstream outfile ("Dataset.txt");
ofstream trajfile ("TrajectoryPoints.txt");
ofstream robot ("robot.txt");
ofstream human ("human.txt");
ofstream robotVel ("robotVel.txt");
ofstream humanVel ("humanVel.txt");
ofstream probability ("probability.txt");

//geometry_msgs::Point robotPosition;
geometry_msgs::Point robotGoalPosition;
geometry_msgs::Point robotFuturePosition;

geometry_msgs::Point hallwayPointL1;
geometry_msgs::Point hallwayPointL2; 

geometry_msgs::PointStamped hallwayPoints1;
geometry_msgs::PointStamped transformedHallwayPoints1;

geometry_msgs::PointStamped hallwayPoints2;
geometry_msgs::PointStamped transformedHallwayPoints2;


/*geometry_msgs::PointStamped personPose;
geometry_msgs::PointStamped transformedPersonPose;

geometry_msgs::PointStamped robotInitialPose;
geometry_msgs::PointStamped transformedRobotInitialPose;

geometry_msgs::PointStamped robotGoalPose;
geometry_msgs::PointStamped transformedRobotGoalPose;

geometry_msgs::PointStamped robotPose;
geometry_msgs::PointStamped transformedRobotPose;*/

geometry_msgs::PointStamped robotPose;
geometry_msgs::PointStamped transformedRobotPose;


double currentTime, startTime;
double robot_t1 = 0, robot_t2 = 0, human_t1 = 0, human_t2 = 0 , minimum_distance = 100000;

ros::Publisher people_features, san_features, start_cmd;
ros::Publisher new_Marker, move_to_goal;
san_feature_extractor::peopleData personDetails;
san_feature_extractor::sanData sanDetails;

geometry_msgs::Point robot_pose;

string Convert (float number){
	ostringstream buff;
	buff<<number;
	return buff.str();   
}

//To transform a point to target frame from fixed frame
void transformPoint(const tf::TransformListener& listener){
  
  hallwayPoints1.header.frame_id = "/robot_0/base_laser_link";
  hallwayPoints1.header.stamp = ros::Time();
  hallwayPoints1.point.x = hallwayPointL1.x;
  hallwayPoints1.point.y = hallwayPointL1.y;
  hallwayPoints1.point.z = 0.0;

  hallwayPoints2.header.frame_id = "/robot_0/base_laser_link";
  hallwayPoints2.header.stamp = ros::Time();
  hallwayPoints2.point.x = hallwayPointL2.x;
  hallwayPoints2.point.y = hallwayPointL2.y;
  hallwayPoints2.point.z = 0.0;

  robotPose.header.frame_id = "/robot_0/odom";
  robotPose.header.stamp = ros::Time();
  robotPose.point.x = robotFuturePosition.x;
  robotPose.point.y = robotFuturePosition.y;
  robotPose.point.z = 0.0;

  /*personPose.header.frame_id = "/robot_1/odom";
  personPose.header.stamp = ros::Time();
  personPose.point.x = personData.posx;
  personPose.point.y = personData.posy;
  personPose.point.z = 0.0;

  robotInitialPose.header.frame_id = "/robot_0/odom";
  robotInitialPose.header.stamp = ros::Time();
  robotInitialPose.point.x = robotPosition.x;
  robotInitialPose.point.y = robotPosition.y;
  robotInitialPose.point.z = 0.0;

  robotGoalPose.header.frame_id = "/robot_0/odom";
  robotGoalPose.header.stamp = ros::Time();
  robotGoalPose.point.x = robotGoalPosition.x;
  robotGoalPose.point.y = robotGoalPosition.y;
  robotGoalPose.point.z = 0.0;
  
  robotPose.header.frame_id = "/robot_0/odom";
  robotPose.header.stamp = ros::Time();
  robotPose.point.x = robotFuturePosition.x;
  robotPose.point.y = robotFuturePosition.y;
  robotPose.point.z = 0.0;


  testrobotPose.header.frame_id = "/robot_0/odom";
  testrobotPose.header.stamp = ros::Time();
  testrobotPose.point.x = test.x;
  testrobotPose.point.y = test.y;
  testrobotPose.point.z = 0.0;*/

  try{

    //Need to transform hallway points from /robot_0/base_laser_link to /world

    listener.transformPoint("/map", hallwayPoints1, transformedHallwayPoints1);
    /*ROS_INFO("Hallway Point 1: (%.2f, %.2f. %.2f) -----> map: (%.2f, %.2f, %.2f) at time %.2f",
        hallwayPoints1.point.x, hallwayPoints1.point.y, hallwayPoints1.point.z,
        transformedHallwayPoints1.point.x, transformedHallwayPoints1.point.y, transformedHallwayPoints1.point.z, transformedHallwayPoints1.header.stamp.toSec());*/

    listener.transformPoint("/map", hallwayPoints2, transformedHallwayPoints2); 
    /*ROS_INFO("Hallway Point 2: (%.2f, %.2f. %.2f) -----> map: (%.2f, %.2f, %.2f) at time %.2f",
        hallwayPoints2.point.x, hallwayPoints2.point.y, hallwayPoints2.point.z,
        transformedHallwayPoints2.point.x, transformedHallwayPoints2.point.y, transformedHallwayPoints2.point.z, transformedHallwayPoints2.header.stamp.toSec());*/


    listener.transformPoint("/map", robotPose, transformedRobotPose); 
    /*ROS_INFO("Future Robot Position: (%.2f, %.2f. %.2f) -----> map: (%.2f, %.2f, %.2f) at time %.2f",
        robotPose.point.x, robotPose.point.y, robotPose.point.z,
        transformedRobotPose.point.x, transformedRobotPose.point.y, transformedRobotPose.point.z, transformedRobotPose.header.stamp.toSec());*/

    //Not required: Person and robot positions are now rbased on world frame

    /*Listener.transformPoint("/map", personPose, transformedPersonPose); 
    ROS_INFO("Person Position: (%.2f, %.2f. %.2f) -----> map: (%.2f, %.2f, %.2f) at time %.2f",
        personPose.point.x, personPose.point.y, personPose.point.z,
        transformedPersonPose.point.x, transformedPersonPose.point.y, transformedPersonPose.point.z, transformedPersonPose.header.stamp.toSec());

    listener.transformPoint("/map", robotInitialPose, transformedRobotInitialPose); 
    ROS_INFO("Robot Initial Position: (%.2f, %.2f. %.2f) -----> map: (%.2f, %.2f, %.2f) at time %.2f",
        robotInitialPose.point.x, robotInitialPose.point.y, robotInitialPose.point.z,
        transformedRobotInitialPose.point.x, transformedRobotInitialPose.point.y, transformedRobotInitialPose.point.z, transformedRobotInitialPose.header.stamp.toSec());

    listener.transformPoint("/map", robotGoalPose, transformedRobotGoalPose); 
    ROS_INFO("Robot Goal Position: (%.2f, %.2f. %.2f) -----> map: (%.2f, %.2f, %.2f) at time %.2f",
        robotGoalPose.point.x, robotGoalPose.point.y, robotGoalPose.point.z,
        transformedRobotGoalPose.point.x, transformedRobotGoalPose.point.y, transformedRobotGoalPose.point.z, transformedRobotGoalPose.header.stamp.toSec());

    listener.transformPoint("/map", testrobotPose, testtransformedRobotPose); 
    ROS_INFO("Test Robot Position: (%.2f, %.2f. %.2f) -----> map: (%.2f, %.2f, %.2f) at time %.2f",
        testrobotPose.point.x, testrobotPose.point.y, testrobotPose.point.z,
        testtransformedRobotPose.point.x, testtransformedRobotPose.point.y, testtransformedRobotPose.point.z, testtransformedRobotPose.header.stamp.toSec());*/
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"robot_0\"base_laser_link\" to \"map\": %s", ex.what());
  }
}

void robotVelocityCallback( const geometry_msgs::Twist::ConstPtr& msg ){
	float time = ros::Time::now().toSec() - robot_t1;
	robotVel<<time<<","<<msg->linear.x<<","<<msg->linear.y<<endl; 
}

void humanVelocityCallback( const geometry_msgs::Twist::ConstPtr& msg ){
	float time = ros::Time::now().toSec() - human_t1;
	humanVel<<time<<","<<msg->linear.x<<","<<msg->linear.y<<endl; 
}

//Function to get the current position of the robot
void robotPositionCallback ( const nav_msgs::Odometry::ConstPtr& msg  ){
	poseX = msg->pose.pose.position.x * 100;//In cm
  	poseY = msg->pose.pose.position.y * 100; //In cm
  
}

//For Simulation - Person Detection
void personPositionCallback ( const nav_msgs::Odometry::ConstPtr& msg ){
	float currentDistance;
	//Topic is not publishing an id. 
	personData.personId = "Person1";

	personData.posx =  msg->pose.pose.position.x; //In m
  	personData.posy = msg->pose.pose.position.y; //In m

  	//ROS_INFO("Person  : (%f , %f)", personData.posx, personData.posy);

  	//personData.xdistance = transformedPersonPose.point.x * 100 ; //In cm
  	//personData.ydistance = transformedPersonPose.point.y * 100; //In cm

  	personData.xdistance = msg->pose.pose.position.x * 100;
  	personData.ydistance = msg->pose.pose.position.y * 100;

  	if( personFound == 0 ){
    	personData.t0 = ros::Time::now().toSec();
    	personData.t1 = ros::Time::now().toSec();
    	personInitialX = personData.xdistance;
    	personInitialY = personData.ydistance;
    	personData.distance_travelled = 0.0;
  		personData.speed = 0.0;    
   		personFound = 1;
  	}	
	else{
		personData.distance_travelled = sqrt(pow((personData.xdistance - personInitialX ),2)+pow((personData.ydistance - personInitialY ),2));
    	//personData.distanceFromPR2 = currentDistance;
    	personData.t0 =  personData.t1; 
    	personData.t1 = ros::Time::now().toSec();
        
    	personData.speed =  personData.distance_travelled / ( personData.t1 - personData.t0);
    	personInitialX = personData.xdistance;
    	personInitialY = personData.ydistance; 
	}
	//For meeting and passing scenarios: See whether human passed the initial robot position
    //if(personData.xdistance <= intialPoseX){
    if(personData.xdistance <= goalPoseX){
    	human_t2 = ros::Time::now().toSec() - human_t1;
    }

}


void featureCalculator(float robotPositionX, float robotPositionY){
  
  float distanceFromPR2, personHallwayR, personHallwayL, robotHallwayR, robotHallwayL, robotHLPersonHL, distanceTravelledbyPR2 = 0, distanceToGoal;
  float timeStamp, time;
  float normalized_time = 0, total_distance = 0;
  
  ostringstream ss;
  string stringData;
  

  if(flag == 0){
    flag =1;
    startTime = ros::Time::now().toSec();
  }
  
  if( personFound == 0 ){
    ROS_INFO("\n\tNo person in the hallway");   
  }
  else if( hallwayFound = 0 ){
    ROS_INFO("\n\tNo hallway data found");
  }
  else{
    currentTime = ros::Time::now().toSec();
    timeStamp = currentTime - startTime;
    
    distanceFromPR2 =  sqrt(pow((personData.xdistance - robotPositionX),2) + pow((personData.ydistance - robotPositionY),2)); //2-D distance 
    distanceTravelledbyPR2 = sqrt(pow((robotPositionX - intialPoseX ), 2) + pow( (robotPositionY - intialPoseY ) ,2));
    distanceToGoal = sqrt(pow((goalPoseX - robotPositionX ), 2) + pow( (goalPoseY - robotPositionY) ,2)); 
    personHallwayL = ( fabs( m*personData.xdistance - personData.ydistance + c ) ) / ( sqrt( pow(m,2) + 1 ) );
    personHallwayR = width - personHallwayR;
    robotHallwayL = ( fabs( m*robotPositionX - robotPositionY + c ) ) / ( sqrt( pow(m,2) + 1 ) );
    robotHallwayR = width - robotHallwayR;
    robotHLPersonHL = robotHallwayL - personHallwayL;
    total_distance = sqrt(pow((goalPoseX - intialPoseX ), 2) + pow( (goalPoseY - intialPoseY) ,2));
    normalized_time = distanceTravelledbyPR2/ total_distance;      


    if(minimum_distance > distanceFromPR2)
    	minimum_distance = distanceFromPR2;
     

    /*ROS_INFO("Robot Position : (%f, %f)",robotPositionX, robotPositionY);
    ROS_INFO("Human Position : (%f, %f)", personData.xdistance, personData.ydistance);
    ROS_INFO("intial Position : (%f, %f)", intialPoseX, intialPoseY);
    ROS_INFO("Goal Position: (%f, %f)", goalPoseX, goalPoseY);
    ROS_INFO("Transformed m & c : %f & %f",m, c);
    ROS_INFO("Hallway width : %f", width);
    ROS_INFO("Robot HallwayL : %f", robotHallwayL);
    ROS_INFO("Person HallwayL : %f", personHallwayL);*/

    //ss << currentTime;
    ss << normalized_time;
    stringData = ss.str();
    ss.str(std::string());
    SANfeatures.push_back(stringData);
    //ROS_INFO("Time - %f - %s",currentTime,stringData.c_str());
      
    ss << distanceTravelledbyPR2;  
    stringData = ss.str();
    ss.str(std::string());
    SANfeatures.push_back(stringData);
    //ROS_INFO("distanceTravelledbyPR2 - %f - %s", distanceTravelledbyPR2, stringData.c_str());
      
    ss << personHallwayL; 
    stringData = ss.str();
    ss.str(std::string());
    SANfeatures.push_back(stringData);
    //ROS_INFO("personHallwayL- %f - %s", personHallwayL, stringData.c_str());
      
    ss << distanceFromPR2; 
    stringData = ss.str();
    ss.str(std::string());
    SANfeatures.push_back(stringData);
    //ROS_INFO("distanceFromPR2 - %f - %s", distanceFromPR2, stringData.c_str());
     
    ss << robotHLPersonHL; 
    stringData = ss.str();
    ss.str(std::string());
    SANfeatures.push_back(stringData);
    //ROS_INFO("robotHLPersonHL - %f - %s", robotHLPersonHL, stringData.c_str());
    ROS_INFO("Normalized Time : %f, Distance travelled: %f, Total distance: %f", normalized_time, distanceTravelledbyPR2, total_distance);

    outfile << normalized_time << ", "<< robotHallwayL << ", " << "0" << ", " << distanceTravelledbyPR2 << ", "<< personHallwayL << ", " << "0" << ", " << "0" << ", "<< distanceFromPR2  << endl;
  }  
}


void initialPoseCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  	std_msgs::String navMsg;  
  	//ROS_INFO("In pose estimate callback");
	geometry_msgs::PoseStamped goalPosition; 


  	robotGoalPosition.x = 32.70;//22.41
  	robotGoalPosition.y = 10.04;// 10.80; //
  	robotGoalPosition.z = 0;
  
  	goalPosition.header.frame_id = "map";
  	goalPosition.pose.position.x = 32.70; //22.41;
  	goalPosition.pose.position.y = 10.04; //10.80; 
  	goalPosition.pose.position.z = 0;

  	goalPosition.pose.orientation.x = 0;
  	goalPosition.pose.orientation.y = 0;
  	goalPosition.pose.orientation.z = 0.006; //0.999; 
  	goalPosition.pose.orientation.w =  0.999; //0.002;

 	goalPoseX = goalPosition.pose.position.x * 100;//In cm
  	goalPoseY = goalPosition.pose.position.y * 100;//In cm

  	//goalPoseX = transformedRobotGoalPose.point.x * 100;//In cm 
  	//goalPoseY = transformedRobotGoalPose.point.y * 100;//In cm

  	move_to_goal.publish(goalPosition);

  	//robotPosition.x = poseX;
  	//robotPosition.y = poseY; 
  	//robotPosition.z = 0;

  	//intialPoseX = transformedRobotInitialPose.point.x * 100;
  	//intialPoseY = transformedRobotInitialPose.point.y * 100;

  	intialPoseX = poseX;
  	intialPoseY = poseY;
  
  	//Start command to start the human simulation
  	navMsg.data = "Start";
  	start_cmd.publish(navMsg);
  	
  	//Initial time to get calculate the time taken by robot ot reach the goal
  	robot_t1 = ros::Time::now().toSec();
  	human_t1 = ros::Time::now().toSec();

  	//Fro testiing services - SAN_NODES 
  	testflag = 1;
}


//For live Data - Person Detection
void peoplePositionCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg){

	float currentDistance;
  	std::vector<people_msgs::PositionMeasurement> people;

  	if( personFound == 0 ){
    	personData.personId = msg->people[0].name.c_str();
    	personData.t0 = ros::Time::now().toSec();
    	personData.t1 = ros::Time::now().toSec();
  		personData.xdistance = 100 * msg->people[0].pos.x; //Converting m to cm
    	personData.ydistance = 100 * msg->people[0].pos.y; //Converting m to cm
    	personData.posx =  100 * msg->people[0].pos.x; //Converting m to cm
    	personData.posy = 100 * msg->people[0].pos.y; //Converting m to cm
    	personInitialX = personData.posx;
    	personInitialY = personData.posy;
    	personData.distance_travelled = 0.0;
    	personData.speed = 0.0;    
    	personFound = 1;
  	}
  	else{
    	for( int i = 0; i < msg->people.size(); i++ )
    	{
      	if( strcmp(msg->people[i].name.c_str() , personData.personId.c_str()) == 0 ) {
	      	personData.xdistance = 100 * msg->people[i].pos.x; //Converting m to cm
    	  	personData.ydistance = 100 * msg->people[i].pos.y; //Converting m to cm
        	personData.posx =  100 * msg->people[i].pos.x; //Converting m to cm
        	personData.posy = 100 * msg->people[i].pos.y; //Converting m to cm
        	personData.distance_travelled = sqrt(pow((personData.xdistance - personInitialX ),2)+pow((personData.ydistance - personInitialX ),2));
        	//personData.distanceFromPR2 = currentDistance;
    	  	personData.t0 =  personData.t1; 
	      	personData.t1 = ros::Time::now().toSec();
        	personData.speed =  personData.distance_travelled / ( personData.t1 - personData.t0);
      	}
    	}
  	}

  	//ROS_INFO("Time T0 : %f", peopleData[j].t0);
  	//ROS_INFO("Time T1 : %f", peopleData[j].t1);
  	//ROS_INFO("Speed : %f", peopleData[j].speed);
  	//ROS_INFO("Distance travelled : %f", peopleData[j].distance_travelled);
          
  	//Publish the details
  	personDetails.frame_id = "base_link";
  	personDetails.personId = personData.personId; 
  	//personDetails.distanceFromPR2 = personData.distanceFromPR2;
  	personDetails.x = personData.posx;
  	personDetails.y = personData.posy;
  	personDetails.t0 = personData.t0; 
  	personDetails.t1 = personData.t1;
 	personDetails.xdistance = personData.xdistance;
	personDetails.ydistance = personData.ydistance;
  	personDetails.distance_travelled = personData.distance_travelled; 
  	personDetails.speed = personData.speed;
  
  	ROS_INFO("Person ID : %s", personDetails.personId.c_str());
  	//ROS_INFO("Time T1 : %f", peopleData[j].t1);
  	//ROS_INFO("Speed : %f", peopleData[j].speed);
  	//ROS_INFO("Distance travelled : %f", peopleData[j].distance_travelled);
   

}

void hallwayDetectionCallback(const hallway::hallwayMsg::ConstPtr& msg){
	//For testing service -  SAN_NODES
  	int  classification;
  	float classifyProbability[4];
  
  	hallwayFound = 1;
  
  	san_feature_extractor::newMarkerMsg MarkerPoints;
  
  	geometry_msgs::Point pointL1;
  	geometry_msgs::Point pointL2; 
  
  	width = msg->width_hallway; //Width of the Hallway

  	hallwayPointL1.x = msg->hallwayPointL1.x;
  	hallwayPointL1.y = msg->hallwayPointL1.y;
  
  	hallwayPointL2.x = msg->hallwayPointL2.x;
  	hallwayPointL2.y = msg->hallwayPointL2.y;
  
  	//Calculating slope ang intecept based on transformed points
  	m = (float)(transformedHallwayPoints2.point.y   - transformedHallwayPoints1.point.y)/(float)(transformedHallwayPoints2.point.x - transformedHallwayPoints1.point.x);
  	c = ( transformedHallwayPoints1.point.y  -  m*transformedHallwayPoints1.point.x ) * 100;  
  
  	//In meters 
  	pointL1.x = transformedHallwayPoints1.point.x ; 
  	pointL1.y = transformedHallwayPoints1.point.y;
    
  	pointL2.x = transformedHallwayPoints2.point.x; 
  	pointL2.y = transformedHallwayPoints2.point.y;

  	MarkerPoints.pointL1 = pointL1;
  	MarkerPoints.pointL2 = pointL2;
  	new_Marker.publish(MarkerPoints);

  	//ROS_INFO("Old Points: %f  , %f , %f  , %f ",msg->hallwayPointL1.x, msg->hallwayPointL1.y, msg->hallwayPointL2.x, msg->hallwayPointL2.y);
  	//ROS_INFO("New Points: %f ,  %f , %f  , %f ",transformedHallwayPoints1.point.x, transformedHallwayPoints1.point.y, transformedHallwayPoints2.point.x, transformedHallwayPoints2.point.y);

  	//FOR TESTING SERVICE IN TRADITIONAL PLANNER- SAN_NODES [Uncomment if block for testing services]
  	if (testflag == 1){ //Do when the pose estimate is set
    
    	featureCalculator(poseX , poseY);
  
    	san_nodes::Classify classifyScenario;
    	classifyScenario.request.sample = SANfeatures;
  
    	if (clientClassify.call(classifyScenario))
    	{
    		classification = classifyScenario.response.classify_label;
      		ROS_INFO("Service for recognizing scenario: %d", classification);
    	}
    	else
    	{
    		ROS_ERROR("Failed to call service classifyScenario");
    	} 
    	classification = 2;

    	san_nodes::Appscore scoreScenario;
    	scoreScenario.request.sample = SANfeatures;
  
    	if (clientAppscore.call(scoreScenario))
    	{
      		//Get the probability of the corresponding Scenario
      		classifyProbability[classification] = scoreScenario.response.classify_probs[classification];
      		if(poseX <= goalPoseX)
      			robot_t2 = ros::Time::now().toSec() - robot_t1;
      		ROS_INFO("Classification probability for scenario - %f", classifyProbability[classification]);
      		
      		//Write scenario and probabilities to file 'SimAppScore.txt'
  			probability<<robot_t2<<","<<classifyProbability[classification]<<endl;
  			trajfile<<"Robot Time: "<<robot_t2<<" Human time: "<<human_t2<<" Minimum distance: "<<minimum_distance<<endl;
  			robot<<robot_t2<<","<<poseX<<","<<poseY<<endl;
  			
  			//For meeting and passing : 
  			//if( personData.xdistance >= intialPoseX )
  			//For walking away from goal: 
  		if( personData.xdistance <= goalPoseX )	
  				human<<robot_t2<<","<<personData.xdistance<<","<<personData.ydistance<<endl;
    	}
    	else
    	{
      	ROS_ERROR("Failed to call service Appscore");
    	}
  
    	//Clear vector<string> features ;
    	SANfeatures.clear();
  	}

}


bool featureExtractionService(san_feature_extractor::Trajectory::Request  &req, san_feature_extractor::Trajectory::Response &res)
{
  
  	float classifyProbability[4];
  	int  classification;

  	//ROS_INFO("In Service");
  
  	//Call featureCalculator function to get the SAn features as a vector of strings
  	robotFuturePosition.x = req.x;
  	robotFuturePosition.y = req.y;
  	robotFuturePosition.z = 0;

  	featureCalculator(transformedRobotPose.point.x * 100, transformedRobotPose.point.y * 100);
  	ROS_INFO("future trajectory points: (%f , %f )", transformedRobotPose.point.x , transformedRobotPose.point.x);
  
  	san_nodes::Classify classifyScenario;
  	classifyScenario.request.sample = SANfeatures;
  
  	if (clientClassify.call(classifyScenario))
  	{
  	 	classification = classifyScenario.response.classify_label;
    	//ROS_INFO("Service for recognizing scenario: %d", classification);
  	}
  	else
  	{
    	//ROS_ERROR("Failed to call service classifyScenario");
  	}	

  	san_nodes::Appscore scoreScenario;
  	scoreScenario.request.sample = SANfeatures;
  	classification =  0;

  	if (clientAppscore.call(scoreScenario))
  	{
    	//Get the probability of the corresponding Scenario
    	classifyProbability[classification] = scoreScenario.response.classify_probs[classification];
    	if(poseX <= goalPoseX)
    		robot_t2 = ros::Time::now().toSec() - robot_t1;
    	//ROS_INFO("Classification probability for scenario - %f", classifyProbability[classification]);
    	//Write scenario and probabilities to file 'SimAppScore.txt'
    	probability<<robot_t2<<","<<classifyProbability[classification]<<endl;
  		trajfile<<"Robot Time: "<<robot_t2<<" Human time: "<<human_t2<<" Minimum distance: "<<minimum_distance<<endl;
  		robot<<robot_t2<<","<<poseX<<","<<poseY<<endl;
  			
  		//For meeting and passing : 
  		if( personData.xdistance >= intialPoseX )
  		//For walking away from goal : if( personData.xdistance <= goalPoseX )	
  			human<<robot_t2<<","<<personData.xdistance<<","<<personData.ydistance<<endl;
 	}
  	else
  	{
    	//ROS_ERROR("Failed to call service Appscore");
  	}
  
  	//Clear vector<string> features ;
  	SANfeatures.clear();
  
  	res.prob = classifyProbability[classification];
  	//ROS_INFO("request: x=%ld, y=%ld", (long int)req.x, (long int)req.y);
  	//ROS_INFO("sending back response: [%ld]", (long int)res.prob);
  	return true;
}


int main( int argc, char* argv[] ){
  
  	//Initialize the ROS system and specify the node name.
  	ros::init(argc,argv,"legdata") ;

  	ros::NodeHandle nh ; 
  	
  	tf::TransformListener listener;
  
  	//Subsribe to 2D Pose Estimate 
  	ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 1000, initialPoseCallback);

  	//ros::Subscriber robot_Pose_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 1000, robotPositionCallback);
  	ros::Subscriber people_sub = nh.subscribe("people_tracker_measurements", 1000, peoplePositionCallback);
  	//Subsribe to hallway
  	ros::Subscriber hallway_sub = nh.subscribe("hallway_data", 1000, hallwayDetectionCallback);

	//For Simulator
	//Subsribe to person postion
	ros::Subscriber person_sub = nh.subscribe("/robot_1/base_pose_ground_truth", 1000, personPositionCallback);
	//Subsribe to robot postion
	ros::Subscriber robotpose_sub = nh.subscribe("/robot_0/base_pose_ground_truth", 1000, robotPositionCallback);
	//Subscirbe to robot and human velocity
	ros::Subscriber robotVel_sub = nh.subscribe("/robot_0/cmd_vel", 1000, robotVelocityCallback);
	ros::Subscriber humanVel_sub = nh.subscribe("/robot_1/cmd_vel", 1000, humanVelocityCallback); 
  	//Publishers for publishing People Data and SAN Features
  	//people_features = nh.advertise<san_feature_extractor::peopleData>("peopleFeatures",1000);
  	//san_features = nh.advertise<san_feature_extractor::sanData>("sanFeatures",1000);
  	
  	new_Marker = nh.advertise<san_feature_extractor::newMarkerMsg>("new_hallway_marker_points", 100);
 	start_cmd = nh.advertise<std_msgs::String>("navigation_command", 1000);
  	//Publish the goal position
   	move_to_goal = nh.advertise< geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
  
  	//Service for feature calculation
  	ros::ServiceServer service = nh.advertiseService("future_trajectory_points", featureExtractionService);
  	clientClassify = nh.serviceClient<san_nodes::Classify>("classify_samples");
 	clientAppscore = nh.serviceClient<san_nodes::Appscore>("give_appscore");
  
  	ROS_INFO("Server ready");
  
  	ros::Rate rate(10.0);
  
  	tf::StampedTransform transform;


  	ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  	//Not required : Robot and human positions are now with respect to world 

  	//Transforms for /robot_0/odom to /map
  	/*try{
    	ROS_INFO("Try");
    	listener.waitForTransform("/robot_0/odom", "map",  ros::Time(0), ros::Duration(10.0));
    	ROS_INFO("Wait");
    	listener.lookupTransform("/robot_0/odom", "map",  ros::Time(0), transform);
    	ROS_INFO("Look");
  	}
  	catch (tf::TransformException &ex){
    	ROS_INFO("Catch");
    	ROS_ERROR("%s",ex.what());
    	ros::Duration(1.0).sleep();
  	}
  	odom_0_OriginX = transform.getOrigin().x();
  	odom_0_OriginY = transform.getOrigin().y();  
  	ROS_INFO("X - %f, Y - %f", odom_0_OriginX, odom_0_OriginY);*/

    
  	rate.sleep();
 
  	ROS_INFO("Out");
  	ros::spin();
  
  	return 0;
}