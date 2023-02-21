#include "rclcpp/rclcpp.hpp"
//#include "ros/ros.h"
//#include "ros/console.h"
#include "std_msgs/msg/string.hpp"

//#include "bigbot/pwm.h" need to figure this out
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/int8.hpp"

//#include "transform_broadcaster.h"

//#include "tf/transform_broadcaster.hpp"
//#include <tf/transform_broadcaster.h>

#include "motorcontrol/SlaveParse.h"
#include "motorcontrol/sensordata.h"
#include "motorcontrol/serial.h"

//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <termios.h>
//#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// this is from r2serial
//#include <pthread.h>
//#include <sys/time.h>
#include <termios.h>


#include "motorcontrol/botconfig.h"

// must be one of these 78.4k/250k/500k/1M
// 78.4k and 250k as custom
// 500k = #define  B500000   0010005
// 1M = #define  B1000000  0010010
// setting to 78400 first try
// 
//#define BAUDRATE B57600 // old standard, keep B
//#define BAUDRATE 78400 // Change as needed, keep B
//#define BAUDRATE B500000
#define BAUDRATE B115200


/* change this definition for the correct port */
//#define MODEMDEVICE std::string "/dev/ttyUSB0"

#define NUMOUTGOING 60

// 3.14159
#define WHEELRADIUS .09763 // in meters 7 11/16in /2
#define BASE 0.76708  //(30.2 inch)

// 0.613434717 meter =  1 rotation

// convert m/s to radians
// (v(m/s) * 2PI)/circumferance
// (.1ms * 2 * 3.14159) /.613434717 = 1.0242
// (.5   * 2 * 3.14159) /.613434717 = 5.1213
// (1    * 2 * 3.14159) /.613434717 = 10.242

// another website said
// radpersecond = v (mps)/wheel radius
// .1/.09763 =1.0242
// .5/.09763 = 5.1213
// 1./.09763 = 10.242

// default min and max PWM
#define MINLEFTDEFAULT 200
#define MAXLEFTDEFAULT 500
#define MINRIGHTDEFAULT 200
#define MAXRIGHTDEFAULT  570



// ros debug flag
#define LOG_INFO_DEBUG 1
/*
state 0 wait for build message
state 1 send echo and wait for response
state 2 send p wait for config
state 3 send sonar
state 4 send gain
state 5 send sensor state = 1
state 6 wait for sensor readingd      */


class MotorControl : public rclcpp::Node
{

protected:


int CommandID=0;  // for ids for commands 
int State =0; // need to use this to do init
char OutgoingBuffer[NUMOUTGOING];
int sensorEnd; // set based on config

//moved from main to here
char port[20];    //port name
int baud;     //baud rate
//std::string serial_port ="/dev/atmegaslave";
std::string serial_port ="/dev/ttyACM0";

//"/dev/atmegaslave"
bool UseSlave = true;
bool ListenPWMMsg = false;




// end of move

BotConfig mconfig;
nav_msgs::msg::Odometry msg;
sensor_msgs::msg::Range  sonarmsg;

sensorData msensor;
Serial mComm;//= new Serial();

short MinLEFT;
short MaxLEFT;
short MinRIGHT;
short MaxRIGHT;
short BMinLEFT;
short BMaxLEFT;
short BMinRIGHT;
short BMaxRIGHT;



const double MaxSpeed =13.3;//4.7; // in radian
const double MinSpeed =0.1;
const double LeftSlope= 231; //(MaxLEFT-MinLEFT)/(MaxSpeed- MinSpeed) ;
const double RightSlope = 286;//(MaxRIGHT - MinRIGHT) / (MaxSpeed - MinSpeed);
const double LeftY = 154;//MaxLEFT - (LeftSlope * MaxSpeed);
const double RightY =143;// MaxRIGHT - (RightSlope * MaxSpeed);

const double RIGHTMETERPERCLICK = 0.0004786; // (.5 rotation plus 8cm/ 808 original values in feet 0.015408; // original 0.016; //  oover rode orig value
const int RIGHTTICKPERROTATION = 1282; // circumference in meter / RIGHTMETERPERCLICK
const double RIGHTTICK2RAD = 0.00490108; //6.283184/1282 //radian in 1 rotation by tick in rotoation

const double LEFTMETERPERCLICK =  0.000937; //(.5rotation (0.306713432) plus 4cm / 370 clicks)
//0.008863831;// o;ringinal values in feet 0.02908081; //0.030287; original
//.5 rotation plus 4cm
const int LEFTTICKPERROTATION = 655; // circumference in meter / LEFTMETERPERCLICK
const double LEFTTICK2RAD = 0.009592647; //6.283184/655 //radian in 1 rotation by tick in rotoation


int lastleftclick=0,lastrightclick=0, deltaLeft, deltaRight,SlaveTimestamp = 0;



// prior requested speed values in feet per second
double mLastRequestedLeftSpeed = 0, mLastRequestedRightSpeed = 0;
// new requested speed
double mRequestedLeftSpeed = 0, mRequestedRightSpeed = 0;
// hold pwm values between set up request and get command call
short mLeftPWM = 0, mRightPWM = 0, mLastLeftPWM = 0, mLastRightPWM = 0;
// sent mLastDirection to invalid vaule to force sending direction for first call
char mDirection = (char)'F', mLastDirection = 'Z';

double lasttheta=0;

bool mNeedNewCommand = false;
bool mNeedDirection = false;
bool mUseSlave = true;
bool mPubVel = false;
rclcpp::Time mLastCommndSent; 
rclcpp::TimerBase::SharedPtr timer_;

geometry_msgs::msg::Pose2D bot_pose; // hold current position of bot.

//int angularVelocitytoPWM(int MotorMax, int MotorMin,double VelocityMax,double VelocityMin,double angularVelocity);

//bool ReadConfig();
//void CalcOdom();
//void CalcOdom2();

//void RequestConfig();
//void SetSonar( int Index);
//void SetGain();
//bool StartSensor();
//bool SetEcho(int echo);

public:
std::string base_frame =  "base_link";
std::string odom_frame =  "odom";

//void SetPose( geometry_msgs::msg::Pose2D new_pose);
//void SetUseSlave(bool UseSlave);
//void CloseSerial();
//bool InitSerial(char * port, int baud);
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ptr_pub;
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr  ptr_pubR;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub2;
rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr substate;


rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ptr_pubvel;
std::unique_ptr<tf2_ros::TransformBroadcaster> ptr_broadcaster;

//void readLoop();
//void fakeReadLoop();
//void TwistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);
//void SetPWMMinMax(int minleft,int maxleft,int minright,int maxright,int bminleft,int bmaxleft,int bminright,int bmaxright);
//void PWMCallBackpwmCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

void SetPose( geometry_msgs::msg::Pose2D new_pose)
{
    bot_pose.x = new_pose.x;
    bot_pose.y = new_pose.y;
    bot_pose.theta = new_pose.theta;

}


MotorControl() : Node("motorcontrol")
{

  
  RCLCPP_INFO(this->get_logger(), "motorControl starting");

  // this will allow or disable serial comms for testing

  //MotorControl mc;

  geometry_msgs::msg::Pose2D startPose;
  startPose.x=0.0;
  startPose.y=0.0;
  startPose.theta=0.0;
  SetPose(startPose);
  baud = BAUDRATE;
  // examples params

  //add_parameter( "port", rclcpp::ParameterValue(serial_port),    "serial port for connection avr", "Should be /dev/ttyUSB0");


  // read port from config
//  rosNode.param<std::string>("/motorControl/port",serial_port,serial_port);

  RCLCPP_INFO(this->get_logger(), "connection initializing (%s) at %d baud.", serial_port.c_str(), baud);

  // baud = 57600;

  //RCLCPP_INFO(this->get_logger(), "override connection initializing (%s) at %d baud", serial_port, baud);
  mLastCommndSent= this->now();
  this->declare_parameter<bool>("useslave",false);
  this->get_parameter("useslave",UseSlave);
  //  rosNode.param<bool>("/motorControl/useslave",UseSlave,UseSlave);
  if (UseSlave)
	{
    RCLCPP_INFO(this->get_logger(), "Using Slave " );
	}
  //SetUseSlave(UseSlave);
  if (UseSlave)
	{
	  if (!InitSerial((char *)serial_port.c_str(), baud) ) 
	  {
		RCLCPP_ERROR(this->get_logger(),"unable to create a new serial port %s baud %d",port, baud);
		throw std::exception();
	  }
	  RCLCPP_INFO(this->get_logger(), "serial connection successful");
	}
  else
    {
	  RCLCPP_INFO(this->get_logger(), "serial connection not used");
    }

  int minleft,maxleft,minright,maxright,LocalDefault,bminleft,bmaxleft,bminright,bmaxright;

/*
  rosNode.param("/motorControl/pwm/forward/left/min",minleft,LocalDefault);
  rosNode.param("/motorControl/pwm/backward/left/min",bminleft,LocalDefault);
  rosNode.param("/motorControl/pwm/forward/left/max",maxleft,LocalDefault);
  rosNode.param("/motorControl/pwm/backward/left/max",bmaxleft,LocalDefault);
  rosNode.param("/motorControl/pwm/forward/right/min",minright,LocalDefault);
  rosNode.param("/motorControl/pwm/backward/right/min",bminright,LocalDefault);
  rosNode.param("/motorControl/pwm/forward/right/max",maxright,LocalDefault);
  rosNode.param("/motorControl/pwm/backward/right/max",bmaxright,LocalDefault);
*/
  LocalDefault = MINLEFTDEFAULT;
  this->declare_parameter<int16_t>("pwm.forward.left.min",LocalDefault);
  this->get_parameter("pwm.forward.left.min",minleft);
  this->declare_parameter<int16_t>("pwm.backward.left.min",LocalDefault);
  this->get_parameter("pwm.backward.left.min",bminleft);
  LocalDefault =MAXLEFTDEFAULT;
  this->declare_parameter<int16_t>("pwm.forward.left.max",LocalDefault);
  this->get_parameter("pwm.forward.left.max",maxleft);
  this->declare_parameter<int16_t>("pwm.backward.left.max",LocalDefault);
  this->get_parameter("pwm.backward.left.max",bmaxleft);  
  LocalDefault =MINRIGHTDEFAULT;
  this->declare_parameter<int16_t>("pwm.forward.right.min",LocalDefault);
  this->get_parameter("pwm.forward.right.min",minright); 
  this->declare_parameter<int16_t>("pwm.backward.right.min",LocalDefault);
  this->get_parameter("pwm.backward.right.min",bminright);     
  LocalDefault =MAXRIGHTDEFAULT;
  this->declare_parameter<int16_t>("pwm.forward.right.max",LocalDefault);
  this->get_parameter("pwm.forward.right.max",maxright); 
  this->declare_parameter<int16_t>("pwm.backward.right.max",LocalDefault);
  this->get_parameter("pwm.backward.right.max",bmaxright); 

   SetPWMMinMax(minleft, maxleft, minright, maxright,bminleft, bmaxleft, bminright, bmaxright);


  this->declare_parameter<std::string>("base_frame",base_frame);
  this->get_parameter("base_frame",base_frame); 
  this->declare_parameter<std::string>("odom_frame",odom_frame);
  this->get_parameter("odom_frame",odom_frame); 

  //Subscribe to ROS messages cmd_vel 
  sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1000, std::bind(&MotorControl::TwistCallBack, this,std::placeholders::_1));

  //rosNode.param<bool>("/motorControl/pwm/listenpwmmsg",ListenPWMMsg,ListenPWMMsg);
  this->declare_parameter<bool>("listenpwmmsg",ListenPWMMsg);
  this->get_parameter("listenpwmmsg",ListenPWMMsg);  
  // subscribe to state topic
  substate  = this->create_subscription<std_msgs::msg::Int8>("getstate", 10, std::bind(&MotorControl::GetStateCallBack, this,std::placeholders::_1));

  if (ListenPWMMsg)
    {

    // bigbot/pwm
    sub2 = this->create_subscription<geometry_msgs::msg::Vector3>("pwm", 100, std::bind(&MotorControl::PWMCallBackpwmCallback, this, std::placeholders::_1));

   	RCLCPP_INFO(this->get_logger(), "Listenning for pwm topic");

    ptr_pubvel = this->create_publisher<geometry_msgs::msg::Vector3>("vel", 100);
    RCLCPP_INFO(this->get_logger(), "Publishing Velocity");
    }
  //Setup to publish ROS messages
  //ros::Publisher pub = rosNode.advertise<nav_msgs::odometry>("/odom", 100);
  ptr_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);

  //Setup to publish ROS messages
  //ros::Publisher pubR = rosNode.advertise<sensor_msgs::range>("/sensor", 100);
  ptr_pubR = this->create_publisher<sensor_msgs::msg::Range>("sensor", 100);
  //  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


  // this is untested transform
  // tf::TransformBroadcaster odom_broadcaster;

  ptr_broadcaster  = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  // end of transform


// need to rip this out of class
  RCLCPP_INFO(this->get_logger(), "rcvThread: receive thread running");

  timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&MotorControl::timer_callback, this));

  if (UseSlave)
	{
    MotorControl::StartComms();
    }
   //Create receive thread

//  pthread_t rcvThrID;   //receive thread ID
//  int err;
//  err = pthread_create(&rcvThrID, NULL, &MotorControl::rcvThread, &mc);
//  if (err != 0) {
//    ROS_ERROR("unable to create receive thread");
//    return 1;
//  }



        

}

~MotorControl() 
{

    if (UseSlave)
	{
    mComm.SerialClose();
    //node->CloseSerial();
    }
}
bool InitSerial(char * port, int baud)
{
    // init sonrarmsg
    sonarmsg.min_range = .043;
    sonarmsg.max_range = 3.0;  // reduced to 3 meters
    sonarmsg.radiation_type = sonarmsg.ULTRASOUND; //sensor_msgs::Range::ULTRASOUND
    sonarmsg.field_of_view= 1; // 1 radian =approx 60degrees

    return mComm.serialInit(port,baud);
}

bool StartComms(void)
{
    int nbrBytes; 
    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"C\n"); 
    mComm.SerialWrite(OutgoingBuffer,nbrBytes);      
    return true;
}
bool SetEcho(int echo)
{
    int nbrBytes;
CommandID++;
if (echo == 1)
    {
    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"E %d 1\n",CommandID);
    }
else
{
    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"E %d 0\n",CommandID);
}
mComm.SerialWrite(OutgoingBuffer,nbrBytes);


return true;
}

void CloseSerial()
{
   mComm.SerialClose();
}

void SetPWMMinMax(int minleft,int maxleft,int minright,int maxright,int bminleft,int bmaxleft,int bminright,int bmaxright)
{
MinLEFT = (short) minleft;
MaxLEFT =(short) maxleft;
MinRIGHT =(short) minright;
MaxRIGHT =(short) maxright;


BMinLEFT = (short)bminleft ;
BMaxLEFT = (short)bmaxleft;
BMinRIGHT = (short)bminright;
BMaxRIGHT = (short)bmaxright;

RCLCPP_INFO(this->get_logger(), "SetPWMMinMax %d %d %d %d  %d %d %d %d",MinLEFT,MaxLEFT,MinRIGHT,MaxRIGHT,BMinLEFT,BMaxLEFT,BMinRIGHT,BMaxRIGHT);

}

//----------------------------------------------------------------------------
// this is to find min pwm for each wheel to move and max PWM at desired max speed
void PWMCallBackpwmCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{


  RCLCPP_INFO(this->get_logger(), "PWMCallBack: %lf %lf %lf",msg->x,msg->y,msg->z);
  mPubVel = true;
  mLeftPWM = msg->x;
  mRightPWM = msg->y;
    
    if (mLeftPWM < 0 || mRightPWM < 0)
		{
		if (mLeftPWM < 0)
			{
            if (mRightPWM < 0)
                {
                mDirection = 'B';
                }
            else
                {
                mDirection ='L';
                }
            }
        else
            {
            mDirection ='R';
            }		
		}
	else
		{
		if (mLeftPWM == 0 && mRightPWM == 0)
			{
			mDirection = 'S';
			}
		else
			{
			mDirection = 'F';
			}
		}
    mLeftPWM = abs(mLeftPWM);
	mRightPWM = abs(mRightPWM);
	
    //only execute code if slave is ready
    if (State == 6)
	    {

      int nbrBytes;
      if (msg->z == 1)
      {

        RCLCPP_INFO(this->get_logger(), "Sending H B 1 Reset encoder counter and turn on sensor");
        nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"H B 1\n");

      }
			
		if (mDirection != mLastDirection)
		    {
		    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"D %d %c\n",CommandID,mDirection);
		    if (mUseSlave)
				{
		    	mComm.SerialWrite(OutgoingBuffer,nbrBytes);
				}
			mLastDirection = mDirection;

			RCLCPP_INFO(this->get_logger(), "PWMCallBack: Change direction Dir %c to %c",mLastDirection , mDirection);

		    }


      nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"M %d %d %d\n",CommandID,mLeftPWM,mRightPWM);
      if (mUseSlave)
      {
        mComm.SerialWrite(OutgoingBuffer,nbrBytes);
      }
      mLastLeftPWM= mLeftPWM;
      mLastRightPWM = mRightPWM;
		RCLCPP_INFO(this->get_logger(), "PWMCallBack: Set pwm L %d R %d",  mLeftPWM,mRightPWM);


	    }
	else
		{
		RCLCPP_INFO(this->get_logger(), "PWMCallBack: slave not started pwm L %d R %d Dir %c %c",  mLeftPWM,mRightPWM,mDirection,mLastDirection);
		}	
}


//-----------------------------------------------

//Process ROS command message, send to uController
void TwistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg )
{
    int nbrBytes;
    double TargetLinearVelocity,targetAngularVelocity,leftVelocity, rightVelocity;//,leftAngular;

    TargetLinearVelocity= msg->linear.x;
    targetAngularVelocity=msg->angular.z;
    RCLCPP_INFO(this->get_logger(), "TargetLinearVelocity %f  targetAngularVelocity %f", TargetLinearVelocity,targetAngularVelocity );

    //only execute code if slave is ready
    if (State == 6)
    {
    bool sendspeed = false;//,sentdirection=false;


    //http://moorerobots.com/blog/post/4 is missing R in divisor per this youtube
    //https://www.youtube.com/watch?v=aE7RQNhwnPQ

    /*
    Vr = (2Vc + WL)/2R
    Vl = (2Vc - WL)/2R
    where Vl and Vr are target velocity or let and right wheel
    Vc is linear velocity (c = center) 
	W = angular velocity 
	L = wheel base
	R = wheel radius
    */
   
    //calculate tangential velocity 
    // 8/18/2018 I think this is already converted into rad per second
    leftVelocity = ((2 * TargetLinearVelocity) - (targetAngularVelocity * BASE))/(2.0 * WHEELRADIUS);
    rightVelocity = ((2 * TargetLinearVelocity) + (targetAngularVelocity * BASE))/(2.0 * WHEELRADIUS);

    rclcpp::Time tnow =this->now();
    rclcpp::Duration dtdiff =tnow - mLastCommndSent;


    double diff = dtdiff.seconds();

    mLastRequestedLeftSpeed = mRequestedLeftSpeed;
    mLastRequestedRightSpeed = mRequestedRightSpeed;

    // calculate angular for left and right wheel
    //leftAngular = leftVelocity/WHEELRADIUS;
    //rightAngular = rightVelocity/WHEELRADIUS;
    //mRequestedLeftSpeed =  leftVelocity/WHEELRADIUS; already have radius added above 8/18/2018
    //mRequestedRightSpeed = rightVelocity/WHEELRADIUS;

    mRequestedLeftSpeed =  leftVelocity;
    mRequestedRightSpeed = rightVelocity;


    if (mRequestedLeftSpeed != mLastRequestedLeftSpeed)
    {
    mLastLeftPWM= mLeftPWM;
    // calculate new pwm
    //    mleftpwm = angularVelocitytoPWM(MaxLEFT, MinLEFT,leftVelocityMaxRads,leftVelocityMinRads,mRequestedLeftSpeed);
    if (mRequestedLeftSpeed < 0)
      {
          mLeftPWM = angularVelocitytoPWM(BMaxLEFT, BMinLEFT,MaxSpeed,MinSpeed,mRequestedLeftSpeed);
      }
        else
      {
          mLeftPWM = angularVelocitytoPWM(MaxLEFT, MinLEFT,MaxSpeed,MinSpeed,mRequestedLeftSpeed);
      }
    sendspeed = true;
    }
    else
    {
        if (diff > 1)
        {
                sendspeed = true;
        }
    }

    if (mRequestedRightSpeed != mLastRequestedRightSpeed)
    {
      // save old values
      mLastRightPWM = mRightPWM;
      //    mrightpwm = angularVelocitytoPWM(MaxRIGHT, MinRIGHT,rightVelocityMaxRads,rightVelocityMinRads,mRequestedRightSpeed);
      if (mRequestedRightSpeed < 0)
      {
      mRightPWM = angularVelocitytoPWM(BMaxRIGHT, BMinRIGHT,MaxSpeed,MinSpeed,mRequestedRightSpeed);
      }
      else
      {
      mRightPWM = angularVelocitytoPWM(MaxRIGHT, MinRIGHT,MaxSpeed,MinSpeed,mRequestedRightSpeed);
      }
    }
    else
    {
        if (diff > 1)
        {
                sendspeed = true;
        }
    }




    if (mRequestedLeftSpeed  < 0 ||  mRequestedRightSpeed < 0)
        {
        if (mRequestedLeftSpeed <  0)
            {
            if (mRequestedRightSpeed < 0)
                {
                mDirection = 'B';
                }
            else
                {
                mDirection ='L';
                }
            }
        else
            {
            mDirection ='R';
            }
        }
    else
        {
        mDirection ='F';
        }

    if (mDirection != mLastDirection)
        {
        nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"D %d %c\n",CommandID,mDirection);
        if (mUseSlave)
        {
        	mComm.SerialWrite(OutgoingBuffer,nbrBytes);
        }

        }


    if (sendspeed)
    {
        nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"M %d %d %d\n",CommandID,mLeftPWM,mRightPWM);
        if (UseSlave)
			{
        	mComm.SerialWrite(OutgoingBuffer,nbrBytes);
			}
        mLastCommndSent = tnow;

    }

    RCLCPP_INFO(this->get_logger(), "TwistCallBack: lin %f ang %f lv %f rv %f rls %f rrs %f L %d R %d Dir %c %c", TargetLinearVelocity , targetAngularVelocity,leftVelocity, rightVelocity,
           mRequestedLeftSpeed, mRequestedRightSpeed, mLeftPWM,mRightPWM,mDirection,mLastDirection);
    mLastDirection = mDirection;
    }
} //ucCommandCallback


void GetStateCallBack(const std_msgs::msg::Int8::SharedPtr msg )
{
    int8_t statecommand = msg->data;
    RCLCPP_INFO(this->get_logger(), "GetStateCallBack %d current MotorControl state = %d",statecommand,State);
    int nbrBytes;
    if (statecommand == 0)
    {
    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"S\n");
    if (UseSlave)
	    {
        mComm.SerialWrite(OutgoingBuffer,nbrBytes);
		}
    }
    else
    {
        if (statecommand == 1)
            {
                MotorControl::SetEcho(0);
            }
        else if  (statecommand == 2)
        {
        MotorControl::RequestConfig(); 
        }
        else if  (statecommand == 3)
        {
        MotorControl::SetGain();
        }
        else if  (statecommand == 4)
        {
        MotorControl::StartSensor();
        }        
        else if  (statecommand ==6)
        {
        MotorControl::StartSensor();
        }        
        State = statecommand;


        /* code */
    }
    
}

void CalcOdom2()
{  
    // code from turtlebot3


  double delta_s, theta, delta_theta;


   int localleftclick,localrightclick,dt;
   double  VelLeft, VelRight, Velth, deltaX, deltaY,linear, angular;//deltath,

//        long lastleftclick=0,lastrightclick=0, deltaLeft, deltaRight;

   //get time period for tick difference
   dt = msensor.getTimeStamp()-SlaveTimestamp ;
   // first time through  SlaveTimestamp=0 default to 100 ms
   if (dt > 200)
   {
       dt = 100;
   }

   SlaveTimestamp = msensor.getTimeStamp();

   localleftclick = msensor.getLeftClick();
   localrightclick = msensor.getRightClick();

   // how many click did wee see.
   deltaLeft =  (localleftclick -lastleftclick);
   deltaRight = (localrightclick - lastrightclick);

   // save current click count for next run
   lastleftclick = localleftclick;
   lastrightclick = localrightclick;

   double dtSeconds = dt/1000.0;

   VelLeft =  LEFTTICK2RAD * (double)deltaLeft; //VelLeft/dtSeconds;
   VelRight = RIGHTTICK2RAD * (double)deltaRight;//VelRight/dtSeconds;

   delta_s = WHEELRADIUS * (deltaLeft + deltaRight) / 2.0;
   theta   = WHEELRADIUS * (deltaRight - deltaLeft) / BASE;  

   delta_theta = theta;// - lasttheta;

  bot_pose.x += delta_s * cos(bot_pose.theta + (delta_theta / 2.0));
  bot_pose.y += delta_s * sin(bot_pose.theta + (delta_theta / 2.0));
  bot_pose.theta += delta_theta;

  linear = delta_s / dtSeconds;
  angular = delta_theta / dtSeconds;

  lasttheta = theta;




    RCLCPP_INFO_EXPRESSION(this->get_logger(),LOG_INFO_DEBUG == 1 && (localleftclick > 0 || localrightclick > 0) ,"locallc %d lastlc %d lrc %d lastrc %d linear %f angular %f Vel l %f r %f th %f | delta x %f y %f new X %f y %f pwm L %d R %d",localleftclick,lastleftclick,localrightclick,lastrightclick,linear, angular,VelLeft,VelRight, Velth, 		deltaX,deltaY, bot_pose.x, bot_pose.y,msensor.getLeftMotor(),msensor.getRightMotor());




   /*
   https://github.com/ros-controls/ros_controllers/blob/kinetic-devel/diff_drive_controller/src/diff_drive_controller.cpp

   */


   /**
    * This is a message object. You stuff it with data, and then publish it.
    */
//x,y,th, VelLeft, VelRight, Velth, deltaLeft, deltaRight

   msg.pose.pose.position.x= bot_pose.x;
   msg.pose.pose.position.y= bot_pose.y;
   msg.pose.pose.position.z= 0;


   //set the position
   //geometry_msgs::msg::Quaternion odom_quat = tf2::createQuaternionMsgFromYaw(bot_pose.theta);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, bot_pose.theta );

  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
  // msg.pose.pose.orientation = odom_quat;

   msg.twist.twist.linear.x = linear; //vx;//vx
   msg.twist.twist.linear.y = 0;//vy
   msg.twist.twist.angular.z = angular;

   msg.header.frame_id = odom_frame;
   msg.child_frame_id = base_frame;


   //---------------------------------------------------------------------------------------
   /*
   http://answers.ros.org/question/11973/gathering-wheel-encoder-data-and-publishing-on-the-odom-topic/
   */

   //first, we'll publish the transform over tf
   geometry_msgs::msg::TransformStamped odom_trans;

   odom_trans.header.stamp = this->now();
   odom_trans.header.frame_id = odom_frame;
   odom_trans.child_frame_id = base_frame;

   odom_trans.transform.translation.x = bot_pose.x;
   odom_trans.transform.translation.y = bot_pose.y;
   odom_trans.transform.translation.z = bot_pose.theta;
   odom_trans.transform.rotation = msg.pose.pose.orientation;

   //send the transform
   //ptr_broadcaster->sendTransform(odom_trans);

   //next, we'll publish the odometry message over ROS
   //  nav_msgs::Odometry odom;
   msg.header.stamp = odom_trans.header.stamp;//CurrentEncoderTime;

       //publish the message

   //---------------------------------------------------------

           ptr_pub->publish(msg);
   //----------------------------------------------------------------------

   // send sonar
   sonarmsg.range = msensor.GetFrontRange();
   ptr_pubR->publish(sonarmsg);

   if (  mPubVel )
   {
      geometry_msgs::msg::Vector3 vel;
      vel.x = VelLeft;
      vel.y = VelRight;
      vel.z=0;
      ptr_pubvel->publish(vel); 
   }


}



/*

void CalcOdom()
{
   int localleftclick,localrightclick,dt;
   double  VelLeft, VelRight, Velth, deltaX, deltaY,linear, angular; //deltath


//        long lastleftclick=0,lastrightclick=0, deltaLeft, deltaRight;

   //get time period for tick difference
   dt = msensor.getTimeStamp()-SlaveTimestamp ;
   // first time through  SlaveTimestamp=0 default to 100 ms
   if (dt > 200)
   {
       dt = 100;
   }
   SlaveTimestamp = msensor.getTimeStamp();

   localleftclick = msensor.getLeftClick();
   localrightclick = msensor.getRightClick();
   // ROS_INFO_EXPRESSION(this->get_logger(),LOG_INFO_DEBUG == 1 && (localleftclick > 0 || localrightclick > 0) ,"CalcOdom localleftclick %d lastleftclick %d localrightclick %d lastrightclick %d",localleftclick,lastleftclick,localrightclick,lastrightclick );

   // how many click did wee see.
   deltaLeft =  (localleftclick -lastleftclick);
   deltaRight = (localrightclick - lastrightclick);
   // save current click count for next run
   lastleftclick = localleftclick;
   lastrightclick = localrightclick;


   // need to figureout how to get lape time correctly
   //dt = (e.current_real -e.last_real).toSec();
   // use clicks to calculate distance moved
   VelLeft = (deltaLeft * LEFTMETERPERCLICK ); //0.008863831
   VelRight = (deltaRight * RIGHTMETERPERCLICK ); //0.004696358

   
   //https://www.youtube.com/watch?v=XbXhA4k7Ur8
   //distance = (2PiR* deltaLeft) /tickperrotation
   


   linear = (VelLeft + VelRight)/2.0;
   angular = (VelRight - VelLeft ) /BASE; // divide by wheel base 0.12005

   // ROS_INFO_EXPRESSION(this->get_logger(),LOG_INFO_DEBUG == 1 && (localleftclick > 0 || localrightclick > 0) ,"CalcOdom linear %f angular %f ", linear, angular);


   double dtSeconds = dt/1000.0;
   VelLeft = VelLeft/dtSeconds;
   VelRight = VelRight/dtSeconds;
   Velth = angular/dtSeconds;

   // ROS_INFO_EXPRESSION(this->get_logger(),LOG_INFO_DEBUG == 1 && (localleftclick > 0 || localrightclick > 0) ,"CalcOdom Vel l %f r %f th %f ",VelLeft,VelRight, Velth);



   if (fabs(angular) < 1e-6)
   {
   const double direction = bot_pose.theta + angular * 0.5;

   /// Runge-Kutta 2nd order integration:
   deltaX =linear * cos(direction);
   deltaY =linear * sin(direction);
   bot_pose.x += deltaX;
   bot_pose.y += deltaY;
   bot_pose.theta += angular *dt;

   }
   else
   {
   /// Exact integration (should solve problems when angular is zero):
   const double oldth = bot_pose.theta;
   const double r = linear/angular;
   bot_pose.theta += angular;
     deltaX= r * (sin(bot_pose.theta) - sin(oldth));
     deltaY = -r * (cos(bot_pose.theta) - cos(oldth));
   bot_pose.x  +=  deltaX;
   bot_pose.y  += deltaY;
   }

   RCLCPP_INFO_EXPRESSION(this->get_logger(),LOG_INFO_DEBUG == 1 && (localleftclick > 0 || localrightclick > 0) ,"locallc %d lastlc %d lrc %d lastrc %d linear %f angular %f Vel l %f r %f th %f | delta x %f y %f new X %f y %f pwm L %d R %d",localleftclick,lastleftclick,localrightclick,lastrightclick,linear, angular,VelLeft,VelRight, Velth, 		deltaX,deltaY, bot_pose.x, bot_pose.y,msensor.getLeftMotor(),msensor.getRightMotor());




   
   //https://github.com/ros-controls/ros_controllers/blob/kinetic-devel/diff_drive_controller/src/diff_drive_controller.cpp

   


   
   // * This is a message object. You stuff it with data, and then publish it.

   //x,y,th, VelLeft, VelRight, Velth, deltaLeft, deltaRight
   msg.pose.pose.position.x= bot_pose.x;
   msg.pose.pose.position.y= bot_pose.y;
   msg.pose.pose.position.z= 0;


   //---------------------------------------------------------------------------------------
  
  // http://answers.ros.org/question/11973/gathering-wheel-encoder-data-and-publishing-on-the-odom-topic/
  
   //geometry_msgs::msg::Quaternion odom_quat = tf2::createQuaternionMsgFromYaw(bot_pose.theta);
   tf2::Quaternion q;
   q.setRPY(0.0, 0.0, bot_pose.theta );

   msg.pose.pose.orientation.x = q.x();
   msg.pose.pose.orientation.y = q.y();
   msg.pose.pose.orientation.z = q.z();
   msg.pose.pose.orientation.w = q.w();

   //first, we'll publish the transform over tf
   geometry_msgs::msg::TransformStamped odom_trans;


   odom_trans.header.stamp = this->now();
   odom_trans.header.frame_id = "odom";
   odom_trans.child_frame_id = base_frame;

   odom_trans.transform.translation.x = bot_pose.x;
   odom_trans.transform.translation.y = bot_pose.y;
   odom_trans.transform.translation.z = 0.0;
   odom_trans.transform.rotation = msg.pose.pose.orientation;

//msg.pose.pose.orientation.x = q.x();

   //send the transform
   ptr_broadcaster->sendTransform(odom_trans);

   //next, we'll publish the odometry message over ROS
   //  nav_msgs::Odometry odom;
   msg.header.stamp = odom_trans.header.stamp;//CurrentEncoderTime;
   msg.header.frame_id = "odom";

   //set the position
   msg.pose.pose.position.x = bot_pose.x;
   msg.pose.pose.position.y = bot_pose.y;
   msg.pose.pose.position.z = 0.0;
   //msg.pose.pose.orientation = odom_quat;

   //set the velocity
   msg.child_frame_id = "base_link";
   msg.twist.twist.linear.x = linear;//deltaX; //vx;//vx
   msg.twist.twist.linear.y = angular;//deltaY;//vy
   msg.twist.twist.angular.z = Velth;


       //publish the message

   //---------------------------------------------------------

           ptr_pub->publish(msg);
   //----------------------------------------------------------------------

   // send sonar
   sonarmsg.range = msensor.GetFrontRange();
   ptr_pubR->publish(sonarmsg);

   if (  mPubVel )
   {
      geometry_msgs::msg::Vector3 vel;
      vel.x = VelLeft;
      vel.y = VelRight;
      vel.z=0;
      ptr_pubvel->publish(vel);
   }

}
*/

bool StartSensor()
{
    int nbrBytes;
    RCLCPP_INFO(this->get_logger(), "Sending H B 1 Reset encoder counter and turn on sensor");
    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"H\nB 3\n");
    mComm.SerialWrite(OutgoingBuffer,nbrBytes);


    return true;
}

void RequestConfig()
{
    int nbrBytes;
    RCLCPP_INFO(this->get_logger(), "Sending P");
    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"P\n");
    mComm.SerialWrite(OutgoingBuffer,nbrBytes);
}
void SetSonar( int Index)
{
int nbrBytes;
    RCLCPP_INFO(this->get_logger(), "Sending A S %d %d", Index,mconfig.SonarAddress(Index));
    nbrBytes=snprintf(OutgoingBuffer, NUMOUTGOING,"A S %d %d\n", Index+1,mconfig.SonarAddress(Index));
    mComm.SerialWrite(OutgoingBuffer,nbrBytes);

}
void SetGain(){
    int nbrBytes;
    RCLCPP_INFO(this->get_logger(), "Sending G %d 80 10\n",CommandID);
    nbrBytes=snprintf(OutgoingBuffer, NUMOUTGOING,"G %d 80 10\n", CommandID);
    mComm.SerialWrite(OutgoingBuffer,nbrBytes);
}
// need to figure these out for left and right wheel

int angularVelocitytoPWM(int MotorMax, int MotorMin,double VelocityMax,double VelocityMin,double angularVelocity)
{
int motorCmd;
double slope,yintercept;
if (angularVelocity == 0)
    {
    return 0;
    }
slope = (MotorMax - MotorMin) / (VelocityMax - VelocityMin );
yintercept = MotorMax - (slope * VelocityMax);



motorCmd = (int) (slope * fabs(angularVelocity) + yintercept);
if (motorCmd > MotorMax)
{
motorCmd = MotorMax;
}
if (motorCmd < MotorMin)
{
motorCmd = MotorMin;
}
//motorCmd = -motorCmd;

/*
lets just abs not matter what
if (angularVelocity > 0) // positive angular velocity
    {
    motorCmd = (int)(slope * angularVelocity + yintercept);
    if (motorCmd > MotorMax)
        {
        motorCmd = MotorMax;
        }
      if (motorCmd < MotorMin)
        {
        motorCmd = MotorMin;
        }
    }
else // negative angular velocity
    {
    motorCmd = (int) (slope * fabs(angularVelocity) + yintercept);
    if (motorCmd > MotorMax)
        {
        motorCmd = MotorMax;
        }
    if (motorCmd < MotorMin)
        {
        motorCmd = MotorMin;
        }
    motorCmd = -motorCmd;
    }
*/
return motorCmd;

}


void readLoop()
{
    char *ptr;
    int strLen,BeginCommand;

    if (mComm.SerialRead())
    {

     if ( mComm.SerialLineAvailable())
     {
       ptr = mComm.SerialGetData();
       strLen = strlen(ptr);
       for (BeginCommand = 0;BeginCommand <=strLen && !(ptr[BeginCommand] == '$' || ptr[BeginCommand] == 'E' || ptr[BeginCommand] == '1' ) ; BeginCommand++)
       {
           ;
       }
       if (ptr[BeginCommand]=='$' && ptr[BeginCommand+1]=='0' )
       {
        if (ptr[BeginCommand+2] == '3')
        {
            // sensor
            if (State == 6 && ptr[(BeginCommand+strLen)-2] =='Z')
            {
 //                sensorData msensor;

                 // 0123456789112345678921234567893123456
                 // $03  2ff5   572   a9710d11dR ad3 688Z
                 //we have a sensor reading
               //  RCLCPP_INFO(this->get_logger(), "sensor reading");
                 msensor.getSensorData(&ptr[BeginCommand],mconfig);
                 // call odom I think
                 CalcOdom2();
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "ELSE sensor reading");
            }



        }
        else if (ptr[BeginCommand+2] == '1')
        {
            // annoucement state 1
            RCLCPP_INFO(this->get_logger(), "Robot Build %s",&ptr[BeginCommand+13]);
            State=1;
            MotorControl::SetEcho(0);
        }
        else if (ptr[BeginCommand+2] == '2')
        {
            // error
            // error
        int slavestate = SlaveParse::ParseHex(&ptr[BeginCommand], 9, 2);
        int result = SlaveParse::ParseInt(&ptr[BeginCommand], 11, 3);
        RCLCPP_ERROR(this->get_logger(), "Slave State %d Error %d",slavestate,result);

        }
        else if (ptr[BeginCommand+2] == '6')
        {
            //current state of slave
            int slavestate = SlaveParse::ParseHex(&ptr[BeginCommand], 6, 2);

            RCLCPP_INFO(this->get_logger(), "Robot state %d slave state  %d",State,slavestate);

        }
        else if (ptr[BeginCommand+2] == '7')
        {
            // config
            RCLCPP_INFO(this->get_logger(), "Slave config received");
            if (ptr[BeginCommand+24] == 'Z' || ptr[BeginCommand+29] == 'Z')
            {
                // we have the whole config pass it to the config object to parse
              mconfig =  BotConfig(&ptr[BeginCommand]);
              if (!mconfig.IsConfigValid() )
              {
                  RCLCPP_ERROR(this->get_logger(), "invalid config setting sonar" );
                  // This can happen if slave code was updated the eeprom config gets cleared
                  // so we need to reset config and reread config
                  mconfig.ResetConfig();
                  for (int Index =0; Index < mconfig.NumberSonars();Index++ )
                  {
                   MotorControl::SetSonar(  Index);
                   State =3;
                   RCLCPP_INFO(this->get_logger(), "MotorControl state 3");

                  }
    
              }
              else
              {
                   RCLCPP_INFO(this->get_logger(), "motorControl BotConfig valid. %d sonars",mconfig.NumberSonars());
              }
              // set this for later use in message $03
              sensorEnd = (mconfig.HasCompass()? 4 :0) + (mconfig.NumberSonars() * 4) + 28;
              // now need to get range and gain every time.
              CommandID++;
              MotorControl::SetGain();
              State = 4;
              RCLCPP_INFO(this->get_logger(), "MotorControl state 4");              
              }
        }
        else if (ptr[BeginCommand+2] == '8')
            {
                // port value
                int slavestate = SlaveParse::ParseHex(&ptr[BeginCommand], 3, 2);
                RCLCPP_INFO(this->get_logger(), "Slave portvalue %d",slavestate);
            }
        else if (ptr[BeginCommand+2] == '9')
            {
            if (State == 1)
                {
                State=2;
                RCLCPP_INFO(this->get_logger(), "MotorControl state 2");                   
                MotorControl::RequestConfig();                
                }
            else if (State == 4)
                {
                State = 6;
                RCLCPP_INFO(this->get_logger(), "MotorControl state 6");   
                MotorControl::StartSensor();              
                }
            else
                {
                State = 6;
                RCLCPP_INFO(this->get_logger(), "MotorControl state 6 else");                 
                MotorControl::StartSensor();
                }
            
            }
        else
            {
                     RCLCPP_ERROR(this->get_logger(), "unknown command %c",ptr[BeginCommand+2] );
            }


       }
       else if( ptr[BeginCommand] == 'E' )
       {
        State=2;
        RCLCPP_INFO(this->get_logger(), "MotorControl state 2");           
        MotorControl::RequestConfig();
       }
       else if( ptr[BeginCommand] == '1' )
       {
        State = 6;
        RCLCPP_INFO(this->get_logger(), "MotorControl state 6 from command 1");           
        MotorControl::StartSensor();
       }
       else
       {
           RCLCPP_ERROR(this->get_logger(),"unknown command %s",ptr );

       }
    }
   }
}

void fakeReadLoop()
{
// need to hard code this so the TwistCallBack will process
State=6;

    auto& clk = *this->get_clock();

   RCLCPP_INFO_THROTTLE(this->get_logger(),clk,10, "fakereadloop");

   int localleftclick;
   int localrightclick;
   unsigned short leftMotor;
   unsigned short rightMotor; 
   char direction;
   double VelLeft;
   double VelRight;


   // get last reported values
   localleftclick = msensor.getLeftClick();
   localrightclick = msensor.getRightClick();
   leftMotor = msensor.getLeftMotor();
   rightMotor =msensor.getRightMotor();
   direction = msensor.getDirection();


    RCLCPP_INFO_EXPRESSION(this->get_logger(),LOG_INFO_DEBUG == 1 && (mRequestedLeftSpeed > 0 || mRequestedRightSpeed > 0) ,"fakeReadLoop input click L %d %d PWM L %d %d Dir %c req speed L %f %f", localleftclick,localrightclick,leftMotor, rightMotor,direction,mRequestedLeftSpeed,mRequestedRightSpeed  );

   //calculate what clicks should have been
   //convert rad to meter per second (mps)
   // wheel velocity(m/s) = wheel rotation(rad/s) * wheel radius(m)
   // then divide by  meter per tick
   // then divide by 10 to get ticks in this time period

	VelLeft = mRequestedLeftSpeed  * WHEELRADIUS; // convert to mps
	VelLeft = VelLeft / LEFTMETERPERCLICK;
	VelLeft = VelLeft /10; 

	VelRight = mRequestedRightSpeed  * WHEELRADIUS; // convert to mps
	VelRight = VelRight / RIGHTMETERPERCLICK;
	VelRight = VelRight /10;

	// handle direction change
	if (direction != mDirection)
		{
		if (mDirection == 'F')
			{
			if(direction == 'L')
               {
               VelLeft = 0;
               if (rightMotor < mRightPWM)
				{
                VelRight = VelRight * (rightMotor/mRightPWM); 
                }
               }
			if(direction == 'R')
               {
               VelRight = 0;
               if (leftMotor < mLeftPWM)
				{
                VelLeft = VelLeft * (leftMotor/mLeftPWM); 
                }
               }
			if(direction == 'B')
               {
               VelLeft = 0;
               VelRight = 0; 
               }
			if(direction == 'S')
               {
               VelLeft = 0;
               VelRight = 0; 
               }
			}
		else
			{
		if (mDirection == 'L')
			{
			if(direction == 'F')
               {
               VelLeft = 0;
  
               if (rightMotor < mRightPWM)
				{
                VelRight = VelRight * (rightMotor/mRightPWM); 
                }
               }
			if(direction == 'R')
               {
               VelRight = 0;
               VelRight = 0; 
               }
			if(direction == 'B')
               {
               VelRight = 0;
               if (leftMotor < mLeftPWM)
				{
                VelLeft = VelLeft * (leftMotor/mLeftPWM); 
                }
               }
			if(direction == 'S')
               {
               VelLeft = 0;
               VelRight = 0; 
               }
			}
			else
				{
				if (mDirection == 'R')
					{
					if(direction == 'F')
				       {
				       VelRight = 0;
				       if (leftMotor < mLeftPWM)
						{
				        VelLeft = VelLeft * (leftMotor/mLeftPWM); 
				        }
				       }
					if(direction == 'R')
				       {
				       VelLeft = 0;
				       VelRight = 0; 
				       }
					if(direction == 'B')
				       {
				       VelLeft = 0;
		  
				       if (rightMotor < mRightPWM)
						{
				        VelRight = VelRight * (rightMotor/mRightPWM); 
				        }
				       }
					if(direction == 'S')
				       {
				       VelLeft = 0;
				       VelRight = 0; 
				       }
					}
				if (mDirection == 'B')
					{
					if(direction == 'F')
				       {
				       VelLeft = 0;
				       VelRight = 0; 
				       }
					if(direction == 'R')
				       {
				       VelLeft = 0;
		  
				       if (rightMotor < mRightPWM)
						{
				        VelRight = VelRight * (rightMotor/mRightPWM); 
				        }
				       }
					if(direction == 'L')
				       {

				       VelRight = 0;
				       if (leftMotor < mLeftPWM)
						{
				        VelLeft = VelLeft * (leftMotor/mLeftPWM); 
				        }

				       }
					if(direction == 'S')
				       {
				       VelLeft = 0;
				       VelRight = 0; 
				       }
					}


				}

			}		
		}
localleftclick = localleftclick + (int) VelLeft;
localrightclick = localrightclick + (int) VelRight;


//RIGHTTICKPERROTATION
//LEFTTICKPERROTATION



    //mLastRequestedLeftSpeed = mRequestedLeftSpeed;
    //mLastRequestedRightSpeed = mRequestedRightSpeed;
    //mLastLeftPWM= mLeftPWM;
    //mLastRightPWM = mRightPWM;
    //mDirection != mLastDirection%c

      RCLCPP_INFO_EXPRESSION(this->get_logger(),LOG_INFO_DEBUG == 1 && (mRequestedLeftSpeed > 0 || mRequestedRightSpeed > 0) ,"fakeReadLoop output click L %d %d PWM L %d %d Dir %c ",localleftclick, localrightclick,mLastLeftPWM, mLastRightPWM, mLastDirection);
msensor.setSensorTestData(localleftclick, localrightclick,mLeftPWM, mRightPWM, mLastDirection);


   CalcOdom2();
}
//MotorControl::timer_callback
void timer_callback()
    {
 //ros::Rate r(100);
 // while (ros::ok()) {
    if (UseSlave)
	  {
      // process incoming
      readLoop();
      }
    else
      {
      fakeReadLoop();
      }
   // ros::spinOnce();

  //}

}

}; // end of class definition
int main(int argc, char **argv)
{

  //Initialize ROS
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
   // moved to destrcutor
   //if (node->UseSlave)
   ///	{
   /// node->CloseSerial();
   /// }
   /// RCLCPP_INFO(node->get_logger(), "r2Serial stopping");

   
    return 0;
}
