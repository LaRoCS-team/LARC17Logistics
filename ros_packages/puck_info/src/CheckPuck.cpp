#include "../include/check_puck/CheckPuck.hpp"

using namespace cv;
using namespace std;

CheckPuck::CheckPuck(): loopRate(2)
{
  hasPuck = false ;
  puckCamera = false;
  puckSensor = false;
  ha = false;
  distance = 60.0;
    distancey = 60.0;
    distancez = 60.0;
  color = 2;

  iLowS = 150;
  iHighS = 255;
  iLowV = 60;
  iHighV = 255;

    //cout << "ImageHeight CONST: " << altura << endl;

  image_transport::ImageTransport it(n);
  sensorSub = n.subscribe("distance_sensors", 10, &CheckPuck::sensorCallback, this);
    puck_info_sub_ = n.subscribe("identifyPuck", 10, &CheckPuck::puckInfoCallback, this);


    //image_transport::ImageTransport imgTrans(n);
    //imgSub = imgTrans.subscribe("image_raw", 2, &CheckPuck::imageCallback, this);

  pub = n.advertise<std_msgs::Bool>("hasPuck",10);
  //stateSub = n.subscribe("/", 1, &CheckPuck::sensorCallback, this);
}

void CheckPuck::sensorCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    //cout << "Estou no callback" << endl;
  distance = msg->points[0].x;
    distancey = msg->points[0].y;
    distancez = msg->points[0].z;
  //std::cout<<distance;
  if(distance < 0.25)
  {
    ha = true;
    puckSensor = true;
    //std::cout<< "PuckSensor:" << puckSensor<<"\n";
  }else{
      if (ha && distance < 0.3)
      {
        puckSensor = true;
      }
      else{
        puckSensor = false;
      }
  }
    ROS_INFO("ha = %s, distancex = %f, distancey = %f, distancez = %f", ha?"true":"false", distance, distancey, distancez);
}

void CheckPuck::puckInfoCallback(const robotino_msgs::PuckInfo::ConstPtr& msg)
{
    puck_center_Y_ = msg->centroid.y;
    //cout << "centroide Y: " << puck_center_Y_;
}

void CheckPuck::pubBool()
{
    if (puck_center_Y_ > altura*0.5)
    {
        puckCamera = true;
    } else{
        puckCamera = false;
    }
    //cout<< "PuckCamera:" << puckCamera<<endl;
    ROS_INFO("puckCamera = %s, puckSensor = %s, puck_center_Y = %f", puckCamera?"true":"false", puckSensor?"true":"false", puck_center_Y_);
//  if(puckSensor && puckCamera)
    if (puckSensor)
  {
    hasPuck = true;
  }else{
    hasPuck = false;
  }

    //cout << "HasPuck: " << hasPuck << endl;

}

/*void CheckPuck::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Treat image from ROS to OpenCV Mat format
    altura = 240;
    cout << "ImageHeight CALL: " << altura << endl;
}*/

void CheckPuck::spin()
{
    ros::Rate lr(2);

    while (n.ok())
    {
        pubBool();

        ROS_INFO("hasPuck = %s", hasPuck?"true":"false");
        //cout << "HasPuck1: " << hasPuck << endl;

        std_msgs::Bool boolMsg;
        boolMsg.data = hasPuck;

        pub.publish(boolMsg);
        ros::spinOnce();
        lr.sleep();
    }
}
