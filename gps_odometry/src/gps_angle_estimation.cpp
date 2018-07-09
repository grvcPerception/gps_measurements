// Jenifer Delgado Rich
// jedelrich@hotmail.com
// Odometria sacada de datos de GPS diferencial (< 2cm de error). El (0,0,0) del eje de referencia es la
// primera posicion que se obtiene del GPS y este eje de referencia esta orientado en Este --> X, Norte --> Y

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>
#include <vector>
#include <termios.h>

#define N_NODOS 1
#define pi 3.14159265359

using namespace Eigen;

double x, y, ang;
bool once = 0;

std::string gps_frame_id;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void gps_odom_callback(const nav_msgs::Odometry& message)
{
	once = 1;
    x = message.pose.pose.position.x;
    y = message.pose.pose.position.y;
}


int main ( int argc, char** argv )
{
    ros::init ( argc, argv, "gps_angle_estimation" );

    ros::NodeHandle node("~");

    std::string gps_topic;

    if(!node.getParam("gps_odom_Topic", gps_topic))
    {
        ROS_ERROR("No port given to 'gpsTopic'");
        exit(0);
    }

    ROS_INFO_STREAM("Subscribed to topic " << gps_topic);
    ROS_INFO_STREAM("Este programa se encarga de estimar el angulo necesario (angle_to_north), medido desde el eje y hasta el norte en sentido +, para que el eje x de la odometria gps mire hacia delante del dron.");
    ROS_INFO_STREAM("Por favor, procure que el gps este en narrow int en todo momento. Pulse 'y' cuando haya movido el dron en linea recta hacia delante, y cualquier otra tecla para leer la posicion actual");

    ros::Subscriber sub = node.subscribe(gps_topic.c_str(), 1, gps_odom_callback);

    while(ros::ok())
    {
    	ros::spinOnce();
    	int c = getch();
    	if(c == 'y')
    	{
    		if(once)
    		{
    			ang = atan(-y/x);
    			if(x*cos(ang+pi) - y*sin(ang+pi) >= 0)
    				ang = ang+pi;

    			ROS_INFO("Boton pulsado, angulo %lf, distancia movida %lf",ang*180/pi,x*cos(ang) - y*sin(ang));
    			ROS_INFO("Saliendo del programa de medida de angulo, pulse Ctrl+c (o espere) para salir de la odometria");
    			break;
    		}
    		else
    		{
    			ROS_INFO("Boton pulsado, pero todavia no hay medidas gps");
    		}
    	}
  	
    }

    return 0;
}
