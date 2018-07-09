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

#define N_NODOS 1
#define pi 3.14159265359


ros::Publisher odom_pub;
ros::Publisher odom_pub2;
ros::Publisher gps_pose_pub;
double ang, offset; /* respecto al norte*/

using namespace Eigen;

double posIni[3], posAct[3], posAnt[3];
bool first_time = true, initPos = false;

std::string gps_frame_id;


void wgs2ecef(double lat,double lon,double alt,double pos_ecef[3])
{
    // De grados a radianes
        lat = lat*pi/180;
        lon = lon*pi/180;

        double aux;

    // Constantes
        // Semi-eje mayor
        float a = 6378137;
        // Factor de aplanamiento
        double f = 1/298.257223563;
        // Semi-eje menor
        double b = a * (1-f);

        // Primera excentricidad
        double e = sqrt(pow(a,2)-pow(b,2))/a;

        // Radio meridiano de curvatura
        double e2 = pow(e,2);
        aux = sin(lat);
        double m_e = (1 - e2*aux);
        m_e = pow(m_e,1.5);
        m_e = (a * (1 - e2)) / m_e;
        // Radio de curvatura vertical principal
        double n = a / sqrt(1 - e2 * pow(aux,2));

        double x_ecef = (n + alt) * cos(lat) * cos(lon);
        double y_ecef = (n + alt) * cos(lat) * sin(lon);
        double z_ecef = ((n * (1 - e2)) + alt) * sin(lat);
/**
        *pos_enu.clear();
        &pos_enu.push_back(x_ecef);
    **/
        pos_ecef[0] = x_ecef;
        pos_ecef[1] = y_ecef;
        pos_ecef[2] = z_ecef;

}

void ecef2enu(double pos_ecef[3],double pos_ecef_ref[3], double lat,double lon, double pos_enu[3])
{
    double U = pos_ecef[0] - pos_ecef_ref[0];
    double V = pos_ecef[1] - pos_ecef_ref[1];
    double W = pos_ecef[2] - pos_ecef_ref[2];

    lat = lat*pi/180;
    lon = lon*pi/180;

    //Coeficientes
    double a11 = -sin(lon);
    double a12 = cos(lon);
    double a13 = 0;
    double a21 = -sin(lat)*cos(lon);
    double a22 = -sin(lat)*sin(lon);
    double a23 = cos(lat);
    double a31 = cos(lat)*cos(lon);
    double a32 = cos(lat)*sin(lon);
    double a33 = sin(lat);

    double East = a11 * U + a12 * V + a13 * W;
    double North = a21 * U + a22 * V + a23 * W;
    double Up = a31 * U + a32 * V + a33 * W;

    pos_enu[0] = East;
    pos_enu[1] = North;
    pos_enu[2] = Up;
}

void enu2ecef(double pos_enu[3], double pos_ref_ecef[3], double lat, double lon, double PlocalECEF[3])
{
    lat = lat * pi/180;
    lon = lon * pi/180;

    //Coeficientes
    double a11 = -sin(lon);
    double a12 = -sin(lat)*cos(lon);
    double a13 = cos(lat)*cos(lon);
    double a21 = cos(lon);
    double a22 = -sin(lat)*sin(lon);
    double a23 = cos(lat)*sin(lon);
    double a31 = 0;
    double a32 = cos(lat);
    double a33 = sin(lat);

    double x_ecef = a11 * pos_enu[0] + a12 * pos_enu[1] + a13 * pos_enu[2];
    double y_ecef = a21 * pos_enu[0] + a22 * pos_enu[1] + a23 * pos_enu[2];
    double z_ecef = a31 * pos_enu[0] + a32 * pos_enu[1] + a33 * pos_enu[2];

    PlocalECEF[0] = x_ecef + pos_ref_ecef[0];
    PlocalECEF[1] = y_ecef + pos_ref_ecef[1];
    PlocalECEF[2] = z_ecef + pos_ref_ecef[2];
}

void ecef2wgs(double pos_ecef[3], double pos_wgs[3])
{
    double X = pos_ecef[0];
    double Y = pos_ecef[1];
    double Z = pos_ecef[2];

    // Set the constants
    double a = 6378137; // semi-major axis in meters
    double f = 1/298.257223563; // the flattening factor
    double b = a*(1-f);  // semiminor axis in meters
    double e = sqrt(pow(a,2) - pow(b,2))/a; // first eccentricity

    double p = (pow(X,2) + pow(Y,2)) / pow(a,2);

    double q = (1-pow(e,2)) / pow(a,2);
    q = q * pow(Z,2);


    double r = (p+q-pow(e,4))/6;

    double s = pow(e,4)*(p*q)/(4*pow(r,3));

    double t = pow(1 + s + sqrt(s*(2+s)), 1.0/3);

    double u = r * (1 + t + (1/t));

    double v = sqrt(pow(u,2) + pow(e,4) * q);

    double w = pow(e,2) * ((u+v-q)/(2*v));

    double k = sqrt(u+v+pow(w,2))-w;

    double D = k * sqrt(pow(X,2) + pow(Y,2)) / (k+pow(e,2));

    // Final Form
    // use the atan2 function for more numerical stability
    double lon = atan2(Y,X)*180/pi;

    double lat = atan2(Z,D)*180/pi;

    double h = ((k + pow(e,2) - 1) / k) * sqrt(pow(D,2) + pow(Z,2));

    pos_wgs[0] = lat;
    pos_wgs[1] = lon;
    pos_wgs[2] = h;
}

geometry_msgs::PoseStamped posegps;
ros::Timer timer;

void gps_odom_callback(const sensor_msgs::NavSatFix& message)
{
    //if(initPos == true)
    //{
        if(message.status.status == -1)
        {
            ROS_ERROR("OdomGPS_callback: El status del GPS no es el correcto para el calculo de odometria\n");
            return;
        }
        nav_msgs::Odometry msgVis;
        tf::TransformBroadcaster odom_broadcaster;
        geometry_msgs::TransformStamped odom_trans;
        static tf::TransformBroadcaster br;
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        double pos_ref_ecef[3], pos_aux_ecef[3];
        double gamma = 0.0, posX, posY;

        // Latitud, longitud y altitud segun WGS84 del punto de referencia. ALEATORIO
        double lat_ref = 37.410922;	//Grados
        double lon_ref = -6.001800;	//Grados
        double alt_ref = 0.2;		//Metros

        wgs2ecef(lat_ref, lon_ref, alt_ref, pos_ref_ecef);

        geometry_msgs::Quaternion q_gps;
        q_gps.x = 0.0; q_gps.y = 0.0; q_gps.z = 0.0; q_gps.w = 1.0;        

        // Latitud, longitud y altitud segun WGS84 del punto de referencia
        if(first_time == true)
        {
            first_time = false;
            std::cout << "GPS_callback [First time]: Primera vez que nos llega medida de GPS" << std::endl;
            wgs2ecef(message.latitude, message.longitude, message.altitude, pos_aux_ecef);
            ecef2enu(pos_aux_ecef, pos_ref_ecef, lat_ref, lon_ref, posIni);


            /** Tf transform **/
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = gps_frame_id; // "/map"
            odom_trans.child_frame_id = message.header.frame_id;
            odom_trans.transform.translation.x = 0.0;
            odom_trans.transform.translation.y = 0.0;
            odom_trans.transform.translation.z = offset;
	    std::cout << "altura " << message.altitude << std::endl;

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(gamma);
            odom_trans.transform.rotation = odom_quat;

            odom_broadcaster.sendTransform(odom_trans);


            /** Odometry message **/
            msgVis.header.frame_id = gps_frame_id; // "/map"
            msgVis.header.stamp = message.header.stamp;
            msgVis.child_frame_id = message.header.frame_id;

            msgVis.pose.pose.position.x = 0.0;
            msgVis.pose.pose.position.y = 0.0;
            msgVis.pose.pose.position.z = offset; 
            msgVis.pose.pose.orientation = odom_quat;

            /** Geometry_msgs Posestamped message **/
            // GPS does not give rotation information
            posegps.header = msgVis.header;
            posegps.pose.position = msgVis.pose.pose.position;
            posegps.pose.orientation = q_gps;



            for(int i = 0; i < 9; i++)
            {
                msgVis.pose.covariance[0] = message.position_covariance[0];
            }
            for(int i = 9; i < 36; i++)
            {
                msgVis.pose.covariance[0] = 0.0;
            }
            odom_pub2.publish(msgVis);
	    timer.start();
            gps_pose_pub.publish(posegps);
            for(int i = 0; i < 3; i++)
            {
               posAnt[i] = posIni[i];
            }
	    posIni[2] = message.altitude;

	    std::cout << "GPS_callback [First time]: La posicion inicial es x: " << odom_trans.transform.translation.x << " y: " << odom_trans.transform.translation.y << std::endl;
	    //std::cout << "GPS_callback [First time]: La posicion inicial es x: " << posIni[0]<< " y: " << posIni[1] << std::endl;

        }
        else
        {
            //std::cout << "GPS_callback: Calculando odometria" << std::endl;
            wgs2ecef(message.latitude, message.longitude, message.altitude, pos_aux_ecef);
            ecef2enu(pos_aux_ecef, pos_ref_ecef, lat_ref, lon_ref, posAct);


            /** Tf transform **/
            odom_trans.header.stamp = message.header.stamp;
            odom_trans.header.frame_id = gps_frame_id; // "/map"
            odom_trans.child_frame_id = message.header.frame_id;


	    posAct[2] = message.altitude;

            odom_trans.transform.translation.x = (posAct[0] - posIni[0])*cos(ang) - (posAct[1] - posIni[1])*sin(ang);
            odom_trans.transform.translation.y = (posAct[0] - posIni[0])*sin(ang) + (posAct[1] - posIni[1])*cos(ang);
            odom_trans.transform.translation.z = (posAct[2] - posIni[2]) + offset;


            gamma = atan2((posAct[0] - posAnt[0])*sin(ang) + (posAct[1] - posAnt[1])*cos(ang),(posAct[0] - posAnt[0])*cos(ang) - (posAct[1] - posAnt[1])*sin(ang));
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(gamma);
            odom_trans.transform.rotation = odom_quat;

            odom_broadcaster.sendTransform(odom_trans);

            /** Odometry message **/
            msgVis.header.frame_id = gps_frame_id; // "/map"
            msgVis.header.stamp = message.header.stamp;
            msgVis.child_frame_id = message.header.frame_id;

            msgVis.pose.pose.position.x = (posAct[0] - posIni[0])*cos(ang) - (posAct[1] - posIni[1])*sin(ang);
            msgVis.pose.pose.position.y = (posAct[0] - posIni[0])*sin(ang) + (posAct[1] - posIni[1])*cos(ang);
            /**msgVis.pose.pose.position.x = posX;
            msgVis.pose.pose.position.y = posY;**/
            msgVis.pose.pose.position.z = (posAct[2] - posIni[2]) + offset;
            msgVis.pose.pose.orientation = odom_quat;

            /** Geometry_msgs Posestamped message **/
            posegps.header = msgVis.header;
            posegps.pose.position = msgVis.pose.pose.position;
            posegps.pose.orientation = q_gps;


            for(int i = 0; i < 9; i++)
            {
                msgVis.pose.covariance[i] = 0.0; /** message.position_covariance[0] **/
            }
            for(int i = 9; i < 36; i++)
            {
                msgVis.pose.covariance[i] = 0.0;
            }

            /*if(message.status.status == 4){
            	odom_pub2.publish(msgVis);
              gps_pose_pub.publish(posegps);
            else*/
            odom_pub2.publish(msgVis);
            gps_pose_pub.publish(posegps);

            for(int i = 0; i < 3; i++)
            {
                posAnt[i] = posAct[i];
            }
	    
	    std::cout << "GPS_callback : La posicion es x: " << odom_trans.transform.translation.x << " y: " << odom_trans.transform.translation.y << std::endl;
	   //std::cout << "GPS_callback: La posicion es x: " << posAct[0]<< " y: " << posAct[1] << std::endl;
        }
    //}
}

void timerCallback(const ros::TimerEvent &){
	gps_pose_pub.publish(posegps);
}


int main ( int argc, char** argv )
{
    ros::init ( argc, argv, "gps_odom_mavros_timer" );

    ros::NodeHandle node("~");

    std::string gps_topic, out_pose_topic;

    if(!node.getParam("gpsTopic", gps_topic))
    {
        ROS_ERROR("No port given to 'gpsTopic'");
        exit(0);
    }

    if(!node.getParam("gps_frame_id", gps_frame_id))
    {
        ROS_ERROR("No port given to 'gps_frame_id'");
        exit(0);
    }

    if(!node.getParam("angle_to_north", ang))
    {
        ROS_WARN("Parameter 'angle_to_north' not set. [Default value = 0]");
        ang = 0.0;
    }

    if(!node.getParam("offset", offset))
    {
        ROS_WARN("Parameter 'offset' not set. [Default value = 0]");
        offset = 0.0;
    }

    if(!node.getParam("out_pose_topic", out_pose_topic))
    {
        ROS_WARN("Parameter 'out_pose_topic' not set");
        exit(0);
    }

    int frame_rate;
    if(!node.getParam("frame_rate", frame_rate))
    {
        ROS_WARN("Parameter 'frame_rate' not set");
        exit(0);
    }

    ROS_INFO_STREAM("Subscribed to topic " << gps_topic);
    ROS_INFO_STREAM("Frame where odom is referenced is " << gps_frame_id);
    ROS_INFO_STREAM("Angle respect to North is " << ang);

    ang = (ang * pi)/180;

    //odom_pub = node.advertise<nav_msgs::Odometry>("/odometry_ground_truth", 1);
    odom_pub2 = node.advertise<nav_msgs::Odometry>("/odometry_ground_truth", 1);
    gps_pose_pub = node.advertise<geometry_msgs::PoseStamped>(out_pose_topic, 1);

    timer = node.createTimer(ros::Duration(ros::Rate(frame_rate)), timerCallback, false, false);
    //timer.start();

    ros::Subscriber sub = node.subscribe(gps_topic.c_str(), 1, gps_odom_callback);

    ros::spin();

    return 0;
}
