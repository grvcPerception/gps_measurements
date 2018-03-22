#include "modulo_gps/gps.h"

ros::Publisher pub_gps;


#define VEL_MIN 0.5
#define NOT_MAGNETOMETRO 0x0024

using namespace std;

int main(int argc, char **argv) {

  // Inicio de ROS
  ros::init(argc, argv, "GPS");
  // Manejador ROS
  ros::NodeHandle n("~");

  // Generación de publicadores
// cambiar todo esto por NavSatFix
  pub_gps = n.advertise<sensor_msgs::NavSatFix>("global", 1000);

  ros::Time tiempo;
  // Creación de mensaje de publicacion de datos
	sensor_msgs::NavSatFix gps_msg;

  // Todo esta correcto, lo especificamos con el correspondiente parametro

       GPS_Management *gps;

      std::string gpsPort;

   if(!n.getParam("gpsPort", gpsPort))
    {
        ROS_ERROR("No port given to 'gpsPort'");
        ROS_WARN("Trying to connect to /dev/ttyUSB0");
        gpsPort = "/dev/ttyUSB0";
    }

      
      // Funcionamiento del modo debug
      gps = new GPS_Management(gpsPort.c_str());
	
      if (gps->isPortOpened()){

        gps->gps_log_general("bestgpsposa", "ontime 0.1");
        //gps->gps_log_general("bestgpsvela", "ontime 0.1");
        gps->gps_log_general("clockmodela", "ontime 0.1");
        while(ros::ok())
        {
        while (gps->getGPSPos().sol_status != "SOL_COMPUTED" && ros::ok())
        {
          gps->rcvData();
          cout << "Esperando Alineamiento de GPS: " << gps->getGPSPos().sol_status << endl;
        }
        cout << "GPS Alineado" << endl;

        bool flagBestGPSPosa = false;
        int typeFrame;
        ROS_INFO("Empiezo a recibir datos");
        while (gps->getGPSPos().sol_status == "SOL_COMPUTED" && ros::ok())
        {
            typeFrame = gps->rcvData();
            switch (typeFrame) {
              case TT_BESTGPSPOSA:
                flagBestGPSPosa = true;
                break;
              default:
                break;
                }

            //Compruebo que recibo todas las tramas
            if (flagBestGPSPosa)
            {
                flagBestGPSPosa=false;

                gps_msg.latitude = gps->getGPSPos().lat;
                gps_msg.longitude = gps->getGPSPos().lon;
                gps_msg.altitude = gps->getGPSPos().hgt;
                cout <<"Estado del GPS: " << gps->getGPSPos().pos_type <<endl;
                if(gps->getGPSPos().pos_type == "NONE")
                        gps_msg.status.status = -1;
                else if(gps->getGPSPos().pos_type == "SINGLE")
                        gps_msg.status.status = 0;
                else if(gps->getGPSPos().pos_type == "WAAS")
                        gps_msg.status.status = 1;
                else if(gps->getGPSPos().pos_type == "PSDIFF")
                        gps_msg.status.status = 2;
                else if(gps->getGPSPos().pos_type == "NARROW_FLOAT")
                        gps_msg.status.status = 3;
                else if(gps->getGPSPos().pos_type == "NARROW_INT")
                        gps_msg.status.status = 4;
                else
                    gps_msg.status.status = 5;

//                if(gps->rcvData() == TT_CLOCKMODEL)
//                {
//                    for(int i = 0; i < 9; i++)
//                        gps_msg.position_covariance[i] = gps->getClockModel().cov_data[i];
//                }
//                else
//                {
                    for(int i = 0; i < 9; i++)
                        gps_msg.position_covariance[i] = 0.0;
//                }

                tiempo = ros::Time::now();
                gps_msg.header.stamp = tiempo;
                gps_msg.header.frame_id = "/gps";
                pub_gps.publish(gps_msg);
             }

            }
//          ros::spinOnce();
      }
	exit(0);}

      else {
        cout << "El puerto no se ha abierto" << endl;
      }


  return 0;
}


