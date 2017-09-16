#include <state_feedback/Gps2xy.h>

using namespace state_feedback;

void Gps2xy::update_ecef_ini()
{
   double N = semi_major/(std::sqrt(1-eccen_sq*std::pow(std::sin(lat_ini),2)));

    // X_ECEF
    pos_ECEF_ini[0] = (N + alt_ini) * std::cos(lat_ini)*std::cos(lon_ini);

    // Y_ECEF
    pos_ECEF_ini[1] = (N + alt_ini) * std::cos(lat_ini)*std::sin(lon_ini);

    // Z_ECEF
    pos_ECEF_ini[2] = (N*(1-eccen_sq) + alt_ini)*std::sin(lat_ini);
}

void Gps2xy::geod2ecef()
{
   double N = semi_major/(std::sqrt(1-eccen_sq*std::pow(std::sin(lat),2)));

    // X_ECEF
    pos_ECEF[0] = (N + alt) * std::cos(lat)*std::cos(lon);

    // Y_ECEF
    pos_ECEF[1] = (N + alt) * std::cos(lat)*std::sin(lon);

    // Z_ECEF
    pos_ECEF[2] = (N*(1-eccen_sq) + alt)*std::sin(lat);
}

void Gps2xy::ecef2enu()
{
    // Calculating difference between initial ECEF coordinates and current ECEF coordinates
    double delta_X = pos_ECEF[0] - pos_ECEF_ini[0];
    double delta_Y = pos_ECEF[1] - pos_ECEF_ini[1];
    double delta_Z = pos_ECEF[2] - pos_ECEF_ini[2];

    // following equations are obtained by,
    // pos_ENU = R_ned2enu * R_ECEF2ned * [delta_X; delta_Y; delta_Z];
    //
    // R_ned2enu = [0 1 0; 1 0 0; 0 0 -1];
    //
    // R_ECEF2enu = [ -sin(lat)*cos(lon) -sin(lat)*sin(lon) cos(lat);
    //  		      -sin(lon)           cos(lon)          0;
    //                -cos(lat)*cos(lon) -cos(lat)*sin(lon) -sin(lat)]

    // X_ENU
    pos_ENU[0] = delta_Y*std::cos(lon) - delta_X*std::sin(lon);

    // Y_ENU
    pos_ENU[1] = delta_Z*std::cos(lat) - delta_X*std::cos(lon)*std::sin(lat) -
                    delta_Y*std::sin(lon)*std::sin(lat);

    // Z_ENU
    pos_ENU[2] = delta_Z*std::sin(lat) + delta_X*std::cos(lon)*std::cos(lat) +
                    delta_Y*std::cos(lat)*std::sin(lon);
}

// GPS_callback function
void Gps2xy::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& data)
{
    lat = data->latitude * M_PI/180.0;
    lon = data->longitude * M_PI/180.0;
    alt = data->altitude;
}


void Gps2xy::init_callback(const std_msgs::Bool::ConstPtr& data)
{
    // if (flag == true) then re-initialises the starting position as initial "latitude" and "longitude"
    if (data->data == true)
    {
        lat_ini = lat;
        lon_ini = lon;
        alt_ini = alt;

        // Update initial starting pos in ECEF frame and store those values in pos_ECEF_ini
        update_ecef_ini();
    }
}

void Gps2xy::ini_cond_callback(const std_msgs::Float32MultiArray::ConstPtr& ini_cond)
{
    lon_ini = ini_cond->data[0] * M_PI/180.0;
    lat_ini = ini_cond->data[1] * M_PI/180.0;
    alt_ini = ini_cond->data[2];
    update_ecef_ini();
}


double Gps2xy::pos_ENU_x() const
{ return pos_ENU[0]; }

double Gps2xy::pos_ENU_y() const
{ return pos_ENU[1]; }

double Gps2xy::pos_ENU_z() const
{ return pos_ENU[2]; }


