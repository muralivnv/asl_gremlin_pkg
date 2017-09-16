#include <state_feedback/GetCompassHdg.h>

void GetCompassHdg::hdg_callback(const std_msgs::Float64::ConstPtr data)
{
    compass_hdg_ = data->data;
}

double GetCompassHdg::data()
{
    return compass_hdg_;
}

double GetCompassHdg::data_ENU()
{
    return utility_pkg::compass_angle_to_polar_angle(compass_hdg_);
}
