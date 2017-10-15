#include <boost/shared_ptr.hpp>
#include <iostream>
#include <geometry_msgs/PoseWithCovariance.h>
#include <chrono>
#include <memory>

#define TIME_NOW std::chrono::system_clock::now()

int main()
{
    boost::shared_ptr<geometry_msgs::PoseWithCovariance> PosePtr, PosePtr2(new geometry_msgs::PoseWithCovariance());
    
    std::unique_ptr<geometry_msgs::PoseWithCovariance> PoseUPtr;
    auto start = TIME_NOW;
    auto end = TIME_NOW;
    
    double total_elapsed = 0.0;
    std::chrono::duration<double> elapsed = end - start;

    for (int i = 0; i < 1000; ++i)
    {
        start = TIME_NOW;
        
        PoseUPtr.reset(new geometry_msgs::PoseWithCovariance(*PosePtr2));
        end = TIME_NOW;
       
        elapsed = end - start;

        total_elapsed += elapsed.count();
    }

    std::cout << "elapsed: " << total_elapsed/1000;

}
