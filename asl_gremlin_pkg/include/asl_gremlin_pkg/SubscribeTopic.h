#ifndef _asl_gremlin_pkg_SBSCRIBERTOPIC_H_
#define _asl_gremlin_pkg_SBSCRIBERTOPIC_H_

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <memory>

namespace asl_gremlin_pkg{

template<typename T>
class SubscribeTopic{
    std::unique_ptr<T> data_;
    ros::Subscriber topic_sub_;

    public:
        SubscribeTopic(ros::NodeHandle&, 
                        const std::string& ,int = 20);
        ~SubscribeTopic();

        void topic_callback(const typename T::ConstPtr& );
        T* get_data();
};

template<typename T>
SubscribeTopic<T>::SubscribeTopic(ros::NodeHandle& nh,
                                const std::string& topic_name,
                                int queue_size)
{
   topic_sub_ = nh.subscribe(   topic_name,
                                queue_size,
                                &SubscribeTopic<T>::topic_callback,
                                this);
   data_.reset(new T());
}


template<typename T>
SubscribeTopic<T>::~SubscribeTopic()
{ data_.reset(); }

template<typename T>
void SubscribeTopic<T>::topic_callback(const typename T::ConstPtr& topic_data)
{
    /* Can't use topic_data.get() as passed is const ptr 
     *  Don't ever use "CONST_CAST" to remove the pointer constness
    */
    data_.reset(new T(*topic_data) ) ;
}

template<typename T>
T* SubscribeTopic<T>::get_data()
{ return data_.get(); }

} //end namespace {asl_gremlin_pkg}


#endif
