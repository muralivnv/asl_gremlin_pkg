/**
 * @brief SubscribeTopic header
 * @file SubscribeTopic.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright 2017.
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#ifndef _asl_gremlin_pkg_SUBSCRIBERTOPIC_H_
#define _asl_gremlin_pkg_SUBSCRIBERTOPIC_H_

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
    data_.reset(new T(*topic_data) ) ;
}

template<typename T>
T* SubscribeTopic<T>::get_data()
{ return data_.get(); }

} //end namespace {asl_gremlin_pkg}


#endif
