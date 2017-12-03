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

template<typename MsgType>
class SubscribeTopic{
    std::unique_ptr<MsgType> data_;
    ros::Subscriber topic_sub_;

    public:
        SubscribeTopic(ros::NodeHandle&, 
                        const std::string& ,int = 20);
        ~SubscribeTopic();

        void topic_callback(const typename MsgType::ConstPtr& );
        MsgType* get_data();
};

template<typename MsgType>
SubscribeTopic<MsgType>::SubscribeTopic(ros::NodeHandle& nh,
                                const std::string& topic_name,
                                int queue_size)
{
   topic_sub_ = nh.subscribe(   topic_name,
                                queue_size,
                                &SubscribeTopic<MsgType>::topic_callback,
                                this);
   data_.reset(new MsgType());
}


template<typename MsgType>
SubscribeTopic<MsgType>::~SubscribeTopic()
{ data_.reset(); }

template<typename MsgType>
void SubscribeTopic<MsgType>::topic_callback(const typename MsgType::ConstPtr& topic_data)
{
    data_.reset(new MsgType(*topic_data) ) ;
}

template<typename MsgType>
inline MsgType* SubscribeTopic<MsgType>::get_data()
{ return data_.get(); }

} //end namespace {asl_gremlin_pkg}


#endif
