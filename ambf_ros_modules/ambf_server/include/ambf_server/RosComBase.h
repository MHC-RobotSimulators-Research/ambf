//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2020, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   1.0$
*/
//==============================================================================

#ifndef ROSCOMBASE_H
#define ROSCOMBASE_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include "ambf_server/CmdWatchDog.h"

class Node{
  public:
    static ros::NodeHandle* getNodePtr(){
        if (s_initialized == false){
            int argc = 0;
            char **argv = 0;
            ros::init(argc, argv, "ambf_comm_node");
            s_nodePtr = new ros::NodeHandle;
            s_initialized = true;
            std::cerr << "INFO! INITIALIZING ROS NODE HANDLE\n";
        }
        s_nodeCounter++;
        return s_nodePtr;
    }

    static void destroyNode(){
        if (s_initialized){
            s_initialized = false;

            std::cerr << "INFO! TOTAL ACTIVE COMM INSTANCES: " << s_nodeCounter << std::endl;
            std::cerr << "INFO! WAITING FOR ALL COMM INSTANCES TO UNREGISTER ... \n";
            while(s_nodeCounter > 0){
                std::cerr << "\tINFO! REMAINING ACTIVE COMMs: " << s_nodeCounter << std::endl;
                usleep(10000);
            }

            std::cerr << "INFO! DESTROYING ROS NODE HANDLE\n";
            ros::shutdown();
            delete s_nodePtr;
        }
    }

    static bool isNodeActive(){
        return s_initialized;
    }

    static void unregisterInstance(){
        s_nodeCounter--;
    }

private:
    static bool s_initialized;
    static unsigned int s_nodeCounter;
    static ros::NodeHandle* s_nodePtr;
};

template <class T_state, class T_cmd>
class RosComBase{
public:
    RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
    ~RosComBase();
    virtual void init() = 0;
    virtual void run_publishers();
    virtual void cleanUp();
    virtual T_cmd get_command(){return m_Cmd;}

    int m_freq_min;
    int m_freq_max;

protected:
    ros::NodeHandle* nodePtr;
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
    boost::shared_ptr<CmdWatchDog> m_watchDogPtr;

    std::string m_namespace;
    std::string m_name;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

    tf::Transform m_trans;
    T_state m_State;
    T_cmd m_Cmd;
    T_cmd m_CmdPrev;

    boost::thread m_thread;
    ros::CallbackQueue m_custom_queue;

    virtual void reset_cmd() = 0;
};

template<class T_state, class T_cmd>
void RosComBase<T_state, T_cmd>::run_publishers(){
    while(Node::isNodeActive()){
        T_state stateCopy = m_State;
        m_pub.publish(stateCopy);
        m_custom_queue.callAvailable();
        if(m_watchDogPtr->is_wd_expired()){
            m_watchDogPtr->consolePrint(m_name);
            reset_cmd();
        }
        m_watchDogPtr->m_ratePtr->sleep();
    }
    Node::unregisterInstance();
}

template<class T_state, class T_cmd>
RosComBase<T_state, T_cmd>::~RosComBase(){
    std::cerr << "INFO! Thread ShutDown: " << m_name << std::endl;
}


#endif
