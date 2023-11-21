/**
 * @file lib_knowrob_interface.hpp
 * @author Marco Masannek
 *
 * @brief Cpp interface class for the knowrob action interfaces
 *
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023 Marco Masannek
 *
 */

#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <knowrob/askoneAction.h>
#include <knowrob/askincrementalAction.h>
#include <knowrob/askallAction.h>
#include <knowrob/tellAction.h>


class IKnowrob
{
    public:
        IKnowrob();
        ~IKnowrob();

    private:
        std::string m_logger_prefix;
};

