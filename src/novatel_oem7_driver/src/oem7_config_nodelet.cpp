////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "novatel_oem7_msgs/Oem7AbasciiCmd.h"

#include <algorithm>

namespace
{

   /**
    * @return true if the string has the specified prefix
    */
   bool isPrefix(const std::string& prefix, const std::string& str)
   {
      auto const diff_pos = std::mismatch(prefix.begin(), prefix.end(), str.begin());
      return diff_pos.first == prefix.end();
   }

}

namespace novatel_oem7_driver
{
  /**
   * Nodelet which configures Oem7 receiver.
   * Sends Oem7 commands using 'Oem7Cmd' service. The commands are obtained from Global parameters.
   *
   */
  class Oem7ConfigNodelet : public nodelet::Nodelet
  {
    ros::Timer serviceCbTimer_; /**< Timer used to execute main service callback. */
    ros::ServiceClient client_; /** Oem7Cmd service */

  public:

      /**
       * Initializes the Config nodelet: Connects to Oem7 Cmd service and launches the configuration.
       */
      void onInit()
      {
        NODELET_INFO_STREAM(getName() << ": Oem7ConfigNodelet v." << novatel_oem7_driver_VERSION << "; "
                                      << __DATE__ << " " << __TIME__);

        client_ = getNodeHandle().serviceClient<novatel_oem7_msgs::Oem7AbasciiCmd>("Oem7Cmd");

        serviceCbTimer_ = getNodeHandle().createTimer(ros::Duration(0.0), &Oem7ConfigNodelet::serviceLoopCb, this, true);
      }

      /**
       * Service loop, obtains and sends the configuration commands sequentially, waiting for a response from a previous command before sending the next one.
       */
      void serviceLoopCb(const ros::TimerEvent& event)
      {
        client_.waitForExistence();

        std::vector<std::string> receiver_init_commands;
        getNodeHandle().getParam("receiver_init_commands", receiver_init_commands);
        for(const auto& cmd : receiver_init_commands)
        {
          issueConfigCmd(cmd);
        }

        NODELET_INFO_STREAM("Oem7 extended initialization commands:");

        std::vector<std::string> receiver_ext_init_commands;
        getNodeHandle().getParam("receiver_ext_init_commands", receiver_ext_init_commands);
        for(const auto& cmd : receiver_ext_init_commands)
        {
          issueConfigCmd(cmd);
        }

        NODELET_INFO_STREAM("Oem7 configuration completed.");
        client_.shutdown();
      }

      /**
       * Executes Driver-specific command, like PAUSE.
       *
       * @return true when the provided command is a recongized internal command.
       */
      bool executeInternalCommand(const std::string& cmd)
      {
        static const std::string CMD_PAUSE("!PAUSE");
   	if(isPrefix(CMD_PAUSE, cmd))
        {
           std::stringstream ss(cmd);
           std::string token;
           ss >> token; // Prefix
           ss >> token; // Period
           int pause_period_sec = 0;
           if(std::stringstream(token) >> pause_period_sec)
           {
	      ROS_INFO_STREAM("Sleeping for " << pause_period_sec << " seconds....");
              ros::Duration(pause_period_sec).sleep();
              ROS_INFO_STREAM("... done sleeping.");
           }
           else
           {
              ROS_ERROR_STREAM("Invalid Driver command syntax: '" << cmd << "'");
           }

           return true;
        }
	else // Not a recognized internal command
        {
           return false;
        }
      }

      /**
       * Issues Oem7 configuration command
       */
      void issueConfigCmd(const std::string& cmd /**< The command to issue */)
      {
        if(!executeInternalCommand(cmd))
        {
           novatel_oem7_msgs::Oem7AbasciiCmd oem7_cmd;
           oem7_cmd.request.cmd = cmd;

           if(client_.call(oem7_cmd)) // BLOCKS with no timeout.
           {
              NODELET_DEBUG_STREAM("Config: '" <<  cmd << "' : Rsp: '" << oem7_cmd.response.rsp << "'");
           }
           else
           {
              NODELET_ERROR_STREAM("Config '" << cmd << "' not executed.");
           }
        }
      }
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ConfigNodelet, nodelet::Nodelet);
