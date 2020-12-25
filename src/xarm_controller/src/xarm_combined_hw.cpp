/* Copyright 2020 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

#include "xarm_combined_hw.h"

namespace xarm_control
{
	bool XArmCombinedHW::need_reset()
	{
	  std::vector<hardware_interface::RobotHWSharedPtr>::iterator robot_hw;
      for (robot_hw = robot_hw_list_.begin(); robot_hw != robot_hw_list_.end(); ++robot_hw)
      {
      	xarm_control::XArmHW* xarm_hw_ptr = dynamic_cast<xarm_control::XArmHW *>((*robot_hw).get());
        if(xarm_hw_ptr->need_reset())
        	return true;
      }
	  return false;
	}
}