/**
 * \file main.cpp
 * \brief
 * \author IAT Digital
 * \version 1.0
 * \date 2025-07-07
 *
 * \copyright Copyright (c) 2014-2025  阿尔特(北京)汽车数字科技有限公司
 *
 * Unpublished copyright. All rights reserved. This material contains
 * proprietary information that should be used or copied only within
 * IAT (Beijing) Automotive Digital Technology Co., Ltd., except with
 * written permission of IAT (Beijing) Automotive Digital Technology Co., Ltd.
 *
 * \par 修改日志:
 * <table>
 * <tr><th>Date           <th>Version    <th>Author            <th>Description
 * <tr><td>2025-07-07     <td>1.0        <td>IAT Digital       <td>Initialize create
 * </table>
 */

#include "ros/ros.h"                         
#include "application_impl.h"



int main(int argc, char* argv[]) {
    
    ros::init(argc, argv, "rov_planning");  
          
                            
    rov_planning::MyApplicationImpl application;
    application.Initialize();
   
    application.Run();
  
    application.Shutdown();                                         
   
   
    return 0;
}
