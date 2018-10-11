/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file idl.thrift
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

/**
* skeletonGenerator_IDL
*
* IDL Interface to skeletonGenerator services.
*/
service skeletonGenerator_IDL
{
    /**
    * Generate skeleton.
    * @param motion_type name of the motion to reproduce
    * @return true/false on success/failure.
    */
    bool generateSkeleton(1:string motion_type);

    /**
    * Start moving.
    * @return true/false on success/failure.
    */
    bool start();

    /**
    * Stop moving.
    * @return true/false on success/failure.
    */
    bool stop();

}
