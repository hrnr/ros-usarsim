/*****************************************************************************
 DISCLAIMER:
 This software was produced by the National Institute of Standards
 and Technology (NIST), an agency of the U.S. government, and by statute is
 not subject to copyright in the United States.  Recipients of this software
 assume all responsibility associated with its operation, modification,
 maintenance, and subsequent redistribution.
 
 See NIST Administration Manual 4.09.07 b and Appendix I.
 *****************************************************************************/
/**
 * @file URDF.cpp
 * @brief This source file will contain all required definitions and basic utilities functions to read and create <b>urdf</b> files.
 *
 * @author Dr. Zeid Kootbally
 *
 * @date 05/20/2013
 */


#include "URDF.h"
#include "Config.h"
#include <stdio.h>
#include <iostream>
#include <ostream>
using namespace std;

URDF::URDF()
{
    //ctor
}

URDF::~URDF()
{
    //dtor
}


/**
 @brief Set in memory the value for the attribute \htmlonly <font color="red">name</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>link</font>\endhtmlonly.
 
 
 \htmlonly
 In the following XML piece, for the node <font color=9900FF>link</font>, the value for the attribute <font color="red">name</font> is "KR60Arm_link0".
 
 <BR><BR> <font color=9900FF> &lt;link </font> <font color="red">name</font> ="KR60Arm_link0"<font color=9900FF>&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;visual&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;geometry&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;box</font> <font color="red">size</font>= "0.80 0.05 0.05"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;/geometry&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;origin</font> <font color="red">xyz</font>="0.06 0.01 -0.40" <font color="red">rpy</font>="0.00 1.43 0.18"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;visual&gt;</font><BR>
 <font color=9900FF> &lt;link&gt;</font><BR>
 \endhtmlonly
 
 @param name Value for the attribute \htmlonly <font color="red">name</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>link</font>\endhtmlonly.
 
 */
void URDF::setLink_name(std::string name){
    m_link_name=name;
}


/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">name</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>link</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">name</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>link</font>\endhtmlonly.
 */
std::string URDF::getLink_name(){
    return m_link_name;
}



/**
 @brief Set in memory the value for the attribute \htmlonly <font color="red">size</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>box</font>\endhtmlonly.
 
 \htmlonly
 In the following XML piece, for the node <font color=9900FF>box</font>, the value for the attribute <font color="red">size</font> is "0.80 0.05 0.05".
 
 <BR><BR> <font color=9900FF> &lt;link </font> <font color="red">name</font> ="KR60Arm_link0"<font color=9900FF>&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;visual&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;geometry&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;box</font> <font color="red">size</font>= "0.80 0.05 0.05"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;/geometry&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;origin</font> <font color="red">xyz</font>="0.06 0.01 -0.40" <font color="red">rpy</font>="0.00 1.43 0.18"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;visual&gt;</font><BR>
 <font color=9900FF> &lt;link&gt;</font><BR>
 \endhtmlonly
 
 @param size Value for the attribute \htmlonly <font color="red">size</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>box</font>\endhtmlonly.
 */
void URDF::setLinkBox(std::string size){
    m_link_box_size=size;
}


/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">size</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>box</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">size</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>box</font>\endhtmlonly.
 */
std::string URDF::getLinkBox_size(){
    return m_link_box_size;
}

/**
 @brief Set in memory the value for the attributes \htmlonly <font color="red">xyz</font>\endhtmlonly and \htmlonly <font color="red">rpy</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 
 \htmlonly
 In the following XML piece, for the node <font color=9900FF>link</font>, the value for the attribute <font color="red">xyz</font> is "0.06 0.01 -0.40"
 and the value for the attribute <font color="red">rpy</font> is "0.00 1.43 0.18".
 
 <BR><BR> <font color=9900FF> &lt;link </font> <font color="red">name</font> ="KR60Arm_link0"<font color=9900FF>&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;visual&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;geometry&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;box</font> <font color="red">size</font>= "0.80 0.05 0.05"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;/geometry&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;origin</font> <font color="red">xyz</font>="0.06 0.01 -0.40" <font color="red">rpy</font>="0.00 1.43 0.18"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF> &lt;visual&gt;</font><BR>
 <font color=9900FF> &lt;link&gt;</font><BR>
 \endhtmlonly
 
 @param size Value for the attribute \htmlonly <font color="red">xyz</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 @param rpy Value for the attribute \htmlonly <font color="red">rpy</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 */
void URDF::setLinkOrigin(std::string xyz, std::string rpy){
    m_link_origin_xyz=xyz;
    m_link_origin_rpy=rpy;
}

/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">xyz</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">xyz</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 */
std::string URDF::getLinkOrigin_xyz(){
    return m_link_origin_xyz;
}

/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">rpy</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">rpy</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 */
std::string URDF::getLinkOrigin_rpy(){
    return m_link_origin_rpy;
}

/**
 @brief Set in memory the value for the attributes \htmlonly <font color="red">name</font>\endhtmlonly and \htmlonly <font color="red">type</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>joint</font>\endhtmlonly.
 
 \htmlonly
 In the following XML piece, for the node <font color=9900FF>joint</font>, the value for the attribute <font color="red">name</font> is "KR60Arm_link0"
 and the value for the attribute <font color="red">type</font> is "revolute".
 <BR><BR> <font color=9900FF> &lt;joint </font> <font color="red">name</font>="KR60Arm_joint_1" <font color="red">type</font>="revolute"<font color=9900FF>&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;parent</font>  <font color="red">link</font>="KR60Arm_link0"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;child</font>  <font color="red">link</font>  ="KR60Arm_link1"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;origin</font>  <font color="red">xyz</font>  ="0.11 0.02 -0.79" <font color="red">rpy</font>="3.14 -0.00 0.00"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;axis</font>  <font color="red">xyz</font>  ="0.0 0.0 1" <font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;limit</font>  <font color="red">effort</font>  ="600.00"
 <font color="red">lower</font>  ="-3.14"  <font color="red">upper</font> ="3.14" <font color="red">velocity</font>="1.0" <font color=9900FF>/&gt;</font><BR>
 <font color=9900FF>&lt;/joint&gt;</font>
 \endhtmlonly
 
 @param name Value for the attribute \htmlonly <font color="red">name</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>joint</font>\endhtmlonly.
 @param type Value for the attribute \htmlonly <font color="red">type</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>joint</font>\endhtmlonly.
 */
void URDF::setJoint(std::string name, std::string type){
    m_joint_name=name;
    m_joint_type=type;
}

/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">name</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>joint</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">name</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>joint</font>\endhtmlonly.
 */
std::string URDF::getJoint_name(){
    return m_joint_name;
}


/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">type</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>joint</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">type</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>joint</font>\endhtmlonly.
 */
std::string URDF::getJoint_type(){
    return m_joint_type;
}

/**
 @brief Set in memory the value for the attribute \htmlonly <font color="red">link</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>parent</font>\endhtmlonly.
 
 \htmlonly
 In the following XML piece, the value for the attribute <font color="red">link</font> is "KR60Arm_link0" for the node <font color=9900FF>parent</font>.
 <BR><BR> <font color=9900FF> &lt;joint </font> <font color="red">name</font>="KR60Arm_joint_1" <font color="red">type</font>="revolute"<font color=9900FF>&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;parent</font>  <font color="red">link</font>="KR60Arm_link0"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;child</font>  <font color="red">link</font>  ="KR60Arm_link1"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;origin</font>  <font color="red">xyz</font>  ="0.11 0.02 -0.79" <font color="red">rpy</font>="3.14 -0.00 0.00"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;axis</font>  <font color="red">xyz</font>  ="0.0 0.0 1" <font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;limit</font>  <font color="red">effort</font>  ="600.00"
 <font color="red">lower</font>  ="-3.14"  <font color="red">upper</font> ="3.14" <font color="red">velocity</font>="1.0" <font color=9900FF>/&gt;</font><BR>
 <font color=9900FF>&lt;/joint&gt;</font>
 \endhtmlonly
 
 @param link Value for the attribute \htmlonly <font color="red">link</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>parent</font>\endhtmlonly.
 */
void URDF::setJointParent(std::string link){
    m_joint_parent_link=link;
}

/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">link</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>parent</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">link</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>parent</font>\endhtmlonly.
 */
std::string URDF::getJointParent_link(){
    return m_joint_parent_link;
}

/**
 @brief Set in memory the value for the attribute \htmlonly <font color="red">link</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>child</font>\endhtmlonly.
 
 
 \htmlonly
 In the following XML piece, the value for the attribute <font color="red">link</font> is "KR60Arm_link1" for the node <font color=9900FF>child</font>.
 <BR><BR> <font color=9900FF> &lt;joint </font> <font color="red">name</font>="KR60Arm_joint_1" <font color="red">type</font>="revolute"<font color=9900FF>&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;parent</font>  <font color="red">link</font>="KR60Arm_link0"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;child</font>  <font color="red">link</font>  ="KR60Arm_link1"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;origin</font>  <font color="red">xyz</font>  ="0.11 0.02 -0.79" <font color="red">rpy</font>="3.14 -0.00 0.00"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;axis</font>  <font color="red">xyz</font>  ="0.0 0.0 1" <font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;limit</font>  <font color="red">effort</font>  ="600.00"
 <font color="red">lower</font>  ="-3.14"  <font color="red">upper</font> ="3.14" <font color="red">velocity</font>="1.0" <font color=9900FF>/&gt;</font><BR>
 <font color=9900FF>&lt;/joint&gt;</font>
 \endhtmlonly
 
 @param link Value for the attribute \htmlonly <font color="red">link</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>child</font>\endhtmlonly.
 */
void URDF::setJointChild(std::string link){
    m_joint_child_link=link;
}


/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">link</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>child</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">link</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>child</font>\endhtmlonly.
 */
std::string URDF::getJointChild_link(){
    return m_joint_child_link;
}


/**
 @brief Set in memory the values for the attributes \htmlonly <font color="red">xyz</font>\endhtmlonly and \htmlonly <font color="red">rpy</font>\endhtmlonly for the XML node
 \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 
 \htmlonly
 In the following XML piece, the value for the attribute <font color="red">xyz</font> is "0.11 0.02 -0.79"
 and the value for the attribute <font color="red">rpy</font> is "3.14 -0.00 0.00" for the node <font color=9900FF>origin</font>.
 <BR><BR> <font color=9900FF> &lt;joint </font> <font color="red">name</font>="KR60Arm_joint_1" <font color="red">type</font>="revolute"<font color=9900FF>&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;parent</font>  <font color="red">link</font>="KR60Arm_link0"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;child</font>  <font color="red">link</font>  ="KR60Arm_link1"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;origin</font>  <font color="red">xyz</font>  ="0.11 0.02 -0.79" <font color="red">rpy</font>="3.14 -0.00 0.00"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;axis</font>  <font color="red">xyz</font>  ="0.0 0.0 1" <font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;limit</font>  <font color="red">effort</font>  ="600.00"
 <font color="red">lower</font>  ="-3.14"  <font color="red">upper</font> ="3.14" <font color="red">velocity</font>="1.0" <font color=9900FF>/&gt;</font><BR>
 <font color=9900FF>&lt;/joint&gt;</font>
 \endhtmlonly
 
 @param xyz Value for the attribute \htmlonly <font color="red">xyz</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 @param rpy Value for the attribute \htmlonly <font color="red">rpy</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 */
void URDF::setJointOrigin(std::string xyz, std::string rpy){
    m_joint_origin_xyz=xyz;
    m_joint_origin_rpy=rpy;
}


/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">xyz</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">xyz</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 */
std::string URDF::getJointOrigin_xyz(){
    return m_joint_origin_xyz;
}

/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">rpy</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">rpy</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>origin</font>\endhtmlonly.
 */
std::string URDF::getJointOrigin_rpy(){
    return m_joint_origin_rpy;
}


/**
 @brief Set in memory the value for the attribute \htmlonly <font color="red">xyz</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>axis</font>\endhtmlonly.
 
 \htmlonly
 In the following XML piece, the value for the attribute <font color="red">xyz</font> is "0.0 0.0 1" for the node <font color=9900FF>axis</font>.
 
 <BR><BR> <font color=9900FF> &lt;joint </font> <font color="red">name</font>="KR60Arm_joint_1" <font color="red">type</font>="revolute"<font color=9900FF>&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;parent</font>  <font color="red">link</font>="KR60Arm_link0"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;child</font>  <font color="red">link</font>  ="KR60Arm_link1"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;origin</font>  <font color="red">xyz</font>  ="0.11 0.02 -0.79" <font color="red">rpy</font>="3.14 -0.00 0.00"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;axis</font>  <font color="red">xyz</font>  ="0.0 0.0 1" <font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;limit</font>  <font color="red">effort</font>  ="600.00"
 <font color="red">lower</font>  ="-3.14"  <font color="red">upper</font> ="3.14" <font color="red">velocity</font>="1.0" <font color=9900FF>/&gt;</font><BR>
 <font color=9900FF>&lt;/joint&gt;</font>
 \endhtmlonly
 
 @param xyz Value for the attribute \htmlonly <font color="red">xyz</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>axis</font>\endhtmlonly.
 */
void URDF::setJointAxis(std::string xyz){
    m_joint_axis_xyz=xyz;
}

/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">xyz</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>axis</font>\endhtmlonly.
 
 @return The stored value of the attribute \htmlonly <font color="red">xyz</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>axis</font>\endhtmlonly.
 */
std::string URDF::getJointAxis_xyz(){
    return m_joint_axis_xyz;
}


/**
 @brief Set in memory the values for the attributes \htmlonly <font color="red">effort</font>, <font color="red">lower</font>, <font color="red">upper</font>, and <font color="red">velocity</font>\endhtmlonly
 for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 
 \htmlonly
 In the following XML piece, for the node <font color=9900FF>limit</font>, the value for the attribute <font color="red">effort</font> is "600.00",
 the value for the attribute <font color="red">lower</font> is "-3.14", the value for the attribute <font color="red">upper</font> is "3.14",
 and the value the value for the attribute <font color="red">velocity</font> is is "1.0".
 
 <BR><BR> <font color=9900FF> &lt;joint </font> <font color="red">name</font>="KR60Arm_joint_1" <font color="red">type</font>="revolute"<font color=9900FF>&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;parent</font>  <font color="red">link</font>="KR60Arm_link0"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;child</font>  <font color="red">link</font>  ="KR60Arm_link1"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;origin</font>  <font color="red">xyz</font>  ="0.11 0.02 -0.79" <font color="red">rpy</font>="3.14 -0.00 0.00"<font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;axis</font>  <font color="red">xyz</font>  ="0.0 0.0 1" <font color=9900FF>/&gt;</font><BR>
 &nbsp;&nbsp;&nbsp;<font color=9900FF>&lt;limit</font>  <font color="red">effort</font>  ="600.00"
 <font color="red">lower</font>  ="-3.14"  <font color="red">upper</font> ="3.14" <font color="red">velocity</font>="1.0" <font color=9900FF>/&gt;</font><BR>
 <font color=9900FF>&lt;/joint&gt;</font>
 \endhtmlonly
 
 @param effort Value for the attribute \htmlonly <font color="red">effort</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 @param lower Value for the attribute \htmlonly <font color="red">lower</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 @param upper Value for the attribute \htmlonly <font color="red">upper</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 @param velocity Value for the attribute \htmlonly <font color="red">velocity</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 */
void URDF::setJointLimit(std::string effort, std::string lower, std::string upper, std::string velocity){
    m_joint_limit_effort=effort;
    m_joint_limit_lower=lower;
    m_joint_limit_upper=upper;
    m_joint_limit_velocity=velocity;
}

/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">effort</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 @return The stored value of the attribute \htmlonly <font color="red">effort</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 */
std::string URDF::getJointLimit_effort(){
    return m_joint_limit_effort;
}

/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">lower</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 @return The stored value of the attribute \htmlonly <font color="red">lower</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 */
std::string URDF::getJointLimit_lower(){
    return m_joint_limit_lower;
}

/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">upper</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 @return The stored value of the attribute \htmlonly <font color="red">upper</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 */
std::string URDF::getJointLimit_upper(){
    return m_joint_limit_upper;
}

/**
 @brief Return the stored value of the attribute \htmlonly <font color="red">velocity</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 @return The stored value of the attribute \htmlonly <font color="red">velocity</font>\endhtmlonly for the XML node \htmlonly <font color=9900FF>limit</font>\endhtmlonly.
 */
std::string URDF::getJointLimit_velocity(){
    return m_joint_limit_velocity;
}
