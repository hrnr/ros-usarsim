
#ifndef URDF_H
#define URDF_H


/**
 * @file URDF.h
 * @brief This header file will contain all required definitions and basic utilities functions to read and create URDF files.
 *
 * @author Dr. Zeid Kootbally
 *
 * @date 05/20/2013
 */


#include <string>
#include "ArmLink.h"
#include "ArmJoint.h"
#include "XmlParser.h"


class URDF
{
public:
    URDF();
    virtual ~URDF();
    //-- URDF Link
    void setLink_name(std::string name);
    std::string getLink_name();
    void setLinkBox(std::string size);
    std::string getLinkBox_size();
    void setLinkOrigin(std::string xyz, std::string rpy);
    std::string getLinkOrigin_xyz();
    std::string getLinkOrigin_rpy();
    //-- URDF Joint
    void setJoint(std::string name, std::string type);
    std::string getJoint_name();
    std::string getJoint_type();
    void setJointParent(std::string link);
    std::string getJointParent_link();
    void setJointChild(std::string link);
    std::string getJointChild_link();
    void setJointOrigin(std::string xyz, std::string rpy);
    std::string getJointOrigin_xyz();
    std::string getJointOrigin_rpy();
    void setJointAxis(std::string xyz);
    std::string getJointAxis_xyz();
    void setJointLimit(std::string effort, std::string lower, std::string upper, std::string velocity);
    std::string getJointLimit_effort();
    std::string getJointLimit_lower();
    std::string getJointLimit_upper();
    std::string getJointLimit_velocity();
    
    
private:
    std::string m_link_name;
    std::string m_link_box_size;
    std::string m_link_origin_xyz;
    std::string m_link_origin_rpy;
    std::string m_joint_name;
    std::string m_joint_type;
    std::string m_joint_parent_link;
    std::string m_joint_child_link;
    std::string m_joint_origin_xyz;
    std::string m_joint_origin_rpy;
    std::string m_joint_axis_xyz;
    std::string m_joint_limit_effort;
    std::string m_joint_limit_lower;
    std::string m_joint_limit_upper;
    std::string m_joint_limit_velocity;
    
    std::vector<ArmLink*> m_armlink;
    std::vector<ArmJoint*> m_armjoint;
};

#endif // URDF_H
