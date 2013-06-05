/*****************************************************************************
 DISCLAIMER:
 This software was produced by the National Institute of Standards
 and Technology (NIST), an agency of the U.S. government, and by statute is
 not subject to copyright in the United States.  Recipients of this software
 assume all responsibility associated with its operation, modification,
 maintenance, and subsequent redistribution.

 See NIST Administration Manual 4.09.07 b and Appendix I.
 *****************************************************************************/

#include "ArmJoint.h"

/**
\brief Default Constructor
*/
ArmJoint::ArmJoint()
{
}

/**
\brief Default Destructor
*/
ArmJoint::~ArmJoint()
{
}

/**
\brief Set the joint name to a member parameter
\param _name Name of the joint to be set
*/
void ArmJoint::setJointName(std::string _name)
{
    m_jointName = _name;
}

/**
\brief Get the joint name
\return The name of the joint
*/
std::string ArmJoint::getJointName()
{
    return m_jointName;
}

/**
\brief Set the name of the joint's parent link
\param _name Name of the parent link
*/
void ArmJoint::setJointParent(std::string _name)
{
    m_jointParent = _name;
}

/**
\brief Get the link's name for the parent of the joint
\return The name of the parent link
*/
std::string ArmJoint::getJointParent()
{
    return m_jointParent;
}

/**
\brief Set the name of the joint's child link
\param _name Name of the child link
*/
void ArmJoint::setJointChild(std::string _name)
{
    m_jointChild = _name;
}

/**
\brief Get the link's name for the child of the joint
\return The name of the child link
*/
std::string ArmJoint::getJointChild()
{
    return m_jointChild;
}


/**
\brief Set the damping value for the joint
\param _damping damping The value for damping
*/
void ArmJoint::setJointDamping(double _damping)
{
    m_damping = _damping;
}

/**
\brief Get the damping value
\return The value for damping
*/
double ArmJoint::getJointDamping()
{
    return m_damping;
}

/**
\brief Set the maximum force value for the joint
\param _maxforce Maximum force value
*/
void ArmJoint::setJointMaxForce(double _maxforce)
{
    m_maxForce = _maxforce;
}

/**
\brief Get the joint maximum force
\return The value for the maximum force
*/
double ArmJoint::getJointMaxForce()
{
    return m_maxForce;
}

/**
\brief Set the lower bound for the joint limit
\param _limitlow Value for the lower bound limit
*/
void ArmJoint::setJointLimitLow(double _limitlow)
{
    m_limitLow = _limitlow;
}

/**
\brief Get the lower bound for the joint limit
\return The value for the lower bound of the joint limit
*/
double ArmJoint::getJointLimitLow()
{
    return m_limitLow;
}

/**
\brief Set the upper bound for the joint limit
\param _limithigh Value for the upper bound limit
*/
void ArmJoint::setJointLimitHigh(double _limithigh)
{
    m_limitHigh = _limithigh;
}

/**
\brief Get the higher bound for the joint limit
\return The value for the higher bound of the joint limit
*/
double ArmJoint::getJointLimitHigh()
{
    return m_limitHigh;
}

/**
\brief Set the values for the joint offset
\param _x Value for the x coodinate
\param _y Value for the y coodinate
\param _z Value for the z coodinate
*/
void ArmJoint::setJointOffsetVector(double _x, double _y, double _z)
{
    m_jointOffsetVector.clear();
    m_jointOffsetVector.push_back(_x);
    m_jointOffsetVector.push_back(_y);
    m_jointOffsetVector.push_back(_z);
}

/**
\brief Get the C++ vector for the joint offset
\return The C++ vector that contains the joint offset values
*/
std::vector<double> ArmJoint::getJointOffsetVector()
{
    return m_jointOffsetVector;
}

/**
\brief Set the values for the joint orientations
\param _roll Value for the roll
\param _pitch Value for the pitch
\param _yaw Value for the yaw
*/
void ArmJoint::setJointRPYVector(double _roll, double _pitch, double _yaw)
{
    m_jointRPYVector.clear();
    m_jointRPYVector.push_back(_roll);
    m_jointRPYVector.push_back(_pitch);
    m_jointRPYVector.push_back(_yaw);
}

/**
\brief Get the C++ vector for the joint rpy
\return The C++ vector that contains the joint rpy values
*/
std::vector<double> ArmJoint::getJointRPYVector()
{
    return m_jointRPYVector;
}
