/*****************************************************************************
 DISCLAIMER:
 This software was produced by the National Institute of Standards
 and Technology (NIST), an agency of the U.S. government, and by statute is
 not subject to copyright in the United States.  Recipients of this software
 assume all responsibility associated with its operation, modification,
 maintenance, and subsequent redistribution.

 See NIST Administration Manual 4.09.07 b and Appendix I.
 *****************************************************************************/

/*!
*  @brief     Robot Arm Joint Data
*  @details   This file consists of functions and procedures for representing arm joints
*  @author    <a href="http://www.nist.gov/el/isd/ks/kootbally.cfm">Zeid Kootbally</a> \a zeid.kootbally\@nist.gov
*  @version   1.0
*  @date      April 24, 2013
*/

#ifndef ARMJOINT_H
#define ARMJOINT_H

#include "ArmLink.h"

class ArmJoint
{
public:
    /** Default constructor */
    ArmJoint();
    /** Default destructor */
    virtual ~ArmJoint();

    //-- G --//
    double getJointDamping();
    std::string getJointChild();
    std::string getJointName();
    std::vector<double> getJointOffsetVector();
    std::string getJointParent();
    std::vector<double> getJointRPYVector();
    double getJointLimitHigh();
    double getJointLimitLow();
    double getJointMaxForce();

    //-- S --//
    void setJointChild(ArmLink);
    void setJointChild(std::string);
    void setJointDamping(double);
    void setJointLimitHigh(double);
    void setJointLimitLow(double);
    void setJointMaxForce(double);
    void setJointName(std::string);
    void setJointOffsetVector(double, double, double);
    void setJointParent(ArmLink);
    void setJointParent(std::string);
    void setJointRPYVector(double, double, double);

private:
    ArmLink *m_armLinkParent;
    ArmLink *m_armLinkChild;
    double m_damping;
    std::string m_jointChild;
    std::vector<double> m_jointOffsetVector;
    std::string m_jointName;
    std::string m_jointParent;
    std::vector<double> m_jointRPYVector;
    double m_limitHigh;
    double m_limitLow;
    double m_maxForce;
};

#endif // ARMJOINT_H
