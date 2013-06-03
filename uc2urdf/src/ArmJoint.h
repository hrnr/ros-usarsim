/*****************************************************************************
 DISCLAIMER:
 This software was produced by the National Institute of Standards
 and Technology (NIST), an agency of the U.S. government, and by statute is
 not subject to copyright in the United States.  Recipients of this software
 assume all responsibility associated with its operation, modification,
 maintenance, and subsequent redistribution.
 
 See NIST Administration Manual 4.09.07 b and Appendix I.
 *****************************************************************************/
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
    void setJointName(std::string name);
    std::string getJointName();
    void setJointParent(ArmLink armlink);
    void setJointParent(std::string name);
    std::string getJointParent();
    void setJointChild(ArmLink armlink);
    void setJointChild(std::string name);
    std::string getJointChild();
    void setJointDamping(double damping);
    double getJointDamping();
    void setJointMaxForce(double maxforce);
    double getJointMaxForce();
    void setJointLimitLow(double limitlow);
    double getJointLimitLow();
    void setJointLimitHigh(double limithigh);
    double getJointLimitHigh();
    void setJointOffsetVector(double x, double y, double z);
    std::vector<double> getJointOffsetVector();
    void setJointRPYVector(double r, double p, double y);
    std::vector<double> getJointRPYVector();
    
private:
    std::string m_jointName;
    std::string m_jointParent;
    std::string m_jointChild;
    ArmLink *m_armLinkParent;
    ArmLink *m_armLinkChild;
    double m_damping;
    double m_maxForce;
    double m_limitLow;
    double m_limitHigh;
    std::vector<double> m_jointOffsetVector;
    std::vector<double> m_jointRPYVector;
};

#endif // ARMJOINT_H
