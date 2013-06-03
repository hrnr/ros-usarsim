#include "ArmJoint.h"

ArmJoint::ArmJoint(){
    //ctor
}

ArmJoint::~ArmJoint(){
    //dtor
}

void ArmJoint::setJointName(std::string name){
    m_jointName = name;
}

std::string ArmJoint::getJointName(){
    return m_jointName;
}

void ArmJoint::setJointParent(std::string name){
    m_jointParent = name;
}

std::string ArmJoint::getJointParent(){
    return m_jointParent;
}

void ArmJoint::setJointChild(std::string name){
    m_jointChild = name;
}

std::string ArmJoint::getJointChild(){
    return m_jointChild;
}

void ArmJoint::setJointDamping(double damping){
    m_damping= damping;
}

double ArmJoint::getJointDamping(){
    return m_damping;
}

void ArmJoint::setJointMaxForce(double maxforce){
    m_maxForce = maxforce;
}

double ArmJoint::getJointMaxForce(){
    return m_maxForce;
}

void ArmJoint::setJointLimitLow(double limitlow){
    m_limitLow = limitlow;
}


double ArmJoint::getJointLimitLow(){
    return m_limitLow;
}

void ArmJoint::setJointLimitHigh(double limithigh){
    m_limitHigh = limithigh;
}


double ArmJoint::getJointLimitHigh(){
    return m_limitHigh;
}

void ArmJoint::setJointOffsetVector(double x, double y, double z){
    m_jointOffsetVector.clear();
    m_jointOffsetVector.push_back(x);
    m_jointOffsetVector.push_back(y);
    m_jointOffsetVector.push_back(z);
}


std::vector<double> ArmJoint::getJointOffsetVector(){
    return m_jointOffsetVector;
}

void ArmJoint::setJointRPYVector(double r, double p, double y){
    m_jointRPYVector.clear();
    m_jointRPYVector.push_back(r);
    m_jointRPYVector.push_back(p);
    m_jointRPYVector.push_back(y);
}


std::vector<double> ArmJoint::getJointRPYVector(){
    return m_jointRPYVector;
}
