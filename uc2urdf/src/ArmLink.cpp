/*****************************************************************************
 DISCLAIMER:
 This software was produced by the National Institute of Standards
 and Technology (NIST), an agency of the U.S. government, and by statute is
 not subject to copyright in the United States.  Recipients of this software
 assume all responsibility associated with its operation, modification,
 maintenance, and subsequent redistribution.

 See NIST Administration Manual 4.09.07 b and Appendix I.
 *****************************************************************************/
#include "ArmLink.h"

ArmLink::ArmLink()
{
    //ctor
}

ArmLink::~ArmLink()
{
    //dtor
}


std::string ArmLink::getLinkName()
{
    return m_linkName;
}

void ArmLink::setLinkName(std::string name)
{
    m_linkName=name;
}

std::string ArmLink::getColladaPath()
{
    return m_colladaPath;
}

void ArmLink::setColladaPath(std::string path)
{
    m_colladaPath=path;
}

std::vector<double> ArmLink::getOffsetVector()
{
    return m_linkOffsetVector;
}


std::vector<double> ArmLink::getRPYVector()
{
    return m_linkRPYVector;
}
void ArmLink::setOffsetVector(double x, double y, double z)
{
    m_linkOffsetVector.clear();
    m_linkOffsetVector.push_back(x);
    m_linkOffsetVector.push_back(y);
    m_linkOffsetVector.push_back(z);
}

void ArmLink::setRPYVector(double roll, double pitch, double yaw)
{
    m_linkRPYVector.clear();
    m_linkRPYVector.push_back(roll);
    m_linkRPYVector.push_back(pitch);
    m_linkRPYVector.push_back(yaw);
}

void ArmLink::setOffsetVector(std::vector<double> myVector)
{
    m_linkOffsetVector=myVector;
}

std::map<std::string,std::vector<double> >  ArmLink::getOffsetMap()
{
    return m_linkOffsetMap;
}

void ArmLink::setOffsetMap()
{
    std::string name = getLinkName();
    std::vector<double> offset = getOffsetVector();
    m_linkOffsetMap.insert(std::pair<std::string,std::vector<double> >(name,offset));
}

void ArmLink::setOffsetMap(std::map<std::string, std::vector<double> > myMap)
{
    //linkOffsetMap(myMap);
    m_linkOffsetMap.swap(myMap);
    //linkOffsetMap=myMap;
}
