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

/**
\brief Default Constructor
*/
ArmLink::ArmLink(){
}

/**
\brief Default Destructor
*/
ArmLink::~ArmLink(){
}

/**
\brief Get the name of the link
\return The _name of the link
*/
std::string ArmLink::getLinkName(){
    return m_linkName;
    }

void ArmLink::setLinkName(std::string _name)
{
    m_linkName=_name;
}

/**
\brief Get the collada path for the link
\return The path to a collada model
*/
std::string ArmLink::getColladaPath()
{
    return m_colladaPath;
}

/**
\brief Set the collada path for the link
\param _path The path to a collada model
*/
void ArmLink::setColladaPath(std::string _path)
{
    m_colladaPath=_path;
}

/**
\brief Get the C++ vector that contains the link offset
\return The offset vector for the link
*/
std::vector<double> ArmLink::getOffsetVector()
{
    return m_linkOffsetVector;
}

/**
\brief Get the C++ vector that contains the link orientation
\return The orientation vector for the link
*/
std::vector<double> ArmLink::getRPYVector()
{
    return m_linkRPYVector;
}

/**
\brief Set the offset for the link
\param _x Value for the x coodinate
\param _y Value for the y coodinate
\param _z Value for the z coodinate
*/
void ArmLink::setOffsetVector(double _x, double _y, double _z)
{
    m_linkOffsetVector.clear();
    m_linkOffsetVector.push_back(_x);
    m_linkOffsetVector.push_back(_y);
    m_linkOffsetVector.push_back(_z);
}
/**
\brief Set the values for the link orientations
\param _roll Value for the roll
\param _pitch Value for the pitch
\param _yaw Value for the yaw
*/
void ArmLink::setRPYVector(double _roll, double _pitch, double _yaw)
{
    m_linkRPYVector.clear();
    m_linkRPYVector.push_back(_roll);
    m_linkRPYVector.push_back(_pitch);
    m_linkRPYVector.push_back(_yaw);
}

/**
\brief Set a C++ vector of offset to another vector
\param _offset_v The vector to assign
*/
void ArmLink::setOffsetVector(std::vector<double> _offset_v)
{
    m_linkOffsetVector=_offset_v;
}

/**
\brief Get the offset C++ map for the link
\return The C++ map of offset for the link
*/
std::map<std::string,std::vector<double> >  ArmLink::getOffsetMap()
{
    return m_linkOffsetMap;
}

/**
\brief Set the values for a C++ map that consists of the name of the link and the associated offset
*/
void ArmLink::setOffsetMap()
{
    std::string name = getLinkName();
    std::vector<double> offset = getOffsetVector();
    m_linkOffsetMap.insert(std::pair<std::string,std::vector<double> >(name,offset));
}

/**
\brief Assign a map to another map
\param _map The C++ map to assign
*/
void ArmLink::setOffsetMap(std::map<std::string, std::vector<double> > _map)
{
    //linkOffsetMap(myMap);
    m_linkOffsetMap.swap(_map);
    //linkOffsetMap=myMap;
}
