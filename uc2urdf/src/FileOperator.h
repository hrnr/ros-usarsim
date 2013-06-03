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
 *  \brief     Class for operations on files
 *  \details   This class is used to manipulate files (open, substract, concat, etc).
 *  \author    <a href="http://www.nist.gov/el/isd/ks/kootbally.cfm">Zeid Kootbally</a> \a zeid.kootbally\@nist.gov
 *  \version   1.0
 *  \date      May 17, 2012
 */

#ifndef FILEOPERATOR_H_
#define FILEOPERATOR_H_

#include "Config.h"
#include "Tools.h"
#include "ArmLink.h"
#include "ArmJoint.h"
#include "URDF.h"
#include "XmlParser.h"
#include <vector>
#include <map>
#include <functional>
#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "Eigen/Geometry"

using Eigen::MatrixXd;
using namespace Eigen;

class FileOperator
{
public:
    FileOperator();
    virtual ~FileOperator();

    //-- A
    void addArmLinkToList(ArmLink*);
    void addArmLinkAtPosition(ArmLink *, int);
    void addArmJointAtPosition(ArmJoint *, int);
    void addLink_base_link();
    void addJoint_mount();
    //-- C
    void changeDotNegativeDecimal (std::string &,std::string);
    void changeDotPositiveDecimal (std::string &,std::string);
    bool contains_word(const std::string&, const std::string&);
    //-- D
    void displayJointAngles();
    void displayJointPosition();
    void displayLinkPosition();
    void displayUCJoint();
    void displayUCLink();
    void displayUrdfJoint();
    void displayUrdfLink();
    //-- G
    std::vector<ArmJoint*> getArmJoint();
    std::vector<ArmLink*> getArmLink();
    void getIniData(std::string);
    std::vector<Vector3d> getJointAngle();
    std::vector<Vector3d> getJointPosition();
    std::vector<Vector3d> getLinkPosition();
    std::vector<URDF*> getUrdfJointList();
    std::vector<URDF*> getUrdfLinkList();
    //-- I
    bool isDotPositiveDecimal(std::string);
    bool isDotNegativeDecimal(std::string);
    //-- R
    void readVector(vector<std::string>);
    void readVector(vector<double>);
    void readMapOfVector(std::map<std::string,std::vector<std::string> >);
    void readMapOfVector(std::map<std::string,std::vector<double> >);
    void readMap(std::map<std::string,std::string>);
    void readUcFile(std::string);
    void readUCJoint(std::string);
    void readUCLink(std::string);
    void readUrdfFile(std::string _input_file);
    void readUrdfJoint(XMLNode);
    void readUrdfLink(XMLNode);
    void readVectorOfVector(std::vector<std::vector<std::string> >);
    void removeDuplicates(std::vector<std::string>&);
    std::string removeParentheses(std::string);
    //-- S
    void setArmJoint(ArmJoint*);
    void setJointAngle(std::vector<Vector3d>);
    void setJointPosition(std::vector<Vector3d>);
    void setLinkPosition(std::vector<Vector3d>);
    std::string stringify(double);
    void setUrdfJointList(std::vector<URDF*>);
    void setUrdfLinkList(std::vector<URDF*>);
    std::vector<std::string> splitString(std::string);
    std::string stripSpace(std::string);
    //-- W
    int writeUrdfFile(std::string);

private:
    std::vector<ArmJoint*> m_armjoint;
    std::vector<ArmLink*> m_armlink;
    /*!
     \brief Joint data read from a urdf file is stored in this list.
     */
    std::vector<URDF*> m_input_urdfjoint_list;
    /*!
     \brief Link data read from a urdf file is stored in this list.
     */
    std::vector<URDF*> m_input_urdflink_list;

    std::vector<Vector3d> m_joint_angle;
    std::vector<Vector3d> m_joint_position;
    std::vector<Vector3d> m_link_position;
};

#endif /* FILEOPERATOR_H_ */
