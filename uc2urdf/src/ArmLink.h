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
 *  @brief     Robot Arm Link Data
 *  @details   This file consists of functions and procedures for representing arm links
 *  @author    <a href="http://www.nist.gov/el/isd/ks/kootbally.cfm">Zeid Kootbally</a> \a zeid.kootbally\@nist.gov
 *  @version   1.0
 *  @date      April 24, 2013
 */

#ifndef ARMLINK_H
#define ARMLINK_H

#include <string>
#include <vector>
#include <map>

class ArmLink
{
public:
    ArmLink();
    virtual ~ArmLink();
    void setLinkName(std::string);
    void setOffsetVector(double, double, double);
    void setRPYVector(double, double, double);
    void setOffsetVector(std::vector<double>);
    void setOffsetMap(std::map<std::string, std::vector<double> >);
    void setOffsetMap();
    void setColladaPath(std::string);
    std::string getLinkName();
    std::vector<double> getOffsetVector();
    std::vector<double> getRPYVector();
    std::map<std::string,std::vector<double> > getOffsetMap();
    std::string getColladaPath();

private:
    std::string m_linkName;
    std::string m_colladaPath;
    std::vector<double> m_linkOffsetVector;
    std::vector<double> m_linkRPYVector;
    std::map<std::string, std::vector<double> > m_linkOffsetMap;

};

#endif // ARMLINK_H
