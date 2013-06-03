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
    void setLinkName(std::string name);
    void setOffsetVector(double x, double y, double z);
    void setRPYVector(double, double, double);
    void setOffsetVector(std::vector<double> myVector);
    void setOffsetMap(std::map<std::string, std::vector<double> > myMap);
    void setOffsetMap();
    void setColladaPath(std::string name);
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
