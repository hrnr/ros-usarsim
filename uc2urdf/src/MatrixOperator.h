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
 *  \brief     Class that consists of functions for matrix operations
 *  \details   This class consists of functions for computing links and joints positions and orientations using matrix transformations
 *  \author    <a href="http://www.nist.gov/el/isd/ks/kootbally.cfm">Zeid Kootbally</a> \a zeid.kootbally\@nist.gov
 *  \version   1.0
 *  \date      May 17, 2013
 */

#ifndef MATRIXOPERATOR_H
#define MATRIXOPERATOR_H

#include "FileOperator.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

using Eigen::MatrixXd;
using namespace Eigen;

class MatrixOperator
{
public:
    MatrixOperator(FileOperator *op);
    MatrixOperator();
    virtual ~MatrixOperator();

    Matrix3d buildRotationMatrix(std::vector<double>);
    void computeLinkOrientation();
    void computeLinkPosition();
    void computeJointOrientation();
    void computeJointPosition();
    Vector3d getAnglesFromRotationMatrix(Matrix3d);
    FileOperator* getFileOperator();
    void setFileOperator(FileOperator *);
    double truncate(double, double);








private:
    FileOperator *m_file_operator;
    std::vector<Vector3d> m_joint_position;
    std::vector<Vector3d> m_link_position;
    std::vector<Vector3d> m_joint_angle;
};

#endif // MATRIXOPERATOR_H
