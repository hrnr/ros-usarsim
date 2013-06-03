/*****************************************************************************
 DISCLAIMER:
 This software was produced by the National Institute of Standards
 and Technology (NIST), an agency of the U.S. government, and by statute is
 not subject to copyright in the United States.  Recipients of this software
 assume all responsibility associated with its operation, modification,
 maintenance, and subsequent redistribution.

 See NIST Administration Manual 4.09.07 b and Appendix I.
 *****************************************************************************/
#include <iostream>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "FileOperator.h"
#include "MatrixOperator.h"
#include "ArmLink.h"
#include "URDF.h"

using Eigen::MatrixXd;
using namespace std;
using namespace Eigen;

std::string input="";
std::string output="";

int main(int argc, char** argv)
{
    std::string sep = "\n---------------------------------------------------------------------------\n";
    if (argc<5 || argc>5) {
        std::cout << sep;
        std::cout << "                          ERROR -- Wrong Usage !!"<<sep;
        std::cout << "Right Usage: ./uc2urdf -input oldurdf.xml -output newurdf.xml\n\n";
        std::cout << "oldurdf.xml: The \"name\" of the original urdf file built from tf data.\n";
        std::cout << "The program will automatically search for this file in ../usarsim_inf/urdf\n\n";
        std::cout << "newurdf.xml: The \"name\" of the output urdf file.\n";
        std::cout << "The program will automatically save the new file in ../etc/"<<sep;

        std::cout << "                              -- EXIT --"<<sep;
        exit(0);

    }
    else if (argc==5){
        input = argv[2];
        output = argv[4];
    }

    std::string input_path = "../etc/";
    std::string output_path = "../etc/";

    std::string urdf_file = input_path.append(input);
    std::string output_urdf_file = output_path.append(output);

    FileOperator *op = new FileOperator();
    URDF *urdf = new URDF();

    //-- Open the KR60Arm.uc file
    string f_kr60Arm = KR60ARM_UC;
    string f_oldURDF = KR60ARM_XML;
    op->readUcFile(f_kr60Arm);
    op->getIniData("COLLADA");
    op->getIniData("LINK");
    op->getIniData("JOINT");
    op->getIniData("LINK_ORIENTATION");
    op->addLink_base_link();
    op->addJoint_mount();
    op->readUrdfFile(urdf_file);
    MatrixOperator *matrixOp = new MatrixOperator(op);
    matrixOp->computeJointPosition();
    matrixOp->computeLinkPosition();
    matrixOp->computeJointAngles();

    //op->displayUrdfLink();
    //op->displayUrdfJoint();
    //op->displayUCLink();
    //op->displayLinkPosition();
    //op->displayUCJoint();
    //op->displayJointPosition();
    //op->displayJointAngles();

std::string sep2 = "\n------------------------------------------------------------------------------------------------------\n";
    if (op->writeUrdfFile(output_urdf_file)==0){
        std::cout << sep2;
        std::cout << "                          "<<output_urdf_file << " was successfully created"<<sep2;
    }
    else{

        std::cout << sep2;
        std::cout << "                          "<<"Error: "<<output_urdf_file << " was not successfully created"<<sep2;
    }

    delete op;
    delete urdf;
    delete matrixOp;

}
