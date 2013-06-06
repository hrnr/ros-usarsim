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
std::string uc="";

int main(int argc, char** argv)
{
    std::string sep = "\n---------------------------------------------------------------------------\n";
    if (argc<7 || argc>7)
    {
        std::cout << sep;
        std::cout << "                          ERROR -- Wrong Usage !!"<<sep;
        std::cout << "Right Usage: ./uc2urdf -input oldurdf.xml -uc file.uc -output newurdf.xml\n\n";
        std::cout << "oldurdf.xml: The \"name\" of the original urdf file built from tf data.\n";
        std::cout << "The program will automatically search for this file in ../../usarsim_inf/urdf\n\n";
        std::cout << "file.uc: The \"name\" of the uc file used to extract link and joint data.\n";
        std::cout << "The program will automatically search for this file in ../etc/\n\n";
        std::cout << "newurdf.xml: The \"name\" of the output urdf file.\n";
        std::cout << "The program will automatically save the new file in ../etc/"<<sep;

        std::cout << "                              -- EXIT --"<<sep;
        exit(0);

    }
    else if (argc==7)
    {
        //-- Making sure we typed "-input"
        if (argv[1] == std::string("-input"))
            input = argv[2];
        else
        {
            std::cout << sep;
            std::cout << "ERROR -- make sure you use the key \"-input\" for the input file !!"<<sep;
            exit(0);
        }

        //-- Making sure we typed "-output"
        if (argv[3] == std::string("-uc"))
            uc = argv[4];
        else
        {
            std::cout << sep;
            std::cout << "ERROR -- make sure you use the key \"-uc\" for the uc file !!"<<sep;
            exit(0);
        }

        //-- Making sure we typed "-uc" for the 4th argument
        if (argv[5] == std::string("-output"))
            output = argv[6];
        else
        {
            std::cout << sep;
            std::cout << "ERROR -- make sure you use the key \"-output\" for the input file !!"<<sep;
            exit(0);
        }

    }

    std::string input_path = "../../usarsim_inf/urdf/";
    std::string output_path = "../etc/";
    std::string uc_path = "../etc/";

    std::string urdf_file = input_path.append(input);
    std::string output_urdf_file = output_path.append(output);
    std::string uc_file = uc_path.append(uc);

    FileOperator *op = new FileOperator();
    URDF *urdf = new URDF();

    //-- Open the KR60Arm.uc file
    //string f_kr60Arm = KR60ARM_UC;
    //string f_oldURDF = KR60ARM_XML;
    op->readUcFile(uc_file);
    op->getIniData("LINK");
    op->getIniData("JOINT");
    op->getIniData("LINK_ORIENTATION");
    op->getIniData("COLLADA");
    op->addLink_base_link();
    op->addJoint_mount();
    //op->displayUCJoint();
    op->readUrdfFile(urdf_file);
    MatrixOperator *matrixOp = new MatrixOperator(op);
    matrixOp->computeLinkPosition();
    matrixOp->computeJointPosition();
    matrixOp->computeJointOrientation();



    //op->displayUrdfLink();
    //op->displayUrdfJoint();
    //op->displayUCLink();
    //op->displayLinkPosition();
    //op->displayUCJoint();
    //op->displayJointPosition();
    //op->displayJointAngles();

    std::string sep2 = "\n------------------------------------------------------------------------------------------------------\n";
    if (op->writeUrdfFile(output_urdf_file)==0)
    {
        std::cout << sep2;
        std::cout << "                          "<<output_urdf_file << " was successfully created"<<sep2;
    }
    else
    {

        std::cout << sep2;
        std::cout << "                          "<<"Error: "<<output_urdf_file << " was not successfully created"<<sep2;
    }

    delete op;
    delete urdf;
    delete matrixOp;

}
