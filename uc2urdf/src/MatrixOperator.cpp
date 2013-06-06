/*****************************************************************************
 DISCLAIMER:
 This software was produced by the National Institute of Standards
 and Technology (NIST), an agency of the U.S. government, and by statute is
 not subject to copyright in the United States.  Recipients of this software
 assume all responsibility associated with its operation, modification,
 maintenance, and subsequent redistribution.

 See NIST Administration Manual 4.09.07 b and Appendix I.
 *****************************************************************************/

#include "MatrixOperator.h"
#include <map>
#include <math.h>

/**
 \brief Constructor.

 Set the file operator @a op to MatrixOperator::m_file_operator.
 */
MatrixOperator::MatrixOperator(FileOperator *op)
{
    setFileOperator(op);
}

/**
 \brief Constructor.
 */
MatrixOperator::MatrixOperator()
{
}

/**
 \brief Destructor.
 */
MatrixOperator::~MatrixOperator()
{
}

/**
 \brief Build the rotation matrix about fixed frame.

 Rotations are applied in reverse order.<br>For instance, if we have first a rotation of \f$ \frac{\pi}{2} \f$ around the \f$z\f$ axis followed by a rotation of \f$ \frac{\pi}{2} \f$ around the \f$y\f$ axis, the rotation matrix \f$R\f$ is:
 \f[

 R=R_{z,\frac{\pi}{2}}R_{y,\frac{\pi}{2}}
 \f]

 \param _rpy_vector A <i>C++</i> vector that contains the rotation angles around the \f$x\f$, \f$y\f$, and \f$z\f$ axes.
 */
Matrix3d MatrixOperator::buildRotationMatrix(std::vector<double> _rpy_vector)
{
    if (_rpy_vector.size()==3)
    {
        Matrix3d rotationMatrix;

        rotationMatrix = AngleAxisd(_rpy_vector[2], Vector3d::UnitZ())
                         * AngleAxisd(_rpy_vector[1], Vector3d::UnitY())
                         * AngleAxisd(_rpy_vector[0], Vector3d::UnitX());

        return rotationMatrix;
    }
    else
    {
        cout << "Issues with buildRotationMatrix"<<endl;
        exit(0);
    }
}

/**
 \brief Build the rotation matrix between two vectors.

 The rotation matrix between two vectors can be found by:
 <ol>
 <li>Taking the dot product.
 <li>Taking the arc cosine of that value to compute the angle \f$\theta\f$.
 <li>Determining the cross product of the vectors: \f$u(a,b,c)\f$.
 <ul>
 <li> If \f$u\f$ is not a unit vector, we make it so by recasting this vector as a specific
 skew-symmetric matrix \f$N\f$ where:
 \f[
 N=\left[ {\begin{array}{ccc}
 0 &  c & -b \\
 -c & 0 & a \\
 b & -a & 0
 \end{array} } \right]
 \f]
 </ul>
 <li>Creating the matrix using the angle and cross vector.
 </ol>

 The formula to compute the rotation matrix \f$R\f$ is given by:
 \f[
 R = I + \sin{\theta} \times N + (1-\cos{\theta})\times N^2
 \f]
 Where \f$I\f$ is a \f$3\times 3\f$ identity matrix.
 \param _vector_source The vector source
 \param _vector_target The vector target
 \return The rotation matrix between \a _vector_source and \a _vector_target
 */
Matrix3d MatrixOperator::buildRotationMatrix(Vector3d _vector_source, Vector3d _vector_target)
{

    Matrix3d I = Matrix3d::Identity();
    if (_vector_source.size()==3)
    {
        if (_vector_target.size()==3)
        {

            double dot = _vector_source.dot(_vector_target);
            if (custom_isnan(dot))
            {
                double angle = (double)acos(dot);
                if (custom_isnan(angle))
                {
                    Vector3d cross = _vector_source.cross(_vector_target);
                    cross.normalize();

                    Matrix3d N;
                    N(0,0)=0;
                    N(0,1)=cross(2);
                    N(0,2)=-cross(1);

                    N(1,0)=-cross(2);
                    N(1,1)=0;
                    N(1,2)=cross(0);

                    N(2,0)=cross(1);
                    N(2,1)=-cross(0);
                    N(2,2)=0;

                    Matrix3d rotation;
                    rotation = I + sin(angle)*N+(1-cos(angle))*(N*N);
                    // Matrix3d rotation = aa(angle, cross);

                    return rotation;
                }
            }
        }
        else
        {
            cout << "Issues with buildRotationMatrix"<<endl;
            exit(0);
        }
    }

}

bool MatrixOperator::custom_isnan(double var)
{
    volatile double d = var;
    return d != d;
}

/**
 \brief Compute the position of each visual element with respect to the reference frame of the link \f$N\f$.

 \image html inertial.png From http://www.ros.org/wiki/urdf/XML/link

 We know:
 <ul>
 <li> The offset of the link \f$ l^0\f$ in the original reference frame \f$O\f$ (retrieved from the uc file).
 <li> The rotations around the \f$x\f$, \f$y\f$, and \f$z\f$ axes from \f$O\f$ to the new reference frame \f$N\f$ (retrieved from the uc file).
 <li> The offset of the joint \f$ j^0\f$ in the original reference frame \f$O\f$ (retrieved from the uc file).
 </ul>
 To compute the position of the link in the new reference frame (\f$ l^N\f$):
 \f[
 l^N = H^{-1} \dot l^0
 \f]
 Where \f$H\f$ is the homogeneous matrix defined as follows:

 \f[
 H = \left[ {\begin{array}{cc}
 R &  j^0 \\
 0 & 1
 \end{array} } \right]
 \f]
 \f$R\f$ is the rotation matrix \f$R=(R_{z,\theta}R_{y,\phi}R_{x,\psi})\f$
 */
void MatrixOperator::computeLinkPosition()
{
    std::vector<ArmJoint*> v_armjoint = m_file_operator->getArmJoint();
    int v_armjoint_size = (int)v_armjoint.size();
    std::vector<Vector3d> link_position_set;


    for (vector<ArmJoint*>::size_type i=1; i<v_armjoint_size; i++)
    {
        std::string child = v_armjoint[i]->getJointChild();

        std::vector<double> v_current_joint_offset = v_armjoint[i]->getJointOffsetVector();
        std::vector<double> v_tmpRPY = v_armjoint[i]->getJointRPYVector();

        //-- Compute the Rotation Matrix
        Matrix3d rotationMatrix = buildRotationMatrix(v_tmpRPY);

        //-- Build the Homogeneous Matrix
        Matrix4d homogeneousMatrix;
        homogeneousMatrix.block(0,0,3,3) << rotationMatrix;
        homogeneousMatrix.col(3) <<
        v_current_joint_offset[0],
        v_current_joint_offset[1],
        v_current_joint_offset[2],1;

        homogeneousMatrix.row(3)<< 0, 0, 0, 1;


        std::vector<ArmLink*> v_armlink = m_file_operator->getArmLink();
        int v_armlink_size = (int)v_armlink.size();

        for (vector<ArmLink*>::size_type j=0; j<v_armlink_size; j++)
        {
            std::vector<double> v_current_link_offset = v_armlink[j]->getOffsetVector();
            std::string s_link_name = v_armlink[j]->getLinkName();

            //-- If the name of the link = the name of the child link
            //- for this joint
            if (s_link_name.compare(child)==0)
            {
                Vector4d v_linkPosition(v_current_link_offset[0],
                                        v_current_link_offset[1],
                                        v_current_link_offset[2],
                                        1);

                Vector3d _position;

                Vector4d final_position;
                final_position=homogeneousMatrix.inverse()*v_linkPosition;


                double scale = 0.001;  // i.e. round to nearest one-hundreth
                _position[0]=floor(final_position[0] / scale + 0.5) * scale;
                _position[1]=floor(final_position[1] / scale + 0.5) * scale;
                _position[2]=floor(final_position[2] / scale + 0.5) * scale;

                //-- Update the current link positions in m_linkOffsetVector with the ones just computed
                v_armlink[j]->setOffsetVector((double)_position[0],(double)_position[1],(double)_position[2]);

                //-- Write this Vector3d in the vector link_position_set
                link_position_set.push_back(_position);
            }
        }
    }
    //-- Set the vector<Vector3d>
    m_file_operator->setLinkPosition(link_position_set);
}


void MatrixOperator::computeLinkOrientation() {}

/*!
 \brief Compute the position of each joint in the parent link reference frame \f$N\f$.
 This is the offset from the parent link to the child link. The joint is located at the origin of the child link, as shown in the figure below.<br>

 \image html joint.png From http://www.ros.org/wiki/urdf/XML/joint
 Since each joint is located at the origin of the child link, we have:
 <ul>
 <li> The offset of the parent link \f$ l^0\f$ in the original reference frame \f$O\f$ (retrieved from the uc file).
 <li> The offset of the joint \f$ j^0\f$ in the original reference frame \f$O\f$ (retrieved from the uc file).
 <li> The rotations around the \f$x\f$, \f$y\f$, and \f$z\f$ axes from \f$O\f$ to the parent link reference frame \f$N\f$ (retrieved from the uc file).
 </ul>
 To compute the position of the joint in the new reference frame:
 \f[
 j^N = H^{-1} \dot j^0
 \f]
 Where \f$H\f$ is the homogeneous matrix defined as follows:

 \f[
 H = \left[ {\begin{array}{cc}
 R &  l^0 \\
 0 & 1
 \end{array} } \right]
 \f]
 \f$R\f$ is the rotation matrix about fixed frame \f$R=(R_{z,\theta}R_{y,\phi}R_{x,\psi})\f$
 */

void MatrixOperator::computeJointPosition()
{
    std::vector<ArmJoint*> v_armjoint = m_file_operator->getArmJoint();

    int v_armjoint_size = (int)v_armjoint.size();
    std::vector<Vector3d> joint_position_set;
    map<int, std::vector<double> > map_tmpOffset;
    map<int, std::vector<double> > map_tmpRPY;

    //-- Store the offset of each v_armjoint in a map
    //-- This map will be used later to retrieve the previous offset
    for (vector<ArmJoint*>::size_type j=0; j<v_armjoint_size; j++){
        map_tmpOffset[j] = v_armjoint[j]->getJointOffsetVector();
        map_tmpRPY[j] = v_armjoint[j]->getJointRPYVector();
    }


    for (vector<ArmJoint*>::size_type i = 0; i < v_armjoint_size; i++)
    {
        //std::vector<double> v_current_tmpOffset = v_armjoint[i]->getJointOffsetVector();

        if (i>0)
        {
            //std::vector<double> v_tmpRPY = v_armjoint[i-1]->getJointRPYVector();

            Matrix3d rotationMatrix = buildRotationMatrix(map_tmpRPY[i-1]);

            //cout << "x y z: "<<map_tmpRPY[i-1][0] << " "<<map_tmpRPY[i-1][1]<<" "<<map_tmpRPY[i-1][2] << endl;
            //cout << "rotation matrix: \n"<<rotationMatrix<< "\n\n"<<endl;

            //-- Declaration of an empty homogeneous matrix
            Matrix4d homogeneousMatrix;

            //-- The first 3x3 block of the homogeneous matrix
            //-- starting at line 0 and column 0 is the rotation matrix
            homogeneousMatrix.block(0,0,3,3) << rotationMatrix;


            //-- The fourth column of the homogenous matrix is the displacement
            homogeneousMatrix.col(3) <<
            map_tmpOffset[i-1][0],
            map_tmpOffset[i-1][1],
            map_tmpOffset[i-1][2], 1;

            //-- The last row of the homogeneous matrix is 0 0 0 1
            homogeneousMatrix.row(3)<< 0, 0, 0, 1;

            Vector4d v_jointPosition(map_tmpOffset[i][0],map_tmpOffset[i][1],map_tmpOffset[i][2],1);
            Vector3d _position;

            Vector4d final_position = homogeneousMatrix.inverse() * v_jointPosition;

            double scale = 0.001;  // i.e. round to nearest one-hundreth
            _position[0]=floor(final_position[0] / scale + 0.5) * scale;
            _position[1]=floor(final_position[1] / scale + 0.5) * scale;
            _position[2]=floor(final_position[2] / scale + 0.5) * scale;

            //-- Update the offset for each joint with the ones just computed
            v_armjoint[i]->setJointOffsetVector((double)_position[0],(double)_position[1],(double)_position[2]);

            joint_position_set.push_back(_position);
        }
    }
    m_file_operator->setJointPosition(joint_position_set);
}


/**
 \brief Truncate a number by defining the number of decimals.

 \param value The value to truncate
 \param decimal The number of decimals in the result
 */
double MatrixOperator::truncate(double value, double decimal)
{
    double f = pow(10.0,decimal);
    return round(value*f)/f;
}

/**
 \brief Compute the joint angles for a joint relative to the reference frame of the parent link.

 In the .uc file, joint angles are relative to the orientation of the first link placed in the environment.<br>
 In ROS, joint angles are relative to the reference frame of its parent link.<br><br>

 In the Figure below Joint2 is offset in the \f$y\f$-direction from link1, a little offset in the negative \f$x\f$-direction from link1, and it is rotated 90 degrees around the \f$z\f$-axis.
 <br>Therefore, the joint angles for Joint2 is described as rpy="0 0 1.57".
 \image html link2.png
 \image latex link.png "Links and Joints" width=10cm

 To describe how this function computes joit angles for a joint relative to its parent link, let's go back to the picture depicted above and use Joint3 as an example.<br>
 Let:
 <ul>
 <li>\f$ R^3_1\f$ the rotation matrix from Link1 to Joint3: The rpy for this joint is given in the .uc file.
 <li>\f$ R^2_1\f$ the rotation matrix from Link1 to Joint2: The rpy for this joint is given in the .uc file.
 <li>\f$ R^3_2\f$ the rotation matrix from Joint2 to Joint3: This rotation matrix is computed using \f$ R^3_1\f$ and \f$ R^2_1\f$.
 </ul>
 We have: \f[ R^3_1 = R^2_1 R^3_2 \Rightarrow R^3_2 = (R^2_1)^{-1} R^3_1\f]

 Once \f$R^3_2\f$ is computed, the function MatrixOperator::getAngles(Matrix3d) is used to compute the joint angles for a joint in the new reference frame.


 */
void MatrixOperator::computeJointOrientation()
{
    std::vector<ArmJoint*> v_armjoint = m_file_operator->getArmJoint();
    int v_armjoint_size = (int)v_armjoint.size();
    std::vector<Vector3d> v_joint_angle;
    map<int, std::vector<double> > map_tmpRPY;
    Matrix3d r12, r13, r23;

    //-- Store the rpy of each v_armjoint in a map
    //-- Doing so allows us to retrieve original rpy data
    //-- and not the ones overwritten by this function
    for (vector<ArmJoint*>::size_type j=0; j<v_armjoint_size; j++)
    {

        map_tmpRPY[j] = v_armjoint[j]->getJointRPYVector();

        //map_tmpOffset[j]=v_tmpOffset;
    }

    for (vector<ArmJoint*>::size_type i = 0; i < v_armjoint_size; i++)
    {
        std::string jointname = v_armjoint[i]->getJointName();
        std::vector<double> current_joint_rpy = v_armjoint[i]->getJointRPYVector();


        //-- Starting from the third joint in the list
        if (i>1)
        {
            //-- Compute the rotation matrix from the original (first) joint to the previous joint: R^2_1
            std::string prev_jointname = v_armjoint[i-1]->getJointName();
            //std::vector<double> previous_joint_rpy = v_armjoint[i-1]->getJointRPYVector();

            /*
            Vector3d source;
            source(0)=map_tmpRPY[i-1][0];
            source(1)=map_tmpRPY[i-1][1];
            source(2)=map_tmpRPY[i-1][2];

            Vector3d target;
            target(0)=map_tmpRPY[i][0];
            target(1)=map_tmpRPY[i][1];
            target(2)=map_tmpRPY[i][2];

            Matrix3d rotationMatrix2 = buildRotationMatrix(source,target);
            cout << rotationMatrix2 <<"\n\n"<< endl;
            */


            /*
            r12 = AngleAxisd(previous_joint_rpy[2], Vector3d::UnitZ())
            * AngleAxisd(previous_joint_rpy[1], Vector3d::UnitY())
            * AngleAxisd(previous_joint_rpy[0], Vector3d::UnitX());
            */

            r12 = AngleAxisd(map_tmpRPY[i-1][2], Vector3d::UnitZ())
                  * AngleAxisd(map_tmpRPY[i-1][1], Vector3d::UnitY())
                  * AngleAxisd(map_tmpRPY[i-1][0], Vector3d::UnitX());

            //-- Compute the rotation matrix from the original (first) joint to the current joint: R^3_1
            r13 = AngleAxisd(map_tmpRPY[i][2], Vector3d::UnitZ())
                  * AngleAxisd(map_tmpRPY[i][1], Vector3d::UnitY())
                  * AngleAxisd(map_tmpRPY[i][0], Vector3d::UnitX());


            //-- Compute the rotation matrix from the previous joint to the current joint: R^3_2
            r23 = r12.inverse()*r13;

            Vector3d v = getAnglesFromRotationMatrix(r23);

            //-- Update joint rpy
            v_armjoint[i]->setJointRPYVector((double)v[0],(double)v[1],(double)v[2]);

            v_joint_angle.push_back(v);
        }
        //-- For the first 2 joints, the joint angles
        else
        {
            Vector3d v(current_joint_rpy[0], current_joint_rpy[1], current_joint_rpy[2]);
            v_joint_angle.push_back(v);
        }
    }
    //-- This is done only for the display on the screen
    m_file_operator->setJointAngle(v_joint_angle);
}

/**
 This function describes a simple technique to find all possible Euler angles from
 a rotation matrix.

 <h2>Rotation matrices</h2>
 We start off with the standard definition of the rotations about the three principle axes.<br>

 A rotation of \f$ \psi \f$ radians about the \f$ x\f$-axis is defined as

 \f[
 R_x(\psi) = {\left[
 \begin{array}{ccc}
 1 & 0 & 0\\
 0 &\cos{\psi} & -\sin{\psi}\\
 0 &\sin{\psi} &\cos{\psi}
 \end{array}\right]}
 \f]

 Similarly, a rotation of \f$ \theta \f$ radians about the \f$ y\f$-axis is defined as

 \f[
 R_y(\theta) = {\left[
 \begin{array}{ccc}
 \cos(\theta) & 0 & \sin{\theta} \\
 0 & 1 & 0 \\
 -\sin{\theta} & 0 & \cos{\theta}
 \end{array}
 \right]}
 \f]

 Finally, a rotation of \f$ \phi \f$ radians about the \f$ z\f$-axis is defined as

 \f[
 R_z(\phi) = {\left[
 \begin{array}{ccc}
 \cos(\phi) & -\sin{\phi} & 0 \\
 \sin{\phi} & \cos(\phi) & 0 \\
 0 & 0 & 1
 \end{array}
 \right]}
 \f]

 The angles \f$ \psi \f$, \f$ \theta \f$, and \f$ \phi \f$ are the Euler angles.

 <h2>Generalized Rotation Matrices</h2>

 A general rotation matrix can will have the form,
 \f[
 R = {\left[
 \begin{array}{ccc}
 R_{11} & R_{12} & R_{13} \\
 R_{21} & R_{22} & R_{23} \\
 R_{31} & R_{32} & R_{33}
 \end{array}\right]}
 \f]

 This matrix can be thought of a sequence of three rotations, one about each
 principle axis. Since matrix multiplication does not commute, the order of the
 axes which one rotates about will affect the result. For this analysis, we will
 rotate first about the \f$ x\f$-axis, then the \f$ y\f$-axis, and finally the \f$ z\f$-axis.

 <h2>C++ Pseudo Code to Compute Euler Angles from a Rotation Matrix</h2>
 The C++ pseudo code for this function is given in the Figure below:<br><br>
 \image html getAnglesFromRotationMatrix.png

 <h2>More than one Solution? </h2>
 It is interesting to note that there is always more than one sequence of rotations
 about the three principle axes that results in the same orientation of an object.
 As we have shown in the above explanation, in the non-degenerate case of cos θ = 0, there
 are two solutions. For the degenerate case of cos θ = 0, an infinite number of
 solutions exist.

 \note The rows and columns for the rotation matrix \f$R\f$ defined above start at index 1 while matrices in the Eigen library start at index 0. Using the elements described above for the
 rotation matrix \f$R\f$, we give the corresponding C++ elements for the Eigen matrix @a mat3d:
 <ul>
 <li> \f$R_{11}\f$=@a mat3d(0,0)
 <li> \f$R_{12}\f$=@a mat3d(0,1)
 <li> \f$R_{13}\f$=@a mat3d(0,2)
 <li> \f$R_{21}\f$=@a mat3d(1,0)
 <li> \f$R_{31}\f$=@a mat3d(2,0)
 <li> \f$R_{32}\f$=@a mat3d(2,1)
 <li> \f$R_{33}\f$=@a mat3d(2,2)
 </ul>

 \param mat3d A rotation matrix
 */
Vector3d MatrixOperator::getAnglesFromRotationMatrix(Matrix3d mat3d)
{

    //cout << "Rotation Matrix:\n\n"<<mat3d << endl;

    double theta_1, theta_2, psi_1, psi_2, phi_1, phi_2, theta, psi, phi;
    Vector3d rpy;

    double r31, r32, r33, r21, r11, r12, r13;
    r11= mat3d(0,0);
    r12= mat3d(0,1);
    r13= mat3d(0,2);
    r21= mat3d(1,0);
    r31= mat3d(2,0);
    r32= mat3d(2,1);
    r33= mat3d(2,2);

    int plus1 = 1;
    int minus1 = -1;

    //cout << "r31: "<<round(r31) << endl;

    if (round(r31)==1)
    {
        phi=0;
        theta=truncate(-0.5*M_PI,3);
        psi=truncate(-phi+atan2(-r12,-r13),3);

        rpy (0)=psi;
        rpy (1)=theta;
        rpy (2)=phi;

        //cout << "1 Solution: "<<endl;
        //cout << psi <<" "<<theta<<" "<<phi<<endl;
    }
    else if (round(r31)==-1)
    {
        phi=0;
        theta=truncate(0.5*M_PI,3);
        psi=truncate(phi+atan2(r12,r13),3);
        rpy (0)=psi;
        rpy (1)=theta;
        rpy (2)=phi;
        //cout << "1 Solution: "<<endl;
        //cout << psi <<" "<<theta<<" "<<phi<<endl;
    }
    else
    {
        //-- Compute theta angles
        theta_1 = truncate(-asin(r31),3);
        theta_2 = truncate(M_PI-theta_1,3);

        //-- Compute psi angles
        psi_1 = truncate(atan2(r32/cos(theta_1),r33/cos(theta_1)),3);
        psi_2 = truncate(atan2(r32/cos(theta_2),r33/cos(theta_2)),3);

        //--Compute phi angles
        phi_1 = truncate(atan2(r21/cos(theta_1),r11/cos(theta_1)),3);
        phi_2 = truncate(atan2(r21/cos(theta_2),r11/cos(theta_2)),3);

        //cout << "2 Solutions" << endl;
        //cout << "psi_1: " << psi_1 << ", psi_2: " << psi_2 << endl;
        //cout << "theta_1: " << theta_1 << ", theta_2: " << theta_2 << endl;
        //cout << "phi_1: " << phi_1 << ", phi_2: " << phi_2 << endl;
        rpy (0)=psi_1;
        rpy (1)=theta_1;
        rpy (2)=phi_1;

        //cout << "2 Solutions: "<<endl;
        //cout << psi_1 <<" "<<theta_1<<" "<<phi_1<<endl;
        //cout << psi_2 <<" "<<theta_2<<" "<<phi_2<<"\n"<<endl;
    }

    return rpy;
}


/*!
\brief Set \a op to MatrixOperator::m_file_operator.
\param op A FileOperator object.
*/
void MatrixOperator::setFileOperator(FileOperator *op)
{
    m_file_operator=op;
}

/*!
\brief Set \a op to MatrixOperator::m_file_operator.
\param op A FileOperator object.
*/
FileOperator* MatrixOperator::getFileOperator()
{
    return m_file_operator;
}
