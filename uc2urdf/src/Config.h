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
 *  @brief     Configuration file
 *  @details   The Configuration file contains paths of files used by the project.
 *  @author    <a href="http://www.nist.gov/el/isd/ks/kootbally.cfm">Zeid Kootbally</a> \a zeid.kootbally\@nist.gov
 *  @version   1.0
 *  @date      April 24, 2013
 */


#ifndef CONFIG_H_
#define CONFIG_H_



/*!
 @def KR60ARM_UC
 @brief File imported from USARSim that consists of all links and joints information
 */
//#define KR60ARM_UC "../etc/kr60Arm.uc"

/*!
 @def COLLADA_INI
 @brief File that contains the path to collada files for each link
 */
#define CONFIG_INI "../etc/config.ini"

/*!
 @def KR60ARM_XML
 @brief URDF file generated for the KR60
 */
//#define KR60ARM_XML "../etc/KR60.xml"


#endif /* CONFIG_H_ */




