package org.nist.usarsim_joint_gui;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author Stephen Balakirsky (added ROS to nunnally code)
 */
public class ArmDOFSlider extends DOFSlider {
    private int link;
    private int actuator;
    private Publisher<control_msgs.FollowJointTrajectoryActionGoal> myPublisher;

    //Holds name and dof info for passing up to movement command
    public ArmDOFSlider(int cNum, int lNum,Publisher<control_msgs.FollowJointTrajectoryActionGoal> publisher){
        super();
	myPublisher = publisher;
        actuator = cNum;
        link = lNum;
    }

    public Publisher<control_msgs.FollowJointTrajectoryActionGoal> getPublisher(){
	return myPublisher;
    }

    public int getLinkNum(){
        return link;
    }
    public int getActNum()
    {
    	return actuator;
    }
}
