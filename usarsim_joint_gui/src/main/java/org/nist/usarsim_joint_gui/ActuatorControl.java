package org.nist.usarsim_joint_gui;
/**
 *
 * @author Stephen Balakirsky (modified Steve Nunally's code to fit ros)
 */

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import java.util.ArrayList;

public class ActuatorControl {
    private String name;
    private int numLinks;
    private ArrayList<Boolean> linksRevolute;
    private ArrayList<Double> linkMin;
    private ArrayList<Double> linkMax;
    private Publisher<control_msgs.FollowJointTrajectoryActionGoal> publisher;
    public ActuatorControl(String name, Node node)
    {
	System.out.println( "Actuator Name: " + name );
	publisher =
	    node.newPublisher((name + "_controller/follow_joint_trajectory/goal"), control_msgs.FollowJointTrajectoryActionGoal._TYPE);
	this.name = name;
	linksRevolute = new ArrayList<Boolean>();
	linkMin = new ArrayList<Double>();
	linkMax = new ArrayList<Double>();
	numLinks = 0;
    }
    //type is true for revolute joint, false for otherwise
    public void addLink(boolean type, double min, double max)
    {
	linksRevolute.add(Boolean.valueOf(type));
	linkMin.add(Double.valueOf(min));
	linkMax.add(Double.valueOf(max));
	numLinks++;
    }

    public Publisher<control_msgs.FollowJointTrajectoryActionGoal> getPublisher()
    {
	return publisher;
    }

    public int getLinkNum()
    {
	return numLinks;
    }
    public boolean getRev(int index)
    {
	return linksRevolute.get(index);
    }
    public double getMax(int index)
    {
	return linkMax.get(index);
    }
    public double getMin(int index)
    {
	return linkMin.get(index);
    }
    public String getName()
    {
	return this.name;
    }
}
