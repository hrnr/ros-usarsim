package org.nist.usarsim_joint_gui;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * BoneSliderPanel.java
 *
 * Created on Jun 28, 2010, 4:00:57 PM
 */

/**
 *
 * @author Stephen Balakirsky (added ROS to nunnally code)
 */
import java.awt.*;
import javax.swing.*;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

public class LinkSliderPanel extends javax.swing.JPanel {

    private AxisSliderPanel axes;
    private JLabel boneName;

    /** Creates new form BoneSliderPanel */
    //Set from MisPkgSliderPanel and holds an array of the AxisSliderPanels for control
    public LinkSliderPanel(String control, char axis, double min, double max, int percision, int controlNum, int linkNum,
			   Publisher<control_msgs.FollowJointTrajectoryActionGoal> publisher) {
        boneName = new JLabel();
        boneName.setFont(new Font("Tahoma", Font.BOLD|Font.ITALIC, 18));
        boneName.setText(control);

        //Sets the size to allow for appropriate sizing further up
        this.setPreferredSize(new Dimension(460+(percision*20)+20,40));
        this.setLayout(new GridLayout(1,1));
        this.add(boneName);

        axes = new AxisSliderPanel(axis, min, max, control, percision, controlNum, linkNum, publisher);
        this.add(axes);
    }

    public ArmDOFSlider getSliders(){
        ArmDOFSlider sliders = (ArmDOFSlider) axes.getSlider();

        return sliders;
    }

    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 400, Short.MAX_VALUE)
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 151, Short.MAX_VALUE)
        );
    }// </editor-fold>//GEN-END:initComponents


    // Variables declaration - do not modify//GEN-BEGIN:variables
    // End of variables declaration//GEN-END:variables

}
