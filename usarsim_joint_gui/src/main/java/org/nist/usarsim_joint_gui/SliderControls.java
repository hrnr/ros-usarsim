package org.nist.usarsim_joint_gui;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author Stephen Balakirsky (modified Steve Nunally's code to fit ros)
 */
import javax.swing.*;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

// ros imports
import org.apache.commons.logging.Log;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
//import org.ros.ParameterTree;

public class SliderControls implements NodeMain{
    private InputPanel panel;
    private SliderPanel slidePanel;
    private JFrame frame;
    private String configFile="src/main/java/org/nist/usarsim_joint_gui/SliderController.ini";

    //Declaration of Config Variables
    private int port;
    private String host;
    private int precision;

    private Log log;
    
    @Override
	public GraphName getDefaultNodeName() {
	return new GraphName("usarsim_joint_gui/SliderControls");
    }

    @Override
	public void onStart(Node node){
	//	ParameterTree params = node.newParameterTree();
	log = node.getLog();
        boolean changeConfig = false;
        String file= "";//params.getString("/usarsim_joint_gui/iniFile");

	//	node.getLog.error("Filename: ");
        frame = new JFrame("Slider Controls");
        frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
        frame.addWindowListener(new ClosingOperation());
        panel = new InputPanel();
        frame.getContentPane().add(panel);
        panel.getButton().addActionListener(new SendPressed());
        frame.pack();
        frame.show();

	//        new SliderControls(changeConfig, file);

    }

    @Override
	public void onShutdown(Node node) {
    }
    
    @Override
	public void onShutdownComplete(Node node) {
    }

    private class ClosingOperation implements WindowListener{
        public void windowActivated(WindowEvent e){}
        public void windowClosed(WindowEvent e){
            if (slidePanel != null)
                slidePanel.closeSocket();
            System.exit(0);
        }
        public void windowClosing(WindowEvent e){
            if (slidePanel != null)
                slidePanel.closeSocket();
            System.exit(0);
        }
        public void windowDeactivated(WindowEvent e){}
        public void windowDeiconified(WindowEvent e){}
        public void windowIconified(WindowEvent e){}
        public void windowOpened(WindowEvent e){}
    }

    private class SendPressed implements ActionListener{
        public void actionPerformed(ActionEvent e){
            // Set the variables from the configuration file
            configure();
            slidePanel = new ActuatorSliderPanel(panel.getClassName(),panel.getLocX(),panel.getLocY(),panel.getLocZ(),panel.getRotR(),panel.getRotP(),panel.getRotW(),precision,host,port);
            frame.getContentPane().removeAll();
            frame.getContentPane().add(slidePanel);
            frame.pack();
            frame.show();
        }
    }

    // Method to set all of the configuration variables based on the SoundServer.ini file
    private void configure(){
        try{
            Scanner config = new Scanner(new File(configFile));

            while(config.hasNextLine()){
                setVariable(config.next(),config);
            }
        }
        catch(FileNotFoundException exp){
            System.out.println(configFile + " does not exist");
            System.exit(1);
        }
        catch(InputMismatchException exp){
            System.out.println("Config file is not formatted correctly.\nCheck README file to see what should be a float, int, and a string");
            System.exit(1);
        }
        catch(Exception exp){
            System.out.println("Check configuration file: " + exp);
            System.exit(1);
        }
    }

    // Var should be one of the configuration variables, then sets it based
    //    on the config file
    private void setVariable(String var, Scanner config){
        config.next();

        if(var.equals("Precision")){
            precision=config.nextInt();
        }else if(var.equals("Host")){
            host = config.next();
        }else if(var.equals("Port")){
            port = config.nextInt();
        }else{
            System.out.println("Unknown variable = " + var);
            config.nextLine();
        }
    }

    /*
    public SliderControls(boolean changeConfig, String file){
        if(changeConfig){
            configFile = file;
        }
    }
    */
}
