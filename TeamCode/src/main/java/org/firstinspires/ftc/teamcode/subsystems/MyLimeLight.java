package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MyLimeLight {
    private Limelight3A limelight;
    private int pipeline = 0;
    private double xLoc;
    private double yLoc;
    private double angle;

    public MyLimeLight(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void start(int input){
        pipeline = input;
        limelight.start();
    }

    public void stop(){
        limelight.stop();
    }

    public boolean update(){
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // Getting numbers from Python
            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                xLoc = pythonOutputs[1];
                yLoc = pythonOutputs[2];
                angle = pythonOutputs[3];
            }
            return true;
        }
        return false;
    }

    public double getxLoc(){
        return xLoc;
    }

    public double getyLoc(){
        return yLoc;
    }

    public double getAngle(){
        return angle;
    }




}
