package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class VerticalFlipper {
    private ServoImplEx left;
    private ServoImplEx right;
    private double min = -135;
    private double max = 135;
    public static double pickupPos = 85;
    public static double dropOffPos = -85;

    public VerticalFlipper(@NonNull HardwareMap hardwareMap){
        this.left = hardwareMap.get(ServoImplEx.class,"vflipleft");
        this.right = hardwareMap.get(ServoImplEx.class,"vflipright");
        this.left.setPwmRange(new PwmControl.PwmRange(500, 2500));
        this.right.setPwmRange(new PwmControl.PwmRange(500, 2500));
        this.right.setDirection(Servo.Direction.REVERSE);
    }

    public void goToPickUp(){
        setPosition(pickupPos);
    }

    public void goToDropOff(){
        setPosition(dropOffPos);
    }

    private double setPosition(double input){
        input = map(input,min,max,0,1);
        this.left.setPosition(input);
        this.right.setPosition(input);
        return input;
    }

    private double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
