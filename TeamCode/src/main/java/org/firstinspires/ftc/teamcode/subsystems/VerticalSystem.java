package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class VerticalSystem {
    //@Config variables
    public static int Height_clip = 475;
    public static int Height_clipClip = 350;
    public static int Height_transfer = 30;
    public static int Height_prepToLift = 0;
    public static int Height_basket = 900;
    public static int Height_sample = 0;
    public static int Height_stow = 300;
    public static int Height_flipUp = 600;
    public static int Height_flipDown = 575;
    public static int gripPause = 1000;
    static int flipperState = -1; //0 means facing forward, 1 means facing back
    static int flipperTarget = -1; //same data as above
    static int gripperState = -1; //0 means closed, 1 means open
    static int gripperTarget = -1;
    static ElapsedTime runtime = new ElapsedTime();


    static VerticalSliders verticalSliders;
    static VerticalFlipper verticalFlipper;
    static VerticalGripper verticalGripper;

    Telemetry dashboardTelemetry;


    public VerticalSystem (@NonNull HardwareMap hardwareMap, Telemetry db) {
        verticalSliders = new VerticalSliders(hardwareMap);
        verticalFlipper = new VerticalFlipper(hardwareMap);
        verticalGripper = new VerticalGripper(hardwareMap);
        dashboardTelemetry = db;
    }

    public void prepToBasket(){
        verticalSliders.setPosition(Height_basket);
        flipperTarget = 1;
        gripperTarget = 0;
    }

    public void prepToStow(){
        verticalSliders.setPosition(Height_stow);
        gripperTarget = 0;
    }

    public void prepToTransfer(){
        verticalSliders.setPosition(Height_transfer);
        flipperTarget = 0;
        gripperTarget = 1;
    }

    public void goHome(){
        verticalSliders.setPosition(-99);
        flipperTarget = 0;
        gripperTarget = 1;
    }

    public void prepToSample(){
        verticalSliders.setPosition(Height_sample);
        verticalFlipper.goToDropOff();
        flipperTarget = 1;
        flipperState = 1;
        gripperTarget = 1;
    }

    public void prepToClip(){
        verticalSliders.setPosition(Height_clip);
        verticalFlipper.goToDropOff();
        flipperTarget = 1;
        flipperState = 1;
        gripperTarget = 0;
    }

    public void clipClip(){
        verticalSliders.setPosition(Height_clipClip);
        gripperTarget=1;
    }


    public void openGripper(){
        gripperTarget = 1;
    }

    public void update(){
        if (gripperState != gripperTarget){
            runtime.reset();
            if (gripperTarget == 1) {
                verticalGripper.goToRelease();
                gripperState = 1;
            } else {
                verticalGripper.goToHold();
                gripperState = 0;
            }
        }
        if (runtime.milliseconds() > gripPause) {
            verticalSliders.update(dashboardTelemetry);
        }
        if (flipperState != flipperTarget && verticalSliders.getPositionMillimeter()>Height_flipDown && verticalSliders.getPositionMillimeter()<Height_flipUp){
            if (flipperTarget == 1) {
                verticalFlipper.goToDropOff();
                flipperState = 1;
            } else {
                verticalFlipper.goToPickUp();
                flipperState = 0;
            }
        }
        dashboardTelemetry.addData("gripper state", gripperState);
    }
}
