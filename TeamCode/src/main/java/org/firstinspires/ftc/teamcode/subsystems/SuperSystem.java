package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SuperSystem {
    //declare objects
    Telemetry dashboardTelemetry;
    public static VerticalSystem verticalSystem;
    public static HorizontalArm horizontalArm;
    public static HorizontalHand horizontalHand;
    public static HorizontalSliders horizontalSliders;

    //set up timers
    ElapsedTime clawTimer = new ElapsedTime();
    public static int clawPause = 500;
    ElapsedTime armTimer = new ElapsedTime();
    boolean armPauseTriggered = false;
    ElapsedTime transferTimer = new ElapsedTime();
    boolean transferTriggered = false;
    public static int transferPause = 1500;
    public static int pickUpPause = 650;
    boolean isPickupPause = false;
    ElapsedTime pickUpPauseTimer = new ElapsedTime();
    ElapsedTime scanPause = new ElapsedTime();
    boolean isScanning = false;
    public static double scanPowerFast = .6;
    public static double scanPowerSlow = .3;
    public static double upperLimit = .05;
    public static double lowerLimit = -.05;

    //set up vision
    RevBlinkinLedDriver blinkin;
    Servo rgbLED;
    public static MyLimeLight myLimeLight;
    DcMotor headlights;
    boolean isBlue = false;
    int toggleState = 0; //0 is yellow, 1 is color, 2 is lift

    public SuperSystem(@NonNull HardwareMap hardwareMap, @NonNull Telemetry db){
        dashboardTelemetry = db;
        headlights = hardwareMap.get(DcMotor.class, "led");
        verticalSystem = new VerticalSystem(hardwareMap, dashboardTelemetry);
        horizontalArm = new HorizontalArm(hardwareMap);
        horizontalHand = new HorizontalHand(hardwareMap);
        myLimeLight = new MyLimeLight(hardwareMap);
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinken");
        rgbLED = hardwareMap.get(Servo.class,"rgbLight");
        horizontalSliders = new HorizontalSliders(hardwareMap, dashboardTelemetry);
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        rgbLED.setPosition(.388);
    }

    public void start(){
        horizontalArm.goToStartPos();
        horizontalHand.wristPickup();
        horizontalHand.handPar();
        verticalSystem.goHome();
        headlights.setPower(0);
    }

    public void update(){
        //@arm transfer position waiting for vgripper to take control
        if (armPauseTriggered && armTimer.milliseconds() > VerticalSystem.gripPause/2.0) {
            horizontalArm.toStowPos();
            horizontalHand.openClaw();
            armPauseTriggered = false;
            setLED();
        }
        //waiting for arm to move out of the way before vsliders going up
        if (transferTriggered && transferTimer.milliseconds() > transferPause && horizontalSliders.getPositionMM() < 10) {
            transferTriggered = false;
            verticalSystem.prepToStow();
            armPauseTriggered = true;
            armTimer.reset();
        }
        //we are scanning for an object
        if (isScanning && scanPause.milliseconds() > 1000 && myLimeLight.update()) {
            horizontalHand.setPositionByCamera(myLimeLight.getAngle());
            //dashboardTelemetry.addData("xloc", myLimeLight.getxLoc());
            //dashboardTelemetry.addData("yloc",myLimeLight.getyLoc());
            //dashboardTelemetry.addData("angle",myLimeLight.getAngle());
            if (myLimeLight.getyLoc() == 0){
                horizontalSliders.manualInput(scanPowerFast);
            } else if (myLimeLight.getyLoc() < lowerLimit){
                //move out - blue
                rgbLED.setPosition(.722); //violet
                horizontalSliders.manualInput(scanPowerSlow);
            } else if (myLimeLight.getyLoc() > upperLimit) {
                //move in
                horizontalSliders.manualInput(-scanPowerSlow);
                rgbLED.setPosition(.333); //orange
            } else {
                //we good
                //isScanning = false;
                rgbLED.setPosition(.5); //sets to green
                horizontalSliders.manualInput(0);

                //dpad down
                horizontalArm.toPickupPos();
                horizontalHand.wristPickup();
                horizontalHand.openClaw();
                myLimeLight.stop();
                isScanning = false;
                headlights.setPower(0);
                isPickupPause = true;
                pickUpPauseTimer.reset();
            }
        }

        //we are trying to pick up the object
        if (isPickupPause && pickUpPauseTimer.milliseconds() > pickUpPause) {
            verticalSystem.prepToTransfer();
            horizontalHand.closeClaw();
            clawTimer.reset();
            while (clawTimer.milliseconds() < clawPause) {}
            horizontalArm.toTransferPos();
            horizontalHand.wristTransfer();
            horizontalHand.handPar();
            transferTriggered = true;
            transferTimer.reset();
            horizontalSliders.setPosition(0);
            isPickupPause = false;
        }

        //update all subsystems
        horizontalArm.update();
        verticalSystem.update();
        horizontalSliders.update();

        dashboardTelemetry.addData("toggleState",toggleState);
    }

    public void setAlliance(boolean input){
        if (input) {
            isBlue = true;
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }

    }

    public void setLED() {
        if (toggleState == 1) {
            if (isBlue) {
                rgbLED.setPosition(.611);
            } else {
                rgbLED.setPosition(.279);
            }
        } else if (toggleState == 2) {
            rgbLED.setPosition(1);
        } else {
            rgbLED.setPosition(.388);
        }
    }

    public void toggle(){
        toggleState += 1;
        if (toggleState > 2) toggleState = 0;
        setLED();
    }

    public void reset(){
        horizontalArm.goToStartPos();
        horizontalHand.wristPickup();
        horizontalHand.handPar();
        verticalSystem.goHome();
        headlights.setPower(0);
    }

    public void scan(){
        scanPause.reset();
        horizontalArm.toScanPos();
        horizontalHand.wristPickup();
        horizontalHand.handPar();
        horizontalHand.openClaw();
        verticalSystem.goHome();
        isScanning = true;
        headlights.setPower(1);

        //***** NEED TO FIX LL PIPELINES BEFORE CHANGING THIS ******
        myLimeLight.start(0); //0 red, 1 blue, 2 yellow
    }

    public void prepToDropOff(){
        if (toggleState == 0) { //0 is yellow, 1 is color, 2 is lift
            verticalSystem.prepToBasket();
        } else if (toggleState == 1) {
            verticalSystem.prepToClip();
        } else {
            verticalSystem.prepToLift();
        }
    }

    public void dropOff() {
        if (toggleState == 0) { //0 is yellow, 1 is color, 2 is lift
            verticalSystem.openGripper();
        } else if (toggleState == 1) {
            verticalSystem.clipClip();
        } else {
            verticalSystem.lift();
        }
    }
}
