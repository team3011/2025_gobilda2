package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@TeleOp(name = "Subsystem_Test2")
@Config
public class Subsystem_Test2 extends OpMode {
    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static VerticalSystem verticalSystem;
    public static HorizontalArm horizontalArm;
    public static HorizontalHand horizontalHand;
    public static HorizontalSliders horizontalSliders;
    public static int iTesting = 100;
    GamepadEx g1;
    double left_y, right_y, left_x, right_x, left_t, right_t;
    ElapsedTime clawTimer = new ElapsedTime();
    public static int clawPause = 500;
    ElapsedTime armTimer = new ElapsedTime();
    boolean armPauseTriggered = false;
    ElapsedTime transferTimer = new ElapsedTime();
    boolean transferTriggered = false;
    public static int transferPause = 1500;
    RevBlinkinLedDriver blinkin;
    Servo rgbLED;
    boolean isScanning = false;
    public static MyLimeLight myLimeLight;
    ElapsedTime scanPause = new ElapsedTime();
    public static double scanPowerFast = .6;
    public static double scanPowerSlow = .3;
    public static int pickUpPause = 650;
    boolean isPickupPause = false;
    ElapsedTime pickUpPauseTimer = new ElapsedTime();
    DcMotor headlights;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        headlights = hardwareMap.get(DcMotor.class, "led");
        verticalSystem = new VerticalSystem(hardwareMap, dashboardTelemetry);
        horizontalArm = new HorizontalArm(hardwareMap);
        horizontalHand = new HorizontalHand(hardwareMap);
        myLimeLight = new MyLimeLight(hardwareMap);
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinken");
        rgbLED = hardwareMap.get(Servo.class,"rgbLight");
        horizontalSliders = new HorizontalSliders(hardwareMap);
        this.g1 = new GamepadEx(gamepad1);
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        // Tell the driver that initialization is complete.
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        horizontalArm.goToStartPos();
        horizontalHand.wristPickup();
        horizontalHand.handPar();
        verticalSystem.goHome();

        headlights.setPower(0);

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //update gamepad values
        this.g1.readButtons();
        this.left_y = zeroAnalogInput(g1.getLeftY());
        this.right_y = zeroAnalogInput(g1.getRightY());
        this.left_x = zeroAnalogInput(g1.getLeftX());
        this.right_x = zeroAnalogInput(g1.getRightX());
        this.left_t = -zeroAnalogInput(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        this.right_t = zeroAnalogInput(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        dashboardTelemetry.addData("XLoc:", myLimeLight.getxLoc());
        dashboardTelemetry.addData("YLoc:", myLimeLight.getyLoc());
        dashboardTelemetry.addData("Angle:", myLimeLight.getAngle());

        //for testing verticalSystem
        if (this.g1.isDown(GamepadKeys.Button.A)){
            verticalSystem.goHome();
        } else if (this.g1.isDown(GamepadKeys.Button.B)) {
            verticalSystem.prepToTransfer();
        } else if (this.g1.isDown(GamepadKeys.Button.Y)) {
            verticalSystem.prepToBasket();
        } else if (this.g1.isDown(GamepadKeys.Button.X)) {
            verticalSystem.openGripper();
        }

        if (armPauseTriggered && armTimer.milliseconds() > VerticalSystem.gripPause/2) {
            horizontalArm.toStowPos();
            horizontalHand.openClaw();
            armPauseTriggered = false;
        }

        //for testing horizontalSystem
        if (this.g1.isDown(GamepadKeys.Button.DPAD_DOWN)){
            horizontalArm.toPickupPos();
            horizontalHand.wristPickup();
            horizontalHand.openClaw();
            rgbLED.setPosition(.279);
            myLimeLight.stop();
            isScanning = false;
            headlights.setPower(0);
        } else if (this.g1.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            scanPause.reset();
            horizontalArm.toScanPos();
            horizontalHand.wristPickup();
            horizontalHand.handPar();
            horizontalHand.openClaw();
            verticalSystem.goHome();
            rgbLED.setPosition(.4);
            isScanning = true;
            headlights.setPower(1);
            myLimeLight.start(0); //0 red, 1 blue, 2 yellow
        } else if (this.g1.isDown(GamepadKeys.Button.DPAD_UP)) {
            rgbLED.setPosition(1);
            verticalSystem.prepToTransfer();
            horizontalHand.closeClaw();
            clawTimer.reset();
            while (clawTimer.milliseconds() < clawPause) {

            }
            horizontalArm.toTransferPos();
            horizontalHand.wristTransfer();
            horizontalHand.handPar();
            transferTriggered = true;
            transferTimer.reset();
            horizontalSliders.setPosition(0);
        }

        if (transferTriggered && transferTimer.milliseconds() > transferPause && horizontalSliders.getPositionMM() < 10) {
            transferTriggered = false;
            verticalSystem.prepToStow();
            armPauseTriggered = true;
            armTimer.reset();
        }

        if (isScanning && scanPause.milliseconds() > 1000 && myLimeLight.update()) {
            horizontalHand.setPositionByCamera(myLimeLight.getAngle());
            dashboardTelemetry.addData("xloc", myLimeLight.getxLoc());
            dashboardTelemetry.addData("yloc",myLimeLight.getyLoc());
            dashboardTelemetry.addData("angle",myLimeLight.getAngle());
            if (myLimeLight.getyLoc() == 0){
                horizontalSliders.manualInput(scanPowerFast);
            } else if (myLimeLight.getyLoc() < .05){
                //move out - blue
                rgbLED.setPosition(.6);
                horizontalSliders.manualInput(scanPowerSlow);
            } else if (myLimeLight.getyLoc() > .15) {
                //move in
                horizontalSliders.manualInput(-scanPowerSlow);
                rgbLED.setPosition(.4);
            } else {
                //we good
                //isScanning = false;
                rgbLED.setPosition(.5);
                horizontalSliders.manualInput(0);

                //dpad down
                horizontalArm.toPickupPos();
                horizontalHand.wristPickup();
                horizontalHand.openClaw();
                rgbLED.setPosition(.279);
                myLimeLight.stop();
                isScanning = false;
                headlights.setPower(0);
                isPickupPause = true;
                pickUpPauseTimer.reset();
            }

            //horizontalHand.setPosition();
        }

        if (isPickupPause && pickUpPauseTimer.milliseconds() > pickUpPause) {
            rgbLED.setPosition(1);
            verticalSystem.prepToTransfer();
            horizontalHand.closeClaw();
            clawTimer.reset();
            while (clawTimer.milliseconds() < clawPause) {

            }
            horizontalArm.toTransferPos();
            horizontalHand.wristTransfer();
            horizontalHand.handPar();
            transferTriggered = true;
            transferTimer.reset();
            horizontalSliders.setPosition(0);
            isPickupPause = false;
        }

        dashboardTelemetry.addData("isScanning",isScanning);

        horizontalArm.update();
        verticalSystem.update();






        //for testing vertical sliders with a set point
//        if (this.g1.isDown(GamepadKeys.Button.A)){
//            verticalSliders.setPosition(0);
//        } else if (this.g1.isDown(GamepadKeys.Button.Y)){
//            verticalSliders.setPosition(iTesting);
//        }
//        verticalSliders.update(1,this.dashboardTelemetry);

        //for testing horizontal sliders with a set point
        if (this.g1.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            horizontalSliders.setPosition(0);
        } else if (this.g1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            horizontalSliders.setPosition(iTesting);
        }
        horizontalSliders.update(1,this.dashboardTelemetry);

        //this is to test vflippy
//        if (this.g1.isDown(GamepadKeys.Button.DPAD_UP)){
//            verticalFlipper.goToPickUp();
//        } else if (this.g1.isDown(GamepadKeys.Button.DPAD_DOWN)){
//            verticalFlipper.goToDropOff();
//        }

        //this is to test vgrippy
//        if (this.g1.isDown(GamepadKeys.Button.DPAD_LEFT)){
//            verticalGripper.goToHold();
//        } else if (this.g1.isDown(GamepadKeys.Button.DPAD_RIGHT)){
//            verticalGripper.goToRelease();
//        }


        this.dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        dashboardTelemetry.update();
    }

    /**
     * removes the analog drift
     * @param input
     * @return
     */
    private double zeroAnalogInput(double input){
        if (Math.abs(input) < 0.05){
            input = 0;
        }
        return input;
    }
}
