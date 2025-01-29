package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalHand;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSliders;
import org.firstinspires.ftc.teamcode.subsystems.MyLimeLight;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSystem;

/** @noinspection unused*/
@TeleOp
@Config
public abstract class JavaCompetitionTeleop extends OpMode {
    protected AllianceColor allianceColor;
    PinpointDrive drive;
    GamepadEx g1;
    double headingOffset;
    double left_y, right_y, left_x, right_x, left_t, right_t;
    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    //***********************************
    //copied from subsystem test 2 01/29
    //***********************************
    public static VerticalSystem verticalSystem;
    public static HorizontalArm horizontalArm;
    public static HorizontalHand horizontalHand;
    public static HorizontalSliders horizontalSliders;
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
    //********************************


    public void init() {
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        g1 = new GamepadEx(gamepad1);
        allianceColor = getAllianceColor();

        //***********************************
        //copied from subsystem test 2 01/29
        headlights = hardwareMap.get(DcMotor.class, "led");
        verticalSystem = new VerticalSystem(hardwareMap, dashboardTelemetry);
        horizontalArm = new HorizontalArm(hardwareMap);
        horizontalHand = new HorizontalHand(hardwareMap);
        myLimeLight = new MyLimeLight(hardwareMap);
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinken");
        rgbLED = hardwareMap.get(Servo.class,"rgbLight");
        horizontalSliders = new HorizontalSliders(hardwareMap);
        //***********************************

        if (allianceColor.equals(AllianceColor.BLUE)) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }

        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
        ***********************************
        copied from subsystem test 2 01/29
        ***********************************
     */
    @Override
    public void start() {
        horizontalArm.goToStartPos();
        horizontalHand.wristPickup();
        horizontalHand.handPar();
        headlights.setPower(0);

    }

    public void loop() {
        //update gamepad values
        g1.readButtons();
        left_y = zeroAnalogInput(g1.getLeftY());
        right_y = zeroAnalogInput(g1.getRightY());
        left_x = zeroAnalogInput(g1.getLeftX());
        right_x = zeroAnalogInput(g1.getRightX());
        left_t = -zeroAnalogInput(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        right_t = zeroAnalogInput(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));


        dashboardTelemetry.addData("y",left_y);
        dashboardTelemetry.addData("x",left_x);
        dashboardTelemetry.addData("dTy",digitalTransmission(left_y));
        dashboardTelemetry.addData("dTx",digitalTransmission(left_x));
        //driver 1
        dashboardTelemetry.addData("color", allianceColor.toString());
        if (allianceColor.equals(AllianceColor.BLUE)) {
            //drive.drive(digitalTransmission(-left_x), digitalTransmission(left_y), right_x, false, dashboardTelemetry);
            drive.drive2(digitalTransmission(left_x),digitalTransmission(left_y),right_x, dashboardTelemetry);
        }

        //this modifies the field centric direction
        //if (g1.wasJustPressed(GamepadKeys.Button.B)) headingOffset = rawHeading;
        //if (g1.wasJustPressed(GamepadKeys.Button.B)) headingOffset = Math.PI/2;
        //if (g1.wasJustPressed(GamepadKeys.Button.A)) headingOffset = 0;


        //***********************************
        //copied from subsystem test 2 01/29
        //***********************************
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
        //***********************************






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

    private double digitalTransmission(double input) {
        if (input < -0.8){
            return 3*input+2;
        } else if (input > 0.8){
            return 3*input-2;
        }
        return .5*input;
    }

    protected abstract AllianceColor getAllianceColor();
}
