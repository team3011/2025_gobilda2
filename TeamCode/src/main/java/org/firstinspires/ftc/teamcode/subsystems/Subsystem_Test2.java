package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@TeleOp(name = "Subsystem_Test2")
@Config
public class Subsystem_Test2 extends OpMode {
    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    //public static HorizontalSliders horizontalSliders;
    public static VerticalSystem verticalSystem;
    public static HorizontalArm horizontalArm;
    public static HorizontalHand horizontalHand;
    GamepadEx g1;
    double left_y, right_y, left_x, right_x, left_t, right_t;
    ElapsedTime clawTimer = new ElapsedTime();
    public static int clawPause = 1000;
    ElapsedTime armTimer = new ElapsedTime();
    boolean armPauseTriggered = false;
    ElapsedTime transferTimer = new ElapsedTime();
    boolean transferTriggered = false;
    public static int transferPause = 1000;
    boolean readyToDrop = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        verticalSystem = new VerticalSystem(hardwareMap, dashboardTelemetry);
        horizontalArm = new HorizontalArm(hardwareMap);
        horizontalHand = new HorizontalHand(hardwareMap);
        //horizontalSliders = new HorizontalSliders(hardwareMap);
        this.g1 = new GamepadEx(gamepad1);


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
        horizontalHand.wristTransfer();
        horizontalHand.handPar();

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
        verticalSystem.update();


        //for testing horizontalSystem
        if (this.g1.isDown(GamepadKeys.Button.DPAD_DOWN)){
            horizontalArm.toPickupPos();
            horizontalHand.wristPickup();
            horizontalHand.handPar();
            horizontalHand.openClaw();
        } else if (this.g1.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            horizontalArm.toScanPos();
            horizontalHand.wristPickup();
            horizontalHand.handPar();
            horizontalHand.openClaw();
            verticalSystem.prepToTransfer();
        } else if (this.g1.isDown(GamepadKeys.Button.DPAD_UP)) {
            horizontalHand.closeClaw();
            clawTimer.reset();
            while (clawTimer.milliseconds() < clawPause) {

            }
            horizontalArm.toTransferPos();
            horizontalHand.wristTransfer();
            horizontalHand.handPar();
            transferTriggered = true;
            transferTimer.reset();
            horizontalHand.handPar();

        }

        if (transferTriggered && transferTimer.milliseconds() > transferPause) {
            transferTriggered = false;
            verticalSystem.prepToStow();
            armPauseTriggered = true;
            armTimer.reset();
        }
        horizontalArm.update();




        //for testing vertical sliders with a set point
//        if (this.g1.isDown(GamepadKeys.Button.A)){
//            verticalSliders.setPosition(0);
//        } else if (this.g1.isDown(GamepadKeys.Button.Y)){
//            verticalSliders.setPosition(iTesting);
//        }
//        verticalSliders.update(1,this.dashboardTelemetry);

        //for testing horizontal sliders with a set point
//        if (this.g1.isDown(GamepadKeys.Button.X)){
//            horizontalSliders.setPosition(0);
//        } else if (this.g1.isDown(GamepadKeys.Button.B)) {
//            horizontalSliders.setPosition(iTesting);
//        }
//        horizontalSliders.update(1,this.dashboardTelemetry);

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
