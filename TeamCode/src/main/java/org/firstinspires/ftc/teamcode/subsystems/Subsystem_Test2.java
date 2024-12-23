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
    VerticalSliders verticalSliders;
    HorizontalSliders horizontalSliders;
    VerticalFlipper verticalFlipper;
    GamepadEx g1;
    double left_y, right_y, left_x, right_x, left_t, right_t;
    ElapsedTime runtime = new ElapsedTime();
    private boolean endGame = false;
    private double testing = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        this.verticalSliders = new VerticalSliders(hardwareMap);
        this.horizontalSliders = new HorizontalSliders(hardwareMap);
        this.verticalFlipper = new VerticalFlipper(hardwareMap);

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



        /*
        //for testing vertical sliders with a set point
        if (this.g1.isDown(GamepadKeys.Button.A)){

            this.verticalSliders.setPosition(0);
        } else if (this.g1.isDown(GamepadKeys.Button.Y)){
            this.verticalSliders.setPosition(verticalSetPosition);
        }
         */

        /*
        //for testing horizontal sliders with a set point
        if (this.g1.isDown(GamepadKeys.Button.A)){
            this.horizontalSliders.setPosition(0);
        } else if (this.g1.isDown(GamepadKeys.Button.Y)){
            this.horizontalSliders.setPosition(horizontalSetPosition);
        }
         */


        /*
        //this is testing the climb
        this.dashboardTelemetry.addData("left_y",left_y);
        if (Math.abs(left_y) > .25) {
            endGame = true;
            this.verticalSliders.manualInput(left_y);
        }
        if (endGame && Math.abs(left_y) < .25) {
            this.verticalSliders.manualInput(.58);
        }
         */

        //this is to test vflippy

        if (this.g1.isDown(GamepadKeys.Button.A)){
            this.verticalFlipper.goToPickUp();
        } else if (this.g1.isDown(GamepadKeys.Button.Y)){
            this.verticalFlipper.goToDropOff();
        }






        //this.horizontalSliders.update(1,this.dashboardTelemetry);
        //this.dashboardTelemetry.addData("hor position", this.horizontalSliders.getPositionMM());


        //this.verticalSliders.update(1,this.dashboardTelemetry);
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
