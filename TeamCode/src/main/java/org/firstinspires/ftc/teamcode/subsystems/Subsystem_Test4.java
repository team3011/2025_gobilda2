package org.firstinspires.ftc.teamcode.subsystems;
/*
grab off wall
hook
spin

 */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@TeleOp(name = "Subsystem_Test4")
@Config
public class Subsystem_Test4 extends OpMode {
    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static VerticalSystem verticalSystem;
    GamepadEx g1;
    private ServoImplEx left;
    private ServoImplEx right;
    PinpointDrive drive;
    double headingOffset;
    double left_y, right_y, left_x, right_x, left_t, right_t;
    public static double pos = 0;
    //this section allows us to access telemetry data from a browser
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        verticalSystem = new VerticalSystem(hardwareMap, dashboardTelemetry);
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
//        verticalSystem.goHome();

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        this.g1.readButtons();
        this.left_y = zeroAnalogInput(g1.getLeftY());
        this.right_y = zeroAnalogInput(g1.getRightY());
        this.left_x = zeroAnalogInput(g1.getLeftX());
        this.right_x = zeroAnalogInput(g1.getRightX());
        this.left_t = -zeroAnalogInput(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        this.right_t = zeroAnalogInput(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        if(this.g1.isDown(GamepadKeys.Button.A)) {
            verticalSystem.prepToSample();
        }else if(this.g1.isDown(GamepadKeys.Button.X)) {
            verticalSystem.prepToClip();
        }else if(this.g1.isDown(GamepadKeys.Button.Y)){
            verticalSystem.goHome();
        }else if(this.g1.isDown(GamepadKeys.Button.B)){
            verticalSystem.clipClip();
        }
        verticalSystem.update();
        dashboardTelemetry.update();
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

    private double digitalTransmission(double input) {
        if (input < -0.8){
            return 3*input+2;
        } else if (input > 0.8){
            return 3*input-2;
        }
        return .5*input;
    }
}
