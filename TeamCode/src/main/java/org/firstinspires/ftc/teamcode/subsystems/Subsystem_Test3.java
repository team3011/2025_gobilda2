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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@TeleOp(name = "Subsystem_Test3")
@Config
public class Subsystem_Test3 extends OpMode {
    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    PinpointDrive drive = new PinpointDrive(hardwareMap,new Pose2d(0, 0, 0));

    GamepadEx g1;
    double left_y, right_y, left_x, right_x, left_t, right_t;
    ElapsedTime runtime = new ElapsedTime();
    private boolean endGame = false;
    public static double dTesting = 0;
    public static int iTesting = 200;

    DcMotorEx test1, test2, test3, test4;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        this.g1 = new GamepadEx(gamepad1);
        test1 = hardwareMap.get(DcMotorEx.class, "rightFront");
        test2 = hardwareMap.get(DcMotorEx.class, "rightBack");
        test3 = hardwareMap.get(DcMotorEx.class, "leftFront");
        test4 = hardwareMap.get(DcMotorEx.class, "leftBack");
        test3.setDirection(DcMotorSimple.Direction.REVERSE);
        test4.setDirection(DcMotorSimple.Direction.REVERSE);

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
        //test1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //test2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        test1.setVelocity(dTesting,AngleUnit.DEGREES);
        //test1.setPower(dTesting);
        dashboardTelemetry.addData("motor1 velocity",test1.getVelocity(AngleUnit.DEGREES));
        dashboardTelemetry.addData("motor1 power", test1.getPower());

        test2.setVelocity(dTesting,AngleUnit.DEGREES);
        //test2.setPower(dTesting);
        dashboardTelemetry.addData("motor2 velocity",test2.getVelocity(AngleUnit.DEGREES));
        dashboardTelemetry.addData("motor2 power", test2.getPower());

        test3.setVelocity(dTesting,AngleUnit.DEGREES);
        //test2.setPower(dTesting);
        dashboardTelemetry.addData("motor3 velocity",test3.getVelocity(AngleUnit.DEGREES));
        dashboardTelemetry.addData("motor3 power", test3.getPower());

        test4.setVelocity(dTesting,AngleUnit.DEGREES);
        //test2.setPower(dTesting);
        dashboardTelemetry.addData("motor4 velocity",test4.getVelocity(AngleUnit.DEGREES));
        dashboardTelemetry.addData("motor4 power", test4.getPower());



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
