package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;


//this line identifies if this is an autonomous or teleop program
@Autonomous(name = "Subsystem_Test")
//this line allows you to modify variables inside the dashboard
@Config
public class Subsystem_Test extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotorEx motor0;
    //public static double power = 0.2;
    Odometry odo;

    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public GoBildaPinpointDriverRR pinpoint;
    public static double power = 1.0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
    //    this.motor0 = hardwareMap.get(DcMotorEx.class,"led");
        this.odo = new Odometry(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        pinpoint.initialize();

        // Tell the driver that initialization is complete.
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        //motor0.setPower(power);
        dashboardTelemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        odo.odoDown();
        runtime.reset();
        //PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        //watch this VIDEO
        //https://www.youtube.com/watch?v=uBwVSRxvpB8
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(0,0,0))
//                        //.strafeTo(new Vector2d(0,64))
//                        //.splineTo(new Vector2d(48, 48), Math.PI / 2)
//                        .splineToConstantHeading(
//                                new Vector2d(48, 48), Math.PI / 2,
//                                // only override velocity constraint
//                                new TranslationalVelConstraint(20.0))
//                        //.waitSeconds(3)
//                        //.strafeTo(new Vector2d(64,64))
//                        .build()
//        );
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //this.motor0.setPower(Subsystem_Test.power);
        //dashboardTelemetry.addData("motor0 power", motor0.getPower());
        pinpoint.update();
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        dashboardTelemetry.addData("Pinpoint", pinpoint.getConnectionInfo());
        dashboardTelemetry.addData("buh init", pinpoint.initialize());
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //this.motor0.setPower(0);
        //dashboardTelemetry.addData("motor0 power", motor0.getPower());
        dashboardTelemetry.update();
    }
}