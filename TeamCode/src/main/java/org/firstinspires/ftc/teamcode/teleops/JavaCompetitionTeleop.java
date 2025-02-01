package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.SuperSystem;

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
    SuperSystem superSystem;
    Odometry odometry;

    public void init() {
        superSystem = new SuperSystem(hardwareMap,dashboardTelemetry);
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        g1 = new GamepadEx(gamepad1);
        allianceColor = getAllianceColor();
        odometry = new Odometry(hardwareMap);

        if (allianceColor.equals(AllianceColor.BLUE)) {
            superSystem.setAlliance(true);
        } else {
            superSystem.setAlliance(false);
        }

        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        superSystem.start();
        odometry.odoUp();
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

        //driver 1
        if (allianceColor.equals(AllianceColor.BLUE)) {
            //drive.drive(digitalTransmission(-left_x), digitalTransmission(left_y), right_x, false, dashboardTelemetry);
            drive.drive2(digitalTransmission(-left_x),digitalTransmission(-left_y),right_x, dashboardTelemetry);
        }

        //this modifies the field centric direction
        //if (g1.wasJustPressed(GamepadKeys.Button.B)) headingOffset = rawHeading;
        //if (g1.wasJustPressed(GamepadKeys.Button.B)) headingOffset = Math.PI/2;
        //if (g1.wasJustPressed(GamepadKeys.Button.A)) headingOffset = 0;

        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            superSystem.toggle();
        }


        //this will run all of our supersystem commands
        if (g1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (this.g1.isDown(GamepadKeys.Button.A)) { //really X
                superSystem.reset();
            } else if (this.g1.isDown(GamepadKeys.Button.B)) { //really O
                superSystem.scan(0);
//                if(superSystem.getToggleState() == 0){
//                    superSystem.scan(1);
//                }else if(superSystem.getToggleState() == 1){
//                    if(allianceColor.equals(AllianceColor.BLUE)){
//                        superSystem.scan(2);
//                    }else if(allianceColor.equals(AllianceColor.RED)){
//                        superSystem.scan(0);
//                    }
//                }
            } else if (this.g1.isDown(GamepadKeys.Button.Y)) { //really ^
                superSystem.prepToDropOff();
            } else if (this.g1.isDown(GamepadKeys.Button.X)) { ////really []
                superSystem.dropOff();
            }
        }else{
            if (this.g1.isDown(GamepadKeys.Button.A)) { //really X
                drive.setHeadingToMaintain(0); // 180 degrees???
            } else if (this.g1.isDown(GamepadKeys.Button.B)) { //really O
                drive.setHeadingToMaintain(90);
            } else if (this.g1.isDown(GamepadKeys.Button.Y)) { //really ^
                drive.setHeadingToMaintain(180);
            } else if (this.g1.isDown(GamepadKeys.Button.X)) { ////really []
                drive.setHeadingToMaintain(-90);
            }
        }



        superSystem.update();
        dashboardTelemetry.update();
    }


    /**
     * removes the analog drift
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
