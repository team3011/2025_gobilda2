package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

/** @noinspection unused*/
@TeleOp
public abstract class JavaCompetitionTeleop extends OpMode {
    protected AllianceColor allianceColor;
    PinpointDrive drive;
    GamepadEx g1;
    double headingOffset;
    double left_y, right_y, left_x, right_x, left_t, right_t;
    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init() {
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        g1 = new GamepadEx(gamepad1);
        allianceColor = getAllianceColor();

        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
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


        dashboardTelemetry.addData("y",digitalTransmission(-left_y));
        //driver 1
        dashboardTelemetry.addData("color", allianceColor.toString());
        if (allianceColor.equals(AllianceColor.BLUE)) {
            drive.drive(digitalTransmission(left_x), digitalTransmission(-left_y), right_x, false, dashboardTelemetry);
        }

        //this modifies the field centric direction
        //if (g1.wasJustPressed(GamepadKeys.Button.B)) headingOffset = rawHeading;
        //if (g1.wasJustPressed(GamepadKeys.Button.B)) headingOffset = Math.PI/2;
        //if (g1.wasJustPressed(GamepadKeys.Button.A)) headingOffset = 0;

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
