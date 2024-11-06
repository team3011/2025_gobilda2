package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

/** @noinspection unused*/
public abstract class JavaCompetitionTeleop extends OpMode {

    protected static AllianceColor allianceColor;

    PinpointDrive drive;
    GamepadEx g1;
    double headingOffset;


    public void init() {
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        g1 = new GamepadEx(gamepad1);
    }

    public void loop() {
        //update gamepad values
        g1.readButtons();

        //update drive Pose
        drive.updatePoseEstimate();

        /* driver 1 */

        double rawHeading = drive.getPinpoint().getHeading();
        Rotation2d heading = Rotation2d.fromDouble(rawHeading - headingOffset);

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        );

        drive.setDrivePowers(new PoseVelocity2d(
                heading.inverse().times(input),
                (gamepad1.left_trigger-gamepad1.right_trigger)*1/2
        ));

        if (g1.wasJustPressed(GamepadKeys.Button.B)) headingOffset = rawHeading;

        /* driver 2 */
    }

    protected abstract AllianceColor getAllianceColor();
}
