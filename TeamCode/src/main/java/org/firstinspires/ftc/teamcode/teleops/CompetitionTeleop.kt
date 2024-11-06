package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive

abstract class CompetitionTeleop : OpMode() {
    private lateinit var drive: PinpointDrive
    private lateinit var g1: PandaGamepad
    private var headingOffset: Double = 0.0

    override fun init() {
        drive = PinpointDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        g1 = PandaGamepad(gamepad1)
    }

    override fun loop() {
        //update gamepad values
        g1.update()

        //update drive Pose
        drive.updatePoseEstimate()

        /* driver 1 */
        val rawHeading = drive.getPinpoint().heading
        val heading: Rotation2d = Rotation2d.fromDouble(rawHeading - headingOffset)

        val input = Vector2d(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble()
        )

        drive.setDrivePowers(
            PoseVelocity2d(
                heading.inverse().times(input),
                ((gamepad1.left_trigger - gamepad1.right_trigger) * 1 / 2).toDouble()
            )
        )

        if (g1.b.justPressed()) headingOffset = rawHeading

        /* driver 2 */
    }

    protected abstract val allianceColor: AllianceColor
}