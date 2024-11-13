package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive

@TeleOp
abstract class CompetitionTeleop : OpMode() {
    private lateinit var drive: PinpointDrive
    private lateinit var g1: PandaGamepad
    private lateinit var g2: PandaGamepad
    private lateinit var claw: Claw
    private lateinit var collectionArm: CollectionArm
    private lateinit var scoringArm: ScoringArm
    private var headingOffset: Double = 0.0
    private val dash: FtcDashboard = FtcDashboard.getInstance()
    private var runningActions: List<Action> = ArrayList<Action>()

    override fun init() {
        drive = PinpointDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        g1 = PandaGamepad(gamepad1)
        g2 = PandaGamepad(gamepad2)
        //claw = Claw(hardwareMap)
        //collectionArm = CollectionArm(hardwareMap)
        //scoringArm = ScoringArm(hardwareMap)

    }

    override fun loop() {

        // update running actions
        val packet = TelemetryPacket()

        val newActions: MutableList<Action> = ArrayList()
        for (action in runningActions) {
            action.preview(packet.fieldOverlay())
            if (action.run(packet)) {
                newActions.add(action)
            }
        }
        runningActions = newActions

        dash.sendTelemetryPacket(packet)

        //update gamepad values
        g1.update()
        g2.update()

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
        /*if (g2.dpadDown.justPressed()) claw.approach()
        if (g2.dpadLeft.justPressed()) claw.close()
        if (g2.dpadRight.justPressed()) claw.open()
        if (g2.dpadUp.justPressed()) claw.close()
        if (g2.leftBumper.justActive()) claw.inBox()
        if (g2.leftBumper.justPressed() && (g2.rightBumper.justPressed())) collectionArm.retract() //debug
        if (g2.rightBumper.justPressed()) collectionArm.extend()
        if (g2.a.justPressed()) scoringArm.score()
        if (g2.b.justPressed()) scoringArm.collect()
        */
        if (g2.leftStickY.isActive()) scoringArm.manual(g2.leftStickY)


    }

    protected abstract val allianceColor: AllianceColor
}