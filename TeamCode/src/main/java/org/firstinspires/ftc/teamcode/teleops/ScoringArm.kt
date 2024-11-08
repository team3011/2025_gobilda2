package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.min


class ScoringArm(hardwareMap: HardwareMap) {


    private val motor = hardwareMap.get(DcMotor::class.java, "scoringArmMotor")
    private val scoringPosition = 0 //INPUT SCORING MOTOR POSITION HERE
    private val collectionPosition = 0 //INPUT COLLECTION MOTOR POSITION HERE
    private val minPosition = 0
    private val maxPosition = 1000
    private val motorPower = 0.6

    inner class GoToPosition (private val position: Int) : Action {
        private var initialized = false
        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                motor.power = motorPower
                motor.targetPosition = position
                initialized = true
            }
            val position = motor.currentPosition
            packet.put("armPosition", position)
            return motor.isBusy
        }
    }

    fun score(): Action = GoToPosition(scoringPosition)
    fun collect(): Action = GoToPosition(collectionPosition)
    fun manual(input: Double): Action {
        val stepSize = 5.0
        var targetPosition = (motor.currentPosition + input * stepSize)
        targetPosition = clamp(targetPosition, minPosition.toDouble(), maxPosition.toDouble())
        return GoToPosition(targetPosition.toInt())
    }
}