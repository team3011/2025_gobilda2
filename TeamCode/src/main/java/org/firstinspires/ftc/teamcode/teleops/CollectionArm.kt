package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap


class CollectionArm(hardwareMap: HardwareMap) {
    private val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "collectionArmMotor")
    private val motorPower = 0.6
    private val collectionDistance = 0 // INPUT CORRECT DISTANCE TO ADD FOR COLLECTING
    private val retractionDistance = 0 // INPUT CORRECT DISTANCE TO ADD FOR RETRACTING

    inner class Extend : Action {
        private var initialized = false
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                motor.power = motorPower
                motor.targetPosition = motor.currentPosition + collectionDistance
                initialized = true
            }
            val pos = motor.currentPosition
            packet.put("collectionArmPosition", pos)
            return motor.isBusy
        }
    }

    inner class Retract : Action {
        private var initialized = false
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                motor.power = motorPower
                motor.targetPosition = motor.currentPosition - retractionDistance
                initialized = true
            }
            val pos = motor.currentPosition
            packet.put("collectionArmPosition", pos)
            return motor.isBusy
        }
    }
    fun extend(): Action {
        return Extend()
    }

    fun retract(): Action {
        return Retract()
    }
}