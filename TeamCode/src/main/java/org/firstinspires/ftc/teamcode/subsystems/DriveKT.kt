package org.firstinspires.ftc.teamcode.subsystems

import androidx.annotation.FloatRange
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.cos
import kotlin.math.sin

class Drive(hardwareMap: HardwareMap) {

    @FloatRange(from = 0.0, to = 1.0)
    var power = 0.0

    private val frontLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "frontLeft")
    private val frontRight: DcMotor = hardwareMap.get(DcMotor::class.java, "frontRight")
    private val backLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "backLeft")
    private val backRight: DcMotor = hardwareMap.get(DcMotor::class.java, "backRight")

    init {

        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
    }

    fun drive(directionRadians: Double, rotation: Double) {
        frontLeft.power = power * sin(directionRadians + Math.PI / 4) + rotation
        frontRight.power = power * cos(directionRadians + Math.PI / 4) - rotation
        backLeft.power = power * cos(directionRadians + Math.PI / 4) + rotation
        backRight.power = power * sin(directionRadians + Math.PI / 4) - rotation
    }
}