package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class Claw (hardwareMap: HardwareMap) {

    private val leftServo: Servo = hardwareMap.get(Servo::class.java, "leftServo")
    private val rightServo: Servo = hardwareMap.get(Servo::class.java, "rightServo")

    fun close(): Action {
        return SequentialAction(
            InstantAction { leftServo.position = 0.7555 },
            InstantAction { rightServo.position = 0.3645 },
            SleepAction(1.0)
        )
    }

    fun open(): Action {
        return SequentialAction(
            InstantAction { leftServo.position = 0.6705 },
            InstantAction { rightServo.position = 0.4955 },
            SleepAction(1.0)
        )
    }

    fun inBox(): Action {
        return SequentialAction(
            InstantAction { leftServo.position = 0.463 },
            InstantAction { rightServo.position = 1.0 },
            SleepAction(1.0)
        )
    }

    fun approach(): Action {
        return SequentialAction(
            InstantAction { leftServo.position = 0.5 },
            InstantAction { rightServo.position = 0.5 },
            SleepAction(1.0)
        )
    }
}