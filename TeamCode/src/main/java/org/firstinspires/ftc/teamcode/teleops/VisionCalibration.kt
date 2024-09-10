package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.vision_pipelines.CalibrationPipeline
import org.opencv.core.Point
import org.openftc.easyopencv.OpenCvCameraFactory

@TeleOp(group = "Calibration")
class VisionCalibration : OpMode() {
    private val timer: ElapsedTime = ElapsedTime()

    private lateinit var pipeline: CalibrationPipeline

    override fun init() {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        val webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "webcam"
            ), cameraMonitorViewId
        )
        pipeline = CalibrationPipeline()
        webcam.setPipeline(pipeline)
        FtcDashboard.getInstance().startCameraStream(webcam, 0.0);
    }

    override fun init_loop() {
        val deltaTime = timer.time()
        timer.reset()
        telemetry.addData("deltaTime", deltaTime)

        val topLeft = pipeline.topLeft
        val bottomRight = pipeline.bottomRight
        telemetry.addData("top left", topLeft.toString())
        telemetry.addData("bottom right", bottomRight.toString())
        pipeline.topLeft = Point(
            topLeft.x + gamepad1.left_stick_x * deltaTime * 10,
            topLeft.y + gamepad1.left_stick_y * deltaTime * 10
        )
        pipeline.bottomRight = Point(
            bottomRight.x + gamepad1.right_stick_x * deltaTime * 10,
            bottomRight.y + gamepad1.right_stick_y * deltaTime * 10
        )
    }

    override fun loop() {}
}
