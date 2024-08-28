package org.firstinspires.ftc.teamcode

import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class CalibrationPipeline : OpenCvPipeline() {
    private val GREEN = Scalar(0.0, 255.0, 0.0)
    private val YELLOW = Scalar(255.0, 255.0, 0.0)

    var topLeft: Point = Point(100.0, 180.0)
    var bottomRight: Point = Point(230.0, 350.0)
    var output: Mat? = null

    override fun processFrame(input: Mat): Mat {
        output = input
        Imgproc.rectangle(output, Rect(topLeft, bottomRight), GREEN, 2);
        return output as Mat
    }
}