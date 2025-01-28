package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Subsystem_limelight")
@Config
public class Subsystem_limelight extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private Limelight3A limelight;
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        // Tell the driver that initialization is complete.
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        dashboardTelemetry.addData("Name", "%s",
                status.getName());
        dashboardTelemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        dashboardTelemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            //dashboardTelemetry.addData("LL Latency", captureLatency + targetingLatency);
            //dashboardTelemetry.addData("Parse Latency", parseLatency);
            //dashboardTelemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
            //dashboardTelemetry.addData("result", result.toString());

            // Getting numbers from Python
            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                double xLoc = pythonOutputs[1];
                double yLoc = pythonOutputs[2];
                double angle = pythonOutputs[3];
                dashboardTelemetry.addData("xloc", xLoc);
                dashboardTelemetry.addData("yloc", yLoc);
                dashboardTelemetry.addData("angle", angle);
            }
        }

        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        dashboardTelemetry.update();
        limelight.stop();
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

}
