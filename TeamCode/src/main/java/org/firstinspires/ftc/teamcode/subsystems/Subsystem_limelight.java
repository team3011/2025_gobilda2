package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleops.AllianceColor;

@TeleOp
public class Subsystem_limelight extends OpMode {
    private Limelight3A limelight;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    GamepadEx g1;
    SuperSystem superSystem;
    Odometry odometry;

    public void init() {
        superSystem = new SuperSystem(hardwareMap,dashboardTelemetry);
        g1 = new GamepadEx(gamepad1);
        odometry = new Odometry(hardwareMap);
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        superSystem.start();
        odometry.odoUp();
    }

    public void loop() {
        //update gamepad values
        g1.readButtons();

        if (g1.stateJustChanged(GamepadKeys.Button.B)) {
            superSystem.scan(1);
        } else if(g1.stateJustChanged(GamepadKeys.Button.A)) {
            superSystem.reset();
        }



        //driver 1
        if (!superSystem.getIsScanning()){
        }else {
            double xOffset = superSystem.getXScanDirection();
            dashboardTelemetry.addData("xOffset", xOffset);
        }
        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            superSystem.toggle();
        }

/*
        //this will run all of our supersystem commands
        if (g1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (this.g1.isDown(GamepadKeys.Button.A)) { //really X
                superSystem.reset();
            } else if (this.g1.isDown(GamepadKeys.Button.B)) { //really O
                if(superSystem.getToggleState() == 0){
                    superSystem.scan(1); //yellow
                }else if(superSystem.getToggleState() == 1){
                    superSystem.scan(2); //blue
                }
            } else if (this.g1.isDown(GamepadKeys.Button.Y)) { //really ^
                superSystem.prepToDropOff();
            } else if (this.g1.isDown(GamepadKeys.Button.X)) { ////really []
                superSystem.dropOff();
            }
        }

         */
        superSystem.update();
        dashboardTelemetry.update();
    }
}
