package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueTeleop.j")
public class BlueTeleop extends JavaCompetitionTeleop{

    @Override
    protected AllianceColor getAllianceColor() {
       return AllianceColor.BLUE;
    }
}
