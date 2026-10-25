package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;

@TeleOp(name = "Competition: RED", group = "Competition")
public class CompetitionRed extends TeleOpBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}
