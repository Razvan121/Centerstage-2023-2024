package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class WristOuttakePosCommand extends InstantCommand {
    public WristOuttakePosCommand(Obot obot, LiftSubsystem.WristState wristState){
        super(
                ()-> obot.liftSubsystem.update(wristState)
        );
    }
}
