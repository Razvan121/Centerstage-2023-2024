package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class LiftPosCommand extends InstantCommand {
    public LiftPosCommand(Obot obot,LiftSubsystem.LiftState liftState){
        super(
                ()-> obot.liftSubsystem.update(liftState)
        );
    }
}
