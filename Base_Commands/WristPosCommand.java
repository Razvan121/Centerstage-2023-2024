package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class WristPosCommand extends InstantCommand {
    public WristPosCommand(Obot obot,IntakeSubsystem.WristState wristState){
        super(
                ()-> obot.intakeSubsystem.update(wristState)
        );
    }
}
