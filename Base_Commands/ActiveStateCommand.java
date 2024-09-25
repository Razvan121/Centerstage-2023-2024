package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class ActiveStateCommand extends InstantCommand {
    public ActiveStateCommand(Obot obot,IntakeSubsystem.ActiveState activeState){
        super(
                () -> Obot.getInstance().intakeSubsystem.update(activeState)
        );
    }
}
