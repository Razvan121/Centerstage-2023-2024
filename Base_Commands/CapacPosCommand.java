package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class CapacPosCommand extends InstantCommand {
    public CapacPosCommand(Obot obot, IntakeSubsystem.CapacState state){
        super(
                () -> obot.intakeSubsystem.update(state)
        );
    }
}
