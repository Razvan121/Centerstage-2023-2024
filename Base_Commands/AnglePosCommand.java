package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class AnglePosCommand extends InstantCommand {
    public AnglePosCommand(Obot obot,LiftSubsystem.AngleState angleState) {
        super(
                () -> Obot.getInstance().liftSubsystem.update(angleState)
        );
    }
}
