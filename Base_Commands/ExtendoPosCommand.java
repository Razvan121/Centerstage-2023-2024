package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class ExtendoPosCommand extends InstantCommand {
    public ExtendoPosCommand(Obot obot,IntakeSubsystem.ExtendoState extendoState){
        super(
                ()-> Obot.getInstance().intakeSubsystem.update(extendoState)
        );
    }
}
