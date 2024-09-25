package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class PitchPosCommand extends InstantCommand {
    public PitchPosCommand(Obot obot,LiftSubsystem.PitchState pitchState){
        super(
                ()-> obot.liftSubsystem.update(pitchState)
        );
    }
}
