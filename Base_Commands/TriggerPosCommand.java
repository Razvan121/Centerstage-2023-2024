package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class TriggerPosCommand extends InstantCommand {
    public TriggerPosCommand(Obot obot,LiftSubsystem.TriggerState triggerState){
        super(
                ()-> obot.liftSubsystem.update(triggerState)
        );
    }
}
