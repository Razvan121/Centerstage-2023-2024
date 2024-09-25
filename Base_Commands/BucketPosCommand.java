package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class BucketPosCommand extends InstantCommand {
    public BucketPosCommand(Obot obot,LiftSubsystem.BucketState bucketState){
        super(
                ()-> obot.liftSubsystem.update(bucketState)
        );
    }
}
