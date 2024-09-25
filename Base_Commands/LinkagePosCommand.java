package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class LinkagePosCommand extends InstantCommand {
    public LinkagePosCommand(Obot obot,LiftSubsystem.LinkageState linkageState){
        super(
                ()-> Obot.getInstance().liftSubsystem.update(linkageState)
        );
    }
}
