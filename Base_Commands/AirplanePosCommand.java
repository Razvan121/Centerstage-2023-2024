package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class AirplanePosCommand extends InstantCommand {
    public AirplanePosCommand(Obot obot,LiftSubsystem.AirplaneState airplaneState){
        super(
                () -> Obot.getInstance().liftSubsystem.update(airplaneState)
        );
    }
}
