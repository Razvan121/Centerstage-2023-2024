package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class TurretPosCommand extends InstantCommand {
    public TurretPosCommand(Obot obot,LiftSubsystem.TurretState turretState){
        super(
                () -> Obot.getInstance().liftSubsystem.update(turretState)
        );
    }
}
