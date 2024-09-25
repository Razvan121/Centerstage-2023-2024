package org.firstinspires.ftc.teamcode.NEDRobot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.ActiveStateCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.BucketPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.CapacPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.ExtendoPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.IntakePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.WristPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class ExtendIntakeCommand extends SequentialCommandGroup {
    public ExtendIntakeCommand(Obot obot){
        super(
                new SequentialCommandGroup(
                        new ExtendoPosCommand(obot,IntakeSubsystem.ExtendoState.EXTEND),
                        new WaitCommand(100),
                        new ActiveStateCommand(obot, IntakeSubsystem.ActiveState.START),
                        new WaitCommand(100),
                        new WristPosCommand(obot, IntakeSubsystem.WristState.OUT_EXTENDED)
                )

        );
    }
}
