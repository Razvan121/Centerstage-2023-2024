package org.firstinspires.ftc.teamcode.NEDRobot.Commands;


import android.app.usage.NetworkStats;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.BucketPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.CapacPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.ExtendoPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.LiftPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.LinkagePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.PitchPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.WristOuttakePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class DepositCommand extends SequentialCommandGroup {
    public DepositCommand(Obot obot){
        super(
                new SequentialCommandGroup(
                        new BucketPosCommand(obot, LiftSubsystem.BucketState.IN),
                        new WaitCommand(250),
                        new InstantCommand(() -> obot.linkage.setPosition(0.65)),
                        new WaitCommand(250),
                        new BucketPosCommand(obot, LiftSubsystem.BucketState.SEMIOUT),
                        new WaitCommand(250),
                        new BucketPosCommand(obot, LiftSubsystem.BucketState.LOW),
                        new WaitCommand(250),
                        new InstantCommand(() -> obot.WristOuttake.setPosition(0.79)),
                        new WaitCommand(400),
                        new PitchPosCommand(obot, LiftSubsystem.PitchState.LOW),
                        new WaitCommand(250),
                        new CapacPosCommand(obot, IntakeSubsystem.CapacState.CLOSED),
                        new InstantCommand(() -> obot.Lift.setMotionProfileTargetPosition(-300))
                )
        );
    }
}
