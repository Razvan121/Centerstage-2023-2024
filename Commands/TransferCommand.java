package org.firstinspires.ftc.teamcode.NEDRobot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.ActiveStateCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.BucketPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.CapacPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.ExtendoPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.LiftPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.LinkagePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.PitchPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.TriggerPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.WristPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Obot obotV2){
        super(
                new SequentialCommandGroup(
                        new ExtendoPosCommand(obotV2, IntakeSubsystem.ExtendoState.HOME),
                        new WaitCommand(250),
                        new WristPosCommand(obotV2, IntakeSubsystem.WristState.TRANSFER),
                        new WaitCommand(250),
                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.IN),
                        new WaitCommand(250),
                        new CapacPosCommand(obotV2, IntakeSubsystem.CapacState.OPENED),
                        new WaitCommand(250),
                        new ActiveStateCommand(obotV2, IntakeSubsystem.ActiveState.STOP),
                        new WaitCommand(250),
                        new PitchPosCommand(obotV2, LiftSubsystem.PitchState.TRANSFER),
                        new WaitCommand(350),
                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.SEMIIN),
                        new WaitCommand(350),
                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.TRANSFER),
                        new WaitCommand(1000),
                        new TriggerPosCommand(obotV2, LiftSubsystem.TriggerState.CLOSE),
                        new WaitCommand(250),
                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.HOME)
                )
        );
    }
}
