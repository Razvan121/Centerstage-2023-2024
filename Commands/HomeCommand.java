package org.firstinspires.ftc.teamcode.NEDRobot.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.AnglePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.BucketPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.ExtendoPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.LiftPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.LinkagePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.PitchPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.TriggerPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.WristOuttakePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

import java.util.function.BooleanSupplier;

public class HomeCommand extends SequentialCommandGroup {
    public HomeCommand(Obot obot){
        super(
                new SequentialCommandGroup(
                        new PitchPosCommand(obot, LiftSubsystem.PitchState.TRANSFER),
                        new WaitCommand(250),
                        new WristOuttakePosCommand(obot, LiftSubsystem.WristState.HORIZONTAL_TRANSFER),
                        new WaitCommand(250),
                        new AnglePosCommand(obot, LiftSubsystem.AngleState.OUT),
                        new WaitCommand(250),
                        new PitchPosCommand(obot, LiftSubsystem.PitchState.HOME),
                        new WaitCommand(250),
                        new BucketPosCommand(obot, LiftSubsystem.BucketState.SEMIOUT),
                        new WaitCommand(250),
                        new LinkagePosCommand(obot, LiftSubsystem.LinkageState.HOME),
                        new WaitCommand(250),
                        new BucketPosCommand(obot, LiftSubsystem.BucketState.IN),
                        new WaitCommand(250),
                        new BucketPosCommand(obot, LiftSubsystem.BucketState.HOME),
                        new TriggerPosCommand(obot, LiftSubsystem.TriggerState.TRANSFER),
                        new WaitCommand(250),
                        new LiftPosCommand(obot,LiftSubsystem.LiftState.HOME)
                )
        );
    }
}
