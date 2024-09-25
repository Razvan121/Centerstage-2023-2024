package org.firstinspires.ftc.teamcode.NEDRobot.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.ActiveStateCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.AnglePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.BucketPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.CapacPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.ExtendoPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.IntakePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.LiftPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.PitchPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.TriggerPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.WristOuttakePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.WristPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Commands.DepositCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Commands.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Commands.HomeCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Commands.TransferCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Commands.TransferExtendoCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import org.firstinspires.ftc.teamcode.photoncore.PhotonCore;

import java.util.function.BooleanSupplier;


@Config
@TeleOp(name = "CommandTeleOpOffSeason", group = "National")

public class CommandTeleopOffSeason extends CommandOpMode {
    private final Obot obotV2 = Obot.getInstance();

    int i = 0, j = 0;
    BooleanSupplier button;
    BooleanSupplier button1;
    double contor = 0;
    double sumLoop;
    public RevColorSensorV3Ex intakeSensorLeft;
    public RevColorSensorV3Ex intakeSensorRight;

    private double distance_to_intake = 1.2;
    private double loopTime = 0;
    private SampleMecanumDrive drive;
    double leftSen, rightSen;
    boolean Scoring = false;
    boolean Outtaking = false;
    boolean In_Intake = false, ClosedLeft = false, ClosedRight = false;
    boolean isHome = true;
    private GamepadEx GamepadEx1, GamepadEx2;
    ElapsedTime voltage_timer;
    VoltageSensor voltageSensor;
    double voltage;


    @Override
    public void initialize() {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeSensorLeft = hardwareMap.get(RevColorSensorV3Ex.class,"intakeSensorLeft");
        intakeSensorLeft.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        intakeSensorRight = hardwareMap.get(RevColorSensorV3Ex.class,"intakeSensorRight");
        intakeSensorRight.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        CommandScheduler.getInstance().reset();
        obotV2.init(hardwareMap,telemetry);
        obotV2.read();
        obotV2.periodic();
        obotV2.intakeSubsystem.update(IntakeSubsystem.WristState.HOME);
        obotV2.intakeSubsystem.update(IntakeSubsystem.CapacState.CLOSED);
        obotV2.Intake.setMotionProfileTargetPosition(10);
        obotV2.liftSubsystem.update(LiftSubsystem.LinkageState.HOME);
        obotV2.liftSubsystem.update(LiftSubsystem.BucketState.HOME);
        obotV2.liftSubsystem.update(LiftSubsystem.PitchState.HOME);
        obotV2.liftSubsystem.update(LiftSubsystem.LiftState.HOME);
        obotV2.airplane.setPosition(0.7);
        obotV2.liftSubsystem.update(LiftSubsystem.TurretState.OUT);
        obotV2.liftSubsystem.update(LiftSubsystem.AngleState.OUT);
        obotV2.liftSubsystem.update(LiftSubsystem.TriggerState.TRANSFER);
        obotV2.liftSubsystem.update(LiftSubsystem.WristState.HORIZONTAL_TRANSFER);
        drive = new SampleMecanumDrive(hardwareMap);
        button = () -> gamepad1.start;
        button1 = () ->gamepad1.start;
        GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx2 = new GamepadEx(gamepad2);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage_timer = new ElapsedTime();

        GamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                        .whenPressed(
                                new DepositCommand(obotV2)
                       );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.B)
                        .whenPressed(
                                new SequentialCommandGroup(
                                        new DepositCommand(obotV2),
                                        new WaitCommand(1000),
                                        new WristOuttakePosCommand(obotV2, LiftSubsystem.WristState.OBLIC_RIGHT)
                                )
                        );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new SequentialCommandGroup(
                                new DepositCommand(obotV2),
                                new WaitCommand(1000),
                                new WristOuttakePosCommand(obotV2, LiftSubsystem.WristState.OBLIC_RIGHT)
                        )
                );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                        .whenPressed(
                                new TransferExtendoCommand(obotV2)
                        );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(
                        new SequentialCommandGroup(
                                new ExtendIntakeCommand(obotV2)
                        )
                );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                        .whenPressed(
                                new SequentialCommandGroup(
                                        new TransferCommand(obotV2),
                                        new InstantCommand(() -> In_Intake = false)
                                )
                        );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new ActiveStateCommand(obotV2, IntakeSubsystem.ActiveState.START),
                                new WaitCommand(200),
                                new WristPosCommand(obotV2, IntakeSubsystem.WristState.OUT_RETRACTED)
                        )
                );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(
                        new SequentialCommandGroup(
                                new HomeCommand(obotV2),
                                new InstantCommand(() -> obotV2.liftSubsystem.setBackdropHeight(1))
                        )
                );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(
                                new SequentialCommandGroup(
                                        new InstantCommand(()->In_Intake=false),
                                        new InstantCommand(()->obotV2.liftSubsystem.incrementBackdropHeight(1)),
                                        new InstantCommand(()->obotV2.Lift.setMotionProfileTargetPosition(obotV2.liftSubsystem.getBackdropHeight())),
                                        new InstantCommand(()->obotV2.PitchOuttake.setPosition(obotV2.liftSubsystem.getPitchBackdropHeight()))
                                )
                        );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(()->In_Intake=false),
                                new InstantCommand(()->obotV2.liftSubsystem.incrementBackdropHeight(-1)),
                                new InstantCommand(()->obotV2.Lift.setMotionProfileTargetPosition(obotV2.liftSubsystem.getBackdropHeight())),
                                new InstantCommand(()->obotV2.PitchOuttake.setPosition(obotV2.liftSubsystem.getPitchBackdropHeight()))

                        )
                );
        GamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                        .whenPressed(
                                new InstantCommand(() -> obotV2.intakeSubsystem.update(IntakeSubsystem.CapacState.OPENED))
                        );
        GamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenPressed(
                                new InstantCommand(() -> obotV2.intakeSubsystem.update(IntakeSubsystem.CapacState.CLOSED))
                        );
        GamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                        .whenPressed(
                                new BucketPosCommand(obotV2, LiftSubsystem.BucketState.IN)
                        );
        GamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whenPressed(
                                new SequentialCommandGroup(
                                        new LiftPosCommand(obotV2, LiftSubsystem.LiftState.HOME),
                                        new WaitCommand(300),
                                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.HANG)
                                )
                        );
        GamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(
                                new SequentialCommandGroup(
                                        new TriggerPosCommand(obotV2, LiftSubsystem.TriggerState.CLOSE),
                                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.LOW),
                                        new WaitCommand(300),
                                        new AnglePosCommand(obotV2, LiftSubsystem.AngleState.OUT),
                                        new WaitCommand(300),
                                        new PitchPosCommand(obotV2, LiftSubsystem.PitchState.LOW)
                                )
                        );


        PhotonCore.enable();

        obotV2.read();
    }
    @Override
    public void run() {
        obotV2.clearBulkCache();
        obotV2.read();

        if(gamepad1.left_trigger>0.9)
        {
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> obotV2.liftSubsystem.update(LiftSubsystem.TriggerState.OPEN)));
        }
        if(gamepad2.left_trigger>0.9)
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> obotV2.leftTrigger.setPosition(0.2)));
        if(gamepad2.right_trigger>0.9)
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> obotV2.rightTrigger.setPosition(0.9)));
        if(gamepad2.left_stick_y > 0)
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> obotV2.Active.setPower(0.7)));
        if (Scoring) {
                drive.setWeightedDrivePower(
                        new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * 0.2,
                                dead(scale(-GamepadEx1.getLeftX(), 0.6), 0) * 0.2,
                                -GamepadEx1.getRightX() * 0.3
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.353 : 1),
                                dead(scale(-GamepadEx1.getLeftX(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.4 : 1),
                                -GamepadEx1.getRightX() * (gamepad1.right_trigger > 0.5 ? 0.3 : 1)
                        )
                );
            }
        if(gamepad2.start)
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> obotV2.airplane.setPosition(0.15)));
        if(In_Intake) {
            Outtaking=false;
            leftSen=intakeSensorLeft.getDistance(DistanceUnit.CM);
            rightSen=intakeSensorRight.getDistance(DistanceUnit.CM);
            if (leftSen<distance_to_intake && !ClosedLeft) {
                i++;
            }
            if(i==4)
            {
                i=0;
                ClosedLeft=true;
            }
            if (rightSen<distance_to_intake && !ClosedRight) {
                j++;
            }
            if(j==4)
            {
                j=0;
                ClosedRight=true;
            }
            if (ClosedRight && ClosedLeft) {
                ClosedRight=false;
                ClosedLeft=false;
                In_Intake = false;
                //CommandScheduler.getInstance().schedule(new TransferCommand(obotV2));
                gamepad1.rumble(1000);
                isHome = true;
            }
        }
        else
        {
            i=0;j=0;ClosedLeft=false;ClosedRight=false;
        }
        //gamepad1.rumble(200); -> cod pentru a vibra controller ul

        super.run();
        obotV2.periodic();
        telemetry.addData("Pos", obotV2.liftSubsystem.getBackdropHeight());
        telemetry.addData("In_Intake",In_Intake);
        telemetry.addData("LeftSen",leftSen);
        telemetry.addData("RightSen",rightSen);
        telemetry.addData("isHome",isHome);
        telemetry.addData("i",i);
        telemetry.addData("j",j);
        telemetry.addData("voltage",voltage);


        double loop = System.nanoTime();
        contor++;
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        sumLoop+=1000000000 / (loop - loopTime);
            loopTime = loop;
        telemetry.update();
        obotV2.write();

    }
    @Override
    public void reset() {
           CommandScheduler.getInstance().reset();
        obotV2.reset();
    }

    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double dead(double x, double k) {
        return Math.abs(x) > k ? x : 0;
    }
}

