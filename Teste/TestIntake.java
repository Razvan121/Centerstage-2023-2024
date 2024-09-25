package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotorEncoder;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;
import org.firstinspires.ftc.teamcode.NEDRobot.util.MathUtil;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileConstraints;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileState;

@Config
@TeleOp(name="Test-Intake")
public class TestIntake extends LinearOpMode {
    public NEDServo leftIntake;
    public NEDServo rightIntake;
    public NEDServo Wrist;
    public NEDServo intakeClaw;

    public static double pos = 0.5521,pos_wrist=0.001,pos_claw=1.123;
    public DcMotorEx intakeLiftMotor;

    public NEDMotorEncoder intakeEncoder;
    private double position;
    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private double voltage = 0.0;
    public static double targetposition,realtarget;
    private double pTargetPosition = 0.0;
    private double pPower =0.0;
    public static double p=0.00,i=0.000,d=0.0000;
    public PIDController controller;
    private double loopTime = 0.0;
    private ElapsedTime timer;
    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private ProfileState state;
    public static double velo=10000,accel=10000,decel=10000;
    private double power;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        intakeLiftMotor = hardwareMap.get(DcMotorEx.class,"intakeLiftMotor");
        intakeEncoder = new NEDMotorEncoder(new MotorEx(hardwareMap, "intakeLiftMotor").encoder);

        leftIntake = new NEDServo(hardwareMap.get(Servo.class,"leftIntake"));
        rightIntake = new NEDServo(hardwareMap.get(Servo.class,"rightIntake"));
        intakeClaw = new NEDServo(hardwareMap.get(Servo.class, "intakeClaw"));
        Wrist = new NEDServo(hardwareMap.get(Servo.class,"Wrist"));
        rightIntake.setDirection(Servo.Direction.REVERSE);

        controller = new PIDController(p,i,d);
        controller.setPID(p,i,d);
        constraints = new ProfileConstraints(velo,accel,decel);
        profile = new AsymmetricMotionProfile(0,0,constraints);
        timer = new ElapsedTime();
        state =new ProfileState();
        voltageTimer = new ElapsedTime();
        voltageTimer.reset();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
        intakeEncoder.encoder.reset();
        waitForStart();
        while(!isStopRequested() && opModeIsActive())
        {
            if (pos != 0.5521) {
                leftIntake.setPosition(pos/360);
                rightIntake.setPosition(pos/360);
            }
            if(pos_wrist!=0.001)
                Wrist.setPosition(pos_wrist);
            if(pos_claw!=1.123)
                intakeClaw.setPosition(pos_claw);
            ///Read
            position=intakeEncoder.getPosition();

            if(p!=controller.getP() || i!=controller.getI() || d!=controller.getD())
                controller.setPID(p,i,d);
            if(gamepad1.a)
            {
                constraints.accel=accel;
                constraints.decel=decel;
                constraints.velo=velo;
                realtarget=targetposition;
                profile = new AsymmetricMotionProfile(position,realtarget,constraints);
                timer.reset();
            }
            ////Periodic

            state = profile.calculate(timer.time());
            realtarget = state.x;
            power=controller.calculate(position,realtarget);
            power= MathUtil.clamp(power,-1,1);
            if (voltageTimer.seconds() > 5) {
                voltage = voltageSensor.getVoltage();
                voltageTimer.reset();
            }
            ///Write
            if (Math.abs(targetposition - pTargetPosition) > 0.005 ||
                    Math.abs(power - pPower) > 0.005) {
                double correction = 1.0;
                if (voltage !=0) correction = 12.0 / voltage;
                intakeLiftMotor.setPower(power*correction);

            }
            pTargetPosition = targetposition;
            pPower = power;


            telemetry.addData("Position",position);
            telemetry.addData("TargetPosition",realtarget);
            telemetry.addData("Current",intakeLiftMotor.getCurrent(CurrentUnit.AMPS));
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
    }
    public double getTargetPosition() {
        return targetposition;
    }

}
