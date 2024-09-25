package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotor;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotorEncoder;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileConstraints;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

@Config
public class Obot {

    public DcMotorEx LeftLiftMotor;
    public DcMotorEx RightLiftMotor;
    public DcMotorEx IntakeLiftMotor;
    public DcMotorEx Active;
    public NEDMotorEncoder liftEncoder;
    public NEDMotorEncoder intakeEncoder;
    public NEDMotor Intake;
    public NEDMotor Lift;
    public IntakeSubsystem intakeSubsystem;
    public LiftSubsystem liftSubsystem;



    public NEDServo Wrist;

    public NEDServo turret;
    public NEDServo WristOuttake;
    public NEDServo PitchOuttake;
    public NEDServo linkage;
    public NEDServo angle;
    public NEDServo leftBucket;
    public NEDServo rightBucket;
    public NEDServo leftTrigger;
    public NEDServo rightTrigger;
    public NEDServo capac;
    public NEDServo airplane;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    /**
     * Voltage timer and voltage value.
     */
    private ElapsedTime voltageTimer;
    private double voltage = 0.0;
    public double sensor=0.0;
    private VoltageSensor voltageSensor;

    /**
     * Singleton variables.
     */
    private static Obot instance = null;
    public boolean enabled;

    public List<LynxModule> modules;
    public DoubleSupplier volt;

    private ArrayList<NEDSubsystem> subsystems;

    /**
     * Creating the singleton the first time, instantiating.
     */
    public static Obot getInstance() {
        if (instance == null) {
            instance = new Obot();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Created at the start of every OpMode.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     * @param telemetry Saved for later in the event FTC Dashboard used
     */
    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.subsystems = new ArrayList<>();

        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        try {
            modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch(Exception e) {}

        liftSubsystem = new LiftSubsystem();
        intakeSubsystem = new IntakeSubsystem();


        //INTAKE//
        IntakeLiftMotor = hardwareMap.get(DcMotorEx.class,"intakeLiftMotor");
        intakeEncoder = new NEDMotorEncoder(new MotorEx(hardwareMap, "intakeLiftMotor").encoder);
        Wrist = new NEDServo(hardwareMap.get(Servo.class,"Wrist"));
        Active = hardwareMap.get(DcMotorEx.class,"active");
        capac = new NEDServo(hardwareMap.get(Servo.class,"capac"));


        //OUTTAKE//
        LeftLiftMotor = hardwareMap.get(DcMotorEx.class,"leftLiftMotor");
        RightLiftMotor = hardwareMap.get(DcMotorEx.class,"rightLiftMotor");
        LeftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEncoder = new NEDMotorEncoder(new MotorEx(hardwareMap, "leftLiftMotor").encoder);

        leftTrigger = new NEDServo(hardwareMap.get(Servo.class, "leftTrigger"));
        rightTrigger = new NEDServo(hardwareMap.get(Servo.class,"rightTrigger"));
        leftBucket = new NEDServo(hardwareMap.get(Servo.class,"leftBucket"));
        rightBucket = new NEDServo(hardwareMap.get(Servo.class,"rightBucket"));
        leftBucket.setDirection(Servo.Direction.REVERSE);
        PitchOuttake = new NEDServo(hardwareMap.get(Servo.class,"PitchOuttake"));
        WristOuttake = new NEDServo(hardwareMap.get(Servo.class,"WristOuttake"));
        turret = new NEDServo(hardwareMap.get(Servo.class,"turret"));
        linkage = new NEDServo(hardwareMap.get(Servo.class,"linkage"));
        angle = new NEDServo(hardwareMap.get(Servo.class,"angle"));
        airplane = new NEDServo(hardwareMap.get(Servo.class,"airplane"));



        voltageTimer = new ElapsedTime();
       // PhotonCore.enable();

       // voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        //volt=this::getVoltage;
        this.Intake = new NEDMotor(IntakeLiftMotor,intakeEncoder)
                .setPIDController(new PIDController(0.04, 0.0, 0.0004))
                .setMotionProfile(0, new ProfileConstraints(10000, 10000, 10000));
        this.Lift = new NEDMotor(LeftLiftMotor,RightLiftMotor, liftEncoder)
                .setPIDController(new PIDController(0.03, 0.0, 0.00025))
                .setMotionProfile(0, new ProfileConstraints(10000, 10000, 10000));


    }

    public void read() {
        intakeSubsystem.read();
        liftSubsystem.read();
    }

    public void write() {
        intakeSubsystem.write();
        liftSubsystem.write();
    }
    public void periodic() {
       /* if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }
        volt=this::getVoltage;*/
        intakeSubsystem.periodic();
        liftSubsystem.periodic();

    }

    public void reset() {
    }

    public void clearBulkCache() {
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();
    }

    public void addSubsystem(NEDSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    public double getVoltage() {
        return voltage;
    }


}