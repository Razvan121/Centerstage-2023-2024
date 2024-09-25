package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;

@TeleOp
@Config

public class TestBucketFull_Trigger extends OpMode {
    public NEDServo leftBucket;
    public NEDServo rightBucket;
    public NEDServo leftTrigger;
    public NEDServo rightTrigger;
    public NEDServo PitchOuttake;
    public NEDServo wristOuttake;
    public NEDServo linkage;
    public NEDServo angle;
    public NEDServo turret;
    public static double pos = 0.124,pos_trig_left=0.124,pos_trig_right=0.124,pos_pitch=0.124,pos_wrist=0.124;
    public static double pos_linkage=0.124,pos_ange=0.124,pos_turret=0.124;

    @Override
    public void init() {

        leftBucket = new NEDServo(hardwareMap.get(Servo.class, "leftBucket"));
        rightBucket = new NEDServo(hardwareMap.get(Servo.class, "rightBucket"));
        rightTrigger = new NEDServo(hardwareMap.get(Servo.class, "rightTrigger"));
        leftTrigger = new NEDServo(hardwareMap.get(Servo.class, "leftTrigger"));
        linkage = new NEDServo(hardwareMap.get(Servo.class, "linkage"));
        angle = new NEDServo(hardwareMap.get(Servo.class, "angle"));
        turret = new NEDServo(hardwareMap.get(Servo.class, "turret"));
        PitchOuttake = new NEDServo(hardwareMap.get(Servo.class, "PitchOuttake"));
        wristOuttake = new NEDServo(hardwareMap.get(Servo.class, "WristOuttake"));
        leftBucket.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (pos !=  0.124){
            leftBucket.setPosition(pos/360);
            rightBucket.setPosition(pos/360);
        }
        if(pos_wrist != 0.124){
            wristOuttake.setPosition(pos_wrist);
        }
        if(pos_pitch != 0.124){
            PitchOuttake.setPosition(pos_pitch);
        }
        if(pos_linkage != 0.124){
            linkage.setPosition(pos_linkage);
        }
        if(pos_ange != 0.124){
            angle.setPosition(pos_ange);
        }
        if(pos_trig_right != 0.124){
            rightTrigger.setPosition(pos_trig_right);
        }
        if(pos_trig_left != 0.124){
            leftTrigger.setPosition(pos_trig_left);
        }
        if(pos_turret != 0.124){
            turret.setPosition(pos_turret);
        }

    }
}