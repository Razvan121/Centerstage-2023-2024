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

public class TestBucket_Trigger extends OpMode {
    public NEDServo leftBucket;
    public NEDServo rightBucket;
    public NEDServo leftTrigger;
    public NEDServo rightTrigger;
    public NEDServo solt;
    public static double posleft = 0.5521, posright = 0.5521,pos_trig_l=0.123,pos_trig_f=0.123,pos=0.345;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftBucket = new NEDServo(hardwareMap.get(Servo.class, "leftBucket"));
        rightBucket = new NEDServo(hardwareMap.get(Servo.class, "rightBucket"));
        leftTrigger = new NEDServo(hardwareMap.get(Servo.class, "leftTrigger"));
        rightTrigger = new NEDServo(hardwareMap.get(Servo.class,"rightTrigger"));
        solt = new NEDServo(hardwareMap.get(Servo.class,"Sort"));
        solt.setDirection(Servo.Direction.REVERSE);
        rightBucket.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(pos !=0.345)
        {
            solt.setPosition(pos);
        }
        if (posleft != 0.5521) {
            leftBucket.setPosition(posleft);
        }
        if(posright!=0.5521) {
            rightBucket.setPosition(posleft);

        }
        if(pos_trig_l!=0.123) {
            leftTrigger.setPosition(pos_trig_l);
        }
        if(pos_trig_f!=0.123)
        {
            rightTrigger.setPosition(pos_trig_f);
        }
    }
}
