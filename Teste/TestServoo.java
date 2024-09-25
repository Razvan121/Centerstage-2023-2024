package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;

@Config
@TeleOp
public class TestServoo extends OpMode {

    NEDServo servo;
    NEDServo ser;
    public static double pos=0.124;

    @Override
    public void init() {
        servo = new NEDServo(hardwareMap.get(Servo.class,"leftBucket"));
        ser = new NEDServo(hardwareMap.get(Servo.class,"turret"));
    }

    @Override
    public void loop() {
        if(pos != 0.124){
            servo.setPosition(pos);
            ser.setPosition(pos);
        }
    }
}
