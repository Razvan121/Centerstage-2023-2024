package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;

@TeleOp
@Config

public class TestAirplaneTrigger extends OpMode {

    NEDServo airplane;
    public static double airplane_pos=0.124;

    @Override
    public void init() {
        airplane = new NEDServo(hardwareMap.get(Servo.class,"airplane"));
    }

    @Override
    public void loop() {
        if(airplane_pos != 0.124)
            airplane.setPosition(airplane_pos);
    }
}
