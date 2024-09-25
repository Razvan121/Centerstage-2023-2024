package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;

@TeleOp
@Config

public class TestIntakeFull_Wrist_Active extends OpMode {

    public NEDServo Wrist;
    public NEDServo capac;
    public DcMotorEx active;

    public static double pos = 0.5521,pos_wrist=0.001,pos_capac=0.124,motor_power=0.124;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Wrist = new NEDServo(hardwareMap.get(Servo.class,"Wrist"));
        capac = new NEDServo(hardwareMap.get(Servo.class,"capac"));
        active = hardwareMap.get(DcMotorEx.class,"active");
    }

    @Override
    public void loop() {
        if(pos_wrist!=0.001)
            Wrist.setPosition(pos_wrist);
        if(motor_power != 0.124)
            active.setPower(motor_power);
        if(pos_capac != 0.124)
            capac.setPosition(pos_capac);
    }
}
