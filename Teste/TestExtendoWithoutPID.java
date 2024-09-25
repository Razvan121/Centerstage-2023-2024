package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class TestExtendoWithoutPID extends LinearOpMode {

    DcMotorEx motor;
    int curr_pos;
    int position;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "intakeLiftMotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while(!isStopRequested()){
            curr_pos=(int)motor.getCurrentPosition();

            if(gamepad1.a){
                motor.setTargetPosition(0);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }
            if(gamepad1.b){
                motor.setTargetPosition(600);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }
            if(gamepad1.y){
                motor.setTargetPosition(1000);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }
            if(gamepad1.dpad_left){
                motor.setTargetPosition(curr_pos+100);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }
            if(gamepad1.dpad_right){
                motor.setTargetPosition(curr_pos-100);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }
        }
    }

}
