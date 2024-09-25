package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;

@TeleOp
public class testOdo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(!isStopRequested()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_x,
                            -gamepad1.left_stick_y,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x ",poseEstimate.getX());
            telemetry.addData("y ",poseEstimate.getY());
            telemetry.addData("heading ",poseEstimate.getHeading());
            telemetry.update();
        }

    }
}
