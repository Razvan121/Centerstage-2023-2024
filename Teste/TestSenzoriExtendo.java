package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;

@TeleOp
public class TestSenzoriExtendo extends LinearOpMode {

    RevColorSensorV3Ex senzor1,senzor2;
    double dist1,dist2;

    @Override
    public void runOpMode() throws InterruptedException {
        senzor1 = hardwareMap.get(RevColorSensorV3Ex.class, "intakeSensorLeft");
        senzor1.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        senzor2 = hardwareMap.get(RevColorSensorV3Ex.class, "intakeSensorRight");
        senzor2.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);

        waitForStart();
        while (opModeIsActive()) {
            dist1 = senzor1.getDistance(DistanceUnit.CM);
            dist2 = senzor2.getDistance(DistanceUnit.CM);

            telemetry.addData("intakeLeft", dist1);
            telemetry.addData("intakeRight", dist2);
            telemetry.update();
        }
    }
}
