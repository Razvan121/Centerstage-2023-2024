package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="SEnsor")
public class TestSensor extends LinearOpMode {
    DistanceSensor awd;
    public boolean time_to_intake=true;

    public double curr_pos;
    public static double Q=0.3,R=5;
    public static int N=3;
    KalmanFilter filter = new KalmanFilter(Q,R,N);
    public double filtered_pos;
    public static int targetPosition=1;
    public int i=0;
    @Override
    public void runOpMode() throws InterruptedException{
        awd = hardwareMap.get(DistanceSensor.class, "intakeSensor");
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
//        awd.initialize();
        while(opModeIsActive() && !isStopRequested()) {
            curr_pos = awd.getDistance(DistanceUnit.CM);
            filtered_pos = filter.estimate(curr_pos);
            if(filtered_pos>targetPosition-0.4 && filtered_pos<targetPosition+0.34 && time_to_intake)
            {
                i++;
            }
            if(i==3 && time_to_intake) {
                time_to_intake=false;
                i=0;
            }
            if(gamepad1.a)
            {
               time_to_intake=true;
            }
            telemetry.addData("i",i);
            telemetry.addData("filtered_value",filtered_pos);
            telemetry.addData("Real_value",curr_pos);
            telemetry.update();
        }
    }


}