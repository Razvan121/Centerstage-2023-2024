package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import org.firstinspires.ftc.teamcode.photoncore.PhotonCore;

@TeleOp
@Disabled
@Config
public class SensorPixel extends LinearOpMode {

        public RevColorSensorV3Ex intakeSensorLeft;
        public RevColorSensorV3Ex intakeSensorRight;
        double current_distance_cm;
        double previous_filtered_cm;
        public static double a=0,targetPosition=0;
        double filtered_distance;
        double min_distance;
        double max_distance;
        double average_distance,sum_distance;
        int cnt;

        public static double G;
        int contor=0;
        double loopTime,sumLoop,averageLoopTime;
      //  ClawSubsystem clawSubsystem;

        @Override
        public void runOpMode() throws InterruptedException {
                PhotonCore.enable();
                //clawSubsystem = new ClawSubsystem(hardwareMap,false);
                telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                intakeSensorLeft = hardwareMap.get(RevColorSensorV3Ex.class,"intakeSensorLeft");
                intakeSensorLeft.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
                intakeSensorRight = hardwareMap.get(RevColorSensorV3Ex.class,"intakeSensorRight");
                intakeSensorRight.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
                waitForStart();
                while(!isStopRequested())
                {
                        telemetry.addData("Left",intakeSensorLeft.getDistance(DistanceUnit.CM));
                        telemetry.addData("Right",intakeSensorRight.getDistance(DistanceUnit.CM));
                        /*filtered_distance= current_distance_cm *(1-a) + previous_filtered_cm*a;
                        previous_filtered_cm=current_distance_cm;

                        if(gamepad1.a)
                        {
                                min_distance=max_distance=average_distance=sum_distance=0;
                                cnt=0;
                        }
                        if(min_distance>filtered_distance)
                                min_distance=filtered_distance;
                        if(max_distance<filtered_distance)
                                max_distance=filtered_distance;
                        sum_distance+=filtered_distance;
                        cnt++;
                        average_distance=sum_distance/cnt;



                      //  if(filtered_distance>0 && filtered_distance<targetPosition)
                        //        clawSubsystem.update(ClawSubsystem.ClawState.CLOSE);
                      //  else
                       //         clawSubsystem.update(ClawSubsystem.ClawState.OPEN);
*/
                        double loop = System.nanoTime();
                        contor++;
                        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
                        loopTime = loop;
                        sumLoop+=loopTime;
                        averageLoopTime=sumLoop/contor;
                        telemetry.addData("AVERAGE_Loop",averageLoopTime);

                       // telemetry.addData("Raw_distance_cm",current_distance_cm);
                        //telemetry.addData("Filtered_Distance",filtered_distance);
                        //telemetry.addData("Min",min_distance);
                        //telemetry.addData("Max",max_distance);
                        //telemetry.addData("Avg",average_distance);
                        telemetry.update();
                }

        }
}
