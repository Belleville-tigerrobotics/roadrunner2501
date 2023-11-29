package org.firstinspires.ftc.teamcode.drive.advanced;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous(name="TigerAutoVisionSample", group="Auto")

public class TigerAutoVisionSample extends LinearOpMode{


    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection=null;
    private VisionPortal visionPortal;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        String curAlliance = "red";

        while (!opModeIsActive() && !isStopRequested()){
            element_zone = teamElementDetection.elementDetection(telemetry);

            if (gamepad1.x){
                curAlliance = "blue";
            }else if (gamepad1.b){
                curAlliance = "red";
            }
            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());


            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }




            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();

    }

}