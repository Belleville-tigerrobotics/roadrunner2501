package org.firstinspires.ftc.teamcode.drive.advanced;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.advanced.TeamElementSubsystem;


@Autonomous(name="AutoVisionSample", group="Auto")

public class AutoVisionSample extends LinearOpMode{


    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection=null;

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
        int detectedZone = 0;
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


            telemetry.update();
        }



        telemetry.addData("Object", "Passed waitForStart");
        telemetry.addData("press back to end", ".");

        //read the camera and get object position now


        telemetry.update();

//this testing loop to enable us to try different positions  we will remove this once we test the camera
        while (!gamepad1.back) {

            //check the detected items here
            //loop here to continue to report the detected color element location
            detectedZone = teamElementDetection.elementDetection(telemetry);
            telemetry.addData("press back to end", ".");

            telemetry.update();

        }
//  Here's the real autonomous code
        if (detectedZone==1) {
            //do roadrunner stuff here for zone 1

        } else if (detectedZone== 2 ) {
            // do roadrunner stuff here for zone 2

        } else {
            //do roadrunner stuff here for zone 3 (which will be the default if we don't detect anything

        }

        // once we're positioned, now let's drop the pixel--same for each location hopefully, so only need this part once




//DONE!
//        PoseStorage.currentPose = drive.getPoseEstimate();

    }

}