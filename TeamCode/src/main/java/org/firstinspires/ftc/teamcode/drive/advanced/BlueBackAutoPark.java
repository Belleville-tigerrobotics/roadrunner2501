package org.firstinspires.ftc.teamcode.drive.advanced;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.wristfloorposition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

/**
 * Example opmode demonstrating how to hand-off the pose from your autonomous opmode to your teleop
 * by passing the data through a static class.
 * <p>
 * This is required if you wish to read the pose from odometry in teleop and you run an autonomous
 * sequence prior. Without passing the data between each other, teleop isn't completely sure where
 * it starts.
 * <p>
 * This example runs the same paths used in the SplineTest tuning opmode. After the trajectory
 * following concludes, it simply sets the static value, `PoseStorage.currentPose`, to the latest
 * localizer reading.
 * However, this method is not foolproof. The most immediate problem is that the pose will not be
 * written to the static field if the opmode is stopped prematurely. To work around this issue, you
 * need to continually write the pose to the static field in an async trajectory follower. A simple
 * example of async trajectory following can be found at
 * https://www.learnroadrunner.com/advanced.html#async-following
 * A more advanced example of async following can be found in the AsyncFollowingFSM.java class.
 * <p>
 * The other edge-case issue you may want to cover is saving the pose value to disk by writing it
 * to a file in the event of an app crash. This way, the pose can be retrieved and set even if
 * something disastrous occurs. Such a sample has not been included.
 */
@Autonomous(group = "advanced")
public class BlueBackAutoPark extends LinearOpMode {
    //   @Override

    public int element_zone = 1;
    private TeamElementSubsystem teamElementDetection=null;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//now adjust some of the drive constants

        Servo rightGrip = hardwareMap.get(Servo.class, "rightGrip");
        Servo leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        Servo wristGrip = hardwareMap.get(Servo.class, "wristGrip");

//set grips to holding position
        leftGrip.setPosition(0);
        rightGrip.setPosition(.3);

//set wrist to upgright for start
        wristGrip.setPosition(.36);//..64

//use a generic pose to start...most important here is 0 degrees for orientation
        Pose2d startPose = new Pose2d(10, 15, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        HardwareStart();

        String curAlliance = "blue";
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

        waitForStart();
        sleep(1000); //pause for detection
//now detect the object
        detectedZone = teamElementDetection.elementDetection(telemetry);
        telemetry.update();

        telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
        telemetry.addData("Found position ", detectedZone);
        telemetry.update();
        sleep(200);

        //setup speed limiter for roadrunner
        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(20),
                new AngularVelocityConstraint( 1)
        ));

        if (detectedZone==1) {
            sleep(100);

        //move forward
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    //                  .setVelConstraint(slowConstraint)
                    .forward(30)

                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj);
            if (isStopRequested()) return;
         //rotate
            drive.turn(Math.toRadians(95));

         //backup so that we can place the pixel
            Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .back(8)
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj2);  //back up a bit

            //now drop the pixel on the floor
            wristGrip.setPosition(wristfloorposition);      //now put the wrist down
            sleep(800);
            leftGrip.setPosition(0.15);
            sleep(800);
            //lift wrist
            wristGrip.setPosition(.36);//..64

            drive.turn(Math.toRadians(-95));

            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .strafeLeft(10)//make sure we dont' crash into the frame
                    .build();

            drive.followTrajectory(traj3);  //now push the item out of the way


            sleep(100);
 //backup to the wall
            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .back(30)
                    .build();

            drive.followTrajectory(traj4);  //now push the item out of the way
    //        drive.turn(Math.toRadians(90));

            sleep(9000);
            //now we can drive to park
            Trajectory traj6 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .strafeLeft(24*2+12)//forward 4 tiles from here should park us
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();

            drive.followTrajectory(traj6);  //now push the item out of the way
//

            rightGrip.setPosition(.1);
            wristGrip.setPosition(.5);
            sleep(500);
            wristGrip.setPosition(.36);
            sleep(500);


//we could add some stuff here to place the other pixel now that we're in front of the board
            //


        } else if (detectedZone== 2 ) {
/*--------------------------------------------------------------------------------------------
 *****  BLUE ZONE 2 BACKSTAGE ********
----------------------------------------------------------------------------------------------
*/
            // do roadrunner stuff here for zone 2
            //put down the claw first
            wristGrip.setPosition(wristfloorposition);//..64
            sleep(200);
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    .forward(27)
                    //                .splineTo(new Vector2d(-54,-42), Math.toRadians(0))

                    .build();
            if (isStopRequested()) return;

            drive.followTrajectory(traj);
            // once we're positioned, now let's drop the pixel--same for each location hopefully, so only need this part once
            sleep(200);
            //now let go of left grip
            leftGrip.setPosition(0.15);
            sleep(800);
            //lift wrist
            wristGrip.setPosition(.36);//..64



            // now let's get in position to be able to make it under the stage
            sleep(800);
            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                   //                  .setVelConstraint(slowConstraint)
                    .back(27) //TODO tune this
                    //                   //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            if (isStopRequested()) return;

            drive.followTrajectory(traj3);  //now push the item out of the way
 //           drive.turn(Math.toRadians(95)); //turn towards the backdrop
//now we can drive to park
            sleep(9000);

            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .strafeLeft(24*2 +15)//forward 4 tiles from here should park us
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            if (isStopRequested()) return;

            drive.followTrajectory(traj4);  //now push the item out of the way
//            drive.turn(Math.toRadians(-95));
//

            rightGrip.setPosition(.1);
            wristGrip.setPosition(.5);
            sleep(500);
            wristGrip.setPosition(.36);
            sleep(500);



        } else {
 /*--------------------------------------------------------------------------------------------
 *****  BLUE ZONE 3 BACKSTAGE  ********
----------------------------------------------------------------------------------------------
*/



            //do roadrunner stuff here for zone 3 (which will be the default if we don't detect anything
            //put down the claw first
            sleep(200);
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    //                  .setVelConstraint(slowConstraint)
                    .forward(30)

                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj);
            drive.turn(Math.toRadians(-99));
            Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .back(10)
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj2);  //back up a bit
            wristGrip.setPosition(wristfloorposition);      //now put the wrist down
            sleep(800);

            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .forward(6)
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj3);  //now push the item out of the way

            // once we're positioned, now let's drop the pixel--same for each location hopefully, so only need this part once
            sleep(200);
            //now let go of left grip
            leftGrip.setPosition(0.15);
            sleep(700);
            //lift wrist
            wristGrip.setPosition(.36);//..64
            sleep(400);
//now turn back
            drive.turn(Math.toRadians(99));

//now backup to the wall again
            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .back(30)//forward 4 tiles from here should park us
                    .build();
            drive.followTrajectory(traj4);  //now push the item out of the way
            sleep(9000);
            //now we can drive to park
            Trajectory traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .strafeLeft(24*2+14)//forward 4 tiles from here should park us
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();

            drive.followTrajectory(traj5);  //now push the item out of the way
//

            rightGrip.setPosition(.1);
            wristGrip.setPosition(.5);
            sleep(500);
            wristGrip.setPosition(.36);
            sleep(500);


//we could add some stuff here to place the other pixel now that we're in front of the board
            //

        }


        if (isStopRequested()) return;



//        sleep (1000);


        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}