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
public class BlueFrontAutoParkCenterAll extends LinearOpMode {
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


        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: 10, y: 15, and facing 90 degrees (turned counter-clockwise)
//        Pose2d startPose = new Pose2d(-66, -42, Math.toRadians(0));
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
        sleep(1000); //pause to make sure we detect
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
  /*--------------------------------------------------------------------------------------------
                *****  BLUE ZONE 1 FRONT ********
----------------------------------------------------------------------------------------------
                */

            //do roadrunner stuff here for zone 1
            //put down the claw first
//            wristGrip.setPosition(wristfloorposition);//..64
            sleep(200);
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    //                  .setVelConstraint(slowConstraint)
                    .forward(30)

                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj);
            if (isStopRequested()) return;
            drive.turn(Math.toRadians(95));
            Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .back(7)
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            if (isStopRequested()) return;
            drive.followTrajectory(traj2);  //back up a bit
            wristGrip.setPosition(wristfloorposition);      //now put the wrist down
            sleep(800);
            leftGrip.setPosition(0.15);
            sleep(800);
            //lift wrist
            wristGrip.setPosition(.36);//..64

            drive.turn(Math.toRadians(200));


            sleep(200);
            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .strafeLeft(40)
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            if (isStopRequested()) return;
            drive.followTrajectory(traj3);  //now push the item out of the way

            // once we're positioned, now let's drop the pixel--same for each location hopefully, so only need this part once
            //         sleep(200);
            //now let go of left grip
            //           leftGrip.setPosition(0.15);
            //           sleep(800);
            //lift wrist
            //           wristGrip.setPosition(.36);//..64

//now we can drive to park
            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .back(24*4-10)//forward 4 tiles from here should park us
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj4);  //now push the item out of the way

            drive.turn(Math.toRadians(-125));

            //drop the other pixel
            wristGrip.setPosition(.6);
            sleep(500);
            rightGrip.setPosition(.1);
            sleep(400);
            wristGrip.setPosition(.36);
            sleep(500);



            drive.turn(Math.toRadians(125));

            Trajectory traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .back(6)//forward 4 tiles from here should park us
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            if (isStopRequested()) return;

            drive.followTrajectory(traj5);  //now push the item out of the way
            drive.turn(Math.toRadians(95));



//we could add some stuff here to place the other pixel now that we're in front of the board
            //


        } else if (detectedZone== 2 ) {
  /*--------------------------------------------------------------------------------------------
                *****  BLUE ZONE 2 FRONT ********
----------------------------------------------------------------------------------------------
                */

            // do roadrunner stuff here for zone 2
            //put down the claw first
            wristGrip.setPosition(wristfloorposition);//..64
            sleep(200);
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    .forward(29)
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
            drive.turn(Math.toRadians(95)); //turn towards the backdrop

            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                   //                  .setVelConstraint(slowConstraint)
                    .strafeRight(2)
                    //
                    .build();
            if (isStopRequested()) return;
            drive.followTrajectory(traj3);

            Trajectory traj3a = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                   //                  .setVelConstraint(slowConstraint)
                    .back(16)
                    //
                    .build();
            if (isStopRequested()) return;
            drive.followTrajectory(traj3a);

            Trajectory traj3b = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                   //                  .setVelConstraint(slowConstraint)
                    .strafeRight(40)
                    //
                    .build();
            if (isStopRequested()) return;
            drive.followTrajectory(traj3b);



            //now we can drive to park
            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .forward(24*4)//forward 3 tiles from here should park us
                    .build();
            if (isStopRequested()) return;
            drive.followTrajectory(traj4);  //now push the item out of the way

            drive.turn(Math.toRadians(30));
//drop the other pixel
            wristGrip.setPosition(.6);
            sleep(400);
            rightGrip.setPosition(.1);
            sleep(200);
            wristGrip.setPosition(.36);
            sleep(500);
            drive.turn(Math.toRadians(-30));

            drive.turn(Math.toRadians(-95));



            Trajectory traj6 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .strafeLeft(20)//forward 4 tiles from here should park us
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            if (isStopRequested()) return;
            drive.followTrajectory(traj6);  //now push the item out of the way


        } else {
  /*--------------------------------------------------------------------------------------------
                *****  BLUE ZONE 3 FRONT ********
----------------------------------------------------------------------------------------------
                */

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
            if (isStopRequested()) return;

            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .forward(6)
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj3);  //now push the item out of the way
            if (isStopRequested()) return;

            // once we're positioned, now let's drop the pixel--same for each location hopefully, so only need this part once
            sleep(200);
            //now let go of left grip
            leftGrip.setPosition(0.15);
            sleep(800);
            //lift wrist
            wristGrip.setPosition(.36);//..64
            if (isStopRequested()) return;

//now we can drive to park


            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .strafeLeft((38))//forward 4 tiles from here should park us
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))  //this would be cool!
                    .build();
            drive.followTrajectory(traj4);  //now push the item out of the way
  //          drive.turn(Math.toRadians(95));

            Trajectory traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
                    .back(24*3+14)//forward 4 tiles from here should park us
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
                    .build();
            if (isStopRequested()) return;
            drive.followTrajectory(traj5);  //now push the item out of the way

            drive.turn(Math.toRadians(190));

//drop the other pixel
            wristGrip.setPosition(.6);
            sleep(500);
            rightGrip.setPosition(.1);
            sleep(400);
            wristGrip.setPosition(.36);
            sleep(500);

            drive.turn(Math.toRadians(-95));


 //           Trajectory traj5b = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //                  .setVelConstraint(slowConstraint)
 //                   .strafeLeft(24*1)//forward 4 tiles from here should park us
                    //                 .splineTo(new Vector2d(-54,-42),Math.toRadians(0))
 //                   .build();
            if (isStopRequested()) return;
 //           drive.followTrajectory(traj5b);

//we could add some stuff here to place the other pixel now that we're in front of the board
            //

        }



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}