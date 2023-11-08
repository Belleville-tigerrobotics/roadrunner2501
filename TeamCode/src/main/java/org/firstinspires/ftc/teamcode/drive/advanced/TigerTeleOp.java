package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced")
public class TigerTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {





        // Initialize SampleMecanumDrive

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        //setup our other hardware
        DistanceSensor gripDistance = hardwareMap.get(DistanceSensor.class, "gripDistance");
        ColorSensor lineFinder = hardwareMap.get(ColorSensor.class, "lineFinder");


        DcMotor lift = hardwareMap.dcMotor.get("lift");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        lift.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        arm.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        double speedMultiplier = .7;

        Servo rightGrip = hardwareMap.get(Servo.class, "rightGrip");
        Servo leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        Servo wristGrip = hardwareMap.get(Servo.class, "wristGrip");
        Servo launcher = hardwareMap.get(Servo.class, "launcher");

        //initialize the launch lock
        launcher.setPosition(.2);

        boolean endgame = false ;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
   // start with our custom stuff
            if (gamepad1.y && gamepad1.a) {  //launch drone
                launcher.setPosition(0);
            }

            if (gamepad1.x) {  //declare endgame for arm power boost
                endgame = !endgame;
            }
            telemetry.addData("Endgame?",endgame);
            //test grippers
            //         leftGrip.setPosition(gamepad2.left_trigger);
            //        wristGrip.setPosition(.72);
            //         telemetry.addData("leftposition", gamepad2.left_trigger);
            //          telemetry.addData("wristgrip", gamepad2.right_trigger);


            if (endgame) {
                arm.setPower((gamepad2.left_trigger - gamepad2.right_trigger - .15) * .5);
            } else {
                arm.setPower((gamepad2.left_trigger - gamepad2.right_trigger) * .5);
            }
            lift.setPower((gamepad1.left_trigger-gamepad1.right_trigger)*.90 );


            //commands to operate the grippers
            if (gamepad2.left_bumper) { //operate left gripper
                leftGrip.setPosition(.15);
            } else {
                leftGrip.setPosition(0);
            }

            if (gamepad2.right_bumper) { //operate right gripper
                rightGrip.setPosition(.1);
            } else {
                rightGrip.setPosition(.3);
            }

            if (gamepad2.a) {  //this is the downon the ground position.  bigger is towards the ground
                wristGrip.setPosition(.73);
            }

            if (gamepad2.y) {
                wristGrip.setPosition(.36);
            }

            if (gamepad2.x) {
                wristGrip.setPosition(.64);
            }




            // here's the drive part of the code

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}