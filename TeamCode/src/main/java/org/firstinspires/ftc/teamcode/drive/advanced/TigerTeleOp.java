package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */




/* Notes
    arm encoder upright:  4338
    lift encoder -3253
    arm when lifted:  2000


 */



@TeleOp(group = "advanced")
public class TigerTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double telemetryLevel = 5;

        int armTargetforLift = 4338;
        int liftTargetforLift = -3253;
        int armLifted = 2000;
        int liftLifted = 0;

        // Our Variables
        double armSpeedMultiplier = 0.5;
        double liftSpeedMultiplier = 0.9;



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
//        NormalizedColorSensor lineFinder = hardwareMap.get(NormalizedColorSensor.class, "lineFinder");

        DcMotor lift = hardwareMap.dcMotor.get("lift");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        lift.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        arm.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        //set arm and lift to use encoders...assume starting position
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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

  //emergency reset for arm and lift encoders
            if (gamepad2.back) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }

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
                arm.setPower((gamepad2.left_trigger - gamepad2.right_trigger - .15) * armSpeedMultiplier);
            } else {
                arm.setPower((gamepad2.left_trigger - gamepad2.right_trigger) * armSpeedMultiplier);
            }
            lift.setPower((gamepad1.left_trigger-gamepad1.right_trigger)*liftSpeedMultiplier );


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
                wristGrip.setPosition(.75);
            }

            if (gamepad2.y) {
                wristGrip.setPosition(.36);
            }

            if (gamepad2.x) {
                wristGrip.setPosition(.69);//..64
            }

            //prep for hanging
            if (gamepad2.dpad_left) {
                wristGrip.setPosition(.5);
                arm.setTargetPosition(armTargetforLift);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                lift.setTargetPosition(liftTargetforLift);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
            }
            //ready to hang
            if (gamepad2.dpad_right) {
                arm.setTargetPosition(armLifted);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                lift.setTargetPosition(liftLifted);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                endgame = true;

            }

            //reset motors if hang doesn't work right
            if (gamepad2.dpad_up) {
                endgame = false;
                arm.setPower(0);
                lift.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

// color sensor stuff
//            NormalizedRGBA colors = lineFinder.getNormalizedColors();
//            telemetry.addData("Color Red", colors.red);
//            telemetry.addData("Color Blue", colors.blue);
//currently all it does is report the red and blue values from the color sensor for testing

            //Range finder stuff
            telemetry.addData("GripDistance", gripDistance.getDistance(DistanceUnit.MM));
//currently all it does is report the distance from the gripper sensor in mm



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

            if (telemetryLevel >2) {
               // Print pose to telemetry
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
            }

            if (telemetryLevel >3) {
                telemetry.addData("Arm Encoder", arm.getCurrentPosition());
                telemetry.addData("Lift Encoder", lift.getCurrentPosition());
            }

            if (telemetryLevel >4) {
            //    telemetry.addData("x", poseEstimate.getX());
            //    telemetry.addData("y", poseEstimate.getY());
            }
            telemetry.update();
        }
    }
}