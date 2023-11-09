package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//This is the new field centric code from gm0.com
//uses the new IMU module to track rotation/position

//test
@TeleOp
public class Airplanetest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        boolean endgame = false;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
 //       frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
 //       backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo launcher = hardwareMap.get(Servo.class, "launcher");

        DistanceSensor gripDistance = hardwareMap.get(DistanceSensor.class, "gripDistance");
        ColorSensor lineFinder = hardwareMap.get(ColorSensor.class, "lineFinder");
        

        DcMotor lift = hardwareMap.dcMotor.get("lift");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        lift.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        arm.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        double speedMultiplier = .7;


        //initialize the launch lock
        launcher.setPosition(.2);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
 //here's our loop
            if (gamepad1.y) {  //launch drone
                launcher.setPosition(0);
            }
            if (gamepad1.a) {  //launch drone
                launcher.setPosition(.2);
            }



            if (gamepad1.x) {  //declare endgame for arm power boost
                endgame = !endgame;
            }
            telemetry.addData("Endgame?",endgame);
            //test grippers

            telemetry.addData("PRESS Y to LAUNCH",0);
            telemetry.addData("PRESS A to LOAD",0);


            if (endgame) {
                arm.setPower((gamepad2.left_trigger - gamepad2.right_trigger - .15) * .5);
            } else {
                arm.setPower((gamepad2.left_trigger - gamepad2.right_trigger) * .5);
            }
            lift.setPower((gamepad1.left_trigger-gamepad1.right_trigger)*.90 );


        }
    }
}