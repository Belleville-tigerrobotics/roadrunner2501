package org.firstinspires.ftc.teamcode.drive.advanced;
//got this from here: https://github.com/FTC14133/FTC14133-2023-2024/blob/Detection-TeamElement/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/TeamElementDetection/TeamElementSubsystem.java

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.opencv.core.Core;
        import org.opencv.core.Mat;
        import org.opencv.core.Rect;
        import org.opencv.core.Scalar;
        import org.opencv.imgproc.Imgproc;
        import org.openftc.easyopencv.OpenCvPipeline;

        import java.util.Arrays;
        import java.util.List;


public class SplitAveragePipeline extends OpenCvPipeline {

    int CAMERA_WIDTH = 640/*800*/;
    int CAMERA_HEIGHT = 480/*448*/;

    List<Integer> ELEMENT_COLOR = Arrays.asList(255, 0, 0); //(red, green, blue)

    int line1x = CAMERA_WIDTH / 3;
    int line2x = (CAMERA_WIDTH / 3) * 2;

    //Telemetry telemetry;

    static int color_zone = 1;

    static double distance1;
    static double distance2;

    //public SplitAveragePipeline(/*Telemetry telemetry, */int iCAMERA_HEIGHT, int iCAMERA_WIDTH){
    //    //this.telemetry = telemetry;
    //    this.CAMERA_HEIGHT = iCAMERA_HEIGHT;
    //    this.CAMERA_WIDTH = iCAMERA_WIDTH;
    //}

    @Override
    public Mat processFrame(Mat input) {


        //input = input.submat(new Rect(0));

        //Defining Zones
        //Rect(top left x, top left y, bottom right x, bottom right y)
        //Mat zone1 = input.submat(new Rect(0, 0, line1x, CAMERA_HEIGHT));
        //Mat zone2 = input.submat(new Rect(line1x, 0, line2x - line1x, CAMERA_HEIGHT));
        //Mat zone3 = input.submat(new Rect(line2x, 0, CAMERA_WIDTH - line2x, CAMERA_HEIGHT));

 //       Mat zone1 = input.submat(new Rect(0, 180, 115, 115));
 //       Mat zone2 = input.submat(new Rect(316, 180, 115, 115));
 //       Mat zone3 = input.submat(new Rect(660, 180, 115, 115));

//        Mat zone1 = input.submat(new Rect(40, 50  , 80, 100));
//        Mat zone2 = input.submat(new Rect(550, 200, 120, 120));
  //      Mat zone3 = input.submat(new Rect(799, 0, 1, 1));

        Mat zone1 = input.submat(new Rect(70, 280  , 90, 80));
        Mat zone2 = input.submat(new Rect(470, 270, 105, 60));
 //       Mat zone3 = input.submat(new Rect(1, 1, 1, 1));


        //Averaging the colors in the zones
        Scalar avgColor1 = Core.mean(zone1);
        Scalar avgColor2 = Core.mean(zone2);
 //       Scalar avgColor3 = Core.mean(zone3);

        //Putting averaged colors on zones (we can see on camera now)
        zone1.setTo(avgColor1);
        zone2.setTo(avgColor2);
 //       zone3.setTo(avgColor3);

        distance1 = color_distance(avgColor1, ELEMENT_COLOR);
        distance2 = color_distance(avgColor2, ELEMENT_COLOR);
 //       double distance3 = color_distance(avgColor3, ELEMENT_COLOR);

        double max_distance = Math.min(distance1, distance2);

        if (max_distance == distance1){
            //telemetry.addData("Zone 1 Has Element", distance1);
            color_zone = 1;

        }else if (max_distance == distance2){
            //telemetry.addData("Zone 2 Has Element", distance2);
            color_zone = 2;
        }

        if (max_distance >200){  //must actually be zone 3 because the distance is bigger than 220 (not found)
            //telemetry.addData("Zone 2 Has Element", distance3);
            color_zone = 3;
        }

   //     telemetry.addData("\nZone 1 Color", avgColor1);
   //     telemetry.addData("Zone 2 Color", avgColor2);
   //     telemetry.addData("Zone 3 Color", avgColor3);

   //     telemetry.update();



        return input;
    }

    public double color_distance(Scalar color1, List color2){
        double r1 = color1.val[0];
        double g1 = color1.val[1];
        double b1 = color1.val[2];

        int r2 = (int) color2.get(0);
        int g2 = (int) color2.get(1);
        int b2 = (int) color2.get(2);

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }

    public void setAlliancePipe(String alliance){
        if (alliance.equals("red")){
            ELEMENT_COLOR = Arrays.asList(255, 0, 0);
        }else{ //blue
            ELEMENT_COLOR = Arrays.asList(0, 0, 255);
        }
    }

    public int get_element_zone(Telemetry telemetry){
            telemetry.addData("\nZone 1 Color", distance1);
            telemetry.addData("Zone 2 Color", distance2);

        return color_zone;
    }

}