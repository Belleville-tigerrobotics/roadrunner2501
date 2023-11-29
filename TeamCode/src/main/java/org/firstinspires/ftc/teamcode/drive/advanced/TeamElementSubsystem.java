package org.firstinspires.ftc.teamcode.drive.advanced;
// Got this from here:  https://github.com/FTC14133/FTC14133-2023-2024/blob/Detection-TeamElement/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/TeamElementDetection/TeamElementSubsystem.java

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.drive.advanced.SplitAveragePipeline;

public class TeamElementSubsystem {
    OpenCvCamera camera;
    SplitAveragePipeline splitAveragePipeline;
    int camW = 640;
    int camH = 480;


    int zone = 1;

    public TeamElementSubsystem(HardwareMap hardwareMap){
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        splitAveragePipeline = new SplitAveragePipeline();
        camera.setPipeline(splitAveragePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
  //              camera.startStreaming(camW, camH);


            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void setAlliance(String alliance){
        splitAveragePipeline.setAlliancePipe(alliance);
    }

    public int elementDetection(Telemetry telemetry) {
        zone = splitAveragePipeline.get_element_zone();
        telemetry.addData("Element Zone", zone);
        return zone;
    }
}