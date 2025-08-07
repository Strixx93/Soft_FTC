package vision; // Adjust to your teamcode package

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName; // Import this!
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;

import java.util.List;

@TeleOp(name = "Red Cube Detector", group = "Vision")
public class RedCubeDetectionOpMode extends LinearOpMode {

    private VisionPortal visionPortal;
    private RedCubeDetectorProcessor redCubeProcessor;

    @Override
    public void runOpMode() {
        redCubeProcessor = new RedCubeDetectorProcessor();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(redCubeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1")) // <--- UNCOMMENT AND USE THIS LINE
                // .setCamera(BuiltinCameraDirection.BACK) // Keep this commented if using webcam
                // .setCameraResolution(new Size(640, 480)) // Optional, but often good
                .build();

        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.addData("Instructions", "Press INIT, then open Camera Stream on DS.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                List<Rect> detectedCubes = redCubeProcessor.getDetectedRectangles();
                Rect largestCube = redCubeProcessor.getLargestDetectedRect();

                telemetry.addData("Detected Red Cubes", detectedCubes.size());

                if (largestCube != null) {
                    int centerX = largestCube.x + largestCube.width / 2;
                    int centerY = largestCube.y + largestCube.height / 2;
                    telemetry.addData("Largest Cube Center (X, Y)", "(" + centerX + ", " + centerY + ")");
                    telemetry.addData("Largest Cube Area", largestCube.area());
                    telemetry.addData("Largest Cube Dimensions (W x H)", largestCube.width + " x " + largestCube.height);
                } else {
                    telemetry.addData("Largest Cube", "None detected");
                }

                telemetry.update();
                sleep(20);
            }
        }

        visionPortal.close();
        redCubeProcessor.close();
        telemetry.addData("Status", "Vision Portal and Processor Closed.");
        telemetry.update();
    }
}