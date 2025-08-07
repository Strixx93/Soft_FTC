package vision; // Ensure this matches your folder structure

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo; // Used for CRServos

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection; // Only if using phone camera
// import org.firstinspires.ftc.robotcore.external.navigation.Size; // Only if setting camera resolution
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@TeleOp(name = "Red Cube Distance Detector", group = "Vision")
public class RedCubeDistanceDetectionOpMode extends LinearOpMode {

    private VisionPortal visionPortal;
    private RedCubeDistanceDetectorProcessor redCubeProcessor;

    private CRServo intake; // Declare your CRServo

    @Override
    public void runOpMode() {
        // 1. Create an instance of our custom VisionProcessor.
        redCubeProcessor = new RedCubeDistanceDetectorProcessor();

        // Initialize your intake CRServo here
        // Make sure "intake" matches the name in your robot configuration
        intake = hardwareMap.get(CRServo.class, "intake");
        // CRServos don't have setDirection or ZeroPowerBehavior like DcMotors
        // Their power directly controls direction and speed (-1.0 to 1.0)

        // 2. Build the VisionPortal.
        visionPortal = new VisionPortal.Builder()
                .addProcessor(redCubeProcessor)
                // Set the camera (e.g., webcam, or built-in phone camera)
                // UNCOMMENT ONE OF THE FOLLOWING LINES TO SELECT YOUR CAMERA:
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1")) // <-- Check case sensitivity of "Webcam1"
                // .setCamera(BuiltinCameraDirection.BACK) // Example for phone's built-in back camera
                // .setCameraResolution(new Size(640, 480)) // Optional: Uncomment and import Size if used
                .build();

        // Initial telemetry messages
        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.addData("Instructions", "Press INIT, then open Camera Stream on DS.");
        telemetry.addData("Calibration Help", "Place cube at KNOWN_DISTANCE_CM and note Pixel Width from DS Telemetry.");
        telemetry.addData("Calibration Help", "Calculate FOCAL_LENGTH_PIXELS and update RedCubeDistanceDetectorProcessor.java");
        telemetry.update();

        waitForStart(); // Wait for the OpMode to start

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get the latest detection result from your processor
                RedCubeDistanceDetectorProcessor.DetectedCube largestCube = redCubeProcessor.getLargestDetectedCube();

                // Start with common telemetry that's always displayed
                telemetry.addData("Live Stream Status", "Viewable from 3-dots menu -> Camera Stream"); // Reminder

                if (largestCube != null) {
                    // A red cube IS detected!
                    telemetry.addData("Detection Status", "Red Cube FOUND!");
                    telemetry.addData("Largest Cube Pixel Width", String.format("%.0f pixels", largestCube.pixelWidth));
                    telemetry.addData("Largest Cube Center (X, Y)", String.format("(%.0f, %.0f)", largestCube.centerX, largestCube.centerY));
                    telemetry.addData("Largest Cube Estimated Distance", String.format("%.1f cm", largestCube.distanceCM));
                    telemetry.addData("Largest Cube Area", String.format("%.0f", largestCube.area));

                    // Activate the intake
                    intake.setPower(0.25); // Set intake CRServo to 25% power
                    telemetry.addData("Robot Action", "Intake ON: Cube Detected!");

                    // Add more complex robot actions here based on distance/center
                    // Example: if (largestCube.distanceCM < 15.0) { /* stop drive, grab */ }

                } else {
                    // No red cube is detected.
                    telemetry.addData("Detection Status", "No Red Cube Detected");
                    telemetry.addData("Largest Cube", "None detected"); // This is fine here if cube is null

                    // Stop the intake or put into search mode
                    intake.setPower(0.0); // Stop the intake if no cube is found
                    telemetry.addData("Robot Action", "Intake OFF: Searching for cube...");

                    // (Optional: add search behavior like turning slowly here)
                }

                telemetry.update(); // Update telemetry once per loop iteration

                sleep(20); // Small delay to prevent overwhelming the system
            }
        }

        // --- Cleanup ---
        // It's crucial to close the VisionPortal and release processor resources when the OpMode stops.
        visionPortal.close();
        redCubeProcessor.close();
        telemetry.addData("Status", "Vision Portal and Processor Closed.");
        telemetry.update();
    }
}