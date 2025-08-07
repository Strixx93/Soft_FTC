package vision; // Adjust to your teamcode package

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * RedCubeDistanceDetectorProcessor is a custom VisionProcessor for FTC robots.
 * It detects red-colored objects (like cubes) using HSV thresholding,
 * morphological operations, and contour analysis. It then calculates and displays
 * an estimated distance to the largest detected red object based on its pixel width
 * and pre-calibrated camera parameters.
 */
public class RedCubeDistanceDetectorProcessor implements VisionProcessor {

    // --- NEW CONSTANTS FOR DISTANCE CALCULATION ---
    // IMPORTANT: You MUST measure and calibrate these values for YOUR specific setup.
    // 1. Measure the REAL-WORLD WIDTH of your red cube (e.g., in centimeters or inches).
    //    Use the dimension that will be facing the camera for this calculation.
    public static final double REAL_WORLD_RED_CUBE_WIDTH_CM = 8.9; // <-- REPLACE WITH YOUR CUBE'S ACTUAL WIDTH IN CM

    // 2. CALIBRATE FOCAL_LENGTH_PIXELS for YOUR ROBOT'S CAMERA:
    //    a. Place your red cube at a KNOWN_DISTANCE_CM (e.g., 50 cm) directly in front of your robot's camera.
    //    b. Run the RedCubeDistanceDetectionOpMode.
    //    c. Look at the Driver Station telemetry output for "Largest Cube Pixel Width: X pixels".
    //    d. Use the formula: FOCAL_LENGTH_PIXELS = (Pixel_Width_at_Known_Distance * KNOWN_DISTANCE_CM) / REAL_WORLD_RED_CUBE_WIDTH_CM
    //       Example: If at 50cm, pixel width is 75px and real width is 8.9cm: FL = (75 * 50) / 8.9 = 421.34
    //    e. Replace the value below with your calculated FOCAL_LENGTH_PIXELS.
    public static final double FOCAL_LENGTH_PIXELS = 684.71; // <-- UPDATE THIS VALUE AFTER CALIBRATION!

    // --- END NEW CONSTANTS ---

    // --- OpenCV Mats for image processing (declared as member variables for reuse) ---
    private Mat hsvFrame = new Mat();          // Stores the frame converted to HSV color space
    private Mat redMask1 = new Mat();          // Mask for the first red HSV range
    private Mat redMask2 = new Mat();          // Mask for the second red HSV range
    private Mat finalRedMask = new Mat();      // Combined and cleaned mask for red
    private Mat hierarchy = new Mat();         // Stores contour hierarchy (needed by findContours)
    private Mat kernel;                        // Structuring element for morphological operations

    // --- HSV color ranges for red (tuned for vibrant reds) ---
    // Hue (H: 0-179), Saturation (S: 0-255), Value (V: 0-255)
    // These values might need fine-tuning based on your specific camera, lighting, and red object.
    private Scalar lowerRed1 = new Scalar(0, 150, 100);
    private Scalar upperRed1 = new Scalar(10, 255, 255);
    private Scalar lowerRed2 = new Scalar(160, 150, 100);
    private Scalar upperRed2 = new Scalar(179, 255, 255);

    // --- Data structure to hold information about a detected cube ---
    public static class DetectedCube {
        public Rect rect;
        public double distanceCM;
        public double pixelWidth;
        public double centerX;
        public double centerY;
        public double area;

        public DetectedCube(Rect rect, double pixelWidth, double distanceCM) {
            this.rect = rect;
            this.pixelWidth = pixelWidth;
            this.distanceCM = distanceCM;
            this.centerX = rect.x + rect.width / 2.0;
            this.centerY = rect.y + rect.height / 2.0;
            this.area = rect.area();
        }
    }

    // --- Detection results (using AtomicReference for thread-safe access from OpMode) ---
    private final AtomicReference<List<DetectedCube>> allDetectedCubesRef = new AtomicReference<>(new ArrayList<>());
    private final AtomicReference<DetectedCube> largestDetectedCubeRef = new AtomicReference<>(null);


    /**
     * Called once when the VisionProcessor is initialized.
     * @param width The width of the camera frame.
     * @param height The height of the camera frame.
     * @param calibration Camera calibration data.
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize the morphological kernel. A 3x3 rectangular kernel is common.
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(3, 3));
    }

    /**
     * Processes each camera frame. This is where the core computer vision logic resides.
     * @param frame The input camera frame (Mat). VisionPortal typically provides RGB frames.
     * @param captureTimeNanos The timestamp of the frame capture.
     * @return The largest detected cube (as a DetectedCube object), or null if none.
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Clear previous detections for the current frame
        List<DetectedCube> currentDetectedCubes = new ArrayList<>();
        DetectedCube currentLargestCube = null;
        double largestArea = 0;

        List<MatOfPoint> contours = new ArrayList<>(); // List to store contours found in the current frame

        // 1. Convert the original RGB frame to HSV color space.
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // 2. Create binary masks for both red HSV ranges.
        Core.inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
        Core.inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
        Core.bitwise_or(redMask1, redMask2, finalRedMask);

        // 3. Perform morphological operations to clean up the mask.
        Imgproc.erode(finalRedMask, finalRedMask, kernel);
        Imgproc.dilate(finalRedMask, finalRedMask, kernel);

        // 4. Find contours (outlines of shapes) in the cleaned binary mask.
        contours.clear();
        Imgproc.findContours(finalRedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // 5. Iterate through detected contours, filter them, and get bounding rectangles.
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if (area > 1000 && area < (frame.rows() * frame.cols() * 0.8)) { // Tune these area thresholds
                Rect rect = Imgproc.boundingRect(contour);

                // --- DISTANCE CALCULATION ---
                double pixelWidth = rect.width;
                double distanceCM = -1.0; // Initialize as -1.0 (invalid)

                if (pixelWidth > 0 && FOCAL_LENGTH_PIXELS > 0 && REAL_WORLD_RED_CUBE_WIDTH_CM > 0) {
                    distanceCM = (REAL_WORLD_RED_CUBE_WIDTH_CM * FOCAL_LENGTH_PIXELS) / pixelWidth;
                }

                DetectedCube detectedCube = new DetectedCube(rect, pixelWidth, distanceCM);
                currentDetectedCubes.add(detectedCube);

                if (area > largestArea) {
                    largestArea = area;
                    currentLargestCube = detectedCube;
                }
            }
            contour.release(); // Release MatOfPoint for each contour
        }

        // Update the AtomicReferences with the results for the OpMode to access
        allDetectedCubesRef.set(currentDetectedCubes);
        largestDetectedCubeRef.set(currentLargestCube);

        // Release Mats that are created per frame or are temporary
        contours.clear();
        hierarchy.release();

        // Return the largest detected cube (or null) to be used by onDrawFrame as userContext
        return currentLargestCube;
    }

    /**
     * Draws overlays on the camera preview displayed on the Driver Station.
     * This method is called after processFrame.
     * @param canvas The Android Canvas to draw on.
     * @param onscreenWidth Width of the canvas in pixels.
     * @param onscreenHeight Height of the canvas in pixels.
     * @param scaleBmpPxToCanvasPx Scale factor from bitmap pixels to canvas pixels.
     * @param scaleCanvasDensity Scale factor for density-independent pixels.
     * @param userContext The object returned by processFrame (the largest detected CubeDetection object or null).
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.GREEN); // Draw green rectangles around detected objects
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 2);

        Paint textPaint = new Paint();
        textPaint.setColor(Color.YELLOW); // Yellow text for distance
        textPaint.setTextSize(scaleCanvasDensity * 18);
        textPaint.setAntiAlias(true);

        // Get the list of all detected cubes from the AtomicReference
        List<DetectedCube> cubesToDraw = allDetectedCubesRef.get();

        // Draw all detected red cube rectangles and their distances
        for (DetectedCube cube : cubesToDraw) {
            // Convert OpenCV Rect to Android Graphics.Rect for drawing
            android.graphics.Rect drawRect = new android.graphics.Rect(
                    Math.round(cube.rect.x * scaleBmpPxToCanvasPx),
                    Math.round(cube.rect.y * scaleBmpPxToCanvasPx),
                    Math.round((cube.rect.x + cube.rect.width) * scaleBmpPxToCanvasPx),
                    Math.round((cube.rect.y + cube.rect.height) * scaleBmpPxToCanvasPx)
            );
            canvas.drawRect(drawRect, paint);

            // Draw distance text above the rectangle
            if (cube.distanceCM != -1.0) { // Only draw if distance was calculated
                String distanceText = String.format("%.1f cm", cube.distanceCM);
                canvas.drawText(distanceText, drawRect.left, drawRect.top - (5 * scaleCanvasDensity), textPaint);
            }
        }

        // Optionally, draw summary info for the largest cube at a fixed position
        DetectedCube largestCube = (DetectedCube) userContext;
        Paint summaryTextPaint = new Paint();
        summaryTextPaint.setColor(Color.WHITE);
        summaryTextPaint.setTextSize(scaleCanvasDensity * 20);
        summaryTextPaint.setAntiAlias(true);

        if (largestCube != null) {
            canvas.drawText("Largest Cube:", 50 * scaleCanvasDensity, 50 * scaleCanvasDensity, summaryTextPaint);
            canvas.drawText(String.format("  Center: (%.0f, %.0f)", largestCube.centerX, largestCube.centerY),
                    50 * scaleCanvasDensity, 75 * scaleCanvasDensity, summaryTextPaint);
            canvas.drawText(String.format("  Distance: %.1f cm", largestCube.distanceCM),
                    50 * scaleCanvasDensity, 100 * scaleCanvasDensity, summaryTextPaint);
            canvas.drawText(String.format("  Area: %.0f", largestCube.area),
                    50 * scaleCanvasDensity, 125 * scaleCanvasDensity, summaryTextPaint);
        } else {
            canvas.drawText("No Red Cube Detected", 50 * scaleCanvasDensity, 50 * scaleCanvasDensity, summaryTextPaint);
        }
    }

    /**
     * Provides the list of all detected red cube bounding rectangles and their calculated distances.
     * This method is thread-safe as it returns a copy of the list.
     * @return A list of DetectedCube objects.
     */
    public List<DetectedCube> getAllDetectedCubes() {
        return allDetectedCubesRef.get();
    }

    /**
     * Provides the DetectedCube object for the largest detected red cube.
     * @return A DetectedCube object, or null if no red cube is detected.
     */
    public DetectedCube getLargestDetectedCube() {
        return largestDetectedCubeRef.get();
    }

    /**
     * Releases all OpenCV Mat resources held by the processor.
     * It's important to call this when the processor is no longer needed.
     */
    public void close() {
        hsvFrame.release();
        redMask1.release();
        redMask2.release();
        finalRedMask.release();
        hierarchy.release();
        if (kernel != null) {
            kernel.release();
        }
        allDetectedCubesRef.set(new ArrayList<>());
        largestDetectedCubeRef.set(null);
    }
}