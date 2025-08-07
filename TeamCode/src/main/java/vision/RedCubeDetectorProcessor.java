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
 * RedCubeDetectorProcessor is a custom VisionProcessor for FTC robots.
 * It detects red-colored objects (like cubes) in the camera stream using HSV thresholding,
 * morphological operations, and contour analysis. It then draws green bounding rectangles
 * around the detected objects on the Driver Station preview.
 *
 * The processor also provides the list of detected rectangles and the center of the largest
 * detected cube to the OpMode for further use (e.g., telemetry, robot control).
 */
public class  RedCubeDetectorProcessor implements VisionProcessor {

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
    // Use the Driver Station camera stream and telemetry to help adjust.
    private Scalar lowerRed1 = new Scalar(0, 170, 100);  // Low end of red hue range
    private Scalar upperRed1 = new Scalar(10, 255, 255); // High end of red hue range
    private Scalar lowerRed2 = new Scalar(160, 150, 100); // Other end of red hue range (red wraps around)
    private Scalar upperRed2 = new Scalar(179, 255, 255); // High end of red hue range

    // --- Detection results (volatile for thread safety, as OpMode reads this) ---
    // Use AtomicReference for thread-safe access to the list of rectangles
    private final AtomicReference<List<Rect>> detectedRectsRef = new AtomicReference<>(new ArrayList<>());
    private final AtomicReference<Rect> largestDetectedRectRef = new AtomicReference<>(null);


    /**
     * Called once when the VisionProcessor is initialized.
     * @param width The width of the camera frame.
     * @param height The height of the camera frame.
     * @param calibration Camera calibration data.
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize the morphological kernel. A 3x3 rectangular kernel is common.
        // This is done once to avoid repeated allocation.
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(3, 3));
    }

    /**
     * Processes each camera frame. This is where the core computer vision logic resides.
     * @param frame The input camera frame (Mat). VisionPortal typically provides RGB frames.
     * @param captureTimeNanos The timestamp of the frame capture.
     * @return An object that will be passed to onDrawFrame as userContext.
     * We'll pass the largest detected rectangle (or null if none).
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Clear previous detections for the current frame
        List<Rect> currentDetectedRects = new ArrayList<>();
        Rect currentLargestRect = null;
        double largestArea = 0;

        // List to store contours found in the current frame.
        // This list is created per frame and released at the end of the method.
        List<MatOfPoint> contours = new ArrayList<>();

        // 1. Convert the original RGB frame to HSV color space.
        // FTC VisionPortal typically provides RGB frames, so use COLOR_RGB2HSV.
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // 2. Create binary masks for both red HSV ranges.
        // Pixels within the range become white (255), others black (0).
        Core.inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
        Core.inRange(hsvFrame, lowerRed2, upperRed2, redMask2);

        // 3. Combine the two red masks using a bitwise OR operation.
        // This merges the two red ranges into a single mask.
        Core.bitwise_or(redMask1, redMask2, finalRedMask);

        // 4. Perform morphological operations to clean up the mask.
        // These operations help remove small noise specks and fill small gaps,
        // leading to more stable and accurate contours.
        // Imgproc.erode: Shrinks white regions (removes small noise)
        // Imgproc.dilate: Expands white regions (fills small holes, reconnects broken parts)
        Imgproc.erode(finalRedMask, finalRedMask, kernel);
        Imgproc.dilate(finalRedMask, finalRedMask, kernel);
        // You could also use Imgproc.morphologyEx(finalRedMask, finalRedMask, Imgproc.MORPH_OPEN, kernel);
        // for a combined erode then dilate operation (opening).

        // 5. Find contours (outlines of shapes) in the cleaned binary mask.
        // RETR_EXTERNAL retrieves only the outermost contours.
        // CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments.
        Imgproc.findContours(finalRedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // 6. Iterate through detected contours, filter them, and get bounding rectangles.
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            // Filter out contours that are too small (likely noise) or too large (e.g., entire background).
            // These thresholds (1000 and 0.8) are examples and should be tuned.
            // A good minimum area helps prevent false positives from random pixel clusters.
            // A maximum area prevents detecting the entire frame if the mask goes wrong.
            if (area > 1000 && area < (frame.rows() * frame.cols() * 0.8)) {
                Rect rect = Imgproc.boundingRect(contour);
                currentDetectedRects.add(rect); // Add to the list of detected rectangles

                // Keep track of the largest detected rectangle
                if (area > largestArea) {
                    largestArea = area;
                    currentLargestRect = rect;
                }
            }
            contour.release(); // Release the MatOfPoint for each contour to prevent memory leaks
        }

        // Update the AtomicReferences with the results for the OpMode to access
        detectedRectsRef.set(currentDetectedRects);
        largestDetectedRectRef.set(currentLargestRect);

        // Release Mats that are created per frame or are temporary
        // (member variables are reused, so no need to release them here)
        contours.clear(); // Clear the list for the next frame
        hierarchy.release(); // Release hierarchy Mat as it's populated anew each frame

        // Return the largest detected rectangle (or null) to be used by onDrawFrame as userContext
        return currentLargestRect;
    }

    /**
     * Draws overlays on the camera preview displayed on the Driver Station.
     * This method is called after processFrame.
     * @param canvas The Android Canvas to draw on.
     * @param onscreenWidth Width of the canvas in pixels.
     * @param onscreenHeight Height of the canvas in pixels.
     * @param scaleBmpPxToCanvasPx Scale factor from bitmap pixels to canvas pixels.
     * @param scaleCanvasDensity Scale factor for density-independent pixels.
     * @param userContext The object returned by processFrame (in this case, the largest detected Rect).
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.GREEN); // Draw green rectangles around detected objects
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 2); // Adjust thickness for visibility

        // Get the list of detected rectangles from the AtomicReference
        List<Rect> rectsToDraw = detectedRectsRef.get();

        // Draw all detected red cube rectangles
        for (Rect rect : rectsToDraw) {
            // Convert OpenCV Rect (pixel coordinates) to Android Graphics.Rect (canvas coordinates)
            android.graphics.Rect drawRect = new android.graphics.Rect(
                    Math.round(rect.x * scaleBmpPxToCanvasPx),
                    Math.round(rect.y * scaleBmpPxToCanvasPx),
                    Math.round((rect.x + rect.width) * scaleBmpPxToCanvasPx),
                    Math.round((rect.y + rect.height) * scaleBmpPxToCanvasPx)
            );
            canvas.drawRect(drawRect, paint);
        }

        // Draw text indicating the position of the largest cube (if found) or status
        Paint textPaint = new Paint();
        textPaint.setColor(Color.WHITE); // White text for contrast
        textPaint.setTextSize(scaleCanvasDensity * 20); // Adjust text size
        textPaint.setAntiAlias(true); // Smooth text edges

        Rect largestRect = (Rect) userContext; // The largest rect passed from processFrame67y

        if (largestRect != null) {
            // Calculate center of the largest cube for display
            int centerX = largestRect.x + largestRect.width / 2;
            int centerY = largestRect.y + largestRect.height / 2;
            canvas.drawText(
                    "Largest Cube: (" + centerX + ", " + centerY + ")",
                    50 * scaleCanvasDensity, // X position
                    50 * scaleCanvasDensity, // Y position
                    textPaint
            );
            canvas.drawText(
                    "Area: " + String.format("%.0f", largestRect.area()),
                    50 * scaleCanvasDensity,
                    80 * scaleCanvasDensity,
                    textPaint
            );
        } else {
            canvas.drawText(
                    "No Red Cube Detected",
                    50 * scaleCanvasDensity,
                    50 * scaleCanvasDensity,
                    textPaint
            );
        }
    }

    /**
     * Provides the list of all detected red cube bounding rectangles.
     * This method is thread-safe as it returns a copy of the list.
     * @return A list of OpenCV Rect objects representing detected red cubes.
     */
    public List<Rect> getDetectedRectangles() {
        return detectedRectsRef.get(); // Returns the current list
    }

    /**
     * Provides the bounding rectangle of the largest detected red cube.
     * @return An OpenCV Rect object, or null if no red cube is detected.
     */
    public Rect getLargestDetectedRect() {
        return largestDetectedRectRef.get();
    }

    /**
     * Releases all OpenCV Mat resources held by the processor.
     * It's important to call this when the processor is no longer needed
     * (e.g., in your OpMode's `stop()` method or after `visionPortal.close()`).
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
        // Clear the atomic references
        detectedRectsRef.set(new ArrayList<>());
        largestDetectedRectRef.set(null);
    }
}