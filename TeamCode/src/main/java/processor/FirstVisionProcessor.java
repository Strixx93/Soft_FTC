package processor;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * FirstVisionProcessor is a VisionProcessor implementation designed to detect
 * the position of an object (e.g., a Team Prop) by analyzing the saturation
 * in three predefined regions of interest (ROIs) on the camera frame.
 * It highlights the selected region on the camera preview.
 */
public class FirstVisionProcessor implements VisionProcessor {

    // Define the regions of interest (ROIs) for left, middle, and right
    // These coordinates and dimensions are specific to the camera setup and object size.
    public Rect rectLeft = new Rect(110, 42, 40, 40);
    public Rect rectMiddle = new Rect(160, 42, 40, 40);
    public Rect rectRight = new Rect(210, 42, 40, 40);

    // Stores the current detected selection, initialized to NONE
    Selected selection = Selected.NONE;

    // Mats (matrices) used for image processing to avoid re-allocation
    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    /**
     * Initializes the vision processor.
     * This method is called once when the vision portal starts.
     *
     * @param width The width of the camera frame.
     * @param height The height of the camera frame.
     * @param calibration The camera calibration data.
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // No specific initialization needed for this processor beyond member variable setup.
    }

    /**
     * Processes each camera frame to determine the object's position.
     * Converts the frame to HSV, calculates average saturation for each ROI,
     * and returns the region with the highest saturation.
     *
     * @param frame The current camera frame in RGB format.
     * @param captureTimeNanos The timestamp of when the frame was captured.
     * @return The detected 'Selected' enum value (LEFT, MIDDLE, or RIGHT).
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the RGB frame to HSV color space for better color analysis.
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Get the average saturation for each defined rectangle.
        // Saturation (S) is the second channel (index 1) in HSV.
        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        double satRectRight = getAvgSaturation(hsvMat, rectRight);

        // Determine which rectangle has the highest saturation.
        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
            return Selected.LEFT;
        } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
            return Selected.MIDDLE;
        } else {
            // If neither left nor middle is the highest, then right must be (or all are equal).
            return Selected.RIGHT;
        }
    }

    /**
     * Calculates the average saturation of a given rectangle within an HSV image.
     *
     * @param input The input Mat (expected to be in HSV format).
     * @param rect The Rect object defining the region of interest.
     * @return The average saturation value for the specified rectangle.
     */
    protected double getAvgSaturation(Mat input, Rect rect) {
        // Extract the sub-matrix corresponding to the rectangle.
        submat = input.submat(rect);
        // Calculate the mean (average) color of the sub-matrix.
        Scalar color = Core.mean(submat);
        // Return the saturation component (index 1) of the mean color.
        return color.val[1];
    }

    /**
     * Converts an OpenCV Rect object to an Android Graphics Rect object,
     * scaling the coordinates and dimensions for drawing on the canvas.
     *
     * @param rect The OpenCV Rect to convert.
     * @param scaleBmpPxToCanvasPx The scaling factor from bitmap pixels to canvas pixels.
     * @return An Android Graphics Rect suitable for drawing.
     */
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    /**
     * Draws overlays on the camera preview canvas to visualize the ROIs and the selected region.
     *
     * @param canvas The Android Canvas to draw on.
     * @param onscreenWidth The width of the screen in pixels.
     * @param onscreenHeight The height of the screen in pixels.
     * @param scaleBmpPxToCanvasPx The scaling factor from bitmap pixels to canvas pixels.
     * @param scaleCanvasDensity The scaling factor for canvas density (for stroke width).
     * @param userContext The user context object, which is the 'Selected' enum from processFrame.
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        // Paint for the selected rectangle (red, thick stroke)
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4); // Make stroke width responsive to screen density

        // Paint for non-selected rectangles (green, same style as selected but different color)
        Paint nonSelectedPaint = new Paint(selectedPaint); // Copy style from selectedPaint
        nonSelectedPaint.setColor(Color.GREEN);

        // Convert OpenCV Rects to Android Graphics Rects for drawing, applying scaling.
        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        // Cast the userContext to the Selected enum to determine which rectangle was chosen.
        selection = (Selected) userContext;

        // Draw the rectangles on the canvas, highlighting the selected one in red.
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE: // Case for when no selection has been made yet or explicitly none.
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
        }
    }

    /**
     * Returns the last detected selection.
     * This method can be called from the OpMode to get the result of the vision processing.
     *
     * @return The 'Selected' enum value representing the detected position.
     */
    public Selected getSelection() {
        return selection;
    }

    /**
     * Enum to represent the possible selections for the object's position.
     */
    public enum Selected {
        NONE,   // No selection made yet or explicitly none.
        LEFT,   // Object detected in the left region.
        MIDDLE, // Object detected in the middle region.
        RIGHT   // Object detected in the right region.
    }
}
