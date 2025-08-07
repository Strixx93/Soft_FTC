/*
package autonomus;

import mechanisms.ProgrammingBoard;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutoFunc {
    private ProgrammingBoard board;
    private boolean isRunning = false;
    private int step = 0;
    private double targetX, targetY, targetHeadingDeg;
    private boolean isCurveMode;
    private double curveRadiusMM;
    private boolean isRightTurn;
    private double[] curvePoints;
    private int curvePointIndex;

    // PID constants (from your FollowerConstants)
    private static final double TRANSLATIONAL_P = 0.15;
    private static final double TRANSLATIONAL_I = 0.0;
    private static final double TRANSLATIONAL_D = 0.0;
    private static final double HEADING_P = 1.0;
    private static final double HEADING_I = 0.0;
    private static final double HEADING_D = 0.0;
    private double headingErrorIntegral = 0;
    private double lastHeadingError = 0;
    private double distanceErrorIntegral = 0;
    private double lastDistanceError = 0;

    public AutoFunc(ProgrammingBoard board) {
        this.board = board;
    }

    // Start sequence to drive to a point (x, y) in mm
    public void driveToPoint(double xMM, double yMM) {
        isRunning = true;
        isCurveMode = false;
        targetX = xMM;
        targetY = yMM;
        step = 0;
        headingErrorIntegral = 0;
        lastHeadingError = 0;
        distanceErrorIntegral = 0;
        lastDistanceError = 0;
    }

    // Start sequence to follow a 90-degree curve with given radius
    public void followCurve(double radiusMM, boolean isRightTurn) {
        isRunning = true;
        isCurveMode = true;
        this.curveRadiusMM = radiusMM;
        this.isRightTurn = isRightTurn;
        step = 0;
        headingErrorIntegral = 0;
        lastHeadingError = 0;
        distanceErrorIntegral = 0;
        lastDistanceError = 0;
        // Generate points for a 90-degree arc (10 segments)
        curvePoints = new double[20]; // 10 points (x, y)
        double angleStep = Math.PI / 2 / 10; // 90 degrees in 10 steps
        for (int i = 0; i < 10; i++) {
            double angle = i * angleStep;
            curvePoints[2 * i] = radiusMM * Math.cos(angle); // x
            curvePoints[2 * i + 1] = radiusMM * Math.sin(angle) * (isRightTurn ? 1 : -1); // y
        }
        curvePointIndex = 0;
    }

    // Compute distance and heading errors
    private double[] computeErrors(double currentX, double currentY, double currentHeadingDeg,
                                   double targetX, double targetY) {
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distanceError = Math.sqrt(dx * dx + dy * dy);
        double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingError = normalizeAngle(targetHeadingDeg - currentHeadingDeg);
        return new double[] {distanceError, headingError};
    }

    // Update the autonomous sequence
    public boolean update() {
        if (!isRunning) return true;

        Pose2D pose = board.getPose();
        double currentX = pose.getX(DistanceUnit.MM);
        double currentY = pose.getY(DistanceUnit.MM);
        double currentHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

        if (!isCurveMode) {
            // Drive to point sequence
            switch (step) {
                case 0: // Turn to face the point
                    double[] errors = computeErrors(currentX, currentY, currentHeadingDeg, targetX, targetY);
                    double headingError = errors[1];
                    if (Math.abs(headingError) < 2) { // Within 2 degrees
                        board.setMecanumPower(0, 0, 0);
                        step++;
                    } else {
                        double turnPower = computePID(headingError, true);
                        board.setMecanumPower(0, 0, clamp(turnPower, -0.5, 0.5));
                    }
                    break;
                case 1: // Drive to point
                    errors = computeErrors(currentX, currentY, currentHeadingDeg, targetX, targetY);
                    double distanceError = errors[0];
                    double headingErrorDrive = errors[1];
                    if (distanceError < 10) { // Within 10 mm
                        board.setMecanumPower(0, 0, 0);
                        board.setIntakePower(board.INTAKE_IN);
                        step++;
                    } else {
                        double forward = computePID(distanceError, false);
                        double strafe = 0; // Mecanum allows strafe, but keeping simple
                        double turn = computePID(headingErrorDrive, true);
                        board.setMecanumPower(clamp(forward, -0.5, 0.5), strafe, clamp(turn, -0.3, 0.3));
                    }
                    break;
                case 2: // Run intake for 1 second
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        // Handle interruption
                    }
                    board.setIntakePower(board.INTAKE_OFF);
                    step++;
                    break;
                case 3: // Reset
                    board.stopDrive();
                    isRunning = false;
                    step = 0;
                    headingErrorIntegral = 0;
                    lastHeadingError = 0;
                    distanceErrorIntegral = 0;
                    lastDistanceError = 0;
                    break;
            }
        } else {
            // Follow curve sequence
            switch (step) {
                case 0: // Drive to next curve point
                    if (curvePointIndex >= 10) {
                        board.setIntakePower(board.INTAKE_IN);
                        step++;
                        break;
                    }
                    double targetXPoint = curvePoints[2 * curvePointIndex];
                    double targetYPoint = curvePoints[2 * curvePointIndex + 1];
                    double[] errors = computeErrors(currentX, currentY, currentHeadingDeg, targetXPoint, targetYPoint);
                    double distanceError = errors[0];
                    double headingError = errors[1];
                    if (distanceError < 10) { // Within 10 mm
                        curvePointIndex++;
                    } else {
                        double forward = computePID(distanceError, false);
                        double strafe = 0;
                        double turn = computePID(headingError, true);
                        board.setMecanumPower(clamp(forward, -0.5, 0.5), strafe, clamp(turn, -0.3, 0.3));
                    }
                    break;
                case 1: // Run intake for 1 second
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        // Handle interruption
                    }
                    board.setIntakePower(board.INTAKE_OFF);
                    step++;
                    break;
                case 2: // Reset
                    board.stopDrive();
                    isRunning = false;
                    step = 0;
                    curvePointIndex = 0;
                    headingErrorIntegral = 0;
                    lastHeadingError = 0;
                    distanceErrorIntegral = 0;
                    lastDistanceError = 0;
                    break;
            }
        }
        return !isRunning;
    }

    // Compute PID control for heading or distance
    private double computePID(double error, boolean isHeading) {
        double pGain = isHeading ? HEADING_P : TRANSLATIONAL_P;
        double iGain = isHeading ? HEADING_I : TRANSLATIONAL_I;
        double dGain = isHeading ? HEADING_D : TRANSLATIONAL_D;
        double integral = isHeading ? headingErrorIntegral : distanceErrorIntegral;
        double lastError = isHeading ? lastHeadingError : lastDistanceError;
        integral += error;
        double derivative = error - lastError;
        if (isHeading) {
            headingErrorIntegral = integral;
            lastHeadingError = error;
        } else {
            distanceErrorIntegral = integral;
            lastDistanceError = error;
        }
        return pGain * error + iGain * integral + dGain * derivative;
    }

    // Normalize angle to [-180, 180]
    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }

    // Clamp value to a range
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // Check if sequence is running
    public boolean isRunning() {
        return isRunning;
    }

    // Force stop the sequence
    public void stop() {
        isRunning = false;
        step = 0;
        curvePointIndex = 0;
        headingErrorIntegral = 0;
        lastHeadingError = 0;
        distanceErrorIntegral = 0;
        lastDistanceError = 0;
        board.stopDrive();
        board.setIntakePower(board.INTAKE_OFF);
    }
}*/
