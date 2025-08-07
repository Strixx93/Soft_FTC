package autonomus;

import static mechanisms.ProgrammingBoard.INTAKE_IN;
import static mechanisms.ProgrammingBoard.INTAKE_OFF;
import static mechanisms.ProgrammingBoard.INTAKE_OUT;
import static mechanisms.ProgrammingBoard.WRIST_INTAKE;
import static mechanisms.ProgrammingBoard.dist;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;

import mechanisms.ProgrammingBoard;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import vision.RedCubeDistanceDetectorProcessor;

@TeleOp(name="TestAutoTeleOp", group="TeleOp")
public class ATest extends LinearOpMode {
    private ProgrammingBoard board;
    private Follower follower;
    private VisionPortal visionPortal;
    private RedCubeDistanceDetectorProcessor redCubeProcessor;
    private ElapsedTime auxTimer = new ElapsedTime();
    private Timer pathTimer;
    private int pathState = -1;
    private PathChain trj1;
    private final Point targetPoint = new Point(24.0, 48.0, Point.CARTESIAN);
    private Pose2D currentPose;
    private double currentXInches, currentYInches, currentHeadingDeg;

    @Override
    public void runOpMode() {
        // Initialize hardware
        board = new ProgrammingBoard();
        board.init(hardwareMap);

        // Initialize Pedro Pathing
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(24, 108, Math.toRadians(-90)));

        // Initialize vision
        redCubeProcessor = new RedCubeDistanceDetectorProcessor();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(redCubeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .build();

        // Initialize timers
        pathTimer = new Timer();

        // Initialize trajectory
        trj1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(24.0, 108.0, Point.CARTESIAN),
                        new Point(24.0, 48.0, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.addData("Vision Status", "VisionPortal and Processor Initialized.");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            // Update position
            updateRobotPosition();

            // TeleOp drive
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x, true);
            follower.update();

            // Vision and obstacle detection
            RedCubeDistanceDetectorProcessor.DetectedCube largestCube = redCubeProcessor.getLargestDetectedCube();
            if (largestCube != null) {
                List<Point> obstacles = detectObstacles(largestCube);
                updateBezierCurve(obstacles, targetPoint);
                telemetry.addData("Detection Status", "Obstacle FOUND!");
                telemetry.addData("Obstacle Distance", "%.1f cm", largestCube.distanceCM);
                telemetry.addData("Obstacle Center X", "%.0f pixels", largestCube.centerX);
            } else {
                telemetry.addData("Detection Status", "No Obstacle Detected");
            }

            // Start autonomous navigation
            if (gamepad1.a && pathState == -1) {
                navigateToPoint(24.0, 48.0, -90.0);
                pathState = 0;
            }

            // Execute trajectory
            if (pathState >= 0) {
                executeTrajectory();
            }

            // Manual intake control
            if (gamepad1.left_bumper) {
                board.setIntakePower(INTAKE_IN);
                telemetry.addData("Manual Intake", "IN");
            } else if (gamepad1.right_bumper) {
                board.setIntakePower(INTAKE_OUT);
                telemetry.addData("Manual Intake", "OUT");
            } else if (largestCube == null) {
                board.setIntakePower(INTAKE_OFF);
                telemetry.addData("Manual Intake", "OFF");
            }

            // Telemetry
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", currentXInches);
            telemetry.addData("Y", currentYInches);
            telemetry.addData("Heading", "%.1f deg", currentHeadingDeg);
            telemetry.update();
        }

        // Cleanup
        visionPortal.close();
        redCubeProcessor.close();
        follower.setTeleOpMovementVectors(0, 0, 0, true); // Stop pathing
        board.stopDrive();
        telemetry.addData("Status", "Stopped.");
        telemetry.update();
    }

    private void updateRobotPosition() {
        //currentPose = board.getPose();
        currentXInches = currentPose.getX(DistanceUnit.MM) / 25.4;
        currentYInches = currentPose.getY(DistanceUnit.MM) / 25.4;
        currentHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);
    }

    private void navigateToPoint(double targetXInches, double targetYInches, double targetHeadingDeg) {
        trj1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(currentXInches, currentYInches, Point.CARTESIAN),
                        new Point(targetXInches, targetYInches, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(currentHeadingDeg), Math.toRadians(targetHeadingDeg))
                .build();
    }

    private List<Point> detectObstacles(RedCubeDistanceDetectorProcessor.DetectedCube cube) {
        List<Point> obstacles = new ArrayList<>();
        if (cube != null) {
            // Assume 640x480 resolution, 60° FOV
            double centerXPixels = cube.centerX - 320; // Center of image = 320
            double angleDeg = centerXPixels * (60.0 / 640.0); // ~0.094° per pixel
            double distanceInches = cube.distanceCM / 2.54;
            // Calculate obstacle position relative to robot
            double obstacleX = currentXInches + distanceInches * Math.cos(Math.toRadians(currentHeadingDeg + angleDeg));
            double obstacleY = currentYInches + distanceInches * Math.sin(Math.toRadians(currentHeadingDeg + angleDeg));
            obstacles.add(new Point(obstacleX, obstacleY, Point.CARTESIAN));
        }
        return obstacles;
    }

    private void updateBezierCurve(List<Point> obstacles, Point targetPoint) {
        if (obstacles.isEmpty()) {
            trj1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(currentXInches, currentYInches, Point.CARTESIAN),
                            targetPoint))
                    .setLinearHeadingInterpolation(Math.toRadians(currentHeadingDeg), Math.toRadians(-90))
                    .build();
        } else {
            Point obstacle = obstacles.get(0);
            // Add control point 6 inches left/right of obstacle
            double offsetX = obstacle.getX() + 6.0 * Math.sin(Math.toRadians(currentHeadingDeg));
            double offsetY = obstacle.getY() - 6.0 * Math.cos(Math.toRadians(currentHeadingDeg));
            Point controlPoint = new Point(offsetX, offsetY, Point.CARTESIAN);
            trj1 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(currentXInches, currentYInches, Point.CARTESIAN),
                            controlPoint,
                            targetPoint))
                    .setLinearHeadingInterpolation(Math.toRadians(currentHeadingDeg), Math.toRadians(-90))
                    .build();
        }
    }

    private void executeTrajectory() {
        switch (pathState) {
            case 0:
                board.armExtendToDistance(dist);
                board.setWristPosition(WRIST_INTAKE);
                board.setIntakePower(INTAKE_IN);
                follower.followPath(trj1);
                pathTimer.resetTimer();
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    board.setIntakePower(INTAKE_IN);
                    auxTimer.reset();
                    pathState = 2;
                } else {
                    // Check for new obstacles
                    RedCubeDistanceDetectorProcessor.DetectedCube cube = redCubeProcessor.getLargestDetectedCube();
                    if (cube != null) {
                        updateBezierCurve(detectObstacles(cube), targetPoint);
                        follower.followPath(trj1);
                    }
                }
                break;
            case 2:
                if (auxTimer.seconds() >= 1.0) {
                    board.setIntakePower(INTAKE_OFF);
                    pathState = -1;
                    follower.setTeleOpMovementVectors(0, 0, 0, true); // Stop pathing
                }
                break;
            default:
                pathState = -1;
                follower.setTeleOpMovementVectors(0, 0, 0, true); // Stop pathing
                board.setIntakePower(INTAKE_OFF);
                break;
        }
    }
}