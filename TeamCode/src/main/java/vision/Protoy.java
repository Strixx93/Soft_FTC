package vision;

import static mechanisms.ProgrammingBoard.dist;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import mechanisms.ProgrammingBoard;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp
public class Protoy extends OpMode {
    //RHardware
    ProgrammingBoard board = new ProgrammingBoard();
    // VisionComp
    private VisionPortal visionPortal;
    private RedCubeDistanceDetectorProcessor redCubeProcessor;
    //Auto
    private Follower follower;
    private ElapsedTime auxTimer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = -1;
    private final Pose startPose = new Pose(24, 108, Math.toRadians(-90));
    private PathChain trj1, trj2, trj3;

    private double distance = 108.000;

    @Override
    public void init() {
        board.init(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        redCubeProcessor = new RedCubeDistanceDetectorProcessor();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(redCubeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .build();


        buildTrajectory();
        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.addData("Vision Status", "VisionPortal and Processor Initialized.");
        telemetry.update();

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        follower.startTeleopDrive();
        opmodeTimer.resetTimer();
        visionPortal.resumeStreaming();
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        RedCubeDistanceDetectorProcessor.DetectedCube largestCube = redCubeProcessor.getLargestDetectedCube();

        if (largestCube != null) {

            telemetry.addData("Detection Status", "Red Cube FOUND!");
            telemetry.addData("Largest Cube Pixel Width", String.format("%.0f pixels", largestCube.pixelWidth));
            telemetry.addData("Largest Cube Center (X, Y)", String.format("(%.0f, %.0f)", largestCube.centerX, largestCube.centerY));
            telemetry.addData("Largest Cube Estimated Distance", String.format("%.1f cm", largestCube.distanceCM));
            telemetry.addData("Largest Cube Area", String.format("%.0f", largestCube.area));

           /* if (board.intake.getPower() != INTAKE_IN && board.intake.getPower() != INTAKE_OUT) { // Only set if not manually overridden
                *//*board.intake.setPower(0.5); // Set intake CRServo to 25% power*//*
            }*/

            distance = 108-(largestCube.distanceCM)/2.54;
            /*buildTrajectory();
            trajectory();*/
            telemetry.addData("Robot Action", "Intake ON: Cube Detected!");
            board.armExtendToDistance(dist);
            board.setWristPositionIN();
            board.setIntakePowerIn();
            //if (largestCube.distanceCM < 15.0) { /* stop drive, grab */ }
            /*if (gamepad1.a && pathState == -1) {
                pathState = 0;
                buildTrajectory();

            }*/

        } else {
            // No red cube is detected.
            telemetry.addData("Detection Status", "No Red Cube Detected");
            telemetry.addData("Largest Cube", "None detected");
            board.setIntakePowerOFF();
            board.setWristPositionOUT();
            /*// Stop the intake or put into search mode if not manually overridden
            if (board.intake.getPower() != INTAKE_IN && board.intake.getPower() != INTAKE_OUT) { // Only set if not manually overridden
                board.intake.setPower(0.0); // Stop the intake if no cube is found
            }*/
            telemetry.addData("Robot Action", "Intake OFF: Searching for cube...");
            // (Optional: add search behavior like turning slowly here)
            /*if (pathState != -1) {
                pathState = -1;
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            }*/

        }
        /*if (pathState >= 0) {
            trajectory();
        }*/

        /* TelemetryOutputsFollower */
        /*telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));*/

        // Add manual intake control from gamepad (example: gamepad1.left_bumper for in, right_bumper for out)
        /*if (gamepad1.left_bumper) {
            board.intake.setPower(INTAKE_IN);
            telemetry.addData("Manual Intake", "IN");
        } else if (gamepad1.right_bumper) {
            board.intake.setPower(INTAKE_OUT);
            telemetry.addData("Manual Intake", "OUT");
        } else if (!gamepad1.left_bumper && !gamepad1.right_bumper && largestCube == null) {
            // Only stop if no cube is detected and no manual override
            board.intake.setPower(INTAKE_OFF);
            telemetry.addData("Manual Intake", "OFF");
        }*/

        /* UpdateTelemetryDriverHub */
        telemetry.update();



    }

    @Override
    public void stop() {
        // --- Cleanup ---
        // It's crucial to close the VisionPortal and release processor resources when the OpMode stops.
        if (visionPortal != null) {
            visionPortal.close();
        }
        if (redCubeProcessor != null) {
            redCubeProcessor.close(); // Assuming RedCubeDistanceDetectorProcessor has a close method
        }
        telemetry.addData("Status", "Vision Portal and Processor Closed.");
        telemetry.update();
    }

    public void buildTrajectory(){
        trj1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(24.000, 108.000, Point.CARTESIAN),
                                new Point(24.000, distance, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        /*trj2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(24.000, 90.000, Point.CARTESIAN),
                                new Point(24.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();*/
    }

    public void trajectory() {
        switch (pathState) {
            case 0:
                follower.followPath(trj1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    loop();
                    pathState=-1;
                    break;
                }
                break;
            default:
                pathState = -1;
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                board.setIntakePower(0.0);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void wait(double t) {
        auxTimer.reset();
        while (auxTimer.seconds() < t) {
            //WaitManualOverride
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
        }
    }
}