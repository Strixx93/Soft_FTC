package vision;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import mechanisms.ProgrammingBoard;
import vision.RedCubeDistanceDetectorProcessor;
import vision.RedCubeDistanceDetectorProcessor.DetectedCube;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "Skt3", group = "Linear Opmode")
public class Skt3 extends LinearOpMode {
    private ProgrammingBoard board;
    private VisionPortal visionPortal;
    private RedCubeDistanceDetectorProcessor cubeProcessor;
    private DcMotor leftDrive, rightDrive;
    private boolean isAutoActive = false;

    // Simplified drivetrain interface for Pedro Pathing
    private class SimpleDrivetrain {
        public void setDrivePower(double leftPower, double rightPower) {
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        public void setTargetDistance(double distanceMM) {
            int ticks = (int) (distanceMM * ProgrammingBoard.COUNTS_PER_MM);
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticks);
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + ticks);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) leftDrive).setVelocity(2500);
            ((DcMotorEx) rightDrive).setVelocity(2500);
        }

        public boolean isBusy() {
            return leftDrive.isBusy() || rightDrive.isBusy();
        }
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        board = new ProgrammingBoard();
        board.init(hardwareMap);

        // Initialize drivetrain (confirm motor names in Robot Configuration)
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive"); // Adjust name
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize VisionPortal
        cubeProcessor = new RedCubeDistanceDetectorProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "webcam1"), cubeProcessor);

        // Initialize Pedro Pathing with simplified drivetrain
        SimpleDrivetrain drivetrain = new SimpleDrivetrain();
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        // Note: Pedro Pathing needs a Localizer. Since you don’t have one, we’ll use encoder-based movement
        // If you have odometry wheels or IMU, share details to add a Localizer
        // For now, we’ll simulate path following with encoder targets

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // Manual control
            if (!isAutoActive) {
                // Tank drive
                double leftPower = -gamepad1.left_stick_y;
                double rightPower = -gamepad1.right_stick_y;
                drivetrain.setDrivePower(leftPower, rightPower);

                // Mechanism controls
                if (gamepad1.a) {
                    board.armRotateToAngle(board.ARM_BASKET_ANGLE); // Rotate to basket
                }
                if (gamepad1.b) {
                    board.armExtendToDistance(board.dist); // Extend arm
                }
                if (gamepad1.x) {
                    board.intake.setPower(board.INTAKE_IN); // Intake on
                } else if (gamepad1.y) {
                    board.intake.setPower(board.INTAKE_OFF); // Intake off
                }
                if (gamepad1.dpad_up) {
                    board.wrist.setPosition(board.WRIST_INTAKE); // Wrist to intake
                } else if (gamepad1.dpad_down) {
                    board.wrist.setPosition(board.WRIST_OUT); // Wrist out
                }
            }

            // Check for red cube and trigger autonomous
            DetectedCube cube = cubeProcessor.getLargestDetectedCube();
            if (cube != null && cube.distanceCM < 50 && !isAutoActive) {
                isAutoActive = true;
                // Define path to cube (using encoder-based movement)
                double distanceMM = cube.distanceCM * 10; // Convert cm to mm
                drivetrain.setTargetDistance(distanceMM);
                board.intake.setPower(board.INTAKE_IN); // Activate intake
            }

            // Update autonomous mode
            if (isAutoActive) {
                if (!drivetrain.isBusy() || gamepad1.left_bumper) {
                    isAutoActive = false;
                    board.intake.setPower(board.INTAKE_OFF);
                    drivetrain.setDrivePower(0, 0);
                    leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            // Telemetry
            telemetry.addData("Mode", isAutoActive ? "Autonomous" : "Manual");
            telemetry.addData("Cube Distance", cube != null ? String.format("%.1f cm", cube.distanceCM) : "N/A");
            telemetry.update();
        }

        // Cleanup
        visionPortal.close();
        cubeProcessor.close();
    }
}