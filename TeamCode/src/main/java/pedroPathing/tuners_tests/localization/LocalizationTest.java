package pedroPathing.tuners_tests.localization;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import com.pedropathing.follower.Follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

import pedroPathing.constants.*;

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot
 * on FTC Dashboard (192/168/43/1:8080/dash). You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
@Config
@TeleOp(group = "Teleop Test", name = "Localization Test")
public class LocalizationTest extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private ElapsedTime loopTimer = new ElapsedTime();
    private double prevHeading;
    private double prevTime;
    /**
     * This initializes the PoseUpdater, the mecanum drive motors, and the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        poseUpdater = new PoseUpdater(hardwareMap, FConstants.class, LConstants.class);
        Constants.setConstants(FConstants.class, LConstants.class);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void start(){
        follower.startTeleopDrive();

        loopTimer.reset();
        prevHeading = follower.getPose().getHeading();
        prevTime = loopTimer.seconds();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the FTC
     * Dashboard telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, true);
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        poseUpdater.update();
        dashboardPoseTracker.update();

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // this is strafing
        double rx = gamepad1.right_stick_x;

        double currentHeading = follower.getPose().getHeading();
        double currentTime = loopTimer.seconds();

        double deltaHeading = currentHeading - prevHeading;
        double deltaTime = currentTime - prevTime;

        double calculatedAngularVelocity = 0.0;
        if (deltaTime>0.001){
            calculatedAngularVelocity = deltaHeading / deltaTime;
        }

        telemetry.addData("Calculated Angular Velocity", calculatedAngularVelocity);

        prevHeading = currentHeading;
        prevTime = currentTime;

        
        telemetryA.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}
