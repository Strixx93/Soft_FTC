package vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "Potop1", group = "Examples")
public class Potop1 extends OpMode {
    private Follower follower;

    static final double COUNTS_PER_MOTOR_REV = 28;   // Encoder counts per motor revolution
    static final double GEAR_REDUCTION = 13.7;       // Gearbox ratio
    static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION; // = 28 * 19.2 = 537.6
    static final double WHEEL_DIAMETER_MM = 38.2;
    static final double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * Math.PI;
    static final double COUNTS_PER_MM = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM);

    private Servo wrist;

    final double ARM_TICKS_PER_DEGREE = 19.7924893140647;

    final double ARM_BASKET_ANGLE = 107.5* ARM_TICKS_PER_DEGREE;

    private DcMotor armExtend, armRotate;

    private CRServo intake;

    final double INTAKE_IN = 0.5;

    final double INTAKE_OFF = 0.0;

    final double INTAKE_OUT = -0.25;

    final double dist = -80* COUNTS_PER_MM;

    final double dist1 = -600* COUNTS_PER_MM;

    final double WRIST_INTAKE = -1;

    final double WRIST_OUT = 0.45;

    private ElapsedTime auxTimer = new ElapsedTime();

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState=-1;
    private final Pose startPose = new Pose(24,108,Math.toRadians(-90));

    private PathChain trj1, trj2;

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        initialize();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        trj1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(24.000, 108.000, Point.CARTESIAN),
                                new Point(24.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        trj2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(24.000, 72.000, Point.CARTESIAN),
                                new Point(24.000, 48.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
        opmodeTimer.resetTimer();
    }

    public void wait(double t){
        auxTimer.reset();
        while (auxTimer.seconds() < t){

        }
    }

    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void initialize(){
        intake = hardwareMap.get(CRServo.class, "intake");
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        wrist = hardwareMap.get(Servo.class, "wrist");


        intake.setPower(INTAKE_OFF);


        armExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtend.setTargetPosition(0);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armRotate.setTargetPosition(0);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void armExtendToDistance(double dist){
        armExtend.setTargetPosition((int)dist);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx)armExtend).setVelocity(2500);
    }

    public void armRotateToAngle(double angle){
        armRotate.setTargetPosition((int)angle);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx)armRotate).setVelocity(2500);
    }



    public void trajectory(){
        switch (pathState){
            case 0:
                armExtendToDistance(dist);
                wrist.setPosition(WRIST_INTAKE);
                intake.setPower(INTAKE_IN);
                follower.followPath(trj1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(trj2);
                    setPathState(2);
                    break;
                }

        }
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

        if(gamepad1.a)pathState=0;
        trajectory();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}