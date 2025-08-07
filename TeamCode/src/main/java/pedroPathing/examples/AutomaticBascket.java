package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
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

@Autonomous
public class AutomaticBascket extends OpMode{
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

    final double INTAKE_IN = 1;

    final double INTAKE_OFF = 0.0;

    final double INTAKE_OUT = -0.25;

    final double dist = -80* COUNTS_PER_MM;

    final double dist1 = -600* COUNTS_PER_MM;

    final double WRIST_INTAKE = -1;

    final double WRIST_OUT = 0.45;

    private ElapsedTime auxTimer = new ElapsedTime();

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(8,80, Math.toRadians(0));
    //private final Pose interPose = new Pose(20, 110, Math.toRadians(45));
    private final Pose endPose = new Pose(48, 113, Math.toRadians(90));

    private final Pose nextPose = new Pose( 51, 120, Math.toRadians(90));
    private final Pose lastPose = new Pose(15, 131.5, Math.toRadians(-45));

    private final Pose nextPose1 = new Pose( 51, 126, Math.toRadians(90));

    private final Pose novaPose = new Pose(14.5, 132.5, Math.toRadians(-45));

    //private Path scoreDone, scoreDone2;
    private PathChain scoreDone, scoreNext, scoreFinal,scoreAnother, scoreDone2, scoreNext2;

    private PathChain line1, line2, line3, line4, line5, line6, line7;

    public void buildPaths(){

        scoreDone = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(endPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                //.addPath(new BezierLine(new Point(interPose), new Point(endPose)))
                //.setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .build();
        scoreNext = follower.pathBuilder()
                .addPath(new BezierLine(new Point(endPose), new Point(nextPose)))
                .setLinearHeadingInterpolation(endPose.getHeading(), nextPose.getHeading())
                .build();

        scoreFinal = follower.pathBuilder()
                .addPath(new BezierLine(new Point(nextPose), new Point(lastPose)))
                .setLinearHeadingInterpolation(nextPose.getHeading(), lastPose.getHeading())
                .build();

        scoreAnother = follower.pathBuilder()
                .addPath(new BezierLine(new Point(nextPose), new Point(startPose)))
                .setLinearHeadingInterpolation(nextPose.getHeading(), startPose.getHeading())
                .build();

        scoreDone2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(nextPose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), nextPose1.getHeading())
                .build();

        scoreNext2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(nextPose1), new Point(novaPose)))
                .setLinearHeadingInterpolation(nextPose1.getHeading(), novaPose.getHeading())
                .build();

        line1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(8.000, 80.000, Point.CARTESIAN),
                                new Point(15.063, 104.435, Point.CARTESIAN),
                                new Point(36.552, 94.192, Point.CARTESIAN),
                                new Point(45.311, 96.629, Point.CARTESIAN),
                                new Point(47.899, 114.479, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(47.899, 114.479, Point.CARTESIAN),
                                new Point(19.000, 109.000, Point.CARTESIAN),
                                new Point(12.358, 128.381, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-50))
                .build();

        line3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(12.358, 128.381, Point.CARTESIAN),
                                new Point(22.656, 95.600, Point.CARTESIAN),
                                new Point(34.327, 120.315, Point.CARTESIAN),
                                new Point(50.817, 94.226, Point.CARTESIAN),
                                new Point(49.100, 124.777, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .build();

        line4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(49.100, 124.777, Point.CARTESIAN),
                                new Point(37.073, 108.987, Point.CARTESIAN),
                                new Point(14.074, 130.441, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-50))
                .build();

        line5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(14.074, 130.441, Point.CARTESIAN),
                                new Point(40.449, 107.596, Point.CARTESIAN),
                                new Point(50.966, 117.506, Point.CARTESIAN),
                                new Point(48.944, 127.213, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .build();

        line6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(48.944, 127.213, Point.CARTESIAN),
                                new Point(49.753, 135.303, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        line7 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(49.753, 135.303, Point.CARTESIAN),
                                new Point(43.685, 106.382, Point.CARTESIAN),
                                new Point(15.578, 132.467, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-50))
                .build();

        /*line6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(49.955, 110.022, Point.CARTESIAN),
                                new Point(44.968, 116.539, Point.CARTESIAN),
                                new Point(47.326, 133.483, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .build();*/

    }


    public void autonomousPathUpdate(){
        //follower.followPath(scoreDone);
        /*switch(pathState){
            case 0:
                follower.followPath(scoreDone);
                armExtendToDistance(dist);
                wrist.setPosition(WRIST_INTAKE);
                intake.setPower(INTAKE_IN);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){

                    follower.followPath(scoreNext);
                    setPathState(2);
                    break;
                }
            case 2:
                if(!follower.isBusy()){

                    follower.followPath(scoreFinal);
                    wait(0.2);
                    intake.setPower(INTAKE_OFF);
                    setPathState(3);
                    break;

                }
            case 3:
                if (!follower.isBusy()){
                    //follower.followPath(scoreNova);
                    wait(1.00);
                    armRotateToAngle(ARM_BASKET_ANGLE);
                    wait(2.00);
                    wrist.setPosition(WRIST_OUT);
                    armExtendToDistance(dist1);
                    wait(2.80);
                    intake.setPower(INTAKE_OUT);
                    setPathState(4);
                    break;
                }
            case 4:
                if(!follower.isBusy()){
                    wait(1.00);
                    armRotateToAngle(3* ARM_TICKS_PER_DEGREE);
                    armExtendToDistance(dist);
                    wrist.setPosition(WRIST_INTAKE);
                    intake.setPower(INTAKE_IN);
                    follower.followPath(scoreAnother);
                    setPathState(5);
                    break;
                }
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(scoreDone2);
                    setPathState(6);
                    break;
                }
            case 6:
                if(!follower.isBusy()){
                    follower.followPath(scoreNext2);
                    wait(0.2);
                    intake.setPower(INTAKE_OFF);
                    setPathState(7);
                    break;
                }
            case 7:
                if(!follower.isBusy()){
                    wait(1.00);
                    armRotateToAngle(ARM_BASKET_ANGLE);
                    wait(2.00);
                    wrist.setPosition(WRIST_OUT);
                    armExtendToDistance(dist1);
                    wait(2.80);
                    intake.setPower(-1.0);
                    setPathState(8);
                    break;
                }


        }*/

        switch (pathState){
            case 0:
                follower.followPath(line1);
                armExtendToDistance(dist);
                wrist.setPosition(WRIST_INTAKE);
                intake.setPower(INTAKE_IN);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    intake.setPower(INTAKE_OFF);
                    setPathState(2);
                    break;
                }
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(line2);
                    setPathState(3);
                    break;
                }
            case 3:
                if(!follower.isBusy()){
                    wait(1.00);
                    armRotateToAngle(ARM_BASKET_ANGLE);
                    wait(2.00);

                    armExtendToDistance(dist1);
                    wait(1.00);
                    wrist.setPosition(WRIST_OUT);
                    wait(0.5);
                    intake.setPower(INTAKE_OUT);
                    wait(0.5);
                    setPathState(4);
                    break;
                }
            case 4:
                if(!follower.isBusy()){
                    wait(1.00);
                    follower.followPath(line3);
                    armRotateToAngle(3* ARM_TICKS_PER_DEGREE);
                    armExtendToDistance(dist);
                    wrist.setPosition(WRIST_INTAKE);
                    intake.setPower(INTAKE_IN);
                    setPathState(5);
                    break;
                }
            case 5:
                if(!follower.isBusy()) {
                    wait(1.00);
                    intake.setPower(INTAKE_OFF);
                    setPathState(6);
                    break;
                }
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(line4);
                    setPathState(7);
                    break;
                }
            case 7:
                if(!follower.isBusy()){
                    wait(0.5);
                    armRotateToAngle(ARM_BASKET_ANGLE);
                    wait(1.00);

                    armExtendToDistance(dist1);
                    wait(1.00);
                    wrist.setPosition(WRIST_OUT);
                    wait(0.5);
                    intake.setPower(INTAKE_OUT);
                    wait(0.5);
                    setPathState(8);
                    break;
                }
            case 8:
                if(!follower.isBusy()){
                    wait(1.00);
                    follower.followPath(line5);
                    armRotateToAngle(3* ARM_TICKS_PER_DEGREE);
                    armExtendToDistance(dist);
                    wrist.setPosition(WRIST_INTAKE);
                    intake.setPower(INTAKE_IN);
                    setPathState(9);
                    break;
                }
            case 9:
                if(!follower.isBusy()){
                    follower.followPath(line6);
                    setPathState(10);
                    break;
                }
            case 10:
                if(!follower.isBusy()) {
                    wait(0.5);
                    intake.setPower(INTAKE_OFF);
                    setPathState(11);
                    break;
                }

            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(line7);
                    setPathState(12);
                    break;
                }
            case 12:
                if(!follower.isBusy()){
                    wait(0.5);
                    armRotateToAngle(ARM_BASKET_ANGLE);
                    wait(1.00);

                    armExtendToDistance(dist1);
                    wait(1.00);
                    wrist.setPosition(WRIST_OUT);
                    wait(0.5);
                    intake.setPower(INTAKE_OUT);
                    wait(0.5);
                    setPathState(13);
                    break;
                }

        }
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

    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
    }

    @Override
    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        initialize();
        buildPaths();
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

    @Override
    public void init_loop(){}

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop(){}

}
