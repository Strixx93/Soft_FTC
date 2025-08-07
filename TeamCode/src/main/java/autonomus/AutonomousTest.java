package autonomus;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//import java.util.Timer;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class AutonomousTest extends OpMode {

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
    private int pathState;

    private Timer pathTimer, actionTimer, opmodeTimer;
    //private Path park, scorePathState;
    private final Pose point0 = new Pose(8.000, 80.000, Math.toRadians(0));
    private PathChain point1, point2, point3, point4, point5,point6;

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


    public void buildPaths() {
        point1 = follower.pathBuilder()
         .addPath(
                // Line 1
                 new BezierCurve(
                         new Point(8.000, 80.000, Point.CARTESIAN),
                         new Point(28.112, 83.730, Point.CARTESIAN),
                         new Point(47.124, 109.416, Point.CARTESIAN)
                )
        )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
        point2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(47.124, 109.416, Point.CARTESIAN),
                                new Point(31.753, 112.247, Point.CARTESIAN),
                                new Point(18.809, 125.393, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-45))
                .build();
        point3=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(18.809, 125.393, Point.CARTESIAN),
                                new Point(28.719, 98.494, Point.CARTESIAN),
                                new Point(48.135, 121.753, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(point1);
                armExtendToDistance(dist);
                wrist.setPosition(WRIST_INTAKE);
                intake.setPower(INTAKE_IN);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(point2,true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                follower.followPath(point3,true);
                setPathState(3);
            }
            break;


        }


    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(point0);
        buildPaths();
    }
    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}




