package mechanisms;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ProgrammingBoard {

    //TargetHardware
    public DcMotor leftFront, leftRear, rightFront, rightRear;
    public DcMotor armExtend, armRotate;
    public CRServo intake;
    public Servo wrist;
    //public GoBildaPinpointDriver odometry;

    //DcMotorConstants
    public static final double COUNTS_PER_MOTOR_REV = 28;   // Encoder counts per motor revolution
    public static final double GEAR_REDUCTION = 13.7;       // Gearbox ratio
    public static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION; // = 28 * 19.2 = 537.6
    public static final double WHEEL_DIAMETER_MM = 38.2;
    public static final double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * Math.PI;
    public static final double COUNTS_PER_MM = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM);
    public static double ARM_TICKS_PER_DEGREE = 19.7924893140647;

    //DcMotorVariables
    public static final double ARM_BASKET_ANGLE = 107.5 * ARM_TICKS_PER_DEGREE;
    public static double dist = -80 * COUNTS_PER_MM;
    public static double dist1 = -600 * COUNTS_PER_MM;

    //CRS&ServoVariables
    public static final double INTAKE_IN = 0.5;
    public static final double INTAKE_OFF = 0.0;
    public static final double INTAKE_OUT = -0.25;
    public static final double WRIST_INTAKE = -1;
    public static final double WRIST_OUT = 0.45;

    public void init(HardwareMap hwMap) {

        intake = hwMap.get(CRServo.class, "intake");
        armExtend = hwMap.get(DcMotor.class, "armExtend");
        armRotate = hwMap.get(DcMotor.class, "armRotate");
        wrist = hwMap.get(Servo.class, "wrist");


        /*intake.setPower(INTAKE_OFF);*/


        armExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtend.setTargetPosition(0);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armRotate.setTargetPosition(0);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /*odometry = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.resetPosAndIMU();*/



    }

    public void setMecanumPower(double forward, double strafe, double turn) {
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;
        double max = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower)));
        if (max > 1) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }
        leftFront.setPower(flPower);
        leftRear.setPower(blPower);
        rightFront.setPower(frPower);
        rightRear.setPower(brPower);
    }

    public void stopDrive() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setIntakePowerIn() {
        intake.setPower(INTAKE_IN);
    }

    public void setIntakePowerOFF() {
        intake.setPower(INTAKE_OFF);
    }

    public void setIntakePowerOUT() {
        intake.setPower(INTAKE_OUT);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

    public void setWristPositionIN() {
        wrist.setPosition(WRIST_INTAKE);
    }

    public void setWristPositionOUT() {
        wrist.setPosition(WRIST_OUT);
    }

    public void armExtendToDistance(double dist){
        armExtend.setTargetPosition((int)dist);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx)armExtend).setVelocity(2500);
    }

    public void armExtendToDistanceIN(){
        armExtend.setTargetPosition((int)dist);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx)armExtend).setVelocity(2500);
    }

    public void armRotateToAngle(double angle){
        armRotate.setTargetPosition((int)angle);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx)armRotate).setVelocity(2500);
    }

    /*public Pose2D getPose() {
        odometry.update();
        return odometry.getPosition();
    }*/
}