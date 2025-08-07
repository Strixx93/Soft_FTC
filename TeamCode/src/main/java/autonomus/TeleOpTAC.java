package pedroPathing;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.FF_Settings.FF_Hardware;
import pedroPathing.FF_Settings.ConstantsTAC;
import pedroPathing.Limelight.ExtCoords;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.util.List;

@Config
@TeleOp
public class TeleOpTAC extends OpMode { // with manual bulk reading
    final static boolean isDebugMode = true;
    FF_Hardware hardware = new FF_Hardware();
    private ElapsedTime runTime;

    List<LynxModule> allHubs;

    double lastLoopTime = 0;
    double avgLoopTime = 0;
    int loopCounter = 0;
    double resumeTime = 0;

    private Limelight3A limelight;

    private final int pipelineYellow = 0;
    private final int pipelineBlue = 6;
    private  final int pipelineRed = 7;
    int desiredPipeline = pipelineYellow; // TODO: make a global variable that can hold two colors (i.e. yellow and red)

    double txValue = 0;
    int roundedTXValue = 0;
    double distanceErrorY = 0;
    double computedY = 0;
    double distanceToMoveYPedro = 0;
    LLResult llresult = null;
    boolean isCoordinateInClawPath = false;
    boolean isLimeLightActive = true;
    boolean isUsingLimeLightToGrabSample = false;
    boolean isFindingSample = false;

    boolean isBlueSide = true;

    final int[] sampleDistanceArray =
            {ExtCoords.txCoord0,
            ExtCoords.txCoord1,
            ExtCoords.txCoord2,
            ExtCoords.txCoord3,
            ExtCoords.txCoord4,
            ExtCoords.txCoord5,
            ExtCoords.txCoord6,
            ExtCoords.txCoord7,
            ExtCoords.txCoord8,
            ExtCoords.txCoord9,
            ExtCoords.txCoord10,
            ExtCoords.txCoord11,
            ExtCoords.txCoord12,
            ExtCoords.txCoord13,
            ExtCoords.txCoord14,
            ExtCoords.txCoord15,
            ExtCoords.txCoord16,
            ExtCoords.txCoord17,
            ExtCoords.txCoord18,
            ExtCoords.txCoord19,
            ExtCoords.txCoord20,
            ExtCoords.txCoord21,
            ExtCoords.txCoord22,
            ExtCoords.txCoord23,
            ExtCoords.txCoord24,
            ExtCoords.txCoord25,
            ExtCoords.txCoord26};

    final double txCoordsMax = 26;
    final double txCoordsMin = 0;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    int targetLiftPosition = 0;
    int liftSpeedIncrement = 40;
    boolean isLiftBeingReset = false;

    int targetExtensionPosition = 0;
    int extensionSpeedIncrement = 75;
    boolean isExtensionBeingReset = false;

    int targetLiftHangPosition = 0;
    int liftHangSpeedIncrement = 75;
    boolean isLiftHangBeingReset = false;
    boolean isMovingWheelsWhileHanging = false;

    boolean isClawOpen = true;
    boolean isLiftClawOpen = true;

    boolean isWaitingToScoreSpecimen = false;

    enum State {
        CLAW_PICKUP_STATE,
        BUCKET_DUMP_STATE,
        SPECIMEN_SCORE_STATE,
        HANG_STATE,
        NONE,
    }
    State currentState = State.NONE;

    enum ClawPickupState {
        // LimeLight specific
        PREPARE_TO_MOVE_EXTENSION_WITH_LIMELIGHT,
        EXTEND_TO_SAMPLE,
        WAIT_FOR_HUMAN_CLAW_ROTATE,
        // not part of LimeLight
        SET_SERVO_POSITIONS1,
        SET_SERVO_POSITIONS2,
        SET_SERVO_POSITIONS3,
        SET_SERVO_POSITIONS4,
        OPEN_LIFT_CLAW,
        CHECK_FOR_COLOR_SENSOR1,
        CHECK_FOR_COLOR_SENSOR,
        PULL_EXTENSION_IN_FOR_HANDOFF,
        CLOSE_LIFT_CLAW,
        OPEN_BOTTOM_CLAW,
        SET_POSITIONS_FOR_SCORING,
        CLAW_PICKUP_IDLE
    }
    ClawPickupState clawPickupCurrentState = ClawPickupState.CLAW_PICKUP_IDLE;

    enum BucketState {
        LIFT_VERTICAL_EXTENSION,
        MOVE_LIFT_TILT,
        DUMP_POSITION,
        BUCKET_LIFT_TILT_HOME_POSITION,
        VERTICAL_SLIDES_DOWN,
        BUCKET_IDLE
    }
    BucketState bucketCurrentState = BucketState.BUCKET_IDLE;

    enum SpecimenState {
        WAIT_TO_GET_OFF_WALL,
        SET_TILT_POSITION,
        WAIT_TO_PROCEED,
        LIFT_UP_TO_SCORE_SPECIMEN,
        RELEASE_CLAW,
        RESET_POSITIONS,
        SPECIMEN_IDLE
    }
    SpecimenState specimenCurrentState = SpecimenState.SPECIMEN_IDLE;

    enum HangState {
        PREPARE_FOR_HANG,
        LIFT_UP_FOR_LEVEL2_HANG,
        IS_WAITING_FOR_HUMAN_TO_START_HANG,
        RELEASE_HOOKS,
        LIFT_DOWN_FOR_LEVEL2_HANG,
        LIFT_UP_FOR_LEVEL3_HANG,
        LIFT_HANG_COMPLETE,
        HANG_IDLE
    }
    HangState hangCurrentState = HangState.HANG_IDLE;
    boolean isPreparingForHang = false;

    enum SampleColor {
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    final double [] LEDColorArray = {
            0,      // no color
            0.279,  // red
            0.333,  // yellow (Gobilda orange)
            0.500,  // green
            0.611,  // blue
            0.722,  // violet
            1.0};   // white

    enum LEDColors {
        NO_COLOR,
        RED,
        YELLOW, // GoBilda Orange
        GREEN,
        BLUE,
        VIOLET,
        WHITE
    }
    LEDColors ledColor = LEDColors.WHITE;

    double clawPosition = 0;
    double clawCurrentSetDeg = 0;
    double clawError = 0;
    double liftClawPosition = 0;
    double liftClawCurrentSetDeg = 0;
    double liftClawError = 0;
    double liftTiltPosition = 0;
    double liftTiltCurrentSetDeg = 0;
    double liftTiltError = 0;

    private Timer pathTimer;
    boolean isSwitchingToDrive = false;
    boolean isDrivingToSample = false;
    private Follower follower;
    final private Pose startPose = new Pose(11, 133, Math.toRadians(325));
    private Pose bucketPose = new Pose(11, 133, Math.toRadians(325));
    private Pose distanceToSampleYPedro = new Pose(0, 0, 0);

    private PathChain scoreBucket, movingToSampleY;
    public void buildPaths() {
        scoreBucket = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(bucketPose)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), bucketPose.getHeading())
                .build();

//        movingToSampleY = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(distanceToSampleYPedro)))
//                .setLinearHeadingInterpolation(follower.getPose().getHeading(), distanceToSampleYPedro.getHeading())
//                .setZeroPowerAccelerationMultiplier(1)
//                .build();
    }

    @Override
    public void init_loop() {

        if (gamepad1.b) {
            isBlueSide = false;
        } else if (gamepad1.x) {
            isBlueSide = true;
        }

        if (gamepad2.y) {
            desiredPipeline = pipelineYellow;
            ledColor = LEDColors.YELLOW;
        } else if (gamepad2.x) {
            desiredPipeline = pipelineBlue;
            isBlueSide = true;
            ledColor = LEDColors.BLUE;
        } else if (gamepad2.b) {
            desiredPipeline = pipelineRed;
            isBlueSide = false;
            ledColor = LEDColors.RED;
        }

        hardware.lightIndicator.setPosition(LEDColorArray[ledColor.ordinal()]);

        telemetry.addLine("Init Loop");
        telemetry.addData("LED Color", ledColor);
        telemetry.addData("desiredPipeline:", desiredPipeline);
        telemetry.addData("isBlueSide", isBlueSide);
        telemetry.update();
    }

    @Override
    public void init() {
        hardware.init(hardwareMap);
        runTime = new ElapsedTime();
        pathTimer = new Timer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        // Set bulk caching mode
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.setMsTransmissionInterval(10);
        telemetry.addLine("INIT ver. 1");
        telemetry.update();
    }

    @Override
    public void start(){
        runTime.reset();

        follower.startTeleopDrive();

        hardware.claw.setPosition(ConstantsTAC.CLAW_OPEN_POSITION);
        hardware.clawTilt.setPosition(ConstantsTAC.CLAW_TILT_UP_POSITION);
        hardware.clawRotate.setPosition(ConstantsTAC.CLAW_ROTATE_VERTICAL_POSITION);
        hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_STRAIGHT_UP_POSITION);
        hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_CLOSE_POSITION);
        hardware.hangRight.setPosition(ConstantsTAC.HANG_RIGHT_SERVO_HOLD_POSITION);
        hardware.hangLeft.setPosition(ConstantsTAC.HANG_LEFT_SERVO_HOLD_POSITION);
        isClawOpen = true;
        isLiftClawOpen = true;

        ledColor = LEDColors.WHITE;
        hardware.headlight.setPosition(1.0);

        hardware.extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.extension.setTargetPosition(targetExtensionPosition);
        hardware.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.lift.setTargetPosition(targetLiftPosition);
        hardware.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.liftHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.liftHang.setTargetPosition(targetLiftHangPosition);
        hardware.liftHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        limelight.pipelineSwitch(desiredPipeline);
        limelight.start();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop()
    {
        try {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
        }
        catch (Exception e) {
            telemetry.addLine("Exception Assigning Gamepads. " + e);
        }

        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        /* Game Controls
         * Gamepad 1:
         * right stick - drive straight
         * left_stick - drive turn

         * right_trigger - liftHang out
         * left_trigger - liftHang in

         * right_bumper - reset bucket score pedro position
         * a - drive to bucket score pedro position

         * Gamepad2:
         * left_stick  - lift
         * right_stick - extension

         * y - pickup claw hover
         * b - pickup claw turn Clockwise 45 degrees
         * x - pickup claw turn Counter Clockwise 45 degrees
         * a - extension retract with LimeLight

         * start - claw down
         * back - claw handoff position

         * dpad_up - getting into lift bucket dump state machine
         * dpad_right - lift high chamber position
         * dpad_left - lift wrist specimen pickup position
         * dpad_down - reset position

         * right_trigger - getting into extension retract state machine
         * left_trigger - getting into lift hang state machine

         * right_bumper - open/close pickup claw
         * left_bumper - open/close lift claw
         */

        // Gamepad 1 Controls
        // Wheel Drive
        {
            if (currentGamepad1.a && !previousGamepad1.a) {
                buildPaths();
                follower.followPath(scoreBucket, true);
                isSwitchingToDrive = true;
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                bucketPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
            }

            if (isSwitchingToDrive) {
                if ((Math.abs(gamepad1.left_stick_y) > 0.1) || (Math.abs(gamepad1.left_stick_x) > 0.1)
                        || (Math.abs(gamepad1.right_stick_x) > 0.1)) {
                    follower.startTeleopDrive();
                    isSwitchingToDrive = false;
                }
            }

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
        }

        //Limelight
        {
            if (currentGamepad2.a && !previousGamepad2.a && !isFindingSample) {
                isFindingSample = true;
                currentState = State.NONE;
                isLimeLightActive = true;
            }

            if (isLimeLightActive) {
                llresult = limelight.getLatestResult();
//                LLStatus status = limelight.getStatus();
//                telemetry.addData("Name", "%s",
//                        status.getName());
//                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                        status.getTemp(), status.getCpu(), (int) status.getFps());
//                telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                        status.getPipelineIndex(), status.getPipelineType());

                if (llresult != null) {
                    // Access general information
//                    double captureLatency = llresult.getCaptureLatency();
//                    double targetingLatency = llresult.getTargetingLatency();
//                    double parseLatency = llresult.getParseLatency();
//                    telemetry.addData("LL Latency", captureLatency + targetingLatency);
//                    telemetry.addData("Parse Latency", parseLatency);

                    txValue = llresult.getTx();
                    roundedTXValue = roundTXValue(txValue);

                    telemetry.addData("tx", llresult.getTx());
                    telemetry.addData("ty", llresult.getTy());

                    computedY = (0.39 * txValue) - 17.3;
                    distanceErrorY = computedY - llresult.getTy();
                    isCoordinateInClawPath = (Math.abs(distanceErrorY) <= getDistanceErrorY(desiredPipeline));

                    if (isCoordinateInClawPath && (roundedTXValue > 0) && (roundedTXValue <= 26)){
                        ledColor = LEDColors.GREEN;
                    } else {
                        ledColor = LEDColors.WHITE;
                    }

                    distanceToMoveYPedro = (Math.abs(distanceErrorY) - getDistanceErrorY(desiredPipeline)) / 3.25; // gives out distance in inches

                    if (currentGamepad1.b && !previousGamepad1.b && !isCoordinateInClawPath) {
                        if (distanceErrorY > 0) { // move left
                            distanceToSampleYPedro = new Pose(follower.getPose().getX(),
                                    (follower.getPose().getY() - distanceToMoveYPedro), follower.getPose().getHeading());
                            isDrivingToSample = true;
                        } else if (distanceErrorY < 0) { // move right
                            distanceToSampleYPedro = new Pose(follower.getPose().getX(),
                                    (follower.getPose().getY() + distanceToMoveYPedro), follower.getPose().getHeading());
                            isDrivingToSample = true;
                        }
                    }

                    if (currentGamepad1.x && !previousGamepad1.x) {
                        if (isDrivingToSample) {
//                            buildPaths();
//                            follower.followPath(movingToSampleY, true);
                            follower.holdPoint(distanceToSampleYPedro);
//                            follower.setMaxPower(0.6);
                            isDrivingToSample = true;
                        }
                    }
                } else {
                    telemetry.addData("Limelight", "No data available");
                }
            } else {
                isCoordinateInClawPath = false;
            }

            if (currentGamepad2.a && !previousGamepad2.a && isCoordinateInClawPath && (roundedTXValue > txCoordsMin) && (roundedTXValue <= txCoordsMax)) {
                currentState = State.CLAW_PICKUP_STATE;
                setClawPickupCurrentState(ClawPickupState.PREPARE_TO_MOVE_EXTENSION_WITH_LIMELIGHT);
            }
        }

        // Lift Hang
        {
            if ((hardware.liftHangTouch.isPressed()) && (currentGamepad1.left_trigger > 0.2)) {
                targetLiftHangPosition -= (int) (Math.abs(currentGamepad1.left_trigger * liftHangSpeedIncrement));
                isLiftHangBeingReset = true;
            } else if (currentGamepad1.right_trigger > 0.2) {
                targetLiftHangPosition += (int) (Math.abs(currentGamepad1.right_trigger * liftHangSpeedIncrement));
                isLiftHangBeingReset = false;
            }

            if (isLiftHangBeingReset) {
                if (!hardware.liftHangTouch.isPressed()) {
                    targetLiftHangPosition = 0;
                    hardware.liftHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    isLiftHangBeingReset = false;
                }
            } else if (targetLiftHangPosition > ConstantsTAC.MAX_VERTICAL_LIFT_HANG_POSITION) {
                targetLiftHangPosition = ConstantsTAC.MAX_VERTICAL_LIFT_HANG_POSITION;
            }

            if (currentGamepad2.back && !previousGamepad2.back && !isPreparingForHang) { // Getting into hang state machine
                isPreparingForHang = true;
                currentState = State.HANG_STATE;
                setHangCurrentState(HangState.PREPARE_FOR_HANG);
            }
        }

        // Gamepad 2 Controls
        // Lift
        {
            if ((hardware.liftTouch.isPressed()) && (currentGamepad2.left_stick_y > 0.2)) {
                currentState = State.NONE;
                targetLiftPosition -= (int) (Math.abs(currentGamepad2.left_stick_y * liftSpeedIncrement));
                isLiftBeingReset = true;
            } else if (currentGamepad2.left_stick_y < -0.2) {
                currentState = State.NONE;
                targetLiftPosition += (int) (Math.abs(currentGamepad2.left_stick_y * liftSpeedIncrement));
                isLiftBeingReset = false;
            } else if (currentGamepad2.dpad_up) { // Getting into bucket state machine
                hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_DOWN_POSITION);
                hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_CLOSE_POSITION);
                isLiftClawOpen = false;
                currentState = State.BUCKET_DUMP_STATE;
                setBucketScoreCurrentState(BucketState.LIFT_VERTICAL_EXTENSION);
            } else if (currentGamepad2.dpad_right && !isWaitingToScoreSpecimen && !previousGamepad2.dpad_right) { // Getting into specimen state machine
                currentState = State.SPECIMEN_SCORE_STATE;
                specimenCurrentState = SpecimenState.WAIT_TO_GET_OFF_WALL;
            } else if (currentGamepad2.dpad_left) { // get ready for specimen pickup
                currentState = State.NONE;
                targetLiftPosition = -80;
                isLiftBeingReset = true;
                hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_SPECIMEN_PICKUP_WIDE);
                hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_OPEN_POSITION);
                isLiftClawOpen = true;
            } else if (currentGamepad2.dpad_down) { // Rest position for everything & prepare for sample dump through back
                currentState = State.NONE;
                hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_SPECIMEN_PICKUP_WIDE);
                hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_CLOSE_POSITION);
                hardware.claw.setPosition(ConstantsTAC.CLAW_OPEN_POSITION);
                hardware.clawTilt.setPosition(ConstantsTAC.CLAW_TILT_UP_POSITION);
                hardware.clawRotate.setPosition(ConstantsTAC.CLAW_ROTATE_VERTICAL_POSITION);
                isLiftClawOpen = true;
                isClawOpen = true;
                targetExtensionPosition = -80;
                isExtensionBeingReset = true;
                targetLiftPosition = -80;
                isLiftBeingReset = true;
                hardware.lift.setPower(0.5);
                targetLiftHangPosition = -80;
                isLiftHangBeingReset = true;
                isLimeLightActive = true;
            }

            if (isLiftBeingReset) {
                if (!hardware.liftTouch.isPressed()) {
                    targetLiftPosition = 0;
                    hardware.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    isLiftBeingReset = false;
                }
            } else if (targetLiftPosition > ConstantsTAC.MAX_VERTICAL_LIFT_POSITION) {
                targetLiftPosition = ConstantsTAC.MAX_VERTICAL_LIFT_POSITION;
            }
        }

        // Extension
        {
            if ((hardware.extensionTouch.isPressed()) && (currentGamepad2.right_stick_y > 0.02)) {
                currentState = State.NONE;
                isFindingSample = false;
                targetExtensionPosition -= (int) Math.abs(currentGamepad2.right_stick_y * extensionSpeedIncrement);
                isExtensionBeingReset = true;
                hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_HOVER_POSITION);
                hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_OPEN_POSITION);
            } else if (currentGamepad2.right_stick_y < -0.02) {
                currentState = State.NONE;
                isFindingSample = false;
                targetExtensionPosition += (int) (Math.abs(currentGamepad2.right_stick_y * extensionSpeedIncrement));
                hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_HOVER_POSITION);
                hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_OPEN_POSITION);
                isExtensionBeingReset = false;
            }

            if ((currentGamepad2.right_trigger > 0.2)) { // getting into extension retract state machine
                isLiftClawOpen = true;
                hardware.clawTilt.setPosition(ConstantsTAC.CLAW_TILT_HOVER);
                currentState = State.CLAW_PICKUP_STATE;
                setClawPickupCurrentState(ClawPickupState.SET_SERVO_POSITIONS1);
            }

            if (isExtensionBeingReset) {
                if (!hardware.extensionTouch.isPressed()) {
                    targetExtensionPosition = 0;
                    hardware.extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    isExtensionBeingReset = false;
                }
            } else if (targetExtensionPosition > ConstantsTAC.MAX_EXTENSION_POSITION) {
                targetExtensionPosition = ConstantsTAC.MAX_EXTENSION_POSITION;
            }
        }

        switch (currentState) {
            case CLAW_PICKUP_STATE: {
                switch (clawPickupCurrentState) {
                    case PREPARE_TO_MOVE_EXTENSION_WITH_LIMELIGHT: {
                        isUsingLimeLightToGrabSample = true;
                        isLimeLightActive = false;
                        hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_HOVER_POSITION);
                        hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_OPEN_POSITION);
                        targetExtensionPosition = (sampleDistanceArray[roundedTXValue]) + (hardware.extension.getCurrentPosition());
                        hardware.extension.setPower(1.0);
                        telemetry.addData("sampleDistanceArray", sampleDistanceArray[roundedTXValue]);
                        setClawPickupCurrentState(ClawPickupState.EXTEND_TO_SAMPLE);
                        break;
                    }

                    case EXTEND_TO_SAMPLE: {
                        if ((Math.abs(hardware.extension.getCurrentPosition() - targetExtensionPosition) < 20)) {
                            if (getRuntime() >= resumeTime) {
                                hardware.claw.setPosition(ConstantsTAC.CLAW_VERY_OPEN_POSITION);
                                hardware.clawTilt.setPosition(ConstantsTAC.CLAW_TILT_HOVER);
                                isLiftClawOpen = true;
                                setClawPickupCurrentState(ClawPickupState.WAIT_FOR_HUMAN_CLAW_ROTATE);
                            }
                        }
                        break;
                    }

                    case WAIT_FOR_HUMAN_CLAW_ROTATE: {
                        if (currentGamepad2.a && isFindingSample) {
                            isFindingSample = false;
                            setClawPickupCurrentState(ClawPickupState.SET_SERVO_POSITIONS1);
                        }
                        break;
                    }

                    case SET_SERVO_POSITIONS1: {
                        ledColor = LEDColors.VIOLET;
                        targetLiftPosition = -80;
                        isLiftBeingReset = true;
                        hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_HOVER_POSITION);
                        hardware.claw.setPosition(ConstantsTAC.CLAW_VERY_OPEN_POSITION);
                        isClawOpen = true;
                        setClawPickupCurrentState(ClawPickupState.SET_SERVO_POSITIONS2);
                        break;
                    }

                    case SET_SERVO_POSITIONS2: {
                        hardware.clawTilt.setPosition(ConstantsTAC.CLAW_TILT_DOWN_POSITION);
                        isLiftClawOpen = false;
                        if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                            setClawPickupCurrentState(ClawPickupState.SET_SERVO_POSITIONS3);
                        }
                        break;
                    }

                    case SET_SERVO_POSITIONS3: {
                        if (isUsingLimeLightToGrabSample && pathTimer.getElapsedTimeSeconds() > 0.1) {
                            isUsingLimeLightToGrabSample = false;
                            hardware.claw.setPosition(ConstantsTAC.CLAW_CLOSE_POSITION);
                            isClawOpen = false;
                            if ((clawError < ConstantsTAC.CLAW_ERROR_THRESHOLD) || (pathTimer.getElapsedTimeSeconds() > 0.2)) {
                                setClawPickupCurrentState(ClawPickupState.SET_SERVO_POSITIONS4);
                            }
                        } else if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                            hardware.claw.setPosition(ConstantsTAC.CLAW_CLOSE_POSITION);
                            isClawOpen = false;
                            if ((clawError < ConstantsTAC.CLAW_ERROR_THRESHOLD) || (pathTimer.getElapsedTimeSeconds() > 0.2)) {
                                setClawPickupCurrentState(ClawPickupState.CHECK_FOR_COLOR_SENSOR);
                            }
                        }
                        break;
                    }

                    case CHECK_FOR_COLOR_SENSOR: {
                        if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                            if (foundColor() == SampleColor.YELLOW) {
                                ledColor = LEDColors.YELLOW;
                                setClawPickupCurrentState(ClawPickupState.SET_SERVO_POSITIONS4);
                            } else if ((foundColor() == SampleColor.RED)) {
                                if (isBlueSide) {
                                    ledColor = LEDColors.RED;
                                    hardware.claw.setPosition(ConstantsTAC.CLAW_VERY_OPEN_POSITION);
                                    setClawPickupCurrentState(ClawPickupState.WAIT_FOR_HUMAN_CLAW_ROTATE);
                                } else {
                                    ledColor = LEDColors.RED;
                                    setClawPickupCurrentState(ClawPickupState.SET_SERVO_POSITIONS4);
                                }
                            } else if (foundColor() == SampleColor.BLUE) {
                                if (!isBlueSide) {
                                    ledColor = LEDColors.BLUE;
                                    hardware.claw.setPosition(ConstantsTAC.CLAW_VERY_OPEN_POSITION);
                                    setClawPickupCurrentState(ClawPickupState.WAIT_FOR_HUMAN_CLAW_ROTATE);
                                } else {
                                    ledColor = LEDColors.BLUE;
                                    setClawPickupCurrentState(ClawPickupState.SET_SERVO_POSITIONS4);
                                }
                            } else if ((foundColor() == SampleColor.NONE)) {
                                if ((pathTimer.getElapsedTimeSeconds() > 1.0)) {
                                    ledColor = LEDColors.GREEN;
                                    setClawPickupCurrentState(ClawPickupState.CLAW_PICKUP_IDLE);
                                }
                            }
                        }
                        break;
                    }

                    case SET_SERVO_POSITIONS4: {
                        hardware.claw.setPosition(ConstantsTAC.CLAW_CLOSE_POSITION);
                        hardware.clawTilt.setPosition(ConstantsTAC.CLAW_TILT_UP_POSITION);
                        setClawPickupCurrentState(ClawPickupState.OPEN_LIFT_CLAW);
                        break;
                    }

                    //TODO: if extension is too close in, the tilt servo will hit the lift tilt servo; you will have to fix this

                    case OPEN_LIFT_CLAW: {
                        hardware.clawRotate.setPosition(ConstantsTAC.CLAW_ROTATE_VERTICAL_POSITION);
                        hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_OPEN_POSITION);
                        isLiftClawOpen = true;
                        setClawPickupCurrentState(ClawPickupState.PULL_EXTENSION_IN_FOR_HANDOFF);
                        break;
                    }

                    case PULL_EXTENSION_IN_FOR_HANDOFF: {
                        targetExtensionPosition = ConstantsTAC.EXTENSION_CLAW_HANDOFF_POSITION;
                        isExtensionBeingReset = false;
                        hardware.extension.setPower(1.0);
                        hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_DOWN_POSITION);
                        if ((Math.abs(hardware.extension.getCurrentPosition() - ConstantsTAC.EXTENSION_CLAW_HANDOFF_POSITION ) < 30)) {
                            setClawPickupCurrentState(ClawPickupState.CLOSE_LIFT_CLAW);
                        }
                        break;
                    }

                    case CLOSE_LIFT_CLAW: {
                        if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                            hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_CLOSE_POSITION);
                            isLiftClawOpen = false;
                            ledColor = LEDColors.GREEN;
                            if ((liftClawError < ConstantsTAC.LIFT_CLAW_ERROR_THRESHOLD) || (pathTimer.getElapsedTimeSeconds() > 0.5)) {
                                setClawPickupCurrentState(ClawPickupState.OPEN_BOTTOM_CLAW);
                            }
                        }
                        break;
                    }

                    case OPEN_BOTTOM_CLAW: {
                        if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                            hardware.claw.setPosition(ConstantsTAC.CLAW_OPEN_POSITION);
                            isClawOpen = true;
                            if ((clawError < 7) || (pathTimer.getElapsedTimeSeconds() > 0.3)) {
                                setClawPickupCurrentState(ClawPickupState.SET_POSITIONS_FOR_SCORING);
                                isLimeLightActive = true;
                            }
                        }
                        break;
                    }

                    case SET_POSITIONS_FOR_SCORING: {
                        hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_HOVER_POSITION);
                        targetExtensionPosition = -80;
                        isExtensionBeingReset = true;
                        setClawPickupCurrentState(ClawPickupState.CLAW_PICKUP_IDLE);
                        break;
                    }

                    case CLAW_PICKUP_IDLE: {
                        // Get out of State Machine
                        isFindingSample = false;
                        currentState = State.NONE;
                        break;
                    }
                }
                break;
            }

            case BUCKET_DUMP_STATE: {
                switch (bucketCurrentState) {
                    case LIFT_VERTICAL_EXTENSION: {
                        ledColor = LEDColors.VIOLET;
                        targetLiftPosition = ConstantsTAC.VERTICAL_LIFT_POSITION_HIGH_BUCKET;
                        isLiftBeingReset = false;
                        hardware.lift.setPower(1.0);
                        hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_STRAIGHT_UP_POSITION);
                        hardware.claw.setPosition(ConstantsTAC.CLAW_OPEN_POSITION);
                        if (hardware.lift.getCurrentPosition() > 1400) {
                            setBucketScoreCurrentState(BucketState.MOVE_LIFT_TILT);
                        }

                        break;
                    }

                    case MOVE_LIFT_TILT: {
                        hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_BUCKET_POSITION);
                        targetExtensionPosition = -80;
                        isExtensionBeingReset = true;
                        if (Math.abs(hardware.lift.getCurrentPosition() - ConstantsTAC.VERTICAL_LIFT_POSITION_HIGH_BUCKET) < 100) {
                            setBucketScoreCurrentState(BucketState.DUMP_POSITION);
                        }
                        break;
                    }

                    case DUMP_POSITION: {
                        if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                            hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_OPEN_POSITION);
                            isLiftClawOpen = true;
                            if ((liftTiltError < ConstantsTAC.LIFT_TILT_ERROR_THRESHOLD)
                                    && (liftClawError < ConstantsTAC.LIFT_CLAW_ERROR_THRESHOLD) || (pathTimer.getElapsedTimeSeconds() > 0.3)) {
                                setBucketScoreCurrentState(BucketState.BUCKET_LIFT_TILT_HOME_POSITION);
                            }
                        }
                        break;
                    }

                    case BUCKET_LIFT_TILT_HOME_POSITION: {
                        if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                            hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_DOWN_POSITION);
                            if ((liftTiltError < ConstantsTAC.LIFT_TILT_ERROR_THRESHOLD) || (pathTimer.getElapsedTimeSeconds() > 0.25)){
                                setBucketScoreCurrentState(BucketState.VERTICAL_SLIDES_DOWN);
                            }
                        }
                        break;
                    }

                    case VERTICAL_SLIDES_DOWN: {
                        targetLiftPosition = -80;
                        isLiftBeingReset = true;
                        hardware.lift.setPower(0.5);
                        setBucketScoreCurrentState(BucketState.BUCKET_IDLE);
                        break;
                    }

                    case BUCKET_IDLE: {
                        // Get out of State Machine
                        currentState = State.NONE;
                        break;
                    }
                }
                break;
            }

            case SPECIMEN_SCORE_STATE: {
                switch (specimenCurrentState) {
                    case WAIT_TO_GET_OFF_WALL: {
                        hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_CLOSE_POSITION);
                        isLiftClawOpen = false;
                        targetLiftPosition = ConstantsTAC.VERTICAL_LIFT_SLIGHTLY_UP;
                        isLiftBeingReset = false;
                        if (Math.abs(hardware.lift.getCurrentPosition() - ConstantsTAC.VERTICAL_LIFT_SLIGHTLY_UP) < 30) {
                            setSpecimenScoreCurrentState(SpecimenState.SET_TILT_POSITION);
                        }
                        break;
                    }

                    case SET_TILT_POSITION: {
                        hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_CHAMBER_POSITION);
                        targetLiftPosition = -20;
                        isLiftBeingReset = true;
                        hardware.lift.setPower(1.0);
                        isWaitingToScoreSpecimen = true;
                        setSpecimenScoreCurrentState(SpecimenState.WAIT_TO_PROCEED);
                        break;
                    }

                    case WAIT_TO_PROCEED: {
                        if (currentGamepad2.dpad_right && isWaitingToScoreSpecimen && !previousGamepad2.dpad_right) {
                            setSpecimenScoreCurrentState(SpecimenState.LIFT_UP_TO_SCORE_SPECIMEN);
                        }
                        break;
                    }

                    case LIFT_UP_TO_SCORE_SPECIMEN: {
                        isWaitingToScoreSpecimen = false;
                        targetLiftPosition = ConstantsTAC.VERTICAL_LIFT_POSITION_HIGH_CHAMBER;
                        isLiftBeingReset = false;
                        hardware.lift.setPower(1.0);
                        if ((hardware.lift.getCurrentPosition() > (380))) {
                            setSpecimenScoreCurrentState(SpecimenState.RELEASE_CLAW);
                        }

                        break;
                    }

                    case RELEASE_CLAW: {
                        hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_OPEN_POSITION);
                        isLiftClawOpen = true;
                        setSpecimenScoreCurrentState(SpecimenState.RESET_POSITIONS);

                        break;
                    }

                    case RESET_POSITIONS: {
                        if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                            hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_SPECIMEN_PICKUP_WIDE);
                            hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_OPEN_POSITION);
                            isLiftClawOpen = true;
                            targetLiftPosition = -80;
                            isLiftBeingReset = true;
                            hardware.lift.setPower(1.0);
                            setSpecimenScoreCurrentState(SpecimenState.SPECIMEN_IDLE);
                        }
                        break;
                    }

                    case SPECIMEN_IDLE: {
                        // Get out of State Machine
                        currentState = State.NONE;
                        break;
                    }
                }
                break;
            }

            case HANG_STATE: {
                switch (hangCurrentState) {
                    case PREPARE_FOR_HANG: {
                        ledColor = LEDColors.VIOLET;
                        hardware.liftTilt.setPosition(ConstantsTAC.LIFT_TILT_HANG_POSITION);
                        hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_OPEN_POSITION);
                        hardware.claw.setPosition(ConstantsTAC.CLAW_OPEN_POSITION);
                        hardware.clawTilt.setPosition(ConstantsTAC.CLAW_TILT_UP_POSITION);
                        hardware.clawRotate.setPosition(ConstantsTAC.CLAW_ROTATE_VERTICAL_POSITION);
                        isLiftClawOpen = true;
                        isClawOpen = false;
                        targetExtensionPosition = -80;
                        isExtensionBeingReset = true;
                        targetLiftPosition = -80;
                        isLiftBeingReset = true;
                        hardware.lift.setPower(0.5);
                        targetLiftHangPosition = -80;
                        isLiftHangBeingReset = true;
                        setHangCurrentState(HangState.LIFT_UP_FOR_LEVEL2_HANG);
                        break;
                    }

                    case LIFT_UP_FOR_LEVEL2_HANG: {
                        targetLiftHangPosition = ConstantsTAC.HANG_RAISE_LEVEL2_POSITION;
                        hardware.liftHang.setPower(1.0);
                        if (Math.abs(hardware.liftHang.getCurrentPosition() - ConstantsTAC.HANG_RAISE_LEVEL2_POSITION) < 50) {
                            setHangCurrentState(HangState.IS_WAITING_FOR_HUMAN_TO_START_HANG);
                        }
                        break;
                    }

                    case IS_WAITING_FOR_HUMAN_TO_START_HANG: {
                        if (currentGamepad2.back && !previousGamepad2.back && isPreparingForHang) {
                            isPreparingForHang = false;
                            setHangCurrentState(HangState.LIFT_DOWN_FOR_LEVEL2_HANG);
                        }
                        break;
                    }

                    case LIFT_DOWN_FOR_LEVEL2_HANG: {
                        if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                            targetLiftHangPosition = ConstantsTAC.HANG_LOWER_LEVEL2_POSITION;
                            hardware.liftHang.setPower(1.0);
                            if (Math.abs(hardware.liftHang.getCurrentPosition() - ConstantsTAC.HANG_LOWER_LEVEL2_POSITION) < 25) {
                                setHangCurrentState(HangState.RELEASE_HOOKS);
                            }
                        }
                        break;
                    }

                    case RELEASE_HOOKS: {
                        if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                            hardware.hangRight.setPosition(ConstantsTAC.HANG_RIGHT_SERVO_RELEASE_POSITION);
                            hardware.hangLeft.setPosition(ConstantsTAC.HANG_LEFT_SERVO_RELEASE_POSITION);
                            setHangCurrentState(HangState.LIFT_UP_FOR_LEVEL3_HANG);
                        }
                        break;
                    }

                    case LIFT_UP_FOR_LEVEL3_HANG: {
                        if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                            isMovingWheelsWhileHanging = true;
                            targetLiftHangPosition = ConstantsTAC.HANG_RAISE_LEVEL3_POSITION;
                            if (Math.abs(hardware.liftHang.getCurrentPosition() - ConstantsTAC.HANG_RAISE_LEVEL3_POSITION) < 50) {
                                setHangCurrentState(HangState.LIFT_HANG_COMPLETE);
                            }
                        }
                        break;
                    }

                    case LIFT_HANG_COMPLETE: {
                        if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                            targetLiftHangPosition = -200;
                            hardware.liftHang.setPower(1.0);
                            if (hardware.liftHang.getCurrentPosition() < 5000) {
                                isMovingWheelsWhileHanging = false;
                                if (Math.abs(hardware.liftHang.getCurrentPosition()) < 50) {
                                    setHangCurrentState(HangState.HANG_IDLE);
                                }
                            }
                        }
                        break;
                    }

                    case HANG_IDLE: {
                        // You will stay in this until you turn OpMode off
                        hardware.liftHang.setPower(1.0);
                        break;
                    }
                }
                break;
            }

            case NONE: {
               // Do nothing
                clawPickupCurrentState = ClawPickupState.CLAW_PICKUP_IDLE;
                bucketCurrentState = BucketState.BUCKET_IDLE;
                specimenCurrentState = SpecimenState.SPECIMEN_IDLE;
                isWaitingToScoreSpecimen = false;
                hangCurrentState = HangState.HANG_IDLE;
                break;
            }
        }

        // handoff position only
        if (currentGamepad2.left_trigger > 0.2) {
            isFindingSample = false;
            currentState = State.CLAW_PICKUP_STATE;
            setClawPickupCurrentState(ClawPickupState.SET_SERVO_POSITIONS4);
        }

        //Servos
        { // Claw Tilt and Rotate servos
            if (currentGamepad2.y && !previousGamepad2.y) { // Hover
                hardware.clawTilt.setPosition(ConstantsTAC.CLAW_TILT_HOVER);
                hardware.claw.setPosition(ConstantsTAC.CLAW_VERY_OPEN_POSITION);
                isClawOpen = true;
            } else if (currentGamepad2.b && !previousGamepad2.b) { // Claw turn clockwise 45 degrees
                hardware.clawRotate.setPosition(hardware.clawRotate.getPosition() - ConstantsTAC.CLAW_ROTATE_45_POSITION_INCREMENT);
            } else if (currentGamepad2.x && !previousGamepad2.x) { // Claw turn counterclockwise 45 degrees
                hardware.clawRotate.setPosition(hardware.clawRotate.getPosition() + ConstantsTAC.CLAW_ROTATE_45_POSITION_INCREMENT);
            } else if (currentGamepad2.start && !previousGamepad2.start) { // Down
                hardware.clawTilt.setPosition(ConstantsTAC.CLAW_TILT_DOWN_POSITION);
                hardware.claw.setPosition(ConstantsTAC.CLAW_VERY_OPEN_POSITION);
                currentState = State.NONE;
            }

            if (hardware.clawRotate.getPosition() > ConstantsTAC.CLAW_ROTATE_MAX) {
                hardware.clawRotate.setPosition(ConstantsTAC.CLAW_ROTATE_MAX);
            } else if (hardware.clawRotate.getPosition() < ConstantsTAC.CLAW_ROTATE_MIN) {
                hardware.clawRotate.setPosition(ConstantsTAC.CLAW_ROTATE_MIN);
            }

            // Claw servo
            if (isClawOpen && currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                hardware.claw.setPosition(ConstantsTAC.CLAW_CLOSE_POSITION);
                isClawOpen = false;
            } else if (!isClawOpen && currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                hardware.claw.setPosition(ConstantsTAC.CLAW_OPEN_POSITION);
                isClawOpen = true;
            }

            // Lift claw servo
            if (isLiftClawOpen && currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_CLOSE_POSITION);
                isLiftClawOpen = false;
            } else if (!isLiftClawOpen && currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                hardware.liftClaw.setPosition(ConstantsTAC.LIFT_CLAW_OPEN_POSITION);
                isLiftClawOpen = true;
            }
        }

        telemetry.addLine(" ");
        telemetry.addData("avgLoopTime (ms)", avgLoopTime);
        telemetry.addData("isLimeLightActive", isLimeLightActive);

        if (isDebugMode) {
            telemetry.addData("distanceErrorY", distanceErrorY);
            telemetry.addData("computedY", computedY);
            telemetry.addData("isCoordinateInClawPath", isCoordinateInClawPath);
            telemetry.addData("isFindingSample", isFindingSample);
            telemetry.addData("distanceToMoveYPedro", distanceToMoveYPedro);
            telemetry.addData("isDrivingToSample", isDrivingToSample);
            telemetry.addLine(" ");
//            telemetry.addData("currentState", currentState);
//            telemetry.addData("clawPickupCurrentState", clawPickupCurrentState);
//            telemetry.addData("bucketCurrentState", bucketCurrentState);
//            telemetry.addData("specimenCurrentState", specimenCurrentState);
//            telemetry.addData("hangCurrentState", hangCurrentState);
//            telemetry.addData("isPreparingForHang", isPreparingForHang);
//            telemetry.addData("lift.getCurrentPosition", hardware.lift.getCurrentPosition());
//            telemetry.addData("liftTouch.isPressed", hardware.liftTouch.isPressed());
//            telemetry.addData("lift.getPower", hardware.lift.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("liftHang.getCurrentPosition", hardware.liftHang.getCurrentPosition());
//            telemetry.addData("liftHangTouch.isPressed", hardware.liftHangTouch.isPressed());
//            telemetry.addData("liftHang.getPower", hardware.liftHang.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("extension.getCurrentPosition", hardware.extension.getCurrentPosition());
//            telemetry.addData("extensionTouch.isPressed", hardware.extensionTouch.isPressed());
//            telemetry.addData("extension.getPower", hardware.extension.getCurrent(CurrentUnit.AMPS));
//            telemetry.addLine(" ");
//            telemetry.addData("clawError", clawError);
//            telemetry.addData("clawTiltError", clawTiltError);
//            telemetry.addData("clawRotateError", clawRotateError);
//            telemetry.addData("liftClawError", liftClawError);
//            telemetry.addData("liftTiltError", liftTiltError);
//            telemetry.addData("foundColor", foundColor());
//            telemetry.addData("headlight.getPosition", hardware.headlight.getPosition());
//            telemetry.addLine(" ");
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("distanceToSampleYPedro", distanceToSampleYPedro);
//            telemetry.addData("bucketPose", bucketPose);
        }
        telemetry.update();

        hardware.lightIndicator.setPosition(LEDColorArray[ledColor.ordinal()]);

        if (isMovingWheelsWhileHanging) {
            hardware.leftFront.setPower(0.1);
            hardware.rightFront.setPower(0.1);
            hardware.leftBack.setPower(0.1);
            hardware.rightBack.setPower(0.1);
        }

        hardware.extension.setTargetPosition(targetExtensionPosition);
        hardware.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.extension.setPower(1.0);

        hardware.lift.setTargetPosition(targetLiftPosition);
        hardware.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.lift.setPower(1.0);

        hardware.liftHang.setTargetPosition(targetLiftHangPosition);
        hardware.liftHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.liftHang.setPower(1.0);

        // Axon feedback calculations
        {
            clawPosition = getAxonPosition(false, hardware.clawInput);
            clawCurrentSetDeg = (hardware.claw.getPosition() * 360);
            clawError = Math.abs(clawCurrentSetDeg - (clawPosition));

            liftClawPosition = getAxonPosition(true, hardware.liftClawInput);
            liftClawCurrentSetDeg = (hardware.liftClaw.getPosition() * 360);
            liftClawError = Math.abs(liftClawCurrentSetDeg - (liftClawPosition));

            liftTiltPosition = getAxonPosition(true, hardware.liftTiltInput);
            liftTiltCurrentSetDeg = (hardware.liftTilt.getPosition() * 360);
            liftTiltError = Math.abs(liftTiltCurrentSetDeg - (liftTiltPosition));
        }

        loopCounter++;
        if ((loopCounter % 50) == 0){
            avgLoopTime = (getRuntime() - lastLoopTime) / 50 * 1000;
            lastLoopTime = getRuntime();
        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private void setClawPickupCurrentState(ClawPickupState newState) {
        clawPickupCurrentState = newState;
        pathTimer.resetTimer();
    }

    public void setBucketScoreCurrentState(BucketState newState) {
        bucketCurrentState = newState;
        pathTimer.resetTimer();
    }

    public void setSpecimenScoreCurrentState(SpecimenState newState) {
        specimenCurrentState = newState;
        pathTimer.resetTimer();
    }

    public void setHangCurrentState(HangState newState) {
        hangCurrentState = newState;
        pathTimer.resetTimer();
    }

    final double getAxonPosition (Boolean norm, AnalogInput axonServo) {
        if (norm) {
            return ((((axonServo.getVoltage() - 3.3) / 3.3) * 360) * -1);
        } else {
            return (((axonServo.getVoltage() / 3.3) * 360));
        }
    }

    public SampleColor foundColor() {
        if ((hardware.clawColor.blue() > ConstantsTAC.BLUE_SAMPLE_THRESHOLD) && hardware.clawColor.green() < 1000) {
            return SampleColor.BLUE;
        } else if ((hardware.clawColor.red() > ConstantsTAC.RED_SAMPLE_THRESHOLD) && (hardware.clawColor.green() < 700)) {
            return SampleColor.RED;
        } else if ((hardware.clawColor.green() > ConstantsTAC.YELLOW_SAMPLE_THRESHOLD) && (hardware.clawColor.red() > 1000)) {
            return SampleColor.YELLOW;
        } else {
            return SampleColor.NONE;
        }
    }

    public int roundTXValue(double value) {
        return (int) Math.round(value);
    }

    public static int calculateAngleOfLongestSide(List<List<Double>> points) {
        final int numSides = 4;
        if (points.size() < numSides) {
            return -1;
            //throw new IllegalArgumentException("The list must contain exactly 4 coordinates.");
        }

        // Calculate the lengths of all sides
        double[] sideLengths = new double[numSides];
        for (int i = 0; i < numSides; i++) {
            int next = (i + 1) % numSides;  // Wrap around to the first point
            double deltaX = points.get(next).get(0) - points.get(i).get(0);
            double deltaY = points.get(next).get(1) - points.get(i).get(1);
            sideLengths[i] = Math.sqrt(deltaX * deltaX + deltaY * deltaY);  // Distance formula
        }

        // Find the index of the longest side
        int longestSideIndex = 0;
        for (int i = 1; i < numSides; i++) {
            if (sideLengths[i] > sideLengths[longestSideIndex]) {
                longestSideIndex = i;
            }
        }

        // Get the two points that define the longest side
        int next = (longestSideIndex + 1) % numSides;
        List<Double> p1 = points.get(longestSideIndex);
        List<Double> p2 = points.get(next);

        // Calculate the angle of the longest side
        double deltaX = p2.get(0) - p1.get(0);
        double deltaY = p2.get(1) - p1.get(1);

        // Use Math.atan2 to calculate the angle
        double angle = Math.atan2(deltaY, deltaX); // Angle in radians

        // Convert the angle to degrees
        return (int) Math.toDegrees(angle);
    }

    public static int normalizeAngle(int angle)
    {
        // If angle is between -90 and 90, it is already normalized.
        if (angle >= -90 && angle <= 90) {
            return angle;
        }

        // If the angle is greater than 90, subtract 180.
        if (angle > 90) {
            return angle - 180;
        }

        // If the angle is less than -90, add 180.
        return angle + 180;
    }

    public static int closestMultipleOf45(int angle) {
        // Ensure the angle is within the range -90 to 90
        if (angle < -90 || angle > 90) {
            return 0;
            //throw new IllegalArgumentException("Angle must be between -90 and 90 degrees.");
        }

        // Calculate the closest multiple of 45
        return Math.round(angle / 45.0f) * 45;
    }

    public int getCalculatedClawAngle(int angle) {
        return closestMultipleOf45(normalizeAngle(angle));
    }

    public double getDistanceErrorY(int pipelineNumber) {
        switch (pipelineNumber) {
            case pipelineYellow: {
                return 3.5;
            }

            case pipelineBlue: {
                return 7;
            }

            case pipelineRed: {
                return 7.5;
            }
        }
        return 10;
    }
}