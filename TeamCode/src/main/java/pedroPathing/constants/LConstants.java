package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        PinpointConstants.forwardY = 4.239;
        PinpointConstants.strafeX = 3.207;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false; // Keep this false

        // THIS IS THE CORRECTION:
        // You assign the enum directly to the encoderResolution field.
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

        // DO NOT set customEncoderResolution if useCustomEncoderResolution is false.
        // PinpointConstants.customEncoderResolution = 13.26291192; // Ensure this line is removed or commented out

        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }
}