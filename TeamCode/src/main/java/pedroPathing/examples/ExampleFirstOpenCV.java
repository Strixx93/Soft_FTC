package pedroPathing.examples;

import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import processor.DrawRectangleProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class ExampleFirstOpenCV extends OpMode {

    private DrawRectangleProcessor drawRectangleProcessor;
    private VisionPortal visionPortal;

    @Override
    public  void init () {
        drawRectangleProcessor = new DrawRectangleProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "Webcam1"), drawRectangleProcessor
                    );
    }
      @Override
    public void init_loop(){}

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }
 @Override
 public void loop () {}
}
