package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import mechanisms.ProgrammingBoard;

@TeleOp
public class HelloWorld extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();
    @Override
    public void init() {
        board.init(hardwareMap);
    }

    double speed;

    @Override
    public void loop() {
        board.leftFront.setPower(0.5);
        board.rightFront.setPower(0.5);
        board.leftFront.setTargetPosition(100);
        board.rightFront.setTargetPosition(100);

    }
}