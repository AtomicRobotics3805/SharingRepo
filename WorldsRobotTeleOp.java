package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp")
public class WorldsRobotTeleOp extends OpMode {
    private WorldRobotDevices robot = new WorldRobotDevices();

    private boolean lastY1 = false;
    private boolean lastA1 = false;
    private boolean holdCapBall = false;
    private boolean releaseLift = false;
    private double liftKp = 0.001;

    @Override
    public void init() {
        robot.init(hardwareMap, this);
    }

    @Override
    public void loop() {
        //Drive 1
        //double leftY1 = -gamepad1.left_stick_y;
        //double rightX1 = gamepad1.right_stick_x;

        //TODO Test this with Brisse
        //double leftY1 = (((Math.pow(-gamepad1.left_stick_y, 3) * 0.8) / 1) / Math.abs(-gamepad1.left_stick_y)) + ((0.2 * -gamepad1.left_stick_y) / Math.abs(-gamepad1.left_stick_y));
        //double rightX1 = (((Math.pow(gamepad1.right_stick_x, 3) * 0.8) / 1) / Math.abs(gamepad1.right_stick_x)) + ((0.2 * gamepad1.right_stick_x) / Math.abs(gamepad1.right_stick_x));

        double leftY1 = Math.pow(-gamepad1.left_stick_y, 3);
        double rightX1 = Math.pow(gamepad1.right_stick_x, 3);

        double left = leftY1 + rightX1;
        double right = leftY1 - rightX1;

        robot.leftFrontMotor.setPower(left);
        robot.leftBackMotor.setPower(left);
        robot.rightFrontMotor.setPower(right);
        robot.rightBackMotor.setPower(right);

        //Collection 1
        robot.collectionMotor.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

        //Flicker 2
        robot.flickerMotor.setPower(gamepad2.left_trigger-gamepad2.right_trigger);

        robot.ballIndexer.setPosition(gamepad2.left_bumper ? robot.BIS_LOAD : (gamepad2.right_bumper ? robot.BIS_REVERSE : robot.BIS_IDLE));

        //Button pushers 2
        if (gamepad2.dpad_left) {
            robot.leftButtonPushers.setPosition(robot.LBPS_LEFT);
            robot.rightButtonPushers.setPosition(robot.RBPS_LEFT);
        } else if (gamepad2.dpad_right) {
            robot.leftButtonPushers.setPosition(robot.LBPS_RIGHT);
            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
        } else {
            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
            robot.leftButtonPushers.setPosition(robot.LBPS_CENTER);
        }

        robot.frontPusher.setPosition(gamepad2.b ? robot.FPS_RELEASE : robot.FPS_HOLD);

        //Lift 2
        double liftTp = ((gamepad2.dpad_up ? 1 : (gamepad2.dpad_down ? -1 : 0)));

        int leftLiftPos = robot.liftLeft.getCurrentPosition();
        int rightLiftPos = robot.liftRight.getCurrentPosition();

        double liftOutput = liftKp * (leftLiftPos - rightLiftPos);

        double leftPower = liftTp - liftOutput;
        double rightPower = liftTp + liftOutput;

        robot.liftLeft.setPower(leftPower);
        robot.liftRight.setPower(rightPower);

        //Release lift 2
        boolean currentA1 = gamepad2.a;
        if (currentA1 && !lastA1) {
            releaseLift = !releaseLift;
        }

        robot.liftRelease.setPosition(releaseLift ? robot.LRS_RELEASE : robot.LRS_HOLD);

        //Cap ball hold 2
        boolean currentY1 = gamepad1.y;
        if (currentY1 && !lastY1) {
            holdCapBall = !holdCapBall;
        }

        robot.capHold.setPosition(holdCapBall ? robot.CBHS_GRIP : robot.CBHS_IDLE);

        lastY1 = currentY1;
        lastA1 = currentA1;
    }
}