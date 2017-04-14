package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import android.text.method.HideReturnsTransformationMethod;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Wall follow Test")
//@Disabled
public class WallFollowTest extends OpMode {
    WorldRobotDevices robot = new WorldRobotDevices();

    private double wallSetPoint = 10.5;

    private double wallKp = 0.05;
    private double wallKi = 0.00001;
    private double wallKd = 0.1;
    private double wallIntegral;
    private double wallPreError;

    private double angleKp = 0.01;
    private double angleKi = 0.00003;
    private double angleKd = 0.02;
    private double angleIntegral;
    private double anglePreError;

    private double yawAngle;

    @Override
    public void init() {
        robot.init(hardwareMap, this);
    }

    @Override
    public void start() {
        robot.imu.startIMU();
    }

    @Override
    public void loop() {
        yawAngle = robot.imu.getYaw();

        if (gamepad1.guide) {
            robot.wallFollowLeft(0.25, wallSetPoint, 0.04, 0.00001, 0);
        } else if (gamepad1.a) {
            robot.wallFollowLeft(-0.25, wallSetPoint, 0.04, 0.00001, 0);
        } else if (gamepad1.b) {
            robot.tankDrive(wallPID(0.25, wallSetPoint) + anglePID(0.25, wallSetPoint), wallPID(-0.25, wallSetPoint) - anglePID(-0.25, wallSetPoint));
        } else {
            robot.tankDrive(0, 0);
        }

        telemetry.addData("Heading", yawAngle);
        telemetry.addData("Light sensors", "L: " + robot.leftLightSensor.getLightDetected() + " R: " + robot.rightLightSensor.getLightDetected());
        telemetry.addData("\nDistance", "L: " + robot.getLeftDistance() + " R: " + robot.getRightDistance());
        telemetry.addData("\nColor sensors (RGB)", "\nLF: %1$s, %2$s, %3$s\nLB: %4$s, %5$s, %6$s\nRF: %7$s, %8$s, %9$s\nRB: %10$s, %11$s, %12$s",
                robot.leftFrontColor.red(), robot.leftFrontColor.green(), robot.leftFrontColor.blue(),
                robot.leftBackColor.red(), robot.leftBackColor.green(), robot.leftBackColor.blue(),
                robot.rightFrontColor.red(), robot.rightFrontColor.green(), robot.rightFrontColor.blue(),
                robot.rightBackColor.red(), robot.rightBackColor.green(), robot.rightBackColor.blue());
    }

    private double anglePID(double speed, double setPoint) {
        double error = yawAngle - setPoint;
        angleIntegral += error;
        double derivative = error - anglePreError;
        double output = Range.clip((angleKp * error) + (angleKi * angleIntegral) + (angleKd * derivative), -1, 1);
        anglePreError = error;

        return output;
    }

    public double wallPID(double speed, double wallSetPoint ) {
        double dist = robot.getLeftDistance();
        double distError = dist - wallSetPoint;


        wallIntegral += distError;
        double wallDerivative = distError - wallPreError;

        wallPreError = distError;

        double output = (wallKp * distError) + (wallKi * wallIntegral) + (wallKd * wallDerivative);

        if (speed < 0) {
            return -output;
        } else {
            return output;
        }
    }
}