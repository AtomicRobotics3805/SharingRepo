package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Autonomous(name = "Sensor Test")
//@Disabled
public class SensorTest extends OpMode {
    WorldRobotDevices robot = new WorldRobotDevices();

    @Override
    public void init() {
        robot.init(hardwareMap, this);
    }

    @Override
    public void loop() {
        telemetry.addData("Light sensors", "L: " + robot.leftLightSensor.getLightDetected() + " R: " + robot.rightLightSensor.getLightDetected());
        telemetry.addData("\nDistance", "L: " + robot.getLeftDistance() + " R: " + robot.getRightDistance());
        telemetry.addData("\nColor sensors (RGB)", "\nLF: %1$s, %2$s, %3$s\nLB: %4$s, %5$s, %6$s\nRF: %7$s, %8$s, %9$s\nRB: %10$s, %11$s, %12$s",
                robot.leftFrontColor.red(), robot.leftFrontColor.green(), robot.leftFrontColor.blue(),
                robot.leftBackColor.red(), robot.leftBackColor.green(), robot.leftBackColor.blue(),
                robot.rightFrontColor.red(), robot.rightFrontColor.green(), robot.rightFrontColor.blue(),
                robot.rightBackColor.red(), robot.rightBackColor.green(), robot.rightBackColor.blue());
    }
}