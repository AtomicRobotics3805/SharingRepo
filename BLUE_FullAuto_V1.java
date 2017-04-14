package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "BLUE Auto 1")
//@Disabled
public class BLUE_FullAuto_V1 extends OpMode {
    private robotPIDNavigator PIDNavigator;
    private WorldRobotDevices robot = new WorldRobotDevices();

    private int mainProgramStep = 0;
    private int shootStep = 0;
    private int beaconStep = 0;
    private int pressBeaconStep = 0;
    private int iLoop = 0;
    private int beaconColor;

    private int baseEncoderTicks;

    //1: 1 = moveForward, 2 = moveBackward, 3 = rotateCW, 4 = rotateCCW, 5 = end navigation
    //2: Tp: the speed at which the robot goes when moving in the correct direction.
    //3: Either the wanted angle, or the goal distance in inches.
    private double[][] movementArray = new double[][]{
            //_,______,______}
            {1,   0.3,    18},
            {6,     0,     0},
            {4,  -0.3,   -50},
            {1,   0.3,    48},
            {3,   0.3,  12.5},
            {2,  -0.3,    -8},
            {4,  -0.3,     0},
            {1,   0.3,     2},
            {7,     0,     0},
            {1,   0.3,     6},
            {8,     0,     0},
            {1,   0.3,    12},
            {3,   0.3,    15},
            {1,   0.3,    12},
            {4,  -0.3,   -35},
            {2,    -1,   -48},
            {5,     0,     0} //Stop all movements
    };

    @Override
    public void init() {
        robot.init(hardwareMap, this);

        PIDNavigator = new robotPIDNavigator(movementArray, this, robot.imu, robot.leftFrontMotor, robot.rightFrontMotor, robot.leftBackMotor, robot.rightBackMotor, 747, 4);
        PIDNavigator.tuneTurnGains(0.04, 0.00002, 0);
        PIDNavigator.tuneForwardsGains(0.0125, 0.00001, 0.07);
        telemetry.addData(">", "Press start when ready...");
    }

    @Override
    public void start() {
        PIDNavigator.initialize();
    }

    @Override
    public void loop() {
        switch (mainProgramStep) {
            case 0: //Navigate
                if (PIDNavigator.navigationType() == 6) { //Shoot
                    mainProgramStep++;
                } else {
                    PIDNavigator.loopNavigation();
                }
                break;
            case 1:/*
                switch (shootStep) {
                    case 0: //Shoot first particle
                        if (robot.flickerMotor.getCurrentPosition() >= 1680) {
                            robot.flickerMotor.setPower(0);
                            shootStep++;
                        } else {
                            robot.flickerMotor.setPower(1);
                        }
                        break;
                    case 1: //Load second particle
                        if (iLoop >= 250) {
                            robot.ballIndexer.setPosition(robot.BIS_IDLE);
                            iLoop = 0;
                            shootStep++;
                        } else {
                            robot.ballIndexer.setPosition(robot.BIS_LOAD);
                            iLoop++;
                        }
                        break;
                    case 2: //Shoot second particle
                        if (robot.flickerMotor.getCurrentPosition() >= 3360) {
                            robot.flickerMotor.setPower(0);
                            PIDNavigator.forceNextMovement();
                            mainProgramStep++;
                        } else {
                            robot.flickerMotor.setPower(1);
                        }
                        break;
                }*/
                PIDNavigator.forceNextMovement();
                mainProgramStep++;
                break;
            case 2: //Navigate
                if (PIDNavigator.navigationType() == 7) { //Wall follow
                    mainProgramStep++;
                } else {
                    PIDNavigator.loopNavigation();
                }
                break;
            case 3: //TODO Score far beacon
                switch (beaconStep) {
                    case 0: //Align with near beacon for encoder comparison
                        if (robot.rightLightSensor.getLightDetected() > robot.RIGHT_LIGHT_SENSOR_THRESHOLD) {
                            baseEncoderTicks = PIDNavigator.wheelEncoderValue();
                            beaconStep++;
                        } else {
                            PIDNavigator.moveNoStop(0.15);
                        }
                        break;
                    case 1: //Move to far beacon line
                        if (PIDNavigator.wheelEncoderValue() >= baseEncoderTicks + PIDNavigator.encoderCountFromInches(30)) {
                            beaconStep++;
                        } else {
                            robot.wallFollowLeft(0.2, 10, 0.05, 0.00001, 0.1);
                            telemetry.addData(">>", "Driving 30 inches");
                        }
                        break;
                    case 2: //Move to far beacon line
                        if (robot.rightLightSensor.getLightDetected() > robot.RIGHT_LIGHT_SENSOR_THRESHOLD) {
                            beaconStep++;
                        } else {
                            PIDNavigator.moveNoStop(0.2);
                            telemetry.addData(">>", "Aligning with line");
                        }
                        break;
                    case 3: //Delay for sensor update
                        if (iLoop >= 20) {
                            iLoop = 0;
                            beaconStep++;
                        } else {
                            iLoop++;
                        }
                        break;
                    case 4: //Realign with far beacon line
                        if (robot.rightLightSensor.getLightDetected() > robot.RIGHT_LIGHT_SENSOR_THRESHOLD) {
                            PIDNavigator.fullStop();
                            beaconColor = robot.leftBeaconStatus();
                            telemetry.addData("Beacon status", beaconColor);
                            beaconStep++;
                        } else {
                            telemetry.addData(">>", "Moving towards line");
                            PIDNavigator.moveNoStop(-0.15);
                        }
                        break;
                    case 5: //Press beacon
                        telemetry.addData(">>", "Pressing button");
                        switch (beaconColor) {
                            case 0: //RB
                                telemetry.addData("Color", "RB");
                                switch (pressBeaconStep) {
                                    case 0:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
                                        }
                                        break;
                                    case 1:
                                        if (iLoop >= 100) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                    case 2:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
                                        }
                                        break;
                                    case 3:
                                        if (iLoop >= 100) {
                                            beaconStep++;
                                            pressBeaconStep = 0;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                }
                                break;
                            case 1: //BR
                                telemetry.addData("Color", "BR");
                                switch (pressBeaconStep) {
                                    case 0:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_LEFT);
                                        }
                                        break;
                                    case 1:
                                        if (iLoop >= 100) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_LEFT);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                    case 2:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_LEFT);
                                        }
                                        break;
                                    case 3:
                                        if (iLoop >= 100) {
                                            beaconStep++;
                                            pressBeaconStep = 0;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                }
                                break;
                            case 2: //RR
                                telemetry.addData("Color", "RR");
                                switch (pressBeaconStep) {
                                    case 0:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
                                        }
                                        break;
                                    case 1:
                                        if (iLoop >= 100) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_LEFT);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                    case 2:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
                                        }
                                        break;
                                    case 3:
                                        if (iLoop >= 100) {
                                            beaconStep++;
                                            pressBeaconStep = 0;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                }
                                break;
                            case 3: //BB
                                telemetry.addData("Color", "BB");
                                beaconStep++;
                                break;
                        }
                        break;
                    case 6: //Move on
                        beaconStep = 0;
                        PIDNavigator.forceNextMovement();
                        mainProgramStep++;
                        break;
                }
                break;
            case 4: //Navigate
                if (PIDNavigator.navigationType() == 8) { //Score second beacon
                    mainProgramStep++;
                } else {
                    PIDNavigator.loopNavigation();
                }
                break;
            case 5: //TODO Score second beacon
                switch (beaconStep) {
                    case 0: //Align with far beacon for encoder comparison
                        if (robot.rightLightSensor.getLightDetected() > robot.RIGHT_LIGHT_SENSOR_THRESHOLD) {
                            baseEncoderTicks = PIDNavigator.wheelEncoderValue();
                            beaconStep++;
                        } else {
                            PIDNavigator.moveNoStop(-0.15);
                        }
                        break;
                    case 1: //Move to far beacon line
                        if (PIDNavigator.wheelEncoderValue() <= baseEncoderTicks - PIDNavigator.encoderCountFromInches(30)) {
                            beaconStep++;
                        } else {
                            robot.wallFollowLeft(-0.2, 10, 0.05, 0.00001, 0.1);
                            telemetry.addData(">>", "Driving 30 inches");
                        }
                        break;
                    case 2: //Move to far beacon line
                        if (robot.rightLightSensor.getLightDetected() > robot.RIGHT_LIGHT_SENSOR_THRESHOLD) {
                            beaconStep++;
                        } else {
                            PIDNavigator.moveNoStop(-0.2);
                            telemetry.addData(">>", "Aligning with line");
                        }
                        break;
                    case 3: //Delay for sensor update
                        if (iLoop >= 20) {
                            iLoop = 0;
                            beaconStep++;
                        } else {
                            iLoop++;
                        }
                        break;
                    case 4: //Realign with far beacon line
                        if (robot.rightLightSensor.getLightDetected() > robot.RIGHT_LIGHT_SENSOR_THRESHOLD) {
                            PIDNavigator.fullStop();
                            beaconColor = robot.leftBeaconStatus();
                            telemetry.addData("Beacon status", beaconColor);
                            beaconStep++;
                        } else {
                            telemetry.addData(">>", "Moving towards line");
                            PIDNavigator.moveNoStop(0.15);
                        }
                        break;
                    case 5: //Press beacon
                        telemetry.addData(">>", "Pressing button");
                        switch (beaconColor) {
                            case 0: //RB
                                telemetry.addData("Color", "RB");
                                switch (pressBeaconStep) {
                                    case 0:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
                                        }
                                        break;
                                    case 1:
                                        if (iLoop >= 100) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                    case 2:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
                                        }
                                        break;
                                    case 3:
                                        if (iLoop >= 100) {
                                            beaconStep++;
                                            pressBeaconStep = 0;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                }
                                break;
                            case 1: //BR
                                telemetry.addData("Color", "BR");
                                switch (pressBeaconStep) {
                                    case 0:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_LEFT);
                                        }
                                        break;
                                    case 1:
                                        if (iLoop >= 100) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_LEFT);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                    case 2:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_LEFT);
                                        }
                                        break;
                                    case 3:
                                        if (iLoop >= 100) {
                                            beaconStep++;
                                            pressBeaconStep = 0;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                }
                                break;
                            case 2: //RR
                                telemetry.addData("Color", "RR");
                                switch (pressBeaconStep) {
                                    case 0:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
                                        }
                                        break;
                                    case 1:
                                        if (iLoop >= 100) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_LEFT);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                    case 2:
                                        if (iLoop >= 150) {
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                            pressBeaconStep++;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
                                        }
                                        break;
                                    case 3:
                                        if (iLoop >= 100) {
                                            beaconStep++;
                                            pressBeaconStep = 0;
                                            iLoop = 0;
                                        } else {
                                            iLoop++;
                                            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
                                        }
                                        break;
                                }
                                break;
                            case 3: //BB
                                telemetry.addData("Color", "BB");
                                beaconStep++;
                                break;
                        }
                        break;
                    case 6:
                        beaconStep = 0;
                        PIDNavigator.forceNextMovement();
                        mainProgramStep++;
                        break;
                }
                break;
            case 6:
                PIDNavigator.loopNavigation();
                break;
        }
    }
}