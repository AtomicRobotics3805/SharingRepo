package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name = "RED Auto")
//@Disabled
public class RED_FullAuto_V1 extends OpMode {
    private robotPIDNavigator PIDNavigator;
    private WorldRobotDevices robot = new WorldRobotDevices();

    private int mainProgramStep = 0;
    private int shootStep = 0;
    private int iLoop = 0;

    //1: 1 = moveForward, 2 = moveBackward, 3 = rotateCW, 4 = rotateCCW, 5 = end navigation
    //2: Tp: the speed at which the robot goes when moving in the correct direction.
    //3: Either the wanted angle, or the goal distance in inches.
    private double[][] movementArray = new double[][]{
           //_,______,______}
            {1,   0.3,    18},
            {6,     0,     0},
            {3,   0.3,    60},
            {1,   0.3,    42},
            {4,  -0.3,     0},
            {1,   0.1,    12},
            {7,     0,     0},
            {5,     0,     0} //Stop all movements
    };

    @Override
    public void init() {
        robot.init(hardwareMap, this);

        PIDNavigator = new robotPIDNavigator(movementArray, this, robot.imu, robot.leftFrontMotor, robot.rightFrontMotor, robot.leftBackMotor, robot.rightBackMotor, 747, 4);
        PIDNavigator.tuneTurnGains(0.02, 0, 0);
        PIDNavigator.tuneForwardsGains(0.02, 0.00001, 0.07);
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
            case 1:
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
                }
                break;
            case 2: //Navigate
                if (PIDNavigator.navigationType() == 7) { //Wall follow
                    mainProgramStep++;
                } else {
                    PIDNavigator.loopNavigation();
                }
                break;
            case 4: //Follow wall
                PIDNavigator.loopNavigation();
                break;
        }
        robot.getRightDistance();
    }
}