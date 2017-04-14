package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "RED Corner")
//@Disabled
public class RED_CornerAuto_V1 extends OpMode {
    private robotPIDNavigator PIDNavigator;
    private WorldRobotDevices robot = new WorldRobotDevices();

    private ElapsedTime runtime = new ElapsedTime();

    private int mainProgramStep = 0;
    private int shootStep = 0;
    private int iLoop = 0;

    //Delay variables
    boolean lastUp;
    boolean lastDown;
    boolean lastStart;
    int delayValue;
    boolean knockCapBall;

    //1: 1 = moveForward, 2 = moveBackward, 3 = rotateCW, 4 = rotateCCW, 5 = end navigation
    //2: Tp: the speed at which the robot goes when moving in the correct direction.
    //3: Either the wanted angle, or the goal distance in inches.
    private double[][] movementArray = new double[][]{
           //_,______,______}
            {6,     0,     0},
            {1,   0.3,     6},
            {3,   0.3,    40},
            {1,   0.3,    24},
            {7,     0,     0},
            {4,  -0.5,  -120},
            {2,  -0.5,   -30},
            {5,     0,     0} //Stop all movements
    };

    @Override
    public void init() {
        robot.init(hardwareMap, this);

        PIDNavigator = new robotPIDNavigator(movementArray, this, robot.imu, robot.leftFrontMotor, robot.rightFrontMotor, robot.leftBackMotor, robot.rightBackMotor, 747, 4);
        PIDNavigator.tuneTurnGains(0.03, 0.00005, 0.05);//0.04, 0.00002, 0);
        PIDNavigator.tuneForwardsGains(0.01, 0.00003, 0.02);//0.0125, 0.00001, 0.07);
        telemetry.addData(">", "Press start when ready...");
    }
    @Override
    public void init_loop() {
        boolean currentUp = gamepad1.dpad_up;
        boolean currentDown = gamepad1.dpad_down;
        boolean currentStart = gamepad1.start;

        telemetry.addData("", "Use D-Pad to adjust delay...\nPress START to toggle cap ball knockoff...\nDelay (seconds): %1$s\nKnocking cap ball: %2$s", delayValue, knockCapBall);

        if (currentUp && !lastUp) {
            delayValue++;
        } else if (currentDown && !lastDown) {
            delayValue--;
        }

        delayValue = Range.clip(delayValue, 0, 15);

        if (currentStart && !lastStart) {
            knockCapBall = !knockCapBall;
        }

        lastUp = currentUp;
        lastDown = currentDown;
        lastStart = currentStart;
    }

    @Override
    public void start() {
        PIDNavigator.initialize();
        runtime.reset();
    }

    @Override
    public void loop() {
        switch (mainProgramStep) {
            case 0: //Wait for delay
                if (runtime.seconds() >= delayValue) {
                    telemetry.addData(">", "Completed");
                    PIDNavigator.forceNextMovement();
                    mainProgramStep++;
                } else {
                    telemetry.addData("Time", runtime.seconds());
                }
                break;
            case 1: //Navigate
                if (PIDNavigator.navigationType() == 7) { //Shoot
                    mainProgramStep++;
                } else {
                    PIDNavigator.loopNavigation();
                }
                break;
            case 2:
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
                            shootStep++;
                        } else {
                            robot.flickerMotor.setPower(1);
                        }
                        break;
                    case 3: //Move on to navigation
                        if (knockCapBall) {
                            PIDNavigator.forceNextMovement();
                            mainProgramStep++;
                        } else {
                            if (PIDNavigator.navigationType() != 5) {
                                PIDNavigator.forceNextMovement();
                            } else {
                                mainProgramStep++;
                            }
                        }
                        break;
                }
                break;
            case 3:
                PIDNavigator.loopNavigation();
                break;
        }
    }
}