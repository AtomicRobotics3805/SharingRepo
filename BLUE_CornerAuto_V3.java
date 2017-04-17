package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "BLUE Corner v3")
//@Disabled
public class BLUE_CornerAuto_V3 extends OpMode {
    private robotPIDNavigator PIDNavigator;
    private WorldRobotDevices robot = new WorldRobotDevices();

    private ElapsedTime runtime = new ElapsedTime();

    private int mainProgramStep = 0;
    private int shootStep = 0;
    private int iLoop = 0;

    //Delay variables
    private boolean lastUp;
    private boolean lastDown;
    private boolean lastLeft;
    private boolean lastRight;
    private int delayValue;

    private int parkState; //0: nothing, 1: center, 2: corner


    //1: 1 = moveForward, 2 = moveBackward, 3 = rotateCW, 4 = rotateCCW, 5 = end navigation
    //2: Tp: the speed at which the robot goes when moving in the correct direction.
    //3: Either the wanted angle, or the goal distance in inches.
    private double[][] movementArray = new double[][]{
           //_,______,______}
            {6,     0,     0},
            {1,   0.3,     6},
            {4,  -0.3,   -30},
            {1,   0.3,    24},
            {7,     0,     0},

            //{4,  -0.5,   -85},
            //{1,   0.5,    60},

            {8,     0,     0}, //CENTER
            {3,   0.5,   120}, //CENTER
            {2,    -1,   -30}, //CENTER

            {9,     0,     0}, //CORNER
            {3,   0.5,    50}, //CORNER
            {-1,    0,   105},
            {2,  -0.5,   -60}, //CORNER

            {5,     0,     0} //Stop all movements
    };

    @Override
    public void init() {
        robot.init(hardwareMap, this);

        PIDNavigator = new robotPIDNavigator(movementArray, this, robot.imu, robot.leftFrontMotor, robot.rightFrontMotor, robot.leftBackMotor, robot.rightBackMotor, 747, 4);
        PIDNavigator.tuneTurnGains(0.035, 0.000025, 0.05);//0.04, 0.00002, 0);
        PIDNavigator.tuneForwardsGains(0.01, 0.00003, 0.02);//0.0125, 0.00001, 0.07);
        telemetry.addData(">", "Press start when ready...");
    }
    @Override
    public void init_loop() {
        boolean currentUp = gamepad1.dpad_up;
        boolean currentDown = gamepad1.dpad_down;
        boolean currentLeft = gamepad1.dpad_left;
        boolean currentRight = gamepad1.dpad_right;

        telemetry.addData("", "Use D-Pad to adjust delay...\nPress BACK to toggle parking...\nDelay (seconds): %1$s\nPark location: %2$s", delayValue, (parkState == 1 ? "CENTER" : (parkState == 2 ? "CORNER" : "NO PARK")));

        if (currentUp && !lastUp) {
            delayValue++;
        } else if (currentDown && !lastDown) {
            delayValue--;
        }
        delayValue = Range.clip(delayValue, 0, 20);


        if (currentRight && !lastRight) {
            parkState++;
        } else if (currentLeft && !lastLeft) {
            parkState--;
        }
        parkState = Range.clip(parkState, 0, 2);

        lastUp = currentUp;
        lastDown = currentDown;
        lastLeft = currentLeft;
        lastRight = currentRight;
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
                        /*if (parkState != 0) {
                            PIDNavigator.forceNextMovement();
                            mainProgramStep++;
                        } else {
                            if (PIDNavigator.navigationType() != 5) {
                                PIDNavigator.forceNextMovement();
                            } else {
                                mainProgramStep++;
                            }
                        }
                        */
                        switch (parkState) {
                            case 0: //Skip to end
                                if (PIDNavigator.navigationType() == 5) {
                                    mainProgramStep++;
                                } else {
                                    PIDNavigator.forceNextMovement();
                                }
                                break;
                            case 1: //Skip to center
                                if (PIDNavigator.navigationType() == 8) {
                                    PIDNavigator.forceNextMovement();
                                    mainProgramStep++;
                                } else {
                                    PIDNavigator.forceNextMovement();
                                }
                                break;
                            case 2: //Skip to corner
                                if (PIDNavigator.navigationType() == 9) {
                                    PIDNavigator.forceNextMovement();
                                    mainProgramStep++;
                                } else {
                                    PIDNavigator.forceNextMovement();
                                }
                                break;
                        }
                        break;
                }
                break;
            case 3:
                PIDNavigator.loopNavigation();
                break;
        }
        PIDNavigator.updateTelemetry();
    }
}