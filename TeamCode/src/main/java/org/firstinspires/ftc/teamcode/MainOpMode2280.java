package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_SERVO_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_SERVO_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_COEFFICIENT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_SERVO_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_SERVO_OPEN_POSITION;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name="MainOpMode22280")
public class MainOpMode2280 extends LinearOpMode  {
    private Motor fL, fR, bL, bR;
    private MecanumDrive m_drive;
    private GamepadEx driverController1;
    private GamepadEx driverController2;

    private Motor lLift, rLift;
    private MotorGroup lift;

    private Servo leftArmServo, rightArmServo;

    private int liftPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        driverController1 = new GamepadEx(gamepad1);
        driverController2 = new GamepadEx(gamepad2);

        fL = new Motor(hardwareMap, "fL"); //port 0
        fR = new Motor(hardwareMap, "fR"); //port 1
        bL = new Motor(hardwareMap, "bL"); //port 2
        bR = new Motor(hardwareMap, "bR"); //port 3

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_drive = new MecanumDrive(fL, fR, bL, bR);

        lLift = new Motor(hardwareMap, "lLift", Motor.GoBILDA.RPM_312);
        rLift = new Motor(hardwareMap, "rLift", Motor.GoBILDA.RPM_312);

        lLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rLift.setInverted(true);

        rLift.resetEncoder();
        lLift.resetEncoder();

        lift = new MotorGroup(lLift, rLift);

        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        lift.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        liftPosition = lLift.getCurrentPosition();
//        Arrays.sort(LIFTPOSITIONS);

        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");

        waitForStart();
        while(opModeIsActive()){
            m_drive.driveRobotCentric(driverController1.getLeftX(), driverController1.getLeftY(), driverController1.getRightX());

            if(gamepad2.x){
                leftArmServo.setPosition(.6);
                rightArmServo.setPosition(.6);
            }else if(gamepad2.b){
                leftArmServo.setPosition(.2);
                rightArmServo.setPosition(.2);
            }

            // lift code

            if (gamepad2.right_trigger > 0){
                liftPosition += gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0) {
                liftPosition -= gamepad2.left_trigger;
            }

            if (liftPosition < 0){
                liftPosition = 0;
            }
            if (liftPosition > 3300){
                liftPosition = 3300;
            }

            lift.setTargetPosition(liftPosition);

            if(lift.atTargetPosition()){
                lift.set(0);
            }else{
                lift.set(1);
            }

            //end lift code

            telemetry.addData("Lift Target Position:", liftPosition);
            telemetry.addData("Lift Position Left:", lLift.getCurrentPosition());
            telemetry.addData("Lift Position Right:", rLift.getCurrentPosition());

            telemetry.addData("Position Left:", lLift.getCurrentPosition());
            telemetry.addData("Position Right:", rLift.getCurrentPosition());

            telemetry.addData("left Arm Servo Position:", leftArmServo.getPosition());
            telemetry.addData("right Arm Servo Position", rightArmServo.getPosition());

            telemetry.update();




        }
    }
}