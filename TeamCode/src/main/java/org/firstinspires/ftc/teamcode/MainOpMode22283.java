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
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp(name="MainOpMode22283")
public class MainOpMode22283 extends LinearOpMode  {
    private Motor fL, fR, bL, bR;
    private MecanumDrive m_drive;
    private GamepadEx driverController1;
    private GamepadEx driverController2;

    private Motor lLift, rLift;
    private MotorGroup lift;
    private boolean maxLiftSpeed = false;

    private ServoEx rightArmServo;
    private ServoEx leftClawServo, rightClawServo;

    private ServoEx airplaneLauncher;

    public void closeClaw(){
        leftClawServo.setPosition(0.35);
        rightClawServo.setPosition(0.35);
    }

    public void openClaw(){
        leftClawServo.setPosition(0.65);
        rightClawServo.setPosition(0.65);
    }

    public void positionArmToScore(){
        rightArmServo.setPosition(.5);
    }

    public void positionArmToRest(){
        rightArmServo.setPosition(.1);
    }
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

        lLift.setInverted(true);

        rLift.resetEncoder();
        lLift.resetEncoder();

        lift = new MotorGroup(lLift, rLift);

        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        lift.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        int targetLiftPosition = lLift.getCurrentPosition();

        rightArmServo = new SimpleServo(hardwareMap, "rightArmServo", 0, 360, AngleUnit.DEGREES);

        rightClawServo = new SimpleServo(hardwareMap, "rightClawServo", 0, 360, AngleUnit.DEGREES);
        leftClawServo = new SimpleServo(hardwareMap, "leftClawServo", 0, 360, AngleUnit.DEGREES);

        leftClawServo.setInverted(true);

        airplaneLauncher = new SimpleServo(hardwareMap, "airplaneLauncher", 0, 360, AngleUnit.DEGREES);
        closeClaw();

        waitForStart();
        while(opModeIsActive()){
            if (gamepad2.right_bumper){
                targetLiftPosition = 2500;
                maxLiftSpeed = true;
            }else if (gamepad2.left_bumper) {
                targetLiftPosition = 0;
                maxLiftSpeed = true;
            }
            else if (gamepad2.right_trigger > 0){
                targetLiftPosition += gamepad2.right_trigger * 25;
                maxLiftSpeed = false;
            } else if (gamepad2.left_trigger > 0) {
                targetLiftPosition -= gamepad2.left_trigger * 25;
                maxLiftSpeed = false;
            }

            if (targetLiftPosition < 0){
                targetLiftPosition = 0;
                maxLiftSpeed = false;
            }
            if (targetLiftPosition > 2600){
                targetLiftPosition = 2300;
                maxLiftSpeed = false;
            }

            lift.setTargetPosition(targetLiftPosition);

            if(lift.atTargetPosition()){
                lift.set(0); // same as .set(0)??? test this!
                maxLiftSpeed = false;
            }else{
                if(maxLiftSpeed){
                    lift.set(1);
                }
                else if (lLift.getCurrentPosition() <= 700 || lLift.getCurrentPosition() >= 1200){
                    lift.set(0.2);
                }else{
                    lift.set(0.85);
                }
            }

            if(gamepad1.right_trigger > 0){
                airplaneLauncher.rotateBy(10);
            }else if (gamepad1.left_trigger>0){
                airplaneLauncher.rotateBy(-10);
            }

//            if(targetLiftPosition < 800){
//                positionArmToRest();
//                closeClaw();
//            }else

            if(gamepad2.a){
                positionArmToRest();
            }else if(gamepad2.y){
                positionArmToScore();
            }

//            if(lLift.getCurrentPosition() > 1000){
//                positionArmToScore();
//            }

            if(gamepad2.x){
                closeClaw();
            }else if(gamepad2.b){
                openClaw();
            }

            m_drive.driveRobotCentric(-driverController1.getLeftX(), -driverController1.getLeftY(), driverController1.getRightX());

            telemetry.addData("Lift Target Position:", targetLiftPosition);
            telemetry.addData("Lift Position Left:", lLift.getCurrentPosition());
            telemetry.addData("Lift Position Right:", rLift.getCurrentPosition());

            telemetry.addData("rightArmServo Position", rightArmServo.getPosition());

            telemetry.addData("rightClawServo Position", rightClawServo.getPosition());
            telemetry.addData("leftClawServo Position", leftClawServo.getPosition());

            telemetry.update();

        }
    }
}