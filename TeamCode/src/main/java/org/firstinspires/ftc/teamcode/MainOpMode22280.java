package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_COEFFICIENT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_TOLERANCE;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="MainOpMode22280")
public class MainOpMode22280 extends LinearOpMode  {
    private Motor fL, fR, bL, bR;
    private MecanumDrive m_drive;
    private GamepadEx driverController1;
    private GamepadEx driverController2;

    private Motor lLift, rLift;
    private MotorGroup lift;

//    private Servo leftServo, rightServo;

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

        lLift = new Motor(hardwareMap, "lLift", Motor.GoBILDA.RPM_223);
        rLift = new Motor(hardwareMap, "rLift", Motor.GoBILDA.RPM_223);

        lLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rLift.setInverted(true);

        rLift.resetEncoder();
        lLift.resetEncoder();

        lift = new MotorGroup(lLift, rLift);

        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        lift.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        int targetLiftPosition = lLift.getCurrentPosition();

//        leftServo = hardwareMap.get(Servo.class, "leftServo");
//        rightServo = hardwareMap.get(Servo.class, "rightServo");

        waitForStart();
        while(opModeIsActive()){
            if (gamepad2.right_trigger > 0){
                targetLiftPosition += gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0) {
                targetLiftPosition -= gamepad2.left_trigger ;
            }

            if (targetLiftPosition < 0){
                targetLiftPosition = 0;
            }
            if (targetLiftPosition > 3300){
                targetLiftPosition = 3300;
            }

            lift.setTargetPosition(targetLiftPosition);

            if(lift.atTargetPosition()){
                lift.set(0); // same as .set(0)??? test this!
            }else{
                lift.set(1);
            }





//            if(gamepad2.x){
//                leftServo.setPosition(LEFT_SERVO_CLOSE_POSITION);
//                rightServo.setPosition(RIGHT_SERVO_CLOSE_POSITION);
//            }else if(gamepad2.b){
//                leftServo.setPosition(LEFT_SERVO_OPEN_POSITION);
//                rightServo.setPosition(RIGHT_SERVO_OPEN_POSITION);
//            }

            m_drive.driveRobotCentric(driverController1.getLeftX(), driverController1.getLeftY(), driverController1.getRightX());

            telemetry.addData("Lift Target Position:", targetLiftPosition);
            telemetry.addData("Lift Position Left:", lLift.getCurrentPosition());
            telemetry.addData("Lift Position Right:", rLift.getCurrentPosition());


//            telemetry.addData("leftServo Position:", leftServo.getPosition());
//            telemetry.addData("rightServo Position", rightServo.getPosition());

            telemetry.update();




        }
    }
}