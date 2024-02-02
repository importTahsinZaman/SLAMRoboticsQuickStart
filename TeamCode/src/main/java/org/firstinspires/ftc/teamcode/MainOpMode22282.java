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

@Disabled
@TeleOp(name="MainOpMode22282")
public class MainOpMode22282 extends LinearOpMode  {
    private Motor fL, fR, bL, bR;
    private MecanumDrive m_drive;
    private GamepadEx driverController1;
    private GamepadEx driverController2;

    private Motor lLift, rLift;
    private MotorGroup lift;

//    private Servo leftServo, rightServo;

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

//        lift.setRunMode(Motor.RunMode.PositionControl);
//        lift.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
//        lift.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        liftPosition = lLift.getCurrentPosition();
//        Arrays.sort(LIFTPOSITIONS);

//        leftServo = hardwareMap.get(Servo.class, "leftServo");
//        rightServo = hardwareMap.get(Servo.class, "rightServo");

        waitForStart();
        while(opModeIsActive()){
            m_drive.driveRobotCentric(driverController1.getLeftX(), driverController1.getLeftY(), driverController1.getRightX());

//            if(gamepad2.x){
//                leftServo.setPosition(LEFT_SERVO_CLOSE_POSITION);
//                rightServo.setPosition(RIGHT_SERVO_CLOSE_POSITION);
//            }else if(gamepad2.b){
//                leftServo.setPosition(LEFT_SERVO_OPEN_POSITION);
//                rightServo.setPosition(RIGHT_SERVO_OPEN_POSITION);
//            }

            // lift code

//            if (gamepad2.right_trigger > 0){
//                liftPosition += gamepad2.right_trigger;
//            } else if (gamepad2.left_trigger > 0) {
//                liftPosition -= gamepad2.left_trigger;
//            }
//
//            if (liftPosition < 0){
//                liftPosition = 0;
//            }
//            if (liftPosition > 3300){
//                liftPosition = 3300;
//            }
//
//            lift.setTargetPosition(liftPosition);
//
//            if(lift.atTargetPosition()){
//                lift.set(0);
//            }else{
//                lift.set(1);
//            }

            if (gamepad2.right_trigger > 0){
                lLift.set(gamepad2.right_trigger);
                rLift.set(gamepad2.right_trigger);
            } else if (gamepad1.left_trigger > 0)  {
                lLift.set(-gamepad1.left_trigger);
                rLift.set(-gamepad1.left_trigger);
            }

            //end lift code

            telemetry.addData("Lift Target Position:", liftPosition);
            telemetry.addData("Lift Position Left:", lLift.getCurrentPosition());
            telemetry.addData("Lift Position Right:", rLift.getCurrentPosition());


//            telemetry.addData("leftServo Position:", leftServo.getPosition());
//            telemetry.addData("rightServo Position", rightServo.getPosition());

            telemetry.update();




        }
    }
}