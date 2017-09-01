package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Created by kylie on 8/24/2017.
 */

@Autonomous(name="test: Auto Drive By Encoder", group="Test")
public class TRChassisTestRevHub1 extends LinearOpMode {

        HardwarePushbot         robot = new HardwarePushbot();
        private ElapsedTime     runtime = new ElapsedTime();

        static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
        static final double     DRIVE_GEAR_REDUCTION    = 2.0;
        static final double     WHEEL_DIAMETER_INCHES   = 4.0;
        static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                            (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     DRIVE_SPEED             = 0.6;
        static final double     TURN_SPEED              = 0.5;

        @Override
        public void runOpMode() {

            robot.init(hardwareMap);

            telemetry.addData("Status", "Resetting Encoders"); //
            telemetry.update();

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Path0", "Starting at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
            telemetry.update();

            waitForStart();

            encoderDrive(DRIVE_SPEED, 6, 6, 5.0);
            encoderDrive(DRIVE_SPEED, 12, -12, 4.0);
            encoderDrive(DRIVE_SPEED, -6, -6, 4.0);

            robot.leftClaw.setPosition(1.0);
            robot.rightClaw.setPosition(0.0);
            sleep(1000);

            telemetry.addData("Path", "Complete");
            telemetry.update();




        }
        public void encoderDrive(double speed,
                                 double leftInches, double rightInches,
                                 double timeoutS) {
            int newLeftTarget;
            int newRightTarget;

            if(opModeIsActive()){

                newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                robot.leftMotor.setTargetPosition(newLeftTarget);
                robot.rightMotor.setTargetPosition(newRightTarget);

                robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                runtime.reset();
                robot.leftMotor.setPower(Math.abs(speed));
                robot.rightMotor.setPower(Math.abs(speed));

                while (opModeIsActive()&&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftMotor.isBusy() && robot.rightMotor.isBusy()))   {

                    telemetry.update();
                }

                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
}

