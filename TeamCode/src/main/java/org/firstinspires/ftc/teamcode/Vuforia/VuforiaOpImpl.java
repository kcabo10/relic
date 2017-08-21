package org.firstinspires.ftc.teamcode.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/*

    Class to track for objects using Vuforia
    Created by Sean 7/23/2017

    For details about this class, see
    https://www.youtube.com/watch?v=2z-o9Ts8XoE
    see 15:40 for demo of code working below

    https://github.com/FIXIT3491
    TODO: add methods/class to navigate to the tracked object

 */
public class VuforiaOpImpl extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor driveR = hardwareMap.dcMotor.get("driveR"); // change to ours, driveR = right drive
        driveR.setDirection(DcMotorSimple.Direction.REVERSE);
        driveR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor driveL = hardwareMap.dcMotor.get("driveL"); // change to ours, driveL = left drive
        driveL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // back or front camera
        params.vuforiaLicenseKey = getLicenseKey();
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // also TEAPOT, none (we can experiment with these).
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, NUMBER_OF_BEACONS);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset(FILE_NAME);
        beacons.setName("Wheels"); // name of image from xml file
        beacons.setName("Tools");
        beacons.setName("Lego");
        beacons.setName("gears");

        waitForStart();

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener)beacons.get(0).getListener();
        beacons.activate();  // vuforia starts tracking for objects (no point calling this until after waitForStart() )


        // *** START navigation code  ** //

        driveL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveL.setPower(0.2);
        driveR.setPower(0.2);

        while (opModeIsActive() && wheels.getRawPose() == null ) {
            idle();
            // robot can't see beacons while in this loop, exit loop when target/beacon found
        }

        driveL.setPower(0.0);
        driveR.setPower(0.0);

        // stop (above) and analyze beacon here...

        VectorF angles = anglesFromTarget(wheels);
        VectorF trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0))-90, new VectorF(500,0,0));
        if(trans.get(0)>0) {
            driveL.setPower(0.02);
            driveR.setPower(-0.02);
        } else {
            // other direction
            driveL.setPower(-0.02);
            driveR.setPower(0.02);
        }

        do {
            if (wheels.getPose()!=null){
                trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0))-90, new VectorF(500,0,0));
            }
            idle();
        } while (opModeIsActive() && Math.abs(trans.get(0)) > 30);

        // stop
        driveL.setPower(0.0);
        driveR.setPower(0.0);

        // use encoders to drive forward so that we're in front of the beacon
        driveL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 150 value below will change depending on our/your robot
        // https://www.youtube.com/watch?v=qDoLmZyH69o, see 10:27 for more information
        // ...  150) / 409.575 * 560 ...  150 value is due to phone being 15cm (150mm) off-center on robot
        // 409.575 is wheel circumference
        // 560 ticks per rotation for Andy Mark 20's,  1120 ticks per rotation for Andy Mark 40's
        driveL.setTargetPosition((int) (driveL.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 409.575 * 1120 )));
        driveR.setTargetPosition((int) (driveL.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 409.575 * 1120 )));

        driveL.setPower(0.3);
        driveR.setPower(0.3);

        while(opModeIsActive() && driveL.isBusy() && driveR.isBusy()) {
            idle();
        }

        driveL.setPower(0.0);
        driveR.setPower(0.0);

        driveL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && (wheels.getPose()==null || Math.abs(wheels.getPose().getTranslation().get(0)) > 10)) {
            if(wheels.getPose()!=null) {
                if(wheels.getPose().getTranslation().get(0) > 0 ){
                    driveL.setPower(-0.3);
                    driveR.setPower(0.3);
                } else { // other way, adjusting for over correction
                    driveL.setPower(0.3);
                    driveR.setPower(-0.3);
                }
            } else {
                driveL.setPower(-0.3);
                driveR.setPower(0.3);
            }
        }

        // robot should be stopped and looking at the beacon now...
        driveL.setPower(0);
        driveR.setPower(0);

        // *** END navigation code  ** //



        while(opModeIsActive()) {
            for (VuforiaTrackable beacon : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();
                if (null!=pose) { // may be null if camera doesn't find target object
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beacon.getName() + "-Translation: ", translation);
                    // Math.atan2(x,y) where x,y are the coordinates coming back from the camera/phone
                    // phone may be in landscape or portrait position, so change accordingly
                    // - landscape: translation.get(0), translation.get(2)
                    // - portrait: translation.get(1), translation.get(2)
                    double degreestoTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beacon.getName() + "-Degrees: ", degreestoTurn);
                }
            }
            telemetry.update();
        }

    }


    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){

        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);

        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }

    private static String getLicenseKey () {
        // url: developer.vuforia.com
        // user/pw: beeppatrol/Team96snore
        return LICENSE_KEY;
    }

    private static final String LICENSE_KEY = "AehWUEP/////AAAAGdLM1Ir3CEUunWFOGlSVegZ02oYjauBrfpYGcP/MNvZGEWO15KaOdjuIx0XAGISDJtiT9pfALwG5bGHfY2d5LVLV3jBq+2vLfcYh7zxUbHOcJpPfbzpUDVkGI5WHZlZ6IaqoCAEPznkxcZ5uyMwfZr1qyZp9LVTTAFhYwjRgSuF4/mcjzI3/ujUOZEKUzIOQbSlAPyNkiNMnRA0RHlzK7djpkXvghYsX7LYJDnJc5Fvpi6mqZqI+lyco0jnUHhMh4l7HczZ1HbKTAwuJFqc3aQab8bnjw9QegJb62vURA/ljwEIEUhT6mEGx+XJSOUA+KCwi/WDnKcZwOZr43VqmHPgLCvJmTFpVeOdBY4ozX5/J";
    private static final int  NUMBER_OF_BEACONS = 4; // number of beacons on the FTC game floor
    private static final String FILE_NAME = "FTC_2016-17";  // name of xml file which contains the name of the images (see the project's FtcRobotController/assets folder)


    // find an absolute position on the field
    // see ftc/doc/tutorial folder in this project, file: FTC_FieldCoordinateSystemDefinition.pdf
    // for information about how the field coordinates work..
    //
    // https://www.youtube.com/watch?v=2z-o9Ts8XoE
    // see @16:00 for brief demo
    // see ConceptVuforiaNavigation.java class

}
