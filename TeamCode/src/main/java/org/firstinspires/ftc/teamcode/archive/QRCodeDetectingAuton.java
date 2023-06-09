package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.archive.auton.AutonBot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Autonomous(name = "QR Code Detecting Auton")
public class QRCodeDetectingAuton extends LinearOpMode {
    public AutonBot robot = new AutonBot();
    public ReadQRCodePipeline pipeline = new ReadQRCodePipeline();

    @Override
    public void runOpMode() {
        /*

        Initialization (User pressed "init")

         */
        robot.init(hardwareMap);

        robot.webcam.setPipeline(pipeline);
        robot.webcam.setMillisecondsPermissionTimeout(2500);
        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
               {
                   @Override
                   public void onOpened() {
                       robot.webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                   }

                   @Override
                   public void onError(int errorCode) {
                       telemetry.addData("error", errorCode);
                   }
               }
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        /*

        Program (user pressed start)

         */

        //=
        // robot.command(3, 10, "move", this);]
        String qrCodeText = pipeline.getQrCode();
        telemetry.addData("QR Code", qrCodeText);
        telemetry.addData("a", pipeline.screen.rows());
        telemetry.update();
        sleep(5000);

        switch (qrCodeText) {
            case "3":
                telemetry.addData("QR Code num", "3");
                break;
            case "2":
                telemetry.addData("QR Code num", "2");
                break;
            case "1":
                telemetry.addData("QR Code num", "1");
            default:
                telemetry.addData("QR Code num", "Default");
                break;
        }
        telemetry.update();
        sleep(5000);

        /*

        Program Done

         */

        telemetry.addData("Status", "Done with auton");
        telemetry.update();
        sleep(5000);
    }

    public void telemetry() {

    }
}
