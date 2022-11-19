package org.firstinspires.ftc.teamcode.auton;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvPipeline;

public class ReadQRCodePipeline extends OpenCvPipeline {
    private volatile String qrCodeOutput = "";
    public Mat screen;
    private QRCodeDetector qrCodeDetector = new QRCodeDetector();

    public Mat processFrame(Mat input)
    {
        screen = input;
        qrCodeOutput = qrCodeDetector.detectAndDecode(input);
        Imgproc.putText(input, qrCodeOutput, new Point(100, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0,0,0), 3);
        return input;
    }
    public String getQrCode() {
        return qrCodeOutput;
    }
}
