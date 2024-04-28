package org.firstinspires.ftc.teamcode.drive.OGCode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PipeLineDetector extends OpenCvPipeline {
    public enum Status{
        VERDE1,
        ROZ2,
        ALBASTRU3
    }
    public Status caz = Status.ALBASTRU3;
    public Telemetry telemetry;
    public Rect r1;
    public int xA = 0, yA = 0;
    public int xB = 0, yB = 0;
    public double sumVerzui = 0 , sumAlbastrui = 0 , sumRozului = 0;
    public double Arie=0;
    public PipeLineDetector(int xAI, int yAI, int xBI, int yBI)
    {
        this.xA=xAI;
        this.yA=yAI;
        this.xB=xBI;
        this.yB=yBI;
    }
    @Override
    public Mat processFrame(Mat input) {
        Mat HSV = new Mat();
        Mat copVerde = new Mat();
        Mat copAlbastru = new Mat();
        Mat copRoz = new Mat();
        Mat bunVerde = new Mat();
        Mat bunAlbastru = new Mat();
        Mat bunRoz = new Mat();
        copVerde = input;
        copAlbastru = input;
        copRoz = input;
        final Point A = new Point (xA,yA);
        final Point B = new Point (xB,yB);
        double arie = (B.x-A.x)*(B.y-A.y);
        arie = Math.abs(arie);
        Imgproc.cvtColor(copVerde, HSV, Imgproc.COLOR_RGB2HSV);
        Scalar lowGreen = new Scalar(30,50,20);
        Scalar highGreen = new Scalar(80,255,255);
        Core.inRange(HSV, lowGreen, highGreen,bunVerde);
        Mat verde = bunVerde.submat(new Rect(A,B));
        Scalar sumVerde = Core.sumElems(verde);
        Imgproc.cvtColor(copAlbastru, HSV, Imgproc.COLOR_RGB2HSV);
        Scalar lowAlbastru = new Scalar(110,50,20);
        Scalar highAlbastru = new Scalar(130,255,255);
        Core.inRange(HSV, lowAlbastru, highAlbastru,bunAlbastru);
        Mat albastru = bunAlbastru.submat(new Rect(A,B));
        Scalar sumAlbastru = Core.sumElems(albastru);

        Imgproc.cvtColor(copRoz, HSV, Imgproc.COLOR_RGB2HSV);
        Scalar lowRoz = new Scalar(160,50,20);
        Scalar highRoz = new Scalar(180,255,255);
        Core.inRange(HSV, lowRoz, highRoz,bunRoz);
        Mat roz = bunRoz.submat(new Rect(A,B));
        Scalar sumRoz = Core.sumElems(roz);

        if(sumVerde.val[0]/arie>0.5) caz = Status.VERDE1;
        else if(sumRoz.val[0]/arie>0.5) caz = Status.ROZ2;
        else caz = Status.ALBASTRU3;
        Imgproc.rectangle(input,A,B, new Scalar(0,0,255),3);
        sumVerzui = sumVerde.val[0];
        sumAlbastrui = sumAlbastru.val[0];
        sumRozului = sumRoz.val[0];
        Arie = arie;
        roz.release();albastru.release(); verde.release(); HSV.release();bunVerde.release();bunRoz.release();bunAlbastru.release();
        return input;
    }
}

