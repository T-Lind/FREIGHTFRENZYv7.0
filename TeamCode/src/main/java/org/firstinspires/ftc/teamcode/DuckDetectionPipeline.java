package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.CubeDetectionPipeline.CB_CHAN_MASK_THRESHOLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.core.Range;
import java.util.ArrayList;
import java.util.List;
import org.opencv.imgcodecs.Imgcodecs;

public class DuckDetectionPipeline extends OpenCvPipeline{
    double ducc_x = -1;
    double distance = -1;
    double angle = -1;

    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

    double FRAME = 1;
    static final int CB_CHAN_IDX = 2;
    static final int FOCAL_LENGTH = 470;
    static final double ducc_y = 36;
    static final double CAM_ANGLE = Math.atan(5.5/1.5);    // IN RADIANS
    @Override
    public Mat processFrame(Mat input) {
        return analyze(input);
    }
    public Mat analyze(Mat input){

        Core.extractChannel(input, input, CB_CHAN_IDX);
        Imgproc.threshold(input, input, 10, 255, Imgproc.THRESH_BINARY_INV);

        Imgproc.dilate(input, input, dilateElement);
        Imgproc.medianBlur(input, input, 11);

        Imgproc.Canny(input, input, 50, 250, 3, true);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        }

        Mat drawing = Mat.zeros(input.size(), CvType.CV_8UC3);
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(255,555,0);
            Imgproc.drawContours(drawing, contoursPolyList, i, color);
            Point p1 = boundRect[i].tl();
            Point p2 = boundRect[i].br();
            if((p2.x-p1.x > 40 && p2.y-p1.y > 20) && Math.abs(p2.x-p1.x)/Math.abs(p2.y-p1.y) > 0.86){
                Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);

                angle = ((p2.x-(input.cols()/2.0))/(input.cols()/2.0))*15;
                ducc_x = Math.tan((angle)*3.14159/180)*ducc_y; // amount to strafe in inches

            }
        }

        Imgproc.line(drawing, new Point(0,input.rows()/2.0),new Point(input.cols(),input.rows()/2.0), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);

        return drawing;
    }
    public double calcDistance(double width){
        double dstnce = (2*FOCAL_LENGTH)/width; // IN INCHES
        return Math.cos(CAM_ANGLE)*dstnce; //FLOOR DISTANCE
    }

//pipeline noice

    public double getDucc_x(){
        return ducc_x;
    }
    public double getDucc_y(){
        return ducc_y;
    }
    public double getAngle(){ return angle; }
    public double getDistance(){ return distance;}
    public double getFRAME(){ return FRAME;}
}
