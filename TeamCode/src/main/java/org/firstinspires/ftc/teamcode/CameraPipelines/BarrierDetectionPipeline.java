package org.firstinspires.ftc.teamcode.CameraPipelines;

import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;

import java.util.ArrayList;

public class BarrierDetectionPipeline extends OpenCvPipeline {
    ArrayList<Double> points = new ArrayList<Double>();
   private Mat canny(Mat src){
      Mat gray = new Mat(src.rows(), src.cols(), src.type());
      Mat edges = new Mat(src.rows(), src.cols(), src.type());
      Mat dst = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));


      Imgproc.cvtColor(src, gray, Imgproc.COLOR_RGB2GRAY);
      Imgproc.blur(gray, edges, new Size(3, 3));
      Imgproc.Canny(edges, edges, 5, 100);
      src.copyTo(dst, edges);

      return dst;
   }
   private Mat houghs(Mat src){
    Mat lines = new Mat(src.rows(), src.cols(), src.type());

    Imgproc.HoughLinesP(src, lines, 1, Math.PI/180, 150);

    for (int i = 0; i < lines.rows(); i++) {
       double[] data = lines.get(i, 0);
       double rho = data[0];
       double theta = data[1];
       double a = Math.cos(theta);
       double b = Math.sin(theta);
       double x0 = a*rho;
       double y0 = b*rho;
       Point pt1 = new Point();
       Point pt2 = new Point();

       pt1.x = Math.round(x0 + 1000*(-b));
       pt1.y = Math.round(y0 + 1000*(a));
       pt2.x = Math.round(x0 - 1000*(-b));
       pt2.y = Math.round(y0 - 1000 *(a));

       points.add(pt1.x);
       points.add(pt1.y);
       points.add(pt2.x);
       points.add(pt2.y);

       Imgproc.line(src, pt1, pt2, new Scalar(0, 255, 255), 3);
    }
    return src;
}


   public Mat processFrame(Mat img) {
      return houghs(canny(img));
   }

   public ArrayList<Double> coordinates(){
    ArrayList<Double> save = points;
    points.clear();
    return save;
   }
}
