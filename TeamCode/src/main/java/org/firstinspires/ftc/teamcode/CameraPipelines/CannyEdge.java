package org.firstinspires.ftc.teamcode.CameraPipelines;

import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
//import org.opencv.core.Point;

public class CannyEdge extends OpenCvPipeline {
   public Mat canny(Mat src){
      Mat gray = new Mat(src.rows(), src.cols(), src.type());
      Mat edges = new Mat(src.rows(), src.cols(), src.type());
      Mat dst = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));

      Imgproc.cvtColor(src, gray, Imgproc.COLOR_RGB2GRAY);
      Imgproc.blur(gray, edges, new Size(3, 3));
      Imgproc.Canny(edges, edges, 5, 100);
      src.copyTo(dst, edges);

      return dst;
   }

   public Mat processFrame(Mat img) {
      return canny(img);
   }
}