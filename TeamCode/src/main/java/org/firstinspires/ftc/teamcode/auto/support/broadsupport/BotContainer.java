package org.firstinspires.ftc.teamcode.auto.support.broadsupport;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CameraPipelines.TSEDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/**
 * Class to slim down autonomous programs for four motor drivetrains.
 */
public class BotContainer {
   private LinearOpMode linearOpMode;
   private PathSequence pathSequence;

   public DcMotorEx leftFront, leftBack, rightFront, rightBack;

   private Telemetry telemetry;

   private WebcamName weCam;
   private OpenCvCamera camera;
   private TSEDetectionPipeline pipeline;

   public final static double wheelR = 0.03715;
   public final static double trackWidth = 0.295;


   public BotContainer(LinearOpMode linearOpMode){
      this.linearOpMode = linearOpMode;

      initMotors();
      initCamera();

      pathSequence.buildAll();
   }

   public void setPathSequence(PathSequence pathSequence) {
      this.pathSequence = pathSequence;
   }

   private void executeAuto(){
      pathSequence.follow();
   }

   private void inInitialization(){
      while (!linearOpMode.opModeIsActive()) {
         // In initialization
      }
   }

   private void initMotors(){
      // build the motor objects
      leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
      leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
      rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
      rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");

      leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   }

   private void initCamera(){
      weCam = hardwareMap.get(WebcamName.class, "Webcam 1");
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      camera = OpenCvCameraFactory.getInstance().createWebcam(weCam, cameraMonitorViewId);
      pipeline = new TSEDetectionPipeline();
      camera.setPipeline(pipeline);
      camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
         // @Override
         public void onOpened() {
            telemetry.update();
            camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
         }
         @Override
         public void onError(int errorCode) {
         }
      });
   }

   public void buildAllPaths(){
      pathSequence.buildAll();
   }
   public void followPaths(){
      pathSequence.follow();
   }
}
