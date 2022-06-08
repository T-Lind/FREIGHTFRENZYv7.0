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
abstract public class BotContainer extends LinearOpMode{
   private PathSequence pathSequence;
   private NeoMarkerList markerList;
   private RunnableCollective runMarkerObject;

   private WebcamName weCam;
   private OpenCvCamera camera;
   private TSEDetectionPipeline pipeline; // Replace with some other pipeline

   protected DcMotorEx leftFront, leftBack, rightFront, rightBack;
   protected final double wheelR = 0.03715;
   protected final double trackWidth = 0.295;


   /**
    * Assign the path sequence to this LinearOpMode
    * @param pathSequence is the set of basic robot instructions to move along
    *                     (final linear wheel velocities).
    */
   protected final void setPathSequence(PathSequence pathSequence) {
      this.pathSequence = pathSequence;
   }

   protected final void setMarkerList(NeoMarkerList markerList){
      this.markerList = markerList;
      runMarkerObject = new RunnableCollective(this.markerList);
   }
   protected final void executeMarkers(){
      // Start the markers
      if(runMarkerObject != null)
         runMarkerObject.activateMarkers();
   }
   protected final void stopMarkers(){
      runMarkerObject.setStopMarkers();
   }

   protected final void executeAuto() {
      // Only execute the auto when the button is pressed
      waitForStart();

      // Start markers
      if (markerList != null)
         executeMarkers();

      // Follow the path
      if (pathSequence != null)
         pathSequence.follow();
   }

   protected final void initialize(){
      initMotors();
      initCamera();
      if(pathSequence != null)
         pathSequence.buildAll();

      do{
         // In initialization
         telemetry.addData("The initialization has started successfully.","");
         telemetry.update();
      }
      while (!opModeIsActive());
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
         @Override
         public void onOpened() {
            telemetry.update();
            camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
         }
         @Override
         public void onError(int errorCode) {
            telemetry.addData("Error in camera initialization! Error code",errorCode);
            telemetry.update();
         }
      });
   }

}
