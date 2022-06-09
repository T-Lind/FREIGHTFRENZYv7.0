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
 * Class to slim down autonomous programs for four motor drivetrains and execute markers.
 */
abstract public class BotContainer extends LinearOpMode{
   // Pathing and marker list objects
   private PathSequence pathSequence;
   private NeoMarkerList markerList;
   private RunnableCollectiveArray runMarkerObject;

   // Camera objects
   private WebcamName weCam;
   private OpenCvCamera camera;
   private TSEDetectionPipeline pipeline; // Replace with some other pipeline

   // These variables are protected so autonomous programs can use them easier
   protected DcMotorEx leftFront, leftBack, rightFront, rightBack;
   protected final double wheelR = 0.03715;
   protected final double trackWidth = 0.295;


   /**
    * Assign the path sequence to this LinearOpMode
    * @param pathSequence is the set of basic robot instructions to move along
    *                     (final linear wheel velocities).
    */
   protected final void setPathSequence(PathSequence pathSequence) {
      assert pathSequence != null : "You must enter a non-null object in BotContainer.setPathSequence()";
      this.pathSequence = pathSequence;
   }

   /**
    * Set the list of markers, also create the RunnableCollective object which creates the thread
    * branches.
    * @param markerList the list of markers (aliases it)
    */
   protected final void setMarkerList(NeoMarkerList markerList){
      assert markerList != null : "You must enter a non-null object in BotContainer.setMarkerList()";
      this.markerList = markerList;
      runMarkerObject = new RunnableCollectiveArray(this.markerList);
   }

   /**
    * Start the markers according to the times given, also check for null
    */
   protected final void executeMarkers(){
      // Start the markers
      if(runMarkerObject != null)
         runMarkerObject.activateMarkers();
   }

   /**
    * Interrupts all threads and stops the markers
    */
   protected final void stopMarkers(){
      runMarkerObject.setStopMarkers();
   }

   /**
    * Follows the path and marker list given, assuming one was given.
    */
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

   /**
    * Initialize all and build the path sequence if not null, relay a successful initialization
    * through the telemetry.
    */
   protected final void initialize(){
      // Initialize motors and camera
      initMotors();
      initCamera();

      // Null check - if not null then build the path sequence
      if(pathSequence != null)
         pathSequence.buildAll();

      do{
         // In initialization
         telemetry.addData("The initialization has started successfully.","");
         telemetry.update();
      }
      while (!opModeIsActive());
   }


   /**
    * Initialize and set the motors' properties
    */
   private void initMotors(){
      // build the motor objects
      leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
      leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
      rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
      rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");

      // Set the usage of encoders
      leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      // Sets zero power behavior to braking
      leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   }

   /**
    * Initialize the camera and set its properties.
    */
   private void initCamera(){
      // Get the webcam from the hardware map
      weCam = hardwareMap.get(WebcamName.class, "Webcam 1");
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

      // Create a new camera object in openCV
      camera = OpenCvCameraFactory.getInstance().createWebcam(weCam, cameraMonitorViewId);

      // instantiate and add the pipeline
      pipeline = new TSEDetectionPipeline();
      camera.setPipeline(pipeline);

      // Start the camera stream or throws an error
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
