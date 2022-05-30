package org.firstinspires.ftc.teamcode.auto;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Static class to slim down autonomous programs for four motor drivetrains.
 */
public class Bot {
   private DcMotorEx leftFront;
   private DcMotorEx leftBack;
   private DcMotorEx rightFront;
   private DcMotorEx rightBack;

   public final static double wheelR = 0.03715;
   public final static double trackWidth = 0.295;

   /**
    * Initialize four motors.
    */
   public Bot(){
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

   /**
    * @return the left front motor
    */
   public DcMotorEx LF(){
      return leftFront;
   }
   /**
    * @return the left back motor
    */
   public DcMotorEx LB(){
      return leftBack;
   }
   /**
    * @return the right front motor
    */
   public DcMotorEx RF(){
      return rightFront;
   }
   /**
    * @return the right back motor
    */
   public DcMotorEx RB(){
      return rightBack;
   }

}
