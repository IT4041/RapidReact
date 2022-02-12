/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;

public class Camera extends SubsystemBase {
  /**
   * Creates a new Camera.
   */
	UsbCamera cam;

  public Camera(){
    cam = CameraServer.startAutomaticCapture();
		cam.setResolution(320, 240);
		cam.setFPS(20);
    this.toBlackandWhite();
  }

  public void startCapture(){
    cam = CameraServer.startAutomaticCapture();
		cam.setResolution(320, 240);
		cam.setFPS(20);
    this.toBlackandWhite();
  }


  @Override
  public void periodic() {
    super.periodic();
    // This method will be called once per scheduler run
  }

  private void toBlackandWhite(){

      CvSink cvSink = CameraServer.getVideo();
      CvSource outputStream = CameraServer.putVideo("GreyScale", 320, 240);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }

  }
}

