/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;


/**
 * This OpMode illustrates how to use the LimelightImageTools
 */
@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
//@Disabled
public class SensorLimelight3ATest extends LinearOpMode {


    @Override
    public void runOpMode()
    {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        limelight.deleteSnapshots();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();


         LimeLightImageTools llIt = new LimeLightImageTools(limelight);  // instantiate the lime light tools class, pulls IP address from LimeLight
        //LimeLightImageTools llIt = new LimeLightImageTools("172.29.0.1"); // alternate constructor that provides the IP address

        // Set the LimeLight to provide the image source for the drivers station
        // "camera stream" during Init.  Great for quick verification of camera or for match setup
        // This probably is legal in matches, but should be verified by users.
        llIt.setDriverStationStreamSource();

        // Start port forwarding to allow accessing the limelight from external devices such as
        // a desktop/laptop, a phone or even the driver station (use browser already on DS) that is
        // connected to the Robot's WIFI network.  Point your favorite browser at:
        // http://192.168.43.1:5800/  - displays raw image before processing
        // http://192.168.43.1:5801/  - opens the limelight configuration tool!
        // http://192.168.43.1:5802/  - display the processed
        // This is probably NOT legal in competition?
        llIt.forwardAll();

        // If using Ftc Dash Board, this will stream the limelight camera to dashboard
        FtcDashboard.getInstance().startCameraStream(llIt.getStreamSource(),10);

        waitForStart();



        /*  *** below here shows how to access a single image and send it to dashboard *** */
        // not really needed if using the startCameraStream() shown above, but might be useful
        // for capturing images at a particular time.


        int frames = 0;
        int droppedFrames = 0;
        boolean showProcessedImage = true;
        boolean lastAButton = false; // Store the state of the A button from the previous cycle
        long startTime = System.nanoTime();

        while (opModeIsActive()) {
            if (gamepad1.a && !lastAButton) {
                showProcessedImage = !showProcessedImage;
                frames = 0;
                droppedFrames = 0;
                startTime = System.nanoTime();
            }
            lastAButton = gamepad1.a;

            Bitmap bmp;
            if (showProcessedImage) {
                bmp = llIt.getProcessedBMP();
            } else {
                bmp = llIt.getRawBMP();
            }

            if (bmp != null) {
                FtcDashboard.getInstance().sendImage(bmp);
                frames++;
            } else {
                droppedFrames++;
            }
            long currentTime = System.nanoTime();
            double elapsedTimeSeconds = (currentTime - startTime) / 1_000_000_000.0;
            double frameRate =  (double)frames/elapsedTimeSeconds;

            RobotLog.d("LL_Test  " + (showProcessedImage ? "Processed":"Raw") + "  good frames = " + frames +"   Dropped frame = "+ droppedFrames + " Frame/second="+ frameRate);
        }

    }
}

