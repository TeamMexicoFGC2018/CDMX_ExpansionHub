/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="RoboPro", group="Iterative Opmode")

public class OmniTriple extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centreDrive = null;
    private DcMotor elevadorDrive = null;
    private CRServo eolicoDrive = null;
    private Servo brazoServo = null;
    private Servo manoServo = null;
    private DcMotor cajasRDrive = null;
    private DcMotor cajasLDrive = null;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDrive  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotor");
        centreDrive = hardwareMap.get(DcMotor.class, "centreMotor");
        elevadorDrive = hardwareMap.get(DcMotor.class, "elevadorMotor");
        //eolicoDrive = hardwareMap.get(CRServo.class, "eolicoServo");
        //brazoServo = hardwareMap.get(Servo.class, "brazoServo");
        //manoServo = hardwareMap.get(Servo.class, "manoServo");
        cajasLDrive = hardwareMap.get(DcMotor.class, "cajasleft");
        cajasRDrive = hardwareMap.get(DcMotor.class, "cajasright");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        centreDrive.setDirection(DcMotor.Direction.FORWARD);
        elevadorDrive.setDirection(DcMotor.Direction.REVERSE);
        cajasRDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    public static double controlP(double pAct, double des) {
      double dif = Math.abs(des-pAct);
      if (dif > 0.3) {
        if (des > pAct) {
          pAct = pAct + 0.1;
        } else if (des < pAct) {
          pAct = pAct - 0.1;
        }
      }  else {
        pAct = des;
      }
      return Range.clip(pAct, -1, +1);
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    double tiempo = 0;
    double leftPower = 0;
    double rightPower = 0;
    double centrePower = 0;
    double eolicoPower = 0;
    boolean presd = false;
    double brazoPosition = 1;
    boolean presd1 = false;
    double manoPosition = 1;
    boolean presd2 = false;

    @Override
    public void loop() {
        double elevadorPower;
        double cajasPower;
        double tiempoActual = runtime.milliseconds();

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.left_stick_x;
        double leftDeseado    = Range.clip(drive + turn, -1.0, 1.0);
        double rightDeseado   = Range.clip(drive - turn, -1.0, 1.0);
        double centreDeseado = gamepad1.right_stick_x;

        //Acceleration control
        if (tiempoActual >= tiempo + 40) {
          leftPower = controlP(leftPower,leftDeseado);
          rightPower = controlP(rightPower,rightDeseado);
          centrePower = controlP(centrePower, centreDeseado);
          tiempo = tiempoActual;
        }

        // Control power of wheels.
        if (gamepad1.right_trigger > 0) {
          leftPower = leftPower * 0.75;
          rightPower = rightPower * 0.75;
          centrePower = centrePower * 0.75;
        } else if(gamepad1.left_trigger>0){
          leftPower = leftPower * 0.25 + leftPower * 0.5 * (1 - gamepad1.left_trigger);
          rightPower = rightPower * 0.25 + rightPower * 0.5 * (1 - gamepad1.left_trigger);
          centrePower = centrePower * 0.25 + centrePower * 0.5 * (1 - gamepad1.left_trigger);
        }

        // Move the lift.
        if (gamepad1.dpad_up) {
          elevadorPower = 0.8;
        } else if (gamepad1.dpad_down) {
          elevadorPower = -0.8;
        } else {
          elevadorPower = 0;
        }

        // Activate the movement of the mechanism to move the air turbine
        /*if (gamepad1.y) {
          presd = true;
        } else if (!gamepad1.y && presd) {
          if (eolicoPower == 0) {
            eolicoPower = 1;
          } else {
            eolicoPower = 0;
          }
          presd = false;
        }*/

        //Move the arm for solar panels
        /*if (gamepad1.left_bumper) {
          presd1 = true;
        } else if (!gamepad1.left_bumper && presd1) {
          if (brazoPosition == 0) {
            brazoPosition = 1;
          } else {
            brazoPosition = 0;
          }
          presd1 = false;
        }

        //Grab or leave the solar panel
        if (gamepad1.right_bumper) {
          presd2 = true;
        } else if (!gamepad1.right_bumper && presd2) {
          if (manoPosition == 0) {
            manoPosition = 0.5;
          } else {
            manoPosition = 0;
          }
          presd2 = false;
        }*/

        //Mechanism to pick boxes
        if (gamepad1.a) {
          cajasPower = 1;
        } else if (gamepad1.b) {
          cajasPower = -1;
        } else {
          cajasPower = 0;
        }

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        centreDrive.setPower(centrePower);
        elevadorDrive.setPower(elevadorPower);
        //eolicoDrive.setPower(eolicoPower);
        //brazoServo.setPosition(brazoPosition);
        //manoServo.setPosition(manoPosition);
        cajasLDrive.setPower(cajasPower);
        cajasRDrive.setPower(cajasPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", rightPower);
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
