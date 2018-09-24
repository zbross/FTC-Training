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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * Welcome to Mr. Bross's basic Training TeleOp Code! There is nothing here but drive code,
 * but wow, it is pretty nice and simple! This is built from a sample code provided by FIRST.
 * Have a look under
 * FTC-Training/FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external.samples/
 * for other great examples!
 */

/*
 * First we have to define that this is a TeleOp Opmode by using @TeleOp. The example below is
 * easy to follow. The name and group are not actually necessary, but useful if you want to have
 * multiple modes. If you do not specify a name, you can only have one mode of that type (one TeleOp
 * and one Autonomous). You can include @Disabled to disable a particular OpMode. I have commented
 * it out because I don't want to disable it. This is useful for testing while reducing human error.
  */

@TeleOp(name="Training TeleOp", group="Linear Opmode")
//@Disabled
public class TrainingTeleOp extends LinearOpMode {

    // Declare OpMode members.
    // Notice that because I am using two motors on each side of the robot, they must be
    // initialized individually.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor rightDrive2 = null;
    private Servo armServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive1  = hardwareMap.get(DcMotor.class, "left_drive_1");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_drive_2");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive_1");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive_2");
        armServo = hardwareMap.get(Servo.class, "arm_servo");

        // Most robots need the motor(s) on one side to be reversed to drive forward.
        // Reverse the motor that runs backwards when connected directly to the battery.
        // This can only be determined with live testing, but is often the left side
        // in a traditional tank drive setup. Make sure that both motors on the same side
        // are driving in the same direction, or else there will be serious problems!
        leftDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);

        // Set the servo to its starting position, in this case, zero
        armServo.setPosition(0);

        // Wait for the game to start (driver presses PLAY).
        waitForStart();
        runtime.reset();

        // This is what will actually happen while the OpMode is running,
        // after the driver presses PLAY and before the driver presses STOP.
        while (opModeIsActive()) {

            // Setup a variable for each set of wheels to save power level for telemetry.
            // Also setup a variable for each stick on the gamepad.
            double leftPower;
            double leftInput;
            double rightPower;
            double rightInput;

            // Choose to drive using either Tank Mode, or POV Mode by commenting out the other.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            // double drive = -gamepad1.left_stick_y;
            // double turn  =  gamepad1.right_stick_x;
            // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            // rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each side.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // We have to invert the y-dimension of each stick, because +1 points backwards.
            leftInput  = -gamepad1.left_stick_y;
            rightInput = -gamepad1.right_stick_y;

            // Now that we have input, we need to save the input to the power variable. At this
            // time, we can choose to optionally scale the input. We do this if we want to make
            // sure that our robot accelerates a certain way. See the note below under scaleInput
            // to see how this works. Both scaled and unscaled options are provided. Use only one.
            leftPower = scaleInput(leftInput); // Scale left input.
            // leftPower = leftInput; // Don't scale left input.
            rightPower= scaleInput(rightInput); // Scale right input.
            // rightPower = rightInput; // Don't scale right input.

            // Send power to wheels. If any calculations or adjustments are necessary, do them first.
            // We used to need to clip the input from -1 to +1, but that is no longer necessary.
            leftDrive1.setPower(leftPower);
            leftDrive2.setPower(leftPower);
            rightDrive1.setPower(rightPower);
            rightDrive2.setPower(rightPower);

            // Servo control: I want to make it so that if the A button is being pressed, the servo is
            // set to 90 degrees (halfway). If the Y button is being pressed, the servo will be set
            // to 180 degrees (all the way). If either button is not being pressed, set back to zero.
            if (gamepad2.a) {
                armServo.setPosition(0.5);
            }
            else if (gamepad2.y) {
                armServo.setPosition(1.0);
            }
            else {
                armServo.setPosition(0.0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal) {
        // Define the values for the scale. Notice how the values don't change by a lot at the
        // beginning but change a lot at the higher levels? This means that there will be a
        // small change in speed when you move the stick a little bit, but a large change
        // in speed as you push more. You can change these values to adjust that.
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}