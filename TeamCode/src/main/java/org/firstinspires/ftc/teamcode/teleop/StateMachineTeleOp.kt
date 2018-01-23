package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.FourBar
import org.firstinspires.ftc.teamcode.hardware.IntelligentMotionEngine

@TeleOp(name = "State Machine Test")
//@Disabled
class StateMachineTeleOp : LinearOpMode() {

    override fun runOpMode() {
        val drivetrain = IntelligentMotionEngine(this)
        val fourBar = FourBar(this)

        waitForStart()

        while (opModeIsActive()) {
            drivetrain.tankMecanum(gamepad1)
            fourBar.startStateControl(gamepad2)
            telemetry.addData("Orientation: ", drivetrain.getOrientation())
            telemetry.addData("Lifts: ", fourBar.getMotorPowers())
            telemetry.addData("Claws: ", fourBar.getClawPositions())
            telemetry.addData("Encoders: ", drivetrain.getEncoderPositions())
            telemetry.update()
        }
        drivetrain.runMotors(0.0)
    }

}