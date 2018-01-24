package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.IntelligentMotionEngine

@Autonomous(name = "Drive Rotations Auto")
class DriveRotationsAuto : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = IntelligentMotionEngine(this)
        drivetrain.setPID(0.69, 0.69, 0.04)
        drivetrain.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION)
        telemetry.addLine(drivetrain.getEncoderPositions().toString())
        waitForStart()
        drivetrain.setTargetPositions(2240)
        drivetrain.runMotors(0.6)
        while (drivetrain.areMotorsBusy()) {idle()}
        drivetrain.runMotors(0.0)
    }

}