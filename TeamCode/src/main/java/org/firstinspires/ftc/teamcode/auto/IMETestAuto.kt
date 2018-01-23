package org.firstinspires.ftc.teamcode.auto

import android.preference.PreferenceManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.IntelligentMotionEngine

@Autonomous(name = "IME Test Auto")
class IMETestAuto : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = IntelligentMotionEngine(this)
        val preferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext)
        telemetry.addLine(preferences.getString("start_point", "nope"))
        telemetry.update()
        waitForStart()
        telemetry.clear()
        telemetry.addData("Intelligent Motion Engine self test", true).setRetained(true)
        telemetry.update()
        sleep(1000)
        telemetry.addLine("Test: Proportional IMU Turn")
        telemetry.update()
        if (opModeIsActive()) {
            drivetrain.pTurn(90)
        } else {stop()}
        telemetry.addLine("Proportional IMU Turn test complete!")
        telemetry.addLine("Returning to start heading for more testing")
        telemetry.update()
        sleep(1000)
        if (opModeIsActive()) {
            drivetrain.pTurn(0)
        } else {stop()}
        telemetry.addLine("Proportional IMU turn test done.")
        telemetry.update()
        sleep(3000)
        telemetry.clear()
        // standard encoder drive
        telemetry.addLine("Test: Standard Encoder Drive")
        telemetry.update()
        if (opModeIsActive()) {
            drivetrain.encoderCountDrive(1120)
        } else {stop()}
        telemetry.addLine("Standard Encoder Drive test complete!")
        telemetry.update()
        sleep(3000)
        telemetry.clear()
        // rotation encoder drive
        telemetry.addLine("Test: Rotation Encoder Drive")
        telemetry.update()
        if (opModeIsActive()) {
            drivetrain.encoderRotationDrive(-1.0)
        } else {stop()}
        telemetry.addLine("Rotation Encoder Drive test complete!")
        telemetry.update()
        sleep(3000)
        telemetry.clear()
        // distance encoder drive
        telemetry.addLine("Test: Distance Encoder Drive")
        telemetry.update()
        if (opModeIsActive()) {
            drivetrain.encoderInchDrive(12.0)
        } else {stop()}
        telemetry.addLine("Distance Encoder Drive test complete!")
        telemetry.update()
        sleep(3000)
        telemetry.clear()
        // rotation encoder strafe
        telemetry.addLine("Test: Rotation Encoder Strafe")
        telemetry.update()
        if (opModeIsActive()) {
            drivetrain.encoderRotationStrafe(5.0)
        } else {stop()}
        telemetry.addLine("Rotation Encoder Strafe test complete!")
        telemetry.addLine("Moving back to test center")
        telemetry.update()
        sleep(3000)
        if (opModeIsActive()) {
            drivetrain.encoderRotationStrafe(-5.0)
        } else {stop()}
        sleep(500)
        if (opModeIsActive()) {
            drivetrain.encoderInchDrive(-12.0)
        } else {stop()}
        telemetry.clearAll()
        telemetry.addLine("IME self-test complete. goodbye")
        telemetry.update()
        sleep(3000)

    }

}