package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo

enum class LiftState {GOING_UP, GOING_DOWN, STATIONARY,}
enum class ClawState {OPEN, CLOSED, MECHANISM_HOLD, FOLDED, STARTING}

class FourBar(parentOpMode: OpMode) {

    private val lift_left: DcMotor = parentOpMode.hardwareMap.dcMotor.get("lift_left")
    private val lift_right: DcMotor = parentOpMode.hardwareMap.dcMotor.get("lift_right")
    private val claw_left: Servo = parentOpMode.hardwareMap.servo.get("claw_left")
    private val claw_right: Servo = parentOpMode.hardwareMap.servo.get("claw_right")

    init {
        lift_left.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift_right.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        //lift_right.direction = DcMotorSimple.Direction.REVERSE
        claw_left.direction = Servo.Direction.REVERSE
        claw_right.direction = Servo.Direction.REVERSE
        parentOpMode.telemetry.addLine("4466 FourBar v1.2")
        parentOpMode.telemetry.addLine("Running as expected.")
        parentOpMode.telemetry.update()
    }

    /**
     * @return a List<Double> of motor powers in the order [left, right].
     */
    fun getMotorPowers() : List<Double> = listOf(lift_left.power, lift_right.power)

    /**
     * @return a List<Double> of servo positions in the order [left, right].
     */
    fun getClawPositions() : List<Double> = listOf(claw_left.position, claw_right.position)

    /**
     * Runs a state-machine controller for the lift and claw.
     * @see LiftState
     * @see ClawState
     */
    fun startStateControl(liftState: LiftState, clawState: ClawState) {
        // lift control loop - would be better with encoders
        when (liftState) {
            // run the lift upwards
            LiftState.GOING_UP -> runLift(0.25)
            // run the lift downwards
            LiftState.GOING_DOWN -> runLift(-0.25)
            // an instant-stop function
            LiftState.STATIONARY -> runLift(0.0)
        }
        // claw control loop
        when (clawState) {
            //TODO all these numbers need tuning
            ClawState.OPEN -> setClaw(0.8)
            ClawState.CLOSED -> setClaw(0.4)
            ClawState.MECHANISM_HOLD -> setClaw(0.15)
            ClawState.FOLDED -> setClaw(0.8)
            ClawState.STARTING -> setClaw(0.15)
        }
    }
    /**
     * Runs a state-machine controller for the lift and claw using a Gamepad for input.
     * @see startStateControl
     */
    fun startStateControl(gamepad: Gamepad) {
        var liftState = LiftState.STATIONARY
        var clawState = ClawState.STARTING
        when {
            gamepad.left_trigger > 0F -> liftState = LiftState.GOING_DOWN
            gamepad.right_trigger > 0F -> liftState = LiftState.GOING_UP
            gamepad.x -> clawState = ClawState.OPEN
            gamepad.b -> clawState = ClawState.CLOSED
        }
        startStateControl(liftState, clawState)
    }

    @Deprecated(replaceWith = ReplaceWith("setClaw(position)"), message = "Method no longer in use")
    fun openClaw(position: Double = 0.8) {
        claw_left.position = position
        claw_right.position = position
    }

    @Deprecated(replaceWith = ReplaceWith("setClaw(position)"), message = "Method no longer in use")
    fun closeClaw(position: Double = 0.4) {
        claw_left.position = position
        claw_right.position = position
    }

    fun runLift(power: Double = 0.0) {
        lift_left.power = power
        lift_right.power = power
    }

    fun setClaw(position: Double) {
        // TODO find and implement position variance in the claw
        claw_left.position = position
        claw_right.position = position
    }

}