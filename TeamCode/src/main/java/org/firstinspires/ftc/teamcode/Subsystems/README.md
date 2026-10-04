# Turret: CR servos + Melonbotics encoder (analog)

Files
- shooter/Turret.java              - NEW closed-loop turret (replaces the servo-position one)
- shooter/TurretAim.java           - goal-tracking math (unchanged, same as before)
- util/MathHelpers.java            - angle wrapping (unchanged)
- opmodes/TurretCalibration.java   - manual drive + live encoder readout (do this first)
- opmodes/TurretTestNew.java       - angle targets + PID tuning

Do NOT use the old ServoEx / TurretTestPosition with this.

Wiring
- Encoder's 3-pin analog output -> Control Hub analog port, named "turretEncoder" in the config
  (use the Melonbotics JST joiner board or the splice described in their analog docs).
- Both turret servos in CR mode, named "turretLeft" / "turretRight".
- Encoder goes on the turret / big gear axis (7mm hex bore), NOT on a servo. Then the reading is the real turret angle.
  Analog output is single-turn (0-360), so that only works unambiguously on the 1:1 turret axis.
  The turret must be physically stopped before +-180 degrees.

Usage
    turret.setTurretAngle(TurretAim.angleToGoal(pose, goalPose, angularVel));
    turret.update();   // every loop!

Setup order
1. TurretCalibration: servo directions, ENCODER_REVERSED, ENCODER_ZERO_DEG
2. TurretTestNew: tune kS, kP, kD in Dashboard (kP/kD/kS defaults are only starting guesses)
