# BIOBUZZ blue/right autonomous draft

The existing TeamCode source layout and hardware movement helpers are retained.
Root-level automonous.java, teleop.java and yosiauto.java are untouched.
Do not copy those root examples wholesale into TeamCode: automonous.java declares
the same BioBuzzAutonomous class and several filenames differ from their public classes.

## Intended sequence
Start EMPTY; collect one expected ball at each of three calibrated pickup points,
navigate to the shooting position, discharge, navigate to the final approach
with intake off, collect exactly one expected ball, then return and discharge.
Navigation never changes inventory. The software estimate cannot exceed four.
A pickup failure or STOP aborts; do not retry because a partial pickup may have occurred.

## Deliberate hardware blocks
The OpMode has @Disabled and ROUTE_CALIBRATED=false. Keep these until validation.
shootExpectedBalls() always stops all motors and returns false. It does not run
endFlip, assume a continuous wheel, report success, or clear the estimated count.
Thus even if the route gate is enabled, execution stops at the first shooting step.
Identify whether endFlip is a flipper, catapult or flywheel; implement bounded,
STOP-aware firing with its required homing/limits and feed mechanism. Only decrement
inventory after a successful discharge cycle; abort on failure. No discharge is
sensor-confirmed by this code.

## Calibration and rules
All pickup locations, including the final (1.48, 0.30) m point, are PROPOSED values,
not verified ball centers. The earlier corridor points are now explicit pickup
targets and MUST be replaced with measured positions; reaching them proves no pickup.
The existing blue (+X) / audience (+Z), start orientation and HIVE aim convention
remain assumptions to verify against the actual event field and Driver Station.
Reference: https://github.com/caryden/ftc-biobuzz-sim
This third-party simulator is not a rule authority or a guarantee of collision-free travel.

Measure motor direction, encoder ticks/rev, gearing, wheel diameter, strafe/turn
corrections, turn radius, intake direction/power/settling, robot footprint, travel
times and shot alignment. Strafe motor signs now match the existing teleop's
mecanum convention (LF+, LB-, RB+, RF-). Pose is commanded encoder dead reckoning,
not measured field localization; slip and aiming errors accumulate. Each driveTo
uses strafe then forward and needs clearance along both segments, including turns.
Tune the entire route to fit the event's autonomous duration; per-move timeouts
alone do not establish a match-time budget.

Four is the user's requested capacity, not a verified current rule limit.
No sensor counts balls: multiple balls in one pass, preloads, missed intakes and
failed shots invalidate the estimate. Use a sensor or mechanical limit to prevent
actual overfill. Verify current event rules for possession, preloads, legal starting
area, game-piece access, shooting zones/targets, partner clearance and autonomous
timing before use. Rule compliance and physical capacity are NOT guaranteed.

## Verification
Run: python tests/check_biobuzz.py (Python 3 plus javac/java on PATH).
The check compiles both production classes against minimal FTC doubles and executes
the sequence, final count, capacity rejection, disabled-shot inventory retention,
failed pickup/STOP cleanup, default route gate and mecanum strafe signs.
This is NOT a real FTC SDK/Android compilation or physics simulation.
The repository has no Gradle wrapper, build files or SDK dependencies. Integrate
TeamCode into the team's FTC SDK project and run its Android build before deployment.
