JCC = javac
JFLAGS = -d . -Xlint:unchecked

SOURCES = SearchProblem.java InformedSearchProblem.java Vector.java Trajectory.java \
	  Environment.java MotionPlanner.java MotionPlannerView.java Robot.java \
	  RobotArm.java PlanarRobot.java PRMPlanner.java RRTPlanner.java

MSOURCES = SearchProblem.java InformedSearchProblem.java Vector.java Trajectory.java \
	  Environment.java MotionPlanner.java MotionPlannerView.java Robot.java \
	  RobotArm.java PlanarRobot.java MPRMPlanner.java RRTPlanner.java



driver: $(SOURCES) MotionPlannerDriver.java
	$(JCC) $(JFLAGS) $(SOURCES) MotionPlannerDriver.java
	java assignment_motion_planning.MotionPlannerDriver

extra: $(SOURCES) MotionPlannerDriver.java
	$(JCC) $(JFLAGS) $(SOURCES) MotionPlannerDriver.java
	java assignment_motion_planning.MotionPlannerDriver


benchmark: $(SOURCES) Benchmark.java
	$(JCC) $(JFLAGS) $(SOURCES) Benchmark.java
	java assignment_motion_planning.Benchmark

clean:
	rm -f ./assignment_motion_planning/*.class
