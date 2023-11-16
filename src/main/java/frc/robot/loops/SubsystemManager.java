package frc.robot.loops;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ForkJoinPool;
import java.util.concurrent.TimeUnit;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Subsystem;
import frc.robot.utils.CrashTrackingRunnable;

public class SubsystemManager {

	private final ForkJoinPool threadPool = new ForkJoinPool(3);
	private List<Subsystem> subsystems = new ArrayList<>();
	public final double period;
	private final Notifier notifier;
	private final Object taskRunningLock = new Object();
	private boolean running;
	private double timestamp = 0;
	private double dT = 0;
	private double loopcnt = 0;


	public SubsystemManager(double period){
		var ss = this;
		CrashTrackingRunnable runnable = new CrashTrackingRunnable() {
			@Override
			public void runCrashTracked() {
				synchronized (taskRunningLock) {
					if (running) {
						double now = Timer.getFPGATimestamp();

						ss.run();

						dT = now - timestamp;
						timestamp = now;
					}
				}
			}
		};

		this.period = period;
		notifier = new Notifier(runnable);
		running = false;
	}

	public void setSubsystems(Subsystem... subsystems) {
		this.subsystems = Arrays.asList(subsystems);
	}

	public boolean checkSubsystems() {

		boolean returnValue = true;

		for (Subsystem s : subsystems) {
			returnValue &= s.checkSystem();
		}

		return returnValue;
	}

	public synchronized void stop() {
		if(running){
			System.out.println("Stopping subsystem loops");
			subsystems.forEach(Subsystem::stop);
			notifier.stop();

			synchronized (taskRunningLock) {
				running = false;
				timestamp = Timer.getFPGATimestamp();
				for (Subsystem subsystem : subsystems) {
					System.out.println("Stopping " + subsystem.getId());
					subsystem.stop();
				}
			}
		}
	}

	public void stopSubsystems() {
		subsystems.forEach(Subsystem::stop);
	}

	public synchronized void run() {
		double ost = Timer.getFPGATimestamp();
		loopcnt += 1;
		Logger.getInstance().recordOutput("Overrun", String.format("%s-startloop  : %s", loopcnt, ost));

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.readPeriodicInputs(st);
			double et = Timer.getFPGATimestamp();

			if (et - st > 0.01) {
				Logger.getInstance().recordOutput("Overrun", String.format("%s.read  : %s", subsystem.getId(), et - st));
				// DriverStation.reportError(String.format("%s.readPeriodicInputs took too long: %s", subsystem.getId(), et - st), false);
			}
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		Logger.getInstance().recordOutput("Overrun", String.format("%s-startProcessLoop  : %s", loopcnt, Timer.getFPGATimestamp()));
		threadPool.submit(() -> subsystems.parallelStream().forEach(loop -> {
			double st = Timer.getFPGATimestamp();
			loop.processLoop(st);
			double et = Timer.getFPGATimestamp();

			if (et - st > 0.01) {
				Logger.getInstance().recordOutput("Overrun", String.format("%s.onLoop: %s", loop.getId(), et - st));
				// DriverStation.reportError(String.format("%s.onLoop took too long: %s", loop.getId(), et - st), false);
			}
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		Logger.getInstance().recordOutput("Overrun", String.format("%s-startWrite  : %s", loopcnt, Timer.getFPGATimestamp()));
		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.writePeriodicOutputs(st);
			double et = Timer.getFPGATimestamp();

			if (et - st > 0.01) {
				Logger.getInstance().recordOutput("Overrun", String.format("%s.write : %s", subsystem.getId(), et - st));
				// DriverStation.reportError(String.format("%s.writePeriodicOutputs took too long: %s", subsystem.getId(), et - st), false);
			}
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		var dt = Timer.getFPGATimestamp();
		Logger.getInstance().recordOutput("Overrun", String.format("%s-startTelem  : %s", loopcnt, dt));
		
		dt = dt - ost;
		Logger.getInstance().recordOutput("Overrun", String.format("%s-looptimebeforeTelem  : %s", loopcnt, dt));
		if(dt > .02){
			Logger.getInstance().recordOutput("Overrun", String.format("whole loop overrun : %s", dt));
			DriverStation.reportWarning(String.format("Loop overrun [%s], skipping telemetry...",dt), false);
			return;
		}

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.outputTelemetry(timestamp);
			double et = Timer.getFPGATimestamp();

			if (et - st > 0.01) {
				Logger.getInstance().recordOutput("Overrun", String.format("%s.Telem : %s", subsystem.getId(), et - st));
				// DriverStation.reportError(String.format("%s.outputTelemetry took too long: %s", subsystem.getId(), et - st), false);
			}
		}));
		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		Logger.getInstance().recordOutput("Overrun", String.format("%s-endloop  : %s", loopcnt, Timer.getFPGATimestamp()));
	}

	public synchronized void start() {
		if (!running) {
			System.out.println("Starting loops");

			synchronized (taskRunningLock) {
				timestamp = Timer.getFPGATimestamp();
				running = true;
			}

			notifier.startPeriodic(period);
		}
	}
}
