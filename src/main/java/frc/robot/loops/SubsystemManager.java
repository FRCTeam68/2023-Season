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

	private double m_readCnt = 0;
    private double m_processCnt = 0;
    private double m_writeCnt = 0;
    private double m_telemCnt = 0;


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

		m_readCnt+=1;
		Logger.getInstance().recordOutput("L1", m_readCnt);

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.readPeriodicInputs(st);
			double et = Timer.getFPGATimestamp();

			Logger.getInstance().recordOutput("N1", subsystem.getId().toString());
			Logger.getInstance().recordOutput("D1", et - st);

			if (et - st > 0.01) {
				Logger.getInstance().recordOutput("O1", true);;
				// DriverStation.reportError(String.format("%s.readPeriodicInputs took too long: %s", subsystem.getId(), et - st), false);
			}
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		m_processCnt+=1;
		Logger.getInstance().recordOutput("L2", m_processCnt);

		threadPool.submit(() -> subsystems.parallelStream().forEach(loop -> {
			double st = Timer.getFPGATimestamp();
			loop.processLoop(st);
			double et = Timer.getFPGATimestamp();
			
			Logger.getInstance().recordOutput("N2", loop.getId().toString());
			Logger.getInstance().recordOutput("D2", et - st);

			if (et - st > 0.01) {
				Logger.getInstance().recordOutput("O2", true);;
				// DriverStation.reportError(String.format("%s.onLoop took too long: %s", loop.getId(), et - st), false);
			}
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		m_writeCnt+=1;
		Logger.getInstance().recordOutput("L3", m_writeCnt);

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.writePeriodicOutputs(st);
			double et = Timer.getFPGATimestamp();

			Logger.getInstance().recordOutput("N3", subsystem.getId().toString());
			Logger.getInstance().recordOutput("D3", et - st);

			if (et - st > 0.01) {
				Logger.getInstance().recordOutput("O3", true);;
				// DriverStation.reportError(String.format("%s.writePeriodicOutputs took too long: %s", subsystem.getId(), et - st), false);
			}
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		var dt = Timer.getFPGATimestamp();
		dt = dt - ost;

		Logger.getInstance().recordOutput("D39", dt);

		if(dt > .02){
			Logger.getInstance().recordOutput("O39", true);;
			DriverStation.reportWarning(String.format("Loop overrun [%s], skipping telemetry...",dt), false);
			return;
		}

		m_telemCnt+=1;
		Logger.getInstance().recordOutput("L39", m_telemCnt);

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.outputTelemetry(timestamp);
			double et = Timer.getFPGATimestamp();

			Logger.getInstance().recordOutput("N4", subsystem.getId().toString());
			Logger.getInstance().recordOutput("D4", et - st);

			if (et - st > 0.01) {
				Logger.getInstance().recordOutput("O4", true);;
				// DriverStation.reportError(String.format("%s.outputTelemetry took too long: %s", subsystem.getId(), et - st), false);
			}
		}));
		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);
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
