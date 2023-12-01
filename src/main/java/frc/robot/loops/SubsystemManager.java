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

			boolean overrun = false;
			double deltatime = et - st;
			if (deltatime > 0.01) {
				overrun=true;
				// DriverStation.reportError(String.format("%s.readPeriodicInputs took too long: %s", subsystem.getId(), deltatime), false);
			}
			Logger.getInstance().recordOutput(String.format("%s_L1delta", subsystem.getId()), deltatime);
			Logger.getInstance().recordOutput(String.format("%s_L1ovrun", subsystem.getId()), overrun);
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		m_processCnt+=1;
		Logger.getInstance().recordOutput("L2", m_processCnt);

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.processLoop(st);
			double et = Timer.getFPGATimestamp();
			
			boolean overrun = false;
			double deltatime = et - st;
			if (deltatime > 0.01) {
				overrun=true;
				// DriverStation.reportError(String.format("%s.processLoop took too long: %s", subsystem.getId(), deltatime), false);
			}
			Logger.getInstance().recordOutput(String.format("%s_L2delta", subsystem.getId()), deltatime);
			Logger.getInstance().recordOutput(String.format("%s_L2ovrun", subsystem.getId()), overrun);
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		m_writeCnt+=1;
		Logger.getInstance().recordOutput("L3", m_writeCnt);

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.writePeriodicOutputs(st);
			double et = Timer.getFPGATimestamp();

			boolean overrun = false;
			double deltatime = et - st;
			if (deltatime > 0.01) {
				overrun=true;
				// DriverStation.reportError(String.format("%s.writePeridicOutputs took too long: %s", subsystem.getId(), deltatime), false);
			}
			Logger.getInstance().recordOutput(String.format("%s_L3delta", subsystem.getId()), deltatime);
			Logger.getInstance().recordOutput(String.format("%s_L3ovrun", subsystem.getId()), overrun);
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		var dt = Timer.getFPGATimestamp();
		dt = dt - ost;

		boolean loopOverrun = false;
	
		if (dt > 0.01) {
			loopOverrun=true;
			DriverStation.reportWarning(String.format("Loop overrun [%s], skipping telemetry...",dt), false);
		}
		Logger.getInstance().recordOutput("LoopDelta", dt);
		Logger.getInstance().recordOutput("LoopOvrun", loopOverrun);

		m_telemCnt+=1;
		Logger.getInstance().recordOutput("L4", m_telemCnt);

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.outputTelemetry(timestamp);
			double et = Timer.getFPGATimestamp();

			boolean overrun = false;
			double deltatime = et - st;
			if (deltatime > 0.01) {
				overrun=true;
				// DriverStation.reportError(String.format("%s.outputTelemtry took too long: %s", subsystem.getId(), deltatime), false);
			}
			Logger.getInstance().recordOutput(String.format("%s_L4delta", subsystem.getId()), deltatime);
			Logger.getInstance().recordOutput(String.format("%s_L4ovrun", subsystem.getId()), overrun);
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
