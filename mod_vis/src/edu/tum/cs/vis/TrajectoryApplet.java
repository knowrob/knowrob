package edu.tum.cs.vis;

import java.io.IOException;
import java.sql.ResultSet;

import edu.tum.cs.util.db.MySQLConnection;
import edu.tum.cs.util.math.Vector3f;
import edu.tum.cs.vis.items.FrameDisplay;
import edu.tum.cs.vis.items.Trajectory;

public class TrajectoryApplet extends AnimatedCanvas {
	private static final long serialVersionUID = 1L;
	/**
	 * whether the trajectory should be translated to be centered around the mean coordinate
	 */
	public boolean centerTrajectory = false;

	public TrajectoryApplet() {		
	}
	
	/**
	 * construct a trajectory from x,y,z data in a MySQL database table in Moritz' KIPM database
	 * @param table
	 */
	public TrajectoryApplet(String table) {
		String host = "atradig131";
		String db = "stt-human-import";
		String user = "tenorth";
		String password = "UEY9KbNb";

		// construct 3D trajectory
		Trajectory traj = new Trajectory();
		try {
			MySQLConnection conn = new MySQLConnection(host, user, password, db);
			ResultSet rs = conn.select("select x,y,z from " + table + " where episode_nr=0 and occurrence_nr=1 order by instance_nr");
			while (rs.next()) {
				float x = rs.getFloat(1);
				float y = rs.getFloat(2);
				float z = rs.getFloat(3);
				System.out.printf("%f/%f/%f\n", x, y, z);
				traj.addPoint(x, y, z);
			}
		}
		catch (Exception e) {
			throw new RuntimeException(e.getClass().getCanonicalName() + ": " + e.getMessage());
		}
		init(traj);
	}
	
	public TrajectoryApplet(java.io.File matlabAsciiFile) throws IOException {
		readTrajectory(matlabAsciiFile);
	}
	
	public Trajectory readTrajectory(java.io.File matlabAsciiFile) throws NumberFormatException, IOException {
		return readTrajectory(matlabAsciiFile, 0);
	}
	
	/**
	 * 
	 * @param matlabAsciiFile
	 * @param startLine  0-based line index indicating the first line to consider
	 * @throws NumberFormatException
	 * @throws IOException
	 */
	public Trajectory readTrajectory(java.io.File matlabAsciiFile, int startLine) throws NumberFormatException, IOException {
		Trajectory traj = new Trajectory();
		traj.readAsc(matlabAsciiFile, startLine);
		init(traj);
		return traj;
	}

	public void init(Trajectory traj) {
		if(centerTrajectory)
			traj.center();
		
		//traj.smoothLines();
//		if(false) {
//			traj.mergePoints();
//			traj.mergeLines();
//	
//			traj.findOscillations();
//			traj.mergeRepeats();
//			//traj.cleanUp();
//			//traj.findskippedPoints();
//			
//			
//			traj.getTransitionPoints();		
//		}
			
		// add coordinate system
		float sceneSize = traj.getMaxAbsCoord();
		add(new CoordinateSystem(sceneSize));	
		this.setSceneSize(sceneSize);
		
		// add trajectory
		addAnimated(traj);
		
		// init camera 
		useCamera = false;
		this.eye = new Vector3f(sceneSize, sceneSize, sceneSize/2);
		this.eyeUp = new Vector3f(sceneSize, sceneSize, -sceneSize);
		this.eyeUp = new Vector3f(0,0,1);
		this.eyeUp.normalize();
		
		add(new FrameDisplay(this, 5, 5+13, LEFT, null));
		
		redraw();
	}
	
	public void keyPressed() {
		if(this.keyCode == java.awt.event.KeyEvent.VK_A) {
			for(DrawableAnimated i : this.animatedItems) {
				if(i instanceof Trajectory) {
					Trajectory traj = (Trajectory) i;
					traj.animationMode = traj.animationMode == Trajectory.AnimationMode.BuildUp ? Trajectory.AnimationMode.AllAtOnce : Trajectory.AnimationMode.BuildUp;
					System.out.println("TrajectoryApplet: Switching animation mode");
				}
			}
			redraw();
		}
		else if(this.keyCode == java.awt.event.KeyEvent.VK_D) {
			for(DrawableAnimated i : this.animatedItems) {
				if(i instanceof Trajectory) {
					Trajectory traj = (Trajectory) i;
					traj.pointsAsSpheres ^= true;
					System.out.println("TrajectoryApplet: Switching point drawing style");
				}
			}
			redraw();
		}
		super.keyPressed();
	}
	
	public static void main(String[] args) {
		try {
			//Canvas applet = new TrajectoryApplet("STT_DETAILED_ABSTRACT_EXP_ISOMAP3D_INTERVAL");
			Canvas applet = new TrajectoryApplet("STT_DETAILED_ABSTRACT_EXP_ISOMAP3D_INTERVAL_LOCAL");
			//Canvas applet = new TrajectoryApplet(new File("/usr/wiss/jain/work/code/GP/sttFlorianJointsLatent.asc"));
			applet.runMain();
		}
		catch (Exception e) {
			e.printStackTrace();
		}		
	}
}
