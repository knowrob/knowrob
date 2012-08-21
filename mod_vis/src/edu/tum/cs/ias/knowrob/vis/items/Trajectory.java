package edu.tum.cs.ias.knowrob.vis.items;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.util.*;

import edu.tum.cs.ias.knowrob.util.datastructures.Hashmap2List;
import edu.tum.cs.ias.knowrob.util.datastructures.Vector3f;
import edu.tum.cs.ias.knowrob.vis.Canvas;
import edu.tum.cs.ias.knowrob.vis.Drawable;
import edu.tum.cs.ias.knowrob.vis.DrawableAnimated;

/**
 * 2D or 3D Trajectory
 * @author jain
 *
 */
public class Trajectory implements Drawable, DrawableAnimated {

	public Vector<Point> points;
	public HashMap<Point,Set<Point>> mergeSet = new HashMap<Point,Set<Point>>();
	public double minDistance = Double.MAX_VALUE;
	public float pointSize = 0.0f, sphereSize = 120.0f;	
	public boolean pointsAsSpheres = true;
	public int pointColor = 0xffcbcbcb, sphereColor = 0x99ffff00; 
	public float minx, miny, minz, maxx, maxy, maxz;
	public float range;
	public double minDis;
	public int lineColor = 0xffffffff;
	public AnimationMode animationMode = AnimationMode.BuildUp;
	public int minStep = 0;
	
	public enum AnimationMode { BuildUp, AllAtOnce };
	
	protected Hashmap2List<Integer, Drawable> animationEffects;
	
	public Trajectory() {
		animationEffects = new Hashmap2List<Integer, Drawable>();
		points = new Vector<Point>();
		resetStats();
	}
	
	protected void resetStats() {
		minx = Float.MAX_VALUE; miny = Float.MAX_VALUE; minz = Float.MAX_VALUE; 
		maxx = Float.MIN_VALUE; maxy = Float.MIN_VALUE; maxz = Float.MIN_VALUE;
		range = 0;
	}
	
	protected void updateStats(float x, float y, float z) {
		minx = Math.min(x, minx);
		miny = Math.min(y, miny);
		minz = Math.min(z, minz);
		maxx = Math.max(x, maxx);
		maxy = Math.max(y, maxy);
		maxz = Math.max(z, maxz);
		float xrange = maxx - minx;
		float yrange = maxy - miny;
		float zrange = maxz - minz;
		range = Math.max(Math.max(xrange, yrange), zrange);
	}
	
	public void addPoint(float x, float y, float z) {
		points.add(new Point(x, y, z, pointColor, pointSize));
		updateStats(x,y,z);
	}
	
	public void setLabels(java.io.File ascFile) throws NumberFormatException, IOException {
		int[] colors = new int[] {0xffff0000, 0xff00ff00, 0xff0000ff, 0xffffff00, 0xffff00ff, 0xff00ffff, 0xffff8800, 0xffff0088, 0xff88ff00, 0xff00ff88, 0xff8800ff, 0xff0088ff};
		BufferedReader r = new BufferedReader(new FileReader(ascFile));
		String line;
		int iLine = 0;
		while((line = r.readLine()) != null) {			
			int l = (int)Math.rint(Double.parseDouble(line));
			this.points.get(iLine).color = colors[l];
			iLine++;
		}
	}
	
	public float getMaxAbsCoord() {
		float x = Math.max(Math.abs(minx), Math.abs(maxx));
		float y = Math.max(Math.abs(miny), Math.abs(maxy));
		float z = Math.max(Math.abs(minz), Math.abs(maxz));
		float xy = Math.max(x, y);
		return Math.max(xy, z);
	}
	
	public void draw(Canvas c, int step) {
		c.pushMatrix();
		
		pointSize = range / 150;
		sphereSize = pointSize * 2;
		//System.out.println(pointSize);
		Point prev = null;
		int s = 0;
		c.sphereDetail(3);
		for(Point p : points) {
			if(s > step && animationMode == AnimationMode.BuildUp)
				break;
			if(s++ < minStep)
				continue;
			if(p.size == 0.0f)
				p.size = pointSize;
			if(pointsAsSpheres)
				p.drawAsSphere(c);
			else
				p.draw(c);
			if(prev != null) { // draw line connecting previous point with current point
				//c.stroke(255,255,255);
				//System.out.printf("%f %f %f -> %f %f %f\n",prev.x, prev.y, prev.z, p.x, p.y, p.z);
				//c.line(prev.v.x, prev.v.y, prev.v.z, p.v.x, p.v.y, p.v.z);
				c.drawLine(prev.v, p.v, lineColor);
			}
			prev = p;
		}
		c.sphereDetail(30);
		
		// stuff for current pos
		if(step < points.size()) {
			Point currentPos = points.get(step);
			
			// draw sphere around it
			new Sphere(currentPos.v.x, currentPos.v.y, currentPos.v.z, sphereSize, sphereColor).draw(c);
			
			// draw animation effects if any
			Vector<Drawable> effects = animationEffects.get(step);
			if(effects != null) {
				System.out.println("drawing effects for step " + step);
				for(Drawable d : effects)
					d.draw(c);
			}
			
			// focus eye on it	
			c.setEyeTarget(currentPos.v);			
		}
		
		c.popMatrix();
	}
	
	public int getNumSteps() {
		return points.size();
	}
	
	public void draw(Canvas c) {
		draw(c, this.getMaxStep());
	}
	
	/**
	 * saves the point sequence to a Matlab-style ASCII file
	 * @param f
	 * @throws FileNotFoundException 
	 */
	public void saveAsc(java.io.File f) throws FileNotFoundException {
		PrintStream s = new PrintStream(f);
		for(Point p : this.points) {
			s.print(p.v.x);
			s.print(' ');
			s.print(p.v.y);
			s.print(' ');
			s.print(p.v.z);
			s.print('\n');
		}
	}
	
	public void readAsc(java.io.File matlabAsciiFile) throws NumberFormatException, IOException {
		readAsc(matlabAsciiFile, 0);
	}
	
	/**
	 * @param matlabAsciiFile
	 * @param startLine  0-based line index indicating the first line to consider
	 * @throws NumberFormatException
	 * @throws IOException
	 */
	public void readAsc(java.io.File matlabAsciiFile, int startLine) throws NumberFormatException, IOException {
		BufferedReader r = new BufferedReader(new FileReader(matlabAsciiFile));
		String line;
		int iLine = 0;
		while((line = r.readLine()) != null) {
			if(iLine++ < startLine)
				continue;
			String[] parts = line.trim().split("\\s+");
			float x = Float.parseFloat(parts[0]);
			float y = Float.parseFloat(parts[1]);
			float z;
			if(parts.length == 3)
				z = Float.parseFloat(parts[2]);
			else 
				z = 0;
			this.addPoint(x, y, z);
		}
	}
	
	public double getMinDistance(){
		// minimum distance between two adjacent points
		double distance = 0;
		int i = 0;
		for (Point p: points){
			if(i >0){
				distance = p.v.distance(points.get(i-1).v);
				if (distance < minDistance)
					minDistance = distance;
			}
			i++;
		}
		return minDistance;
	}
	
	public void merge(Point p1, Point p2){
		Set<Point> s1 = mergeSet.get(p1);
		Set<Point> s2 = mergeSet.get(p2);
		Set<Point> s;
		if(s1 != null && s2 != null){
			s1.addAll(s2);
			for(Point p : s2){
				mergeSet.put(p,s1);
			}
			s = s1;
		}
		else {
			if(s1 != null){
				s = s1;
			}
			if(s2 != null){
				s = s2;
			}
			else{
				s = new HashSet<Point>();
			}
		}
		s.add(p1);
		s.add(p2);
		mergeSet.put(p1,s);
		mergeSet.put(p2,s);
	}

	/**
	 * merges points that are close to each other
	 */
	public void mergePoints() {
		System.out.println("Merging points...");
		float proximity_threshold = range/50;		
		float direction_threshold = 60; // in degrees
		
		int i = 0;
		java.util.PriorityQueue<Double> min_distances = new PriorityQueue<Double>();
		for(Point p : points) {
			if(i >= 3) {		
				// check previous points for proximity
				double min_distance = Double.MAX_VALUE;
				int min_distance_point_idx = -1;
				for(int j = i-3; j >= 0; j--) {
					Point p2 = points.get(j);
					double dist = p.v.distance(p2.v);
					if(dist != 0.0 && dist < min_distance) {
						min_distance = dist;
						min_distance_point_idx = j;
					}
				}
				min_distances.add(min_distance);
				
				// merge if distance to closest previous point is small enough
				if(min_distance < proximity_threshold) {
					Point p2 = points.get(min_distance_point_idx);
					
					// ... and directions are similar
					boolean dirSimilar = false;
					if(min_distance_point_idx == 0)
						dirSimilar = true;
					else {
						Vector3f dir1 = new Vector3f(p.v);
						dir1.sub(points.get(i-1).v);
						Vector3f dir2 = new Vector3f(p2.v);
						dir2.sub(points.get(min_distance_point_idx-1).v);
						double angle = dir1.angle(dir2);
						//System.out.println("angle = " + angle * 180/Math.PI);
						dirSimilar = angle < Math.PI*direction_threshold/180;
					}
					
					if(dirSimilar) { // merge p and p2
						// animation events
						PointPair pp = new PointPair(new Point(p.v, 0xffff00ff, pointSize), new Point(p2.v, 0xffff00ff, pointSize), 0xffff0000);
						animationEffects.put(i, pp);
						animationEffects.put(min_distance_point_idx, pp);
						
						// actual merge
						// any point that is equal to point p or p2 is merged
						merge(p,p2);
						Vector3f newPos = new Vector3f((p.v.x+p2.v.x) / 2, (p.v.y+p2.v.y) / 2, (p.v.z+p2.v.z) / 2);
						for (Point p3 : points){
							if(p3.v.distance(p.v) == 0 || p3.v.distance(p2.v) == 0){
								p3.v = newPos;
								p3.color = 0xffff0000;
								merge(p,p3);
							}
						}
						
					}
				}					
			}
			
			i++;
		}
		
		// print minimum distances
		Double prev_d = null;
		double max_diff = Double.MIN_VALUE;
		double max_diff_value = 0;
		while(!min_distances.isEmpty()) {
			double d = min_distances.remove();
			if(prev_d != null) {
				double diff = d-prev_d;
				if(diff > max_diff) {
					max_diff = diff;
					max_diff_value = d;
				}
			}
			prev_d = d;
			//System.out.println(d);
		}
		System.out.println("max diff: " + max_diff + " @ " + max_diff_value);
		
	}
	/**
	 * starts from already merged points and tries to merge their successors
	 */
	public void mergeLines(){
		System.out.println("Merging lines...");
		double thresh = range/30.0;
		double dirthresh = 60;
		System.out.println(thresh);
		//how many points should be considered
		int fut = 2;
		int past = 2;
		for(int k = 0; k < 20; k++){
			HashSet<Set<Point>> setOfSets = new HashSet<Set<Point>>(mergeSet.values());
			for(Set<Point> s : setOfSets){
				HashSet<Point> curSet = new HashSet<Point>(s);
				for(Point p1 : s){
					curSet.remove(p1);
					// merge Points that have the same distance from starting
					// point and that are already close to each other
					for(Point p2 : curSet){
						for (int l = 1; l <= fut; l++){
							if(points.indexOf(p1)+l < points.size() && points.indexOf(p2)+l < points.size()){
								Point p1Post1 = points.get(points.indexOf(p1)+l);
								Point p2Post1 = points.get(points.indexOf(p2)+l);
								Vector3f dir11 = new Vector3f(p1Post1.v);
								dir11.sub(p1.v);
								Vector3f dir21 = new Vector3f(p2Post1.v);
								dir21.sub(p2.v);
								double angle11 = dir11.angle(dir21);
								if (angle11 < dirthresh*Math.PI/180 && p1Post1.v.distance(p2Post1.v) < thresh){
									// actual merge
									//merge(p1Post1,p2Post1);
									Vector3f newPos = new Vector3f((p1Post1.v.x+p2Post1.v.x) / 2, (p1Post1.v.y+p2Post1.v.y) / 2, (p1Post1.v.z+p2Post1.v.z) / 2);
									for(Point p : points){
										if (p.v.distance(p1Post1.v) == 0 || p.v.distance(p2Post1.v) == 0){
											p.v = newPos;
											p.color = 0xffff0000;
										}
									}
								}
							}
						}
						// same for prior points						
						for (int l = 1; l <= past; l++){
							if(points.indexOf(p1)-l > 0 && points.indexOf(p2)-l > 0){
								Point p1Post1 = points.get(points.indexOf(p1)-l);
								Point p2Post1 = points.get(points.indexOf(p2)-l);
								Vector3f dir11 = new Vector3f(p1Post1.v);
								dir11.sub(p1.v);
								Vector3f dir21 = new Vector3f(p2Post1.v);
								dir21.sub(p2.v);
								double angle11 = dir11.angle(dir21);
								if (angle11 < dirthresh*Math.PI/180 && p1Post1.v.distance(p2Post1.v) < thresh){
								// actual merge
								//merge(p1Post1, p2Post1);
								Vector3f newPos = new Vector3f((p1Post1.v.x+p2Post1.v.x) / 2, (p1Post1.v.y+p2Post1.v.y) / 2, (p1Post1.v.z+p2Post1.v.z) / 2);
								p1Post1.v = newPos;
								p2Post1.v = newPos;
								p1Post1.color = 0xffff0000;
								p2Post1.color = 0xffff0000;
								}
							}
						}
					}
				}
			}
			updateMerge();
		}
	}
	
	/**
	 * updates the mergeSet of the trajectory
	 */
	public void updateMerge(){
		int i = 0;
		for(Point p : points){
			for(int j = i+1; j < points.size(); j++){
				if (p.v.distance(points.get(j).v) == 0)
					merge(p,points.get(j));
			}
			i++;
		}
	}
	
	/**
	 * Detects straight lines in the isomap and smoothes them
	 */
	public void smoothLines(){
		int i = 0;
		for(Point p : points){
			Vector<Point> succ = new Vector<Point>();
			Vector<Point> line = new Vector<Point>();
			for(int j = i+2; j < points.size(); j++){
				Point pEnd = points.get(j);
				Vector3f dir = new Vector3f(pEnd.v);
				dir.sub(p.v);
				double dist = 0.0;
				line.clear();
				for(int k = i+1; k < j; k++){
					Point p2 = points.get(k);
					double a = dir.x*p2.v.x + dir.y*p2.v.y + dir.z*p2.v.z;
					double lambda = (a - (dir.x*p.v.x + dir.y*p.v.y + dir.z*p.v.z))/(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);
					Point l = new Point(p);
					l.v.x += lambda * dir.x;
					l.v.y += lambda * dir.y;
					l.v.z += lambda * dir.z;
					line.addElement(l);
					dist += l.v.distance(p2.v);
				}
				if (dist / (j-i) < 200){
					succ = line;
				}
				else {
					if(j - i > 5){
						System.out.println("Smoothing line");
						//System.out.println("Size of succ: "+ succ.size());
						int n = 0;
						for(int k = i+1; k < j-1; k++){
							points.get(k).v = succ.get(n).v;
							points.get(k).color = 0xffff0000;
							n++;
						}
						i = j;
					}
					else
						i++;
					break;
				}
			}
		}
	}
	
	public void findOscillations(){
		int i = 0;
		int j = 0;
		double thresh = range/50.0;
		//double thresh = 150;
		System.out.println(thresh);
		int num = 0;
		int minnum = 5;
		Vector<Point> vec = new Vector<Point>();
		for(Point p : points){
			if(i>1){
				num = 0;
				j = i;
				vec.clear();
				// appends very close preceding points
				while(j > 0 && p.v.distance(points.get(j).v) < thresh){
					vec.add(points.get(j));
					j--;
					num++;
				}
				if(num > minnum){
					// visualize oscillations and actual merge
					Point medPoint = new Point(0,0,0,pointColor,pointSize); 
					for(Point medP : vec){
						medPoint.v.add(medP.v);
					}
					medPoint.v.scale(1/(float)num);
					float medSize = (float)Math.log(num) * range/150;
					for(Point oscP : vec){
						if (oscP != vec.get(0))
							merge(oscP,vec.get(0));
						oscP.copyPos(medPoint);
						oscP.color = 0xff0000ff;
						oscP.size = medSize;
					}
				}
			}
			i++;
		}
	}
	
	/**
	 * centers the trajectory around the mean position
	 */
	public void center() {
		// determine mean point
		float x = 0, y = 0, z = 0;
		for(Point p : points) {
			x += p.v.x;
			y += p.v.y;
			z += p.v.z;
		}
		x /= points.size();
		y /= points.size();
		z /= points.size();
		Vector3f mean = new Vector3f(x, y, z);
		// subtract mean from all points and update stats
		resetStats();
		for(Point p : points) { 
			p.v.sub(mean);
			updateStats(p.v.x, p.v.y, p.v.z);
		}
	}
	/**
	 * looks for small repeats in the trajectory, i.e. small oscillations
	 * with only 2 points (z-y-z)
	 */
	public void mergeRepeats(){
		int i = 0;
		for(Point p : points){
			int j = 1;
			int num = 0;
			while(i+j < points.size() && num < 3){
				Point p2 = points.get(i+j);
				Point prep2 = points.get(i+j-1);
				if(p2.v.distance(prep2.v) > 0.1){
					num++;
					if (p2.v.distance(p.v) < 0.1){
						merge(p,prep2);
						Vector3f newPos = new Vector3f((p.v.x+prep2.v.x) / 2, (p.v.y+prep2.v.y) / 2, (p.v.z+prep2.v.z) / 2);
						for (Point p3 : points){
							if(p3.v.distance(p.v) == 0 || p3.v.distance(prep2.v) == 0){
								p3.v = newPos;
								p3.color = 0xffff0000;
							}
						}
					}
				}
				j++;
			}
			i++;
		}
	}
	
	public void getTransitionPoints(){
		double threshDist = 0.0f;
		double threshAngle = Math.PI * 00/180;
		HashSet<Set<Point>> setOfSets = new HashSet<Set<Point>>(mergeSet.values());
		for(Set<Point> s : setOfSets){
			HashSet<Point> curSet = new HashSet<Point>(s);
			for(Point p1 : s){
				curSet.remove(p1);
				for(Point p2 : curSet){
					// the first part is for searching forwards
					if(points.indexOf(p1)+1 < points.size() && points.indexOf(p2)+1 < points.size()){
						Point succP1 = points.get(points.indexOf(p1)+1);
						Point succP2 = points.get(points.indexOf(p2)+1);
						// look only for real successor not merged points
						if(p1.v.distance(succP1.v) != 0 && p2.v.distance(succP2.v) != 0){
							Vector3f dir1 = new Vector3f(p1.v);
							dir1.sub(succP1.v);
							Vector3f dir2 = new Vector3f(p2.v);
							dir2.sub(succP2.v);
							double angle = dir1.angle(dir2);
							// check for distance and angle threshold
							if (succP1.v.distance(succP2.v) > threshDist && angle >= threshAngle){
								// check, whether the direction of the trajectory is the same!
								int k = 1;
								Point pre1 = new Point(p1);
								Point pre2 = new Point(p2);
								while(points.indexOf(p1)-k > 0){
									pre1 = points.get(points.indexOf(p1)-k);
									if (pre1.v.distance(p1.v) > 0.1){
										k = 1;
										break;
									}
									else
										k++;
								}
								while(points.indexOf(p2)-k > 0){
									pre2 = points.get(points.indexOf(p2)-k);
									if (pre2.v.distance(p2.v) > 0.1){
										break;
									}
									else
										k++;
								}
								if(pre1.v.distance(pre2.v) > 0.1)
									continue;
								// however: we do not want to find 'fake' transition points in lines
								// where there was just a 'oversight' by the merging algorithm,
								// therefore we look whether the trajectories meet again soon
								int l = 1;
								int fut1 = 0;
								int fut2 = 0;
								Set<Point> succ1 = new HashSet<Point>();
								Set<Point> succ2 = new HashSet<Point>();
								while(points.indexOf(p1)+l < points.size() && points.indexOf(p2)+l < points.size()){
									if(fut1 < 4){
										Point p11 = points.get(points.indexOf(p1)+l);
										Point p12 = points.get(points.indexOf(p1)+l-1);
										succ1.add(p11);
										if(p11.v.distance(p12.v) >= range/200 && !mergeSet.get(p1).contains(p11)){
											fut1++;
										}
									}
									if(fut2 < 4){
										Point p21 = points.get(points.indexOf(p2)+l);
										Point p22 = points.get(points.indexOf(p2)+l-1);
										succ2.add(p21);
										if(p21.v.distance(p22.v) >= range/200 && !mergeSet.get(p1).contains(p21)){
											fut2++;
										}
									}
									l++;
								}
								boolean realTransition = true;
								for(Point pSucc1 : succ1){
									for(Point pSucc2 : succ2){
										if(pSucc1.v.distance(pSucc2.v) <= range/200)
											realTransition = false;
									}
								}
								if(realTransition){
									System.out.println(">>>>>>> Got a transition point");
									p1.size = range/75;
									p1.color = 0xff00ff00;
								}
							}
						}
					}
					// Same for the past!
					if(points.indexOf(p1)-1 > 0 && points.indexOf(p2)-1 > 0){
						Point preP1 = points.get(points.indexOf(p1)-1);
						Point preP2 = points.get(points.indexOf(p2)-1);
						// look only for real predecessor not merged points
						if(p1.v.distance(preP1.v) != 0 && p2.v.distance(preP2.v) != 0){
							Vector3f dir1 = new Vector3f(p1.v);
							dir1.sub(preP1.v);
							Vector3f dir2 = new Vector3f(p2.v);
							dir2.sub(preP2.v);
							double angle = dir1.angle(dir2);
							// check for distance and angle threshold
							if (preP1.v.distance(preP2.v) > threshDist && angle >= threshAngle){
								// check, whether the direction of the trajectory is the same!
								int k = 1;
								Point succ1 = new Point(p1);
								Point succ2 = new Point(p2);
								while(points.indexOf(p1)+k < points.size()){
									succ1 = points.get(points.indexOf(p1)+k);
									if (succ1.v.distance(p1.v) > 0.1){
										k = 1;
										break;
									}
									else
										k++;
								}
								while(points.indexOf(p2)+k < points.size()){
									succ2 = points.get(points.indexOf(p2)+k);
									if (succ2.v.distance(p2.v) > 0.1){
										break;
									}
									else
										k++;
								}
								if(succ1.v.distance(succ2.v) > 0.1)
									continue;
								int l = 1;
								int fut1 = 0;
								int fut2 = 0;
								Set<Point> prec1 = new HashSet<Point>();
								Set<Point> prec2 = new HashSet<Point>();
								while(points.indexOf(p1)-l > 0 && points.indexOf(p2)-l > 0){
									if(fut1 < 4){
										Point p11 = points.get(points.indexOf(p1)-l);
										Point p12 = points.get(points.indexOf(p1)-l+1);
										prec1.add(p11);
										if(p11.v.distance(p12.v) >= range/200 && !mergeSet.get(p1).contains(p11)){
											fut1++;
										}
									}
									if(fut2 < 4){
										Point p21 = points.get(points.indexOf(p2)-l);
										Point p22 = points.get(points.indexOf(p2)-l+1);
										prec2.add(p21);
										if(p21.v.distance(p22.v) >= range/200 && !mergeSet.get(p1).contains(p21)){
											fut2++;
										}
									}
									l++;
								}
								boolean realTransition = true;
								for(Point pSucc1 : prec1){
									for(Point pSucc2 : prec2){
										if(pSucc1.v.distance(pSucc2.v) <= range/200)
											realTransition = false;
									}
								}
								if(realTransition){
									System.out.println(">>>>>>> Got a transition point");
									p1.size = range/75;
									p1.color = 0xff00ff00;
								}
							}
						}
					}
				}
			}
		}
	}

	public int getMaxStep() {
		return getNumSteps()-1;
	}
	
	public class PointPair implements Drawable {

		protected Point p1, p2;
		protected int linecolor;
		
		public PointPair(Point p1, Point p2, int linecolor) {
			this.p1 = p2;
			this.p2 = p2;
			this.linecolor = linecolor;
		}
		
		public void draw(Canvas c) {
			p1.size = pointSize;
			p2.size = pointSize;
			p1.draw(c);
			p2.draw(c);
			c.drawLine(p1.v, p2.v, linecolor);
		}		
	}
}
