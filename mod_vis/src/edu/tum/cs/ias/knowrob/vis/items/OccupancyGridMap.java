package edu.tum.cs.ias.knowrob.vis.items;

import java.awt.Image;
import java.awt.image.BufferedImage;
import java.awt.image.RescaleOp;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Vector;

import javax.imageio.ImageIO;
import javax.vecmath.Vector3d;

import org.yaml.snakeyaml.Yaml;

import processing.core.PImage;

import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.utils.ResourceRetriever;
import edu.tum.cs.ias.knowrob.vis.Canvas;

public class OccupancyGridMap extends Item {

	private String map = null;
	private String img = null;
	
	private PImage mapImg = null;
	
	private double ymlResolution;
	private Vector3d ymlOrigin;
	private boolean ymlNegate;
	
	public OccupancyGridMap(String identifier)
	{
		super(null,null);
		
		HashMap<String, Vector<String>> nfo = PrologInterface
				.executeQuery(
						"rdf_has("+identifier+",'http://www.roboearth.org/kb/roboearth.owl#linkToMapFile',literal(Map)),rdf_has("+identifier+",'http://www.roboearth.org/kb/roboearth.owl#linkToImageFile',literal(Img))");
		
		if(nfo!=null) {
			if (nfo.get("Map") != null && nfo.get("Map").size() > 0) {
				map = PrologInterface.removeSingleQuotes(nfo.get("Map").get(0));
			}
			if (nfo.get("Img") != null && nfo.get("Img").size() > 0) {
				img = PrologInterface.removeSingleQuotes(nfo.get("Img").get(0));
			}

			//Parse Yaml

			if (!parseYaml(ResourceRetriever.retrieve(map)))
			{
				System.err.println("Couldn't parse YAML file for " + identifier);
				return;
			}

			//Init image
			File imgFile = ResourceRetriever.retrieve(img);

			BufferedImage bimg = null;
			try {
				bimg = ImageIO.read(imgFile);
			} catch (IOException e) {
				System.err.println("Couldn't read file: " + imgFile.getAbsolutePath());
				e.printStackTrace();
			}	

			try
			{
				if (ymlNegate)
				{
					RescaleOp op = new RescaleOp(-1.0f, 255f, null);
					bimg = op.filter(bimg, null);
				}

				// Convert BufferedImage to Image otherwise PImage constructor will fail!!
				Image i = bimg.getScaledInstance(bimg.getWidth(), bimg.getHeight(),0);
				mapImg = new PImage(i);
			} catch (Exception e) {
				e.printStackTrace();
				System.err.println("Couln't initialize image for: " + identifier);
			}
		}
	}
	
	private boolean parseYaml(File file)
	{
		Yaml yaml = new Yaml();
		try {
			Object parsed = yaml.load(new FileReader(file));
			if (parsed.getClass() != LinkedHashMap.class)
			{
				System.err.println("Invalid yaml object: " + parsed.getClass());
				return false;
			} else {
				@SuppressWarnings("unchecked")
				LinkedHashMap<Object, Object> map = (LinkedHashMap<Object, Object>) parsed;
				
				ymlResolution = 0;
				ymlOrigin = null;
				ymlNegate = false;
				
				for (Object k : map.keySet())
				{
					if (k.getClass() == String.class)
					{
						String key = (String)k;
						
						if (key.equals("resolution"))
						{
							ymlResolution = (Double)map.get(key);
						}
						else if (key.equals("origin"))
						{
							@SuppressWarnings("unchecked")
							ArrayList<Double> par = (ArrayList<Double>)map.get(key);
							ymlOrigin = new Vector3d(par.get(0),par.get(1),par.get(2));
						}
						else if (key.equals("negate"))
						{
							ymlNegate = (((Integer)map.get(key))==1);
						}
					}
				}
				return (ymlOrigin != null && ymlResolution != 0);
				
			}
			
		} catch (FileNotFoundException e) {
			System.err.println("File not found for OccupancyGridMap: " + file.getAbsolutePath());
			return false;
		} catch (Exception e) {
			e.printStackTrace();
			return false;
		}
	}
	
	@Override
	protected void drawIt(Canvas c) {
		if (mapImg == null || ymlOrigin == null)
			return;
		
		float scale = (float)ymlResolution;
		c.pushMatrix();
		c.scale(scale);
		c.image(mapImg, (float)ymlOrigin.x*1/scale,(float)ymlOrigin.y*1/scale);
		c.popMatrix();
	}

}
