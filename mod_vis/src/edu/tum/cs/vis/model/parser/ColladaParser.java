package edu.tum.cs.vis.model.parser;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;

import org.xml.sax.SAXException;

import processing.core.PApplet;
import processing.core.PImage;

import com.dddviewr.collada.Collada;
import com.dddviewr.collada.Input;
import com.dddviewr.collada.Source;
import com.dddviewr.collada.effects.Effect;
import com.dddviewr.collada.effects.EffectAttribute;
import com.dddviewr.collada.effects.Lambert;
import com.dddviewr.collada.geometry.Geometry;
import com.dddviewr.collada.geometry.LibraryGeometries;
import com.dddviewr.collada.geometry.Mesh;
import com.dddviewr.collada.geometry.Primitives;
import com.dddviewr.collada.geometry.Triangles;
import com.dddviewr.collada.materials.Material;
import com.dddviewr.collada.visualscene.InstanceMaterial;

import edu.tum.cs.vis.model.util.Color;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * Connection between dae4j library and processing applet.
 * Used to parse COLLADA (.dae) files and draw them on the processing applet.
 * Also .kml files are supported. These are exctracted and the dae file init will be loaded.
 * 
 * @author Stefan Profanter
 *
 */
public class ColladaParser extends ModelParser {

	private String textureBasePath = "";
	private Boolean texturesInitialized = false;
	
    private static Material FindMaterial(String id, Collada collada)
    {
    	for (Material m : collada.getLibraryMaterials().getMaterials())
    	{
    		if (m.getId().compareTo(id)==0)
    			return m;
    	}
    	return null;
    	
    }
    
    private static float[][] SourceToFloat(Source s)
    {
    	float ret[][] = new float[s.getAccessor().getCount()][s.getAccessor().getStride()];
    	for (int i = 0, k = 0; i<s.getAccessor().getCount(); i++)
        {
            for (int j = 0; j < s.getAccessor().getStride(); j++, k++)
            ret[i][j] = s.getFloatArray().get(k);
        }
    	return ret;
    }
    
    //Hide constructor
    private ColladaParser() {
    	
    }
    
    
	@Override
	public void draw(PApplet applet) {

		if (!texturesInitialized)
			setTextureImage(applet);

		applet.noStroke();

		applet.fill(127);

		drawTriangles(applet);
	}

	/**
	 * Load a model from a file
	 * @param filename The filename to the model
	 * @return The loaded Model
	 */
	public static ColladaParser loadModel(String filename,
			String textureBasePath) {
		Collada collada = null;
		

		try {
			collada = Collada.readFile(filename);
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (SAXException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		ColladaParser parser = new ColladaParser();
		parser.textureBasePath = textureBasePath;
		if (!parser.textureBasePath.endsWith("/")
				&& !parser.textureBasePath.endsWith("\\"))
			parser.textureBasePath += "/";
		parser.loadTriangles(collada);

		parser.centerModel();

		return parser;
	}

	private void loadTriangles(Collada collada) {
		triangles.clear();
		LibraryGeometries geo = collada.getLibraryGeometries();
		for (Geometry g : geo.getGeometries()) {
			Mesh m = g.getMesh();
			for (Primitives p : m.getPrimitives()) {
				if (!(p instanceof Triangles)) {
					continue;
				}
				Triangles t = (Triangles) p;

				Material mat = null;
				for (InstanceMaterial material : collada
						.getLibraryVisualScenes().getScene(
								collada.getScene().getInstanceVisualScene()
										.getUrl().substring(1)).getNodes().get(
								0).getInstanceGeometry().getInstanceMaterials()) {
					if (material.getSymbol().compareTo(t.getMaterial()) == 0) {
						mat = FindMaterial(material.getTarget().substring(1),
								collada);
						break;
					}
				}

				Source vertexSource = null;
				for (Input in : m.getVertices().getInputs()) {
					if (in.getSemantic().compareTo("POSITION") == 0) {
						vertexSource = m.getSource(in.getSource().substring(1));
					}
				}

				// commons for the set of triangles
				float[][] vertPoints = SourceToFloat(vertexSource);

				int stride = t.getInputs().size();
				int count = t.getCount();

				int[][] indexes = new int[count * 3][stride];

				for (int i = 0, k = 0; i < count * 3; i++) {
					for (int j = 0; j < stride; j++, k++)
						indexes[i][j] = t.getData()[k];
				}

				int vertexOffset = 0, textureOffset = 0;
				Source textureSource = null;
				for (Input i : t.getInputs()) {
					if (i.getSemantic().compareTo("VERTEX") == 0)
						vertexOffset = i.getOffset();
					else if (i.getSemantic().compareTo("TEXCOORD") == 0) {
						textureOffset = i.getOffset();
						textureSource = m.getSource(i.getSource().substring(1));
					}

				}

				float[][] texturePoints = null; // dummy init
				if (textureSource != null) {
					// Triangle has texture
					texturePoints = SourceToFloat(textureSource);
				}

				Effect effect = collada.findEffect(mat.getInstanceEffect()
						.getUrl().substring(1));
				Lambert lambert = (Lambert) effect.getEffectMaterial();
				EffectAttribute diffuse = lambert.getDiffuse();
				String texturePath = null;
				Color materialColor = new Color(0.5f, 0.5f, 0.5f, 1.0f);
				if (diffuse.getTexture() != null) {
					texturePath = collada.findImage(
							effect.findNewParam(
									effect.findNewParam(
											diffuse.getTexture().getTexture())
											.getSampler2D().getSource())
									.getSurface().getInitFrom()).getInitFrom();
				} else {
					float[] colors = diffuse.getData();
					materialColor = new Color(colors[0], colors[1], colors[2],
							colors[3]);
				}

				// one single triangle from the triangle-set
				for (int i = 0; i < indexes.length; i++) {
					Triangle tri = new Triangle();

					if (texturePath != null) {
						tri.imageFileName = texturePath;
						tri.containsTexture = true;
					} else {
						tri.colour = materialColor;
					}

					for (int v = 0; v < 3; v++)
					{
						// set the vertices for Point
						tri.position[v] = new Point3f(
								vertPoints[indexes[i][vertexOffset]][0],
								vertPoints[indexes[i][vertexOffset]][1],
								vertPoints[indexes[i][vertexOffset]][2]);
	
						// set the texture for Point
						if (tri.containsTexture) {
							tri.texPosition[v] = new Point2f(
									texturePoints[indexes[i][textureOffset]][0],
									texturePoints[indexes[i][textureOffset]][1]);
						}
	
						i++;
					}
					i--;

					// add it to the Collection
					triangles.add(tri);
				}
			}

		}
		texturesInitialized = false;
	}

	/**
	 * per definition is: textureProcessing x = textureSketchup x *
	 * picture.width textureProcessing y = picture.height - textureSketchup y *
	 * picture.height it also creates an PImage to each Triangle (if it contains
	 * any Texture)
	 */
	private void setTextureImage(PApplet applet) {
		// load all Texture-Images only once (memory efficiency)
		HashMap<String, PImage> pictures = new HashMap<String, PImage>();
		for (Triangle tri : triangles) {
			String texfile = textureBasePath + tri.imageFileName;
			if (tri.containsTexture && pictures.get(texfile) == null) {
				PImage tex = applet.loadImage(texfile);
				pictures.put(texfile, tex);
			}
		}

		for (Triangle tri : triangles) {
			if (tri.containsTexture) {
				String texfile = textureBasePath + tri.imageFileName;
				// PImage tex = applet.loadImage(texfile);
				PImage tex = pictures.get(texfile);
				float AprocX = tri.texPosition[0].x * tex.width;
				float AprocY = tex.height - tri.texPosition[0].y * tex.height;
				float BprocX = tri.texPosition[1].x * tex.width;
				float BprocY = tex.height - tri.texPosition[1].y * tex.height;
				float CprocX = tri.texPosition[2].x * tex.width;
				float CprocY = tex.height - tri.texPosition[2].y * tex.height;

				tri.texPosition[0].x = AprocX;
				tri.texPosition[0].y = AprocY;
				tri.texPosition[1].x = BprocX;
				tri.texPosition[1].y = BprocY;
				tri.texPosition[2].x = CprocX;
				tri.texPosition[2].y = CprocY;

				tri.imageReference = tex;
			}
		}
		texturesInitialized = true;
	}
}
