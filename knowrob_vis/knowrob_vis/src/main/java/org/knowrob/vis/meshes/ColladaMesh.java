package org.knowrob.vis.meshes;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.math.BigInteger;

import javax.vecmath.Matrix4d;
import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;

import org.knowrob.vis.collada_1_4_1.Accessor;
import org.knowrob.vis.collada_1_4_1.Asset;
import org.knowrob.vis.collada_1_4_1.BindMaterial;
import org.knowrob.vis.collada_1_4_1.COLLADA;
import org.knowrob.vis.collada_1_4_1.CommonColorOrTextureType;
import org.knowrob.vis.collada_1_4_1.CommonFloatOrParamType;
import org.knowrob.vis.collada_1_4_1.CommonNewparamType;
import org.knowrob.vis.collada_1_4_1.Effect;
import org.knowrob.vis.collada_1_4_1.FloatArray;
import org.knowrob.vis.collada_1_4_1.FxSampler2DCommon;
import org.knowrob.vis.collada_1_4_1.FxSurfaceCommon;
import org.knowrob.vis.collada_1_4_1.FxSurfaceInitFromCommon;
import org.knowrob.vis.collada_1_4_1.Geometry;
import org.knowrob.vis.collada_1_4_1.Image;
import org.knowrob.vis.collada_1_4_1.InputLocal;
import org.knowrob.vis.collada_1_4_1.InputLocalOffset;
import org.knowrob.vis.collada_1_4_1.InstanceEffect;
import org.knowrob.vis.collada_1_4_1.InstanceGeometry;
import org.knowrob.vis.collada_1_4_1.InstanceMaterial;
import org.knowrob.vis.collada_1_4_1.InstanceWithExtra;
import org.knowrob.vis.collada_1_4_1.LibraryEffects;
import org.knowrob.vis.collada_1_4_1.LibraryGeometries;
import org.knowrob.vis.collada_1_4_1.LibraryImages;
import org.knowrob.vis.collada_1_4_1.LibraryMaterials;
import org.knowrob.vis.collada_1_4_1.LibraryVisualScenes;
import org.knowrob.vis.collada_1_4_1.Material;
import org.knowrob.vis.collada_1_4_1.Matrix;
import org.knowrob.vis.collada_1_4_1.Mesh;
import org.knowrob.vis.collada_1_4_1.Node;
import org.knowrob.vis.collada_1_4_1.ObjectFactory;
import org.knowrob.vis.collada_1_4_1.Param;
import org.knowrob.vis.collada_1_4_1.Polylist;
import org.knowrob.vis.collada_1_4_1.ProfileCOMMON;
import org.knowrob.vis.collada_1_4_1.Source;
import org.knowrob.vis.collada_1_4_1.Vertices;
import org.knowrob.vis.collada_1_4_1.UpAxisType;
import org.knowrob.vis.collada_1_4_1.VisualScene;

/**
 * @author Daniel Beßler <danielb@cs.uni-bremen.de>
 */
public class ColladaMesh {
	COLLADA rootElement = new COLLADA();

	Mesh mesh = new Mesh();
	Source posSource = new Source();
	Source norSource = new Source();
	Object indicesObj = null;

	LibraryMaterials mat_lib;
	LibraryImages img_lib;
	LibraryEffects effects_lib;
	Effect phong_material = null;
	
	LibraryVisualScenes visual_scene_lib;
	VisualScene visual_scene;
	Node mesh_node;
	Matrix mesh_transform = null;

	ObjectFactory colladaFactory = new ObjectFactory();
	
	public ColladaMesh() {
		rootElement.setVersion("1.4.1");
		
		/////////////////////////////
		/////////// Asset
		Asset asset = new Asset();
		Asset.Unit unit = new Asset.Unit();
		unit.setMeter(1.0);
		unit.setName("meter");
		asset.setUnit(unit);
		asset.setUpAxis(UpAxisType.Z_UP);
		rootElement.setAsset(asset);
		
		/////////////////////////////
		/////////// Material
		mat_lib = new LibraryMaterials();
		mat_lib.getMaterials().add(createMaterial());
		rootElement.getLibraryAnimationsAndLibraryAnimationClipsAndLibraryCameras().add(mat_lib);
		
		/////////////////////////////
		/////////// Images
		img_lib = new LibraryImages();
		rootElement.getLibraryAnimationsAndLibraryAnimationClipsAndLibraryCameras().add(img_lib);

		/////////////////////////////
		/////////// Effects
		effects_lib = new LibraryEffects();
		rootElement.getLibraryAnimationsAndLibraryAnimationClipsAndLibraryCameras().add(effects_lib);
		
		/////////////////////////////
		/////////// Geometry
		LibraryGeometries geom_lib = new LibraryGeometries();
		
		Geometry geom = new Geometry();
		geom.setId("mesh");
		geom.setName("mesh");
		
		//<mesh>
		// <source id="positions" name="position">...</source>
		mesh.getSources().add(posSource);
		// <source id="normals" name="normal">...</source>
		mesh.getSources().add(norSource);
		// <vertices id="vertices">...</vertices>
		Vertices vType = new Vertices();
		vType.setId("vertices");
		InputLocal vInput = new InputLocal();
		vInput.setSource("#positions");
		vInput.setSemantic("POSITION");
		vType.getInputs().add(vInput);
		mesh.setVertices(vType);
		geom.setMesh(mesh);
		//</mesh>
		
		geom_lib.getGeometries().add(geom);
		rootElement.getLibraryAnimationsAndLibraryAnimationClipsAndLibraryCameras().add(geom_lib);
		
		/////////////////////////////
		/////////// Visual Scene
		visual_scene_lib = new LibraryVisualScenes();
		visual_scene = new VisualScene();
		visual_scene.setId("VisualSceneNode");
		visual_scene.setName("untitled");
		mesh_node = createMeshNode();
		visual_scene.getNodes().add(mesh_node);
		visual_scene_lib.getVisualScenes().add(visual_scene);
		rootElement.getLibraryAnimationsAndLibraryAnimationClipsAndLibraryCameras().add(visual_scene_lib);
		
		/////////////////////////////
		/////////// Scene
		COLLADA.Scene scene = new COLLADA.Scene();
		InstanceWithExtra instanceVisualScene = new InstanceWithExtra();
		instanceVisualScene.setUrl("#VisualSceneNode");
		scene.setInstanceVisualScene(instanceVisualScene);
		rootElement.setScene(scene);
	}
	
	//////////////////////////////
	///////////// Marshaling
	//////////////////////////////

	public void marshal(OutputStream out, boolean pretty) throws JAXBException, IOException {
		JAXBContext jc = JAXBContext.newInstance( "org.knowrob.vis.collada" );
		Marshaller m = jc.createMarshaller();
		if(pretty) {
			m.setProperty(Marshaller.JAXB_FRAGMENT, Boolean.TRUE);
			m.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
		}
		
		// HACK: OpenJDK inserts strange namespace and JS COLLADA parser does not like
		// it (with SUN JDK this does not happen). The hack is not nice but it works for now...
		ByteArrayOutputStream proxyStream = new ByteArrayOutputStream();
		m.marshal(rootElement, proxyStream);
		String colladaString = proxyStream.toString();
		String fixedCollada = colladaString.replaceAll("ns3:", "");
		out.write(fixedCollada.getBytes());
	}
	
	public String marshal(String meshName) throws JAXBException, IOException {
		// The user name of the docker user
		String userName = System.getenv("VIRTUAL_HOST");
		if(userName==null) {
			// Unix user name (is always 'ros' for in docker container)
			userName = System.getenv("USER");
		}
		if(userName==null) {
			userName = "ros";
		}
		// Webserver resolves meshes by relative path
		String meshFileRel = "/"+userName+"/"+meshName+".dae";
		
		File meshUserDir = new File("/home/ros/mesh_data/"+userName);
		if(!meshUserDir.isDirectory()) {
			meshUserDir.mkdirs();
		}
		
		// TODO(daniel): Think about this and about mesh handling in general for the none docker case!
		// TODO(daniel): Wipe user subdir on logout!
		File meshFileAbs = new File(meshUserDir, meshName+".dae");
		marshal(new FileOutputStream(meshFileAbs), true);
		
		return "package:/"+meshFileRel;
	}
	
	//////////////////////////////
	///////////// Material
	//////////////////////////////
	
	public ProfileCOMMON setPhongMaterial(double[] ambient, double[] diffuse, double[] specular) {
		if(phong_material != null) {
			effects_lib.getEffects().remove(phong_material);
		}
		phong_material = new Effect();
		phong_material.setId("Material-fx");
		
		ProfileCOMMON matProfile = new ProfileCOMMON();
		ProfileCOMMON.Technique matTechnique = new ProfileCOMMON.Technique();
		matTechnique.setSid("common");
		ProfileCOMMON.Technique.Phong phongShading = new ProfileCOMMON.Technique.Phong();
		phongShading.setEmission(createColor(new double[] {0.0, 0.0, 0.0, 1.0}));
		phongShading.setAmbient(createColor(ambient));
		phongShading.setDiffuse(createColor(diffuse));
		phongShading.setSpecular(createColor(specular));
		phongShading.setShininess(createFloat(16.0));
		phongShading.setIndexOfRefraction(createFloat(1.0));
		matTechnique.setPhong(phongShading);
		matProfile.setTechnique(matTechnique);
		
		phong_material.getFxProfileAbstracts().add(
				colladaFactory .createProfileCOMMON(matProfile));
		
		effects_lib.getEffects().add(phong_material);
		
		return matProfile;
	}
	
	//////////////////////////////
	///////////// Vertex Data
	//////////////////////////////
	
	public void setMeshTransformation(Matrix4d transformationMatrix) {
		if(mesh_transform!=null) {
			mesh_node.getLookatsAndMatrixesAndRotates().remove(mesh_transform);
		}
		mesh_transform = new Matrix();
		
		mesh_transform.getValues().add(transformationMatrix.m00);
		mesh_transform.getValues().add(transformationMatrix.m01);
		mesh_transform.getValues().add(transformationMatrix.m02);
		mesh_transform.getValues().add(transformationMatrix.m03);
		
		mesh_transform.getValues().add(transformationMatrix.m10);
		mesh_transform.getValues().add(transformationMatrix.m11);
		mesh_transform.getValues().add(transformationMatrix.m12);
		mesh_transform.getValues().add(transformationMatrix.m13);
		
		mesh_transform.getValues().add(transformationMatrix.m20);
		mesh_transform.getValues().add(transformationMatrix.m21);
		mesh_transform.getValues().add(transformationMatrix.m22);
		mesh_transform.getValues().add(transformationMatrix.m23);
		
		mesh_transform.getValues().add(transformationMatrix.m30);
		mesh_transform.getValues().add(transformationMatrix.m31);
		mesh_transform.getValues().add(transformationMatrix.m32);
		mesh_transform.getValues().add(transformationMatrix.m33);
		
		mesh_node.getLookatsAndMatrixesAndRotates().add(mesh_transform);
	}
	
	public void setPositions(double[] positions) {
		posSource.setId("positions");
		posSource.setName("position");
		posSource.setFloatArray(createFloatArray("positions", positions));
		posSource.setTechniqueCommon(createAccessorXYZ("positions", positions));
	}
	
	public void setNormals(double[] normals) {
		norSource.setId("normals");
		norSource.setName("normal");
		norSource.setFloatArray(createFloatArray("normals", normals));
		norSource.setTechniqueCommon(createAccessorXYZ("normals", normals));
	}
	
	//////////////////////////////
	///////////// Vertex Data Access
	//////////////////////////////
	
	public Polylist setTrianglePolyList(int[] indices) {
		// 3 vertices per face and 2 components (position and normal)
		int[] faces = new int[indices.length/(3*2)];
		for(int i=0; i<faces.length; ++i) faces[i]=3;
		
		Polylist polyList = new Polylist();
		polyList.setCount(createBigInt(faces.length));
		polyList.setMaterial("MaterialSG");
		// position, normal input
		polyList.getInputs().add(createOffsetType(0, "VERTEX", "#vertices"));
		polyList.getInputs().add(createOffsetType(1, "NORMAL", "#normals"));
		// vertices per face
		for(int f : faces) polyList.getVcount().add(createBigInt(f));
		// indices
		for(int i : indices) polyList.getP().add(createBigInt(i));
		setIndices(polyList);
		return polyList;
	}
	
    // TODO: support other primitives.
    //@XmlElement(name = "lines", type = LinesType.class),
    //@XmlElement(name = "tristrips", type = TristripsType.class),
    //@XmlElement(name = "linestrips", type = LinestripsType.class),
    //@XmlElement(name = "polygons", type = PolygonsType.class),
    //@XmlElement(name = "triangles", type = TrianglesType.class),
    //@XmlElement(name = "trifans", type = TrifansType.class)

	//////////////////////////////
	///////////// Helper
	//////////////////////////////

	private void setIndices(Polylist polyList) {
		if(indicesObj!=null) {
	        mesh.getLinesAndLinestripsAndPolygons().remove(indicesObj);
		}
		indicesObj = polyList;
        mesh.getLinesAndLinestripsAndPolygons().add(indicesObj);
	}

	protected InputLocalOffset createOffsetType(int offset, String sem, String sourceUrl) {
		InputLocalOffset offsetType = new InputLocalOffset();
		offsetType.setOffset(createBigInt(offset));
		offsetType.setSemantic(sem);
		offsetType.setSource(sourceUrl);
		return offsetType;
	}

	private Source.TechniqueCommon createAccessorXYZ(String id, double[] values) {
		Source.TechniqueCommon posTechnique = new Source.TechniqueCommon();
		Accessor posAccessor = new Accessor();
		posAccessor.setCount(createBigInt(values.length/3));
		posAccessor.setOffset(createBigInt(0));
		posAccessor.setStride(createBigInt(3));
		posAccessor.setSource("#"+id+"-array");
		String[] elems = new String[] { "X", "Y", "Z" };
		for(String name : elems) {
			Param param = new Param();
			param.setName(name);
			param.setType("float");
			posAccessor.getParams().add(param);
		}
		posTechnique.setAccessor(posAccessor);
		return posTechnique;
	}

	protected Source.TechniqueCommon createAccessorUV(String id, double[] values) {
		Source.TechniqueCommon posTechnique = new Source.TechniqueCommon();
		Accessor posAccessor = new Accessor();
		posAccessor.setCount(createBigInt(values.length/2));
		posAccessor.setOffset(createBigInt(0));
		posAccessor.setStride(createBigInt(3));
		posAccessor.setSource("#"+id+"-array");
		String[] elems = new String[] { "U", "V" };
		for(String name : elems) {
			Param param = new Param();
			param.setName(name);
			param.setType("float");
			posAccessor.getParams().add(param);
		}
		posTechnique.setAccessor(posAccessor);
		return posTechnique;
	}

	protected FloatArray createFloatArray(String id, double[] values) {
		FloatArray floatArray = new FloatArray();
		floatArray.setId(id+"-array");
		floatArray.setCount(createBigInt(values.length));
		for(double v : values) floatArray.getValues().add(v);
		return floatArray;
	}

	private CommonColorOrTextureType createColor(double[] values) {
		CommonColorOrTextureType e = new CommonColorOrTextureType();
		CommonColorOrTextureType.Color c = new CommonColorOrTextureType.Color();
		for(double v : values) c.getValues().add(v);
		e.setColor(c);
		return e;
	}

	public void addDiffuseTexturePhong(
			ProfileCOMMON effectProfile,
			String id, String texcoId, String path) {
		////////////////////////////
		// Add texture to image library
		////////////////////////////
		Image imgType = new Image(); {
			// <library_images>    
			//  <image id="$id">
			//    <init_from>$path</init_from>
			//  </image>
			// </library_images>
			imgType.setId(id);
			imgType.setInitFrom(path);
			img_lib.getImages().add(imgType);
		}

		////////////////////////////
		// Add texture to effects library
		////////////////////////////
		CommonNewparamType texSurfaceParam = new CommonNewparamType(); {
			// <newparam sid="$id-surface">  
			//  <surface type="2D">
			//    <init_from>$id</init_from>
			//  </surface>
			// </newparam>
			texSurfaceParam.setSid(id+"-surface");
			FxSurfaceCommon surface = new FxSurfaceCommon();
			surface.setType("2D");
			FxSurfaceInitFromCommon initFrom = new FxSurfaceInitFromCommon();
			initFrom.setValue(imgType);
			surface.getInitFroms().add(initFrom);
			texSurfaceParam.setSurface(surface);
			effectProfile.getImagesAndNewparams().add(texSurfaceParam);
		}
		
		CommonNewparamType texSamplerParam = new CommonNewparamType(); {
			// <newparam sid="$id-sampler">  
			//  <sampler2D>
			//    <source>$id-surface</source>
			//  <sampler2D>
			// </newparam>
			texSamplerParam.setSid(id+"-sampler");
			FxSampler2DCommon texSampler = new FxSampler2DCommon();
			texSampler.setSource(id + "-surface");
			texSamplerParam.setSampler2D(texSampler);
			effectProfile.getImagesAndNewparams().add(texSamplerParam);
		}
		
		CommonColorOrTextureType e = new CommonColorOrTextureType(); {
            // <diffuse>
            //   <texture texture="$id-sampler" texcoord="$id-texco"/>
            // </diffuse>
			CommonColorOrTextureType.Texture tex = new CommonColorOrTextureType.Texture();
			tex.setTexcoord(texcoId);
			tex.setTexture(id + "-sampler");
			e.setTexture(tex);
			effectProfile.getTechnique().getPhong().setDiffuse(e);
		}
	}

	private Node createMeshNode() {
		Node meshNode = new Node();
		meshNode.setId("Mesh");
		meshNode.setName("Mesh");
		
		InstanceGeometry meshGeom = new InstanceGeometry();
		meshGeom.setUrl("#mesh");
		BindMaterial meshBindMatetrial = new BindMaterial();
		BindMaterial.TechniqueCommon meshBindMaterialTechnique = new BindMaterial.TechniqueCommon();
		InstanceMaterial matType = new InstanceMaterial();
		matType.setSymbol("MaterialSG");
		matType.setTarget("#Material");
		meshBindMaterialTechnique.getInstanceMaterials().add(matType);
		meshBindMatetrial.setTechniqueCommon(meshBindMaterialTechnique);
		meshGeom.setBindMaterial(meshBindMatetrial);
		meshNode.getInstanceGeometries().add(meshGeom);
		
		return meshNode;
	}

	private Material createMaterial() {
		Material material = new Material();
		material.setId("Material");
		material.setName("Material");
		InstanceEffect matEffectRef = new InstanceEffect();
		matEffectRef.setUrl("#Material-fx");
		material.setInstanceEffect(matEffectRef);
		return material;
	}
	
	private BigInteger createBigInt(int i) {
		return new BigInteger(new Integer(i).toString());
	}

	private CommonFloatOrParamType createFloat(double d) {
		CommonFloatOrParamType e = new CommonFloatOrParamType();
		CommonFloatOrParamType.Float v = new CommonFloatOrParamType.Float();
		v.setValue(d);
		e.setFloat(v);
		return e;
	}
	
	//////////////////////////////
	///////////// Factory
	//////////////////////////////
	
	public static ColladaMesh createCube() {
		ColladaMesh m = new ColladaMesh();
		m.setPositions(new double[] {
			-0.5,  0.5,  0.5,
			 0.5,  0.5,  0.5,
			-0.5, -0.5,  0.5,
			 0.5, -0.5,  0.5,
			-0.5,  0.5, -0.5,
			 0.5,  0.5, -0.5,
			-0.5, -0.5, -0.5,
			 0.5, -0.5, -0.5 // 8 vertices
		});
		m.setNormals(new double[] {
			 0,  0,  1,
			 0,  1,  0,
			 0, -1,  0,
			-1,  0,  0,
			 1,  0,  0,
			 0,  0, -1,
			 0,  0,  1,
			 0,  1,  0,
			 0, -1,  0,
			-1,  0,  0,
			 1,  0,  0,
			 0,  0, -1 // 12 normals
		});
		m.setTrianglePolyList(new int[] {
			2, 0, 3, 0, 1,  0, 1,  1, 5,  1, 4,  1, 7,  2,
			3, 2, 2, 2, 4,  3, 6,  3, 2,  3, 7,  4, 5,  4,
			1, 4, 7, 5, 6,  5, 4,  5, 0,  6, 2,  6, 1,  6,
			0, 7, 1, 7, 4,  7, 6,  8, 7,  8, 2,  8, 0,  9,
			4, 9, 2, 9, 3, 10, 7, 10, 1, 10, 5, 11, 7, 11,
			4, 11 // 72 indices (6 per face for normals and positions)
		});
		return m;
	}
	
	//////////////////////////////
	///////////// Testing
	//////////////////////////////

	public static void main(String[] args) {
		System.out.println("----------------------------------");
		System.out.println("------------ WRITE ----------------");
		System.out.println("----------------------------------");
		try {
			ColladaMesh m = createCube();
			
			m.setPhongMaterial(
					new double[] {0.0, 0.0, 0.0, 1.0},
					new double[] {0.137255, 0.403922, 0.870588, 1},
					new double[] {0.5, 0.5, 0.5, 1});
			
			m.marshal(System.out, true);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
}
