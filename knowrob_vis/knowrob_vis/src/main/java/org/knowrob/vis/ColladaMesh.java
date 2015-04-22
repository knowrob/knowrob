package org.knowrob.vis;

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

import org.knowrob.vis.collada.AccessorType;
import org.knowrob.vis.collada.AssetType;
import org.knowrob.vis.collada.BindMaterialType;
import org.knowrob.vis.collada.COLLADA;
import org.knowrob.vis.collada.EffectType;
import org.knowrob.vis.collada.FloatArrayType;
import org.knowrob.vis.collada.FxCommonColorOrTextureType;
import org.knowrob.vis.collada.FxCommonColorOrTextureType.Color;
import org.knowrob.vis.collada.FxCommonFloatOrParamType;
import org.knowrob.vis.collada.GeometryType;
import org.knowrob.vis.collada.InputLocalOffsetType;
import org.knowrob.vis.collada.InputLocalType;
import org.knowrob.vis.collada.InstanceEffectType;
import org.knowrob.vis.collada.InstanceGeometryType;
import org.knowrob.vis.collada.InstanceMaterialType;
import org.knowrob.vis.collada.InstanceWithExtraType;
import org.knowrob.vis.collada.LibraryEffectsType;
import org.knowrob.vis.collada.LibraryGeometriesType;
import org.knowrob.vis.collada.LibraryMaterialsType;
import org.knowrob.vis.collada.LibraryVisualScenesType;
import org.knowrob.vis.collada.MaterialType;
import org.knowrob.vis.collada.MatrixType;
import org.knowrob.vis.collada.MeshType;
import org.knowrob.vis.collada.NodeType;
import org.knowrob.vis.collada.PType;
import org.knowrob.vis.collada.ParamType;
import org.knowrob.vis.collada.PolylistType;
import org.knowrob.vis.collada.ProfileCommonType;
import org.knowrob.vis.collada.SourceType;
import org.knowrob.vis.collada.UpAxisEnum;
import org.knowrob.vis.collada.SourceType.TechniqueCommon;
import org.knowrob.vis.collada.VerticesType;
import org.knowrob.vis.collada.VisualSceneType;

/**
 * @author Daniel Beßler <danielb@cs.uni-bremen.de>
 */
public class ColladaMesh {
	COLLADA rootElement = new COLLADA();

	MeshType mesh = new MeshType();
	SourceType posSource = new SourceType();
	SourceType norSource = new SourceType();
	Object indicesObj = null;

	LibraryMaterialsType mat_lib;
	LibraryEffectsType effects_lib;
	EffectType phong_material = null;
	
	LibraryVisualScenesType visual_scene_lib;
	VisualSceneType visual_scene;
	NodeType mesh_node;
	MatrixType mesh_transform = null;

	public ColladaMesh() {
		rootElement.setVersion("1.4.1");
		
		/////////////////////////////
		/////////// Asset
		AssetType asset = new AssetType();
		AssetType.Unit unit = new AssetType.Unit();
		unit.setMeter(1.0);
		unit.setName("meter");
		asset.setUnit(unit);
		asset.setUpAxis(UpAxisEnum.Z_UP);
		rootElement.setAsset(asset);
		
		/////////////////////////////
		/////////// Material
		mat_lib = new LibraryMaterialsType();
		mat_lib.getMaterials().add(createMaterial());
		rootElement.getLibraryAnimationsAndLibraryAnimationClipsAndLibraryCameras().add(mat_lib);

		/////////////////////////////
		/////////// Effects
		effects_lib = new LibraryEffectsType();
		rootElement.getLibraryAnimationsAndLibraryAnimationClipsAndLibraryCameras().add(effects_lib);
		
		/////////////////////////////
		/////////// Geometry
		LibraryGeometriesType geom_lib = new LibraryGeometriesType();
		
		GeometryType geom = new GeometryType();
		geom.setId("mesh");
		geom.setName("mesh");
		
		//<mesh>
		// <source id="positions" name="position">...</source>
		mesh.getSources().add(posSource);
		// <source id="normals" name="normal">...</source>
		mesh.getSources().add(norSource);
		// <vertices id="vertices">...</vertices>
		VerticesType vType = new VerticesType();
		vType.setId("vertices");
		InputLocalType vInput = new InputLocalType();
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
		visual_scene_lib = new LibraryVisualScenesType();
		visual_scene = new VisualSceneType();
		visual_scene.setId("VisualSceneNode");
		visual_scene.setName("untitled");
		mesh_node = createMeshNode();
		visual_scene.getNodes().add(mesh_node);
		visual_scene_lib.getVisualScenes().add(visual_scene);
		rootElement.getLibraryAnimationsAndLibraryAnimationClipsAndLibraryCameras().add(visual_scene_lib);
		
		/////////////////////////////
		/////////// Scene
		COLLADA.Scene scene = new COLLADA.Scene();
		InstanceWithExtraType instanceVisualScene = new InstanceWithExtraType();
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
	
	public ProfileCommonType.Technique.Phong setPhongMaterial(double[] ambient, double[] diffuse, double[] specular) {
		if(phong_material != null) {
			effects_lib.getEffects().remove(phong_material);
		}
		phong_material = new EffectType();
		phong_material.setId("Material-fx");
		
		ProfileCommonType matProfile = new ProfileCommonType();
		ProfileCommonType.Technique matTechnique = new ProfileCommonType.Technique();
		matTechnique.setSid("common");
		ProfileCommonType.Technique.Phong phongShading = new ProfileCommonType.Technique.Phong();
		phongShading.setEmission(createColor(new double[] {0.0, 0.0, 0.0, 1.0}));
		phongShading.setAmbient(createColor(ambient));
		phongShading.setDiffuse(createColor(diffuse));
		phongShading.setSpecular(createColor(specular));
		phongShading.setShininess(createFloat(16.0));
		phongShading.setIndexOfRefraction(createFloat(1.0));
		matTechnique.setPhong(phongShading);
		matProfile.setTechnique(matTechnique);
		phong_material.getProfileCOMMONsAndProfileBRIDGEsAndProfileGLES2s().add(matProfile);
		effects_lib.getEffects().add(phong_material);
		
		return phongShading;
	}
	
	//////////////////////////////
	///////////// Vertex Data
	//////////////////////////////
	
	public void setMeshTransformation(Matrix4d transformationMatrix) {
		if(mesh_transform!=null) {
			mesh_node.getLookatsAndMatrixesAndRotates().remove(mesh_transform);
		}
		mesh_transform = new MatrixType();
		
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
	
	public void setTrianglePolyList(int[] indices) {
		// 3 vertices per face and 2 components (position and normal)
		int[] faces = new int[indices.length/(3*2)];
		for(int i=0; i<faces.length; ++i) faces[i]=3;
		
		PolylistType polyList = new PolylistType();
		polyList.setCount(createBigInt(faces.length));
		polyList.setMaterial("MaterialSG");
		// position, normal input
		polyList.getInputs().add(createOffsetType(0, "VERTEX", "#vertices"));
		polyList.getInputs().add(createOffsetType(1, "NORMAL", "#normals"));
		// vertices per face
		for(int f : faces) polyList.getVcount().add(createBigInt(f));
		// indices
		PType pType = new PType();
		for(int i : indices) pType.getValues().add(createBigInt(i));
		polyList.setP(pType);
		setIndices(polyList);
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

	private void setIndices(PolylistType polyList) {
		if(indicesObj!=null) {
	        mesh.getLinesAndLinestripsAndPolygons().remove(indicesObj);
		}
		indicesObj = polyList;
        mesh.getLinesAndLinestripsAndPolygons().add(indicesObj);
	}

	private InputLocalOffsetType createOffsetType(int offset, String sem, String sourceUrl) {
		InputLocalOffsetType offsetType = new InputLocalOffsetType();
		offsetType.setOffset(createBigInt(offset));
		offsetType.setSemantic(sem);
		offsetType.setSource(sourceUrl);
		return offsetType;
	}

	private TechniqueCommon createAccessorXYZ(String id, double[] values) {
		SourceType.TechniqueCommon posTechnique = new SourceType.TechniqueCommon();
		AccessorType posAccessor = new AccessorType();
		posAccessor.setCount(createBigInt(values.length/3));
		posAccessor.setOffset(createBigInt(0));
		posAccessor.setStride(createBigInt(3));
		posAccessor.setSource("#"+id+"-array");
		String[] elems = new String[] { "X", "Y", "Z" };
		for(String name : elems) {
			ParamType param = new ParamType();
			param.setName(name);
			param.setType("float");
			posAccessor.getParams().add(param);
		}
		posTechnique.setAccessor(posAccessor);
		return posTechnique;
	}

	private FloatArrayType createFloatArray(String id, double[] values) {
		FloatArrayType floatArray = new FloatArrayType();
		floatArray.setId(id+"-array");
		floatArray.setCount(createBigInt(values.length));
		for(double v : values) floatArray.getValues().add(v);
		return floatArray;
	}

	private FxCommonColorOrTextureType createColor(double[] values) {
		FxCommonColorOrTextureType e = new FxCommonColorOrTextureType();
		Color c = new Color();
		for(double v : values) c.getValues().add(v);
		e.setColor(c);
		return e;
	}

	private NodeType createMeshNode() {
		NodeType meshNode = new NodeType();
		meshNode.setId("Mesh");
		meshNode.setName("Mesh");
		
		InstanceGeometryType meshGeom = new InstanceGeometryType();
		meshGeom.setUrl("#mesh");
		BindMaterialType meshBindMatetrial = new BindMaterialType();
		BindMaterialType.TechniqueCommon meshBindMaterialTechnique = new BindMaterialType.TechniqueCommon();
		InstanceMaterialType matType = new InstanceMaterialType();
		matType.setSymbol("MaterialSG");
		matType.setTarget("#Material");
		meshBindMaterialTechnique.getInstanceMaterials().add(matType);
		meshBindMatetrial.setTechniqueCommon(meshBindMaterialTechnique);
		meshGeom.setBindMaterial(meshBindMatetrial);
		meshNode.getInstanceGeometries().add(meshGeom);
		
		return meshNode;
	}

	private MaterialType createMaterial() {
		MaterialType material = new MaterialType();
		material.setId("Material");
		material.setName("Material");
		InstanceEffectType matEffectRef = new InstanceEffectType();
		matEffectRef.setUrl("#Material-fx");
		material.setInstanceEffect(matEffectRef);
		return material;
	}
	
	private BigInteger createBigInt(int i) {
		return new BigInteger(new Integer(i).toString());
	}

	private FxCommonFloatOrParamType createFloat(double d) {
		FxCommonFloatOrParamType e = new FxCommonFloatOrParamType();
		FxCommonFloatOrParamType.Float v = new FxCommonFloatOrParamType.Float();
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
			4, 11 // 72 indices
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
