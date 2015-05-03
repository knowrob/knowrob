package org.knowrob.vis.meshes;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.math.BigInteger;
import java.util.LinkedList;
import java.util.List;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
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
	Source uvSource = new Source();
	Object indicesObj = null;
	List<Double> posArray = new LinkedList<Double>();
	List<Double> norArray = new LinkedList<Double>();
	List<Double> uvArray = new LinkedList<Double>();
	List<Integer> indexArray = new LinkedList<Integer>();

	LibraryMaterials mat_lib;
	LibraryImages img_lib;
	LibraryEffects effects_lib;
	Effect phong_material = null;
	
	LibraryVisualScenes visual_scene_lib;
	VisualScene visual_scene;
	Node mesh_node;
	InstanceMaterial instanceMaterial;
	Matrix mesh_transform = null;

	ObjectFactory colladaFactory = new ObjectFactory();

	protected int numSources = 1;

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
		posSource.setId("positions");
		posSource.setName("position");
		// <source id="normals" name="normal">...</source>
		norSource.setId("normals");
		norSource.setName("normal");
		// <source id="uv" name="texco">...</source>
		uvSource.setId("uv");
		uvSource.setName("texco");
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
		JAXBContext jc = JAXBContext.newInstance( "org.knowrob.vis.collada_1_4_1" );
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
	
	public void updateGeometry() {
		updateVertexData();
		updateIndexData();
	}
	
	private void updateVertexData() {
		if(posArray.size()==0) return;
		{
			double _posArray[] = listToArray(posArray);
			posSource.setFloatArray(createFloatArray("positions", _posArray));
			posSource.setTechniqueCommon(createAccessorXYZ("positions", _posArray));
			if(!mesh.getSources().contains(posSource))
				mesh.getSources().add(posSource);
		}
		if(norArray.size()>0) {
			double _norArray[] = listToArray(norArray);
			norSource.setFloatArray(createFloatArray("normals", _norArray));
			norSource.setTechniqueCommon(createAccessorXYZ("normals", _norArray));
			if(!mesh.getSources().contains(norSource))
				mesh.getSources().add(norSource);
		}
		if(uvArray.size()>0) {
			double _uvArray[] = listToArray(uvArray);
			uvSource.setFloatArray(createFloatArray("uv", _uvArray));
			uvSource.setTechniqueCommon(createAccessorUV("uv", _uvArray));
			if(!mesh.getSources().contains(uvSource))
				mesh.getSources().add(uvSource);
			
			InstanceMaterial.BindVertexInput uvVertexInput = new InstanceMaterial.BindVertexInput();
			uvVertexInput.setSemantic("UVMap");
			uvVertexInput.setInputSemantic("TEXCOORD");
			uvVertexInput.setInputSet(createBigInt(0));
			instanceMaterial.getBindVertexInputs().add(uvVertexInput);
		}
	}
	
	private void updateIndexData() {
		// 3 vertices per face and numSources components (position, normal, uv, ...)
		int[] faces = new int[indexArray.size()/(3*numSources)];
		for(int i=0; i<faces.length; ++i) faces[i]=3;
		
		Polylist polyList = new Polylist();
		polyList.setCount(createBigInt(faces.length));
		polyList.setMaterial("MaterialSG");
		// position, normal input
		polyList.getInputs().add(createOffsetType(0, "VERTEX", "#vertices"));
		int offset=1;
		if(norArray.size()>0) {
			polyList.getInputs().add(createOffsetType(offset, "NORMAL", "#normals"));
			offset += 1;
		}
		if(uvArray.size()>0) {
			polyList.getInputs().add(createOffsetType(offset, "TEXCOORD", "#uv", 0));
			offset += 1;
		}
		// vertices per face
		for(int f : faces) polyList.getVcount().add(createBigInt(f));
		// indices
		for(int i : indexArray) polyList.getP().add(createBigInt(i));
		setIndices(polyList);
	}
	
	void addPosition(Vector3d p) {
		posArray.add(p.x);
		posArray.add(p.y);
		posArray.add(p.z);
	}
	
	void addNormal(Vector3d n) {
		norArray.add(n.x);
		norArray.add(n.y);
		norArray.add(n.z);
	}
	
	void addUV(Vector2d uv) {
		uvArray.add(uv.x);
		uvArray.add(uv.y);
	}

	protected void addIndices(int[] indices) {
		for(int i : indices) indexArray.add(i);
	}
	
	private double[] listToArray(List<Double> arr) {
		double out[] = new double[arr.size()];
		int i=0;
		for(Double v : arr)  {
			out[i] = v.doubleValue();
			i += 1;
		}
		return out;
	}
	
	//////////////////////////////
	///////////// Box primitive
	//////////////////////////////
	
	public void addBox(Vector3d pos, Vector3d dx, Vector3d dy, Vector3d dz) {
		// lookup index offset
		int offsetPos = posArray.size()/3;
		int offsetNor = norArray.size()/3;
		// add vertex data
		addBoxVertexData(pos,dx,dy,dz);
		// add index data (2 triangle faces)
		for(int i=0; i<6; ++i) {
			addIndices(new int[] {
					offsetPos,   offsetNor,
					offsetPos+1, offsetNor,
					offsetPos+2, offsetNor,
					offsetPos,   offsetNor,
					offsetPos+2, offsetNor,
					offsetPos+3, offsetNor });
			offsetPos += 4;
			offsetNor += 1;
		}
		numSources = 2;
	}
	
	public void addBox(Vector3d pos, Vector3d dx, Vector3d dy, Vector3d dz, Vector2d uv[]) {
		// lookup index offset
		int offsetPos = posArray.size()/3;
		int offsetNor = norArray.size()/3;
		int offsetUV = uvArray.size()/2;
		// add vertex data
		addBoxVertexData(pos,dx,dy,dz,uv);
		// add index data (2*6 triangle faces)
		for(int i=0; i<6; ++i) {
			addIndices(new int[] {
					offsetPos,   offsetNor, offsetUV,
					offsetPos+1, offsetNor, offsetUV+1,
					offsetPos+2, offsetNor, offsetUV+3,
					offsetPos,   offsetNor, offsetUV+1,
					offsetPos+2, offsetNor, offsetUV+2,
					offsetPos+3, offsetNor, offsetUV+3 });
			offsetPos += 4;
			if(uv.length>4) offsetUV += 4;
			offsetNor += 1;
		}
		numSources = 3;
	}
	
	public void addBoxVertexData(Vector3d pos, Vector3d dx, Vector3d dy, Vector3d dz) {
		Vector3d p0, p1, p2, p3;
		// Top face
		p0 = new Vector3d(pos); p0.sub(dx); p0.sub(dy); p0.add(dz); addPosition(p0);
		p1 = new Vector3d(pos); p1.sub(dx); p1.add(dy); p0.add(dz); addPosition(p1);
		p2 = new Vector3d(pos); p2.add(dx); p2.add(dy); p0.add(dz); addPosition(p2);
		p3 = new Vector3d(pos); p3.add(dx); p3.sub(dy); p0.add(dz); addPosition(p3);
		addNormal(new Vector3d(dz.x, dz.y, dz.z));
		// Front face
		p0 = new Vector3d(pos); p0.sub(dx); p0.sub(dy); p0.sub(dz); addPosition(p0);
		p1 = new Vector3d(pos); p1.sub(dx); p1.sub(dy); p0.add(dz); addPosition(p1);
		p2 = new Vector3d(pos); p2.add(dx); p2.sub(dy); p0.add(dz); addPosition(p2);
		p3 = new Vector3d(pos); p3.add(dx); p3.sub(dy); p0.sub(dz); addPosition(p3);
		addNormal(new Vector3d(-dy.x, -dy.y, -dy.z));
		// Left face
		p0 = new Vector3d(pos); p0.sub(dx); p0.sub(dy); p0.sub(dz); addPosition(p0);
		p1 = new Vector3d(pos); p1.sub(dx); p1.add(dy); p0.sub(dz); addPosition(p1);
		p2 = new Vector3d(pos); p2.sub(dx); p2.add(dy); p0.add(dz); addPosition(p2);
		p3 = new Vector3d(pos); p3.sub(dx); p3.sub(dy); p0.add(dz); addPosition(p3);
		addNormal(new Vector3d(-dx.x, -dx.y, -dx.z));
		// Back face
		p0 = new Vector3d(pos); p0.sub(dx); p0.add(dy); p0.add(dz); addPosition(p0);
		p1 = new Vector3d(pos); p1.sub(dx); p1.add(dy); p0.sub(dz); addPosition(p1);
		p2 = new Vector3d(pos); p2.add(dx); p2.add(dy); p0.sub(dz); addPosition(p2);
		p3 = new Vector3d(pos); p3.add(dx); p3.add(dy); p0.add(dz); addPosition(p3);
		addNormal(new Vector3d(dy.x, dy.y, dy.z));
		// Right face
		p0 = new Vector3d(pos); p0.sub(dx); p0.sub(dy); p0.add(dz); addPosition(p0);
		p1 = new Vector3d(pos); p1.sub(dx); p1.add(dy); p0.add(dz); addPosition(p1);
		p2 = new Vector3d(pos); p2.sub(dx); p2.add(dy); p0.sub(dz); addPosition(p2);
		p3 = new Vector3d(pos); p3.sub(dx); p3.sub(dy); p0.sub(dz); addPosition(p3);
		addNormal(new Vector3d(dx.x, dx.y, dx.z));
		// Bottom face
		p0 = new Vector3d(pos); p0.sub(dx); p0.sub(dy); p0.sub(dz); addPosition(p0);
		p1 = new Vector3d(pos); p1.sub(dx); p1.add(dy); p0.sub(dz); addPosition(p1);
		p2 = new Vector3d(pos); p2.add(dx); p2.add(dy); p0.sub(dz); addPosition(p2);
		p3 = new Vector3d(pos); p3.add(dx); p3.sub(dy); p0.sub(dz); addPosition(p3);
		addNormal(new Vector3d(-dz.x, -dz.y, -dz.z));
	}

	public void addBoxVertexData(Vector3d pos, Vector3d dx, Vector3d dy, Vector3d dz, Vector2d uv[]) {
		addBoxVertexData(pos,dx,dy,dz);
		for(Vector2d v : uv) addUV(v);
	}
	
	//////////////////////////////
	///////////// Quad primitive
	//////////////////////////////
	
	public void addQuad(Vector3d pos, Vector3d dx, Vector3d dy, Vector3d nor) {
		// lookup index offset
		int offsetPos = posArray.size()/3;
		int offsetNor = norArray.size()/3;
		// add vertex data
		addQuadVertexData(pos,dx,dy,nor);
		// add index data (2 triangle faces)
		addIndices(new int[] {
				offsetPos,   offsetNor,
				offsetPos+1, offsetNor,
				offsetPos+2, offsetNor,
				offsetPos,   offsetNor,
				offsetPos+2, offsetNor,
				offsetPos+3, offsetNor
		});
		numSources = 2;
	}

	public void addQuad(Vector3d pos, Vector3d dx, Vector3d dy, Vector3d nor, Vector2d uv[]) {
		// lookup index offset
		int offsetPos = posArray.size()/3;
		int offsetNor = norArray.size()/3;
		int offsetUV = uvArray.size()/2;
		// add vertex data
		addQuadVertexData(pos,dx,dy,nor,uv);
		// add index data (2 triangle faces)
		addIndices(new int[] {
				offsetPos,   offsetNor, offsetUV,
				offsetPos+1, offsetNor, offsetUV+1,
				offsetPos+2, offsetNor, offsetUV+3,
				offsetPos,   offsetNor, offsetUV+1,
				offsetPos+2, offsetNor, offsetUV+2,
				offsetPos+3, offsetNor, offsetUV+3
		});
		numSources = 3;
	}
	
	public void addQuadVertexData(Vector3d pos, Vector3d dx, Vector3d dy, Vector3d nor) {
		Vector3d p0 = new Vector3d(pos); p0.sub(dx); p0.sub(dy); addPosition(p0);
		Vector3d p1 = new Vector3d(pos); p1.sub(dx); p1.add(dy); addPosition(p1);
		Vector3d p2 = new Vector3d(pos); p2.add(dx); p2.add(dy); addPosition(p2);
		Vector3d p3 = new Vector3d(pos); p3.add(dx); p3.sub(dy); addPosition(p3);
		addNormal(nor);
	}

	public void addQuadVertexData(Vector3d pos, Vector3d dx, Vector3d dy, Vector3d nor, Vector2d uv[]) {
		addQuadVertexData(pos,dx,dy,nor);
		for(Vector2d v : uv) addUV(v);
	}

	//////////////////////////////
	///////////// Helper
	//////////////////////////////

	private void setIndices(Object indices) {
		if(indicesObj!=null) {
	        mesh.getLinesAndLinestripsAndPolygons().remove(indicesObj);
		}
		indicesObj = indices;
        mesh.getLinesAndLinestripsAndPolygons().add(indicesObj);
	}

	protected InputLocalOffset createOffsetType(int offset, String sem, String sourceUrl) {
		InputLocalOffset offsetType = new InputLocalOffset();
		offsetType.setOffset(createBigInt(offset));
		offsetType.setSemantic(sem);
		offsetType.setSource(sourceUrl);
		return offsetType;
	}

	protected InputLocalOffset createOffsetType(int offset, String sem, String sourceUrl, int set) {
		InputLocalOffset offsetType = createOffsetType(offset,sem,sourceUrl);
		offsetType.setSet(createBigInt(set));
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
		posAccessor.setStride(createBigInt(2));
		posAccessor.setSource("#"+id+"-array");
		String[] elems = new String[] { "S", "T" };
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

	protected CommonColorOrTextureType createColor(double[] values) {
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
            //   <texture texture="$id-sampler" texcoord="$texcoId"/>
            // </diffuse>
			CommonColorOrTextureType.Texture tex = new CommonColorOrTextureType.Texture();
			tex.setTexcoord(texcoId);
			tex.setTexture(id + "-sampler");
			e.setTexture(tex);
			effectProfile.getTechnique().getPhong().setDiffuse(e);
		}
	}

	protected Node createMeshNode() {
		Node meshNode = new Node();
		meshNode.setId("Mesh");
		meshNode.setName("Mesh");
		
		InstanceGeometry meshGeom = new InstanceGeometry();
		meshGeom.setUrl("#mesh");
		BindMaterial meshBindMatetrial = new BindMaterial();
		BindMaterial.TechniqueCommon meshBindMaterialTechnique = new BindMaterial.TechniqueCommon();
		instanceMaterial = new InstanceMaterial();
		instanceMaterial.setSymbol("MaterialSG");
		instanceMaterial.setTarget("#Material");
		meshBindMaterialTechnique.getInstanceMaterials().add(instanceMaterial);
		meshBindMatetrial.setTechniqueCommon(meshBindMaterialTechnique);
		meshGeom.setBindMaterial(meshBindMatetrial);
		meshNode.getInstanceGeometries().add(meshGeom);
		
		return meshNode;
	}

	protected Material createMaterial() {
		Material material = new Material();
		material.setId("Material");
		material.setName("Material");
		InstanceEffect matEffectRef = new InstanceEffect();
		matEffectRef.setUrl("#Material-fx");
		material.setInstanceEffect(matEffectRef);
		return material;
	}
	
	protected BigInteger createBigInt(int i) {
		return new BigInteger(new Integer(i).toString());
	}

	protected CommonFloatOrParamType createFloat(double d) {
		CommonFloatOrParamType e = new CommonFloatOrParamType();
		CommonFloatOrParamType.Float v = new CommonFloatOrParamType.Float();
		v.setValue(d);
		e.setFloat(v);
		return e;
	}
	
	//////////////////////////////
	///////////// Factory
	//////////////////////////////
	
	/*
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
	*/
	
	//////////////////////////////
	///////////// Testing
	//////////////////////////////

	/*
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
	*/
}
