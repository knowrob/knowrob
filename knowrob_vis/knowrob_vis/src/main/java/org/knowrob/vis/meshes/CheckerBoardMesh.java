package org.knowrob.vis.meshes;

import java.util.LinkedList;
import java.util.List;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.knowrob.interfaces.mongo.types.Designator;
import org.knowrob.vis.collada_1_4_1.InstanceMaterial;
import org.knowrob.vis.collada_1_4_1.Polylist;
import org.knowrob.vis.collada_1_4_1.ProfileCOMMON;
import org.knowrob.vis.collada_1_4_1.Source;

public class CheckerBoardMesh extends ColladaMesh {
	
	static class CheckerBoardCell {
		final Vector3d position;
		final List<String> ingredients;
		
		public CheckerBoardCell(Vector3d position) {
			this.position = position;
			ingredients = new LinkedList<String>();
		}
		public CheckerBoardCell(Vector3d position, String[] ingredients) {
			this.position = position;
			this.ingredients = new LinkedList<String>();
			for(String x : ingredients) this.ingredients.add(x);
		}
		public boolean isEmpty() {
			return ingredients.isEmpty();
		}
	}
	
	static class CheckerBoard {
		final Vector3d position;
		final Quat4d orientation;
		final Vector3d xVector;
		final Vector3d yVector;
		final Vector3d normal;
		
		
		final Vector2d boardSize;
		final Vector2d cellSize;
		final int width;
		final int height;
		final List<CheckerBoardCell> cells;
		
		public CheckerBoard(
				Vector3d position,
				Quat4d orientation,
				Vector2d boardSize,
				Vector2d cellSize) {
			this.position = position;
			this.orientation = orientation;
			this.boardSize = boardSize;
			this.cellSize = cellSize;
			this.width = (int)(boardSize.x/cellSize.x);
			this.height = (int)(boardSize.y/cellSize.y);
			this.cells = new LinkedList<CheckerBoardCell>();
			
			Matrix4d mat = new Matrix4d(orientation, new Vector3d(0.0,0.0,0.0), 1.0);
			this.normal = new Vector3d(0.0,0.0,-1.0); mat.transform(this.normal);
			this.xVector = new Vector3d(1.0,0.0,0.0); mat.transform(this.xVector);
			this.yVector = new Vector3d(0.0,1.0,0.0); mat.transform(this.yVector);
		}
		
		public void addCell(CheckerBoardCell cell) {
			cells.add(cell);
		}
	}
	
	CheckerBoard board = null;
	Source cellTexcoSource = new Source();

	public CheckerBoardMesh(Vector3d position, Quat4d orientation, Vector2d boardSize, Vector2d cellSize) {
		super();
		board = new CheckerBoard(position, orientation, boardSize, cellSize);
		
		mesh.getSources().add(cellTexcoSource);
	}
	
	public void addCell(Vector3d position, String[] ingredients) {
		if(ingredients==null || ingredients.length==0) return; // ignore empty cells
		CheckerBoardCell cell = new CheckerBoardCell(position);
		for(String x : ingredients) cell.ingredients.add(x);
		board.cells.add(cell);
	}
	
	public void setCellTexco(double[] uv) {
		cellTexcoSource.setId("cell-texcos");
		cellTexcoSource.setName("cell-texco");
		cellTexcoSource.setFloatArray(createFloatArray("cell-texcos", uv));
		cellTexcoSource.setTechniqueCommon(createAccessorUV("cell-texcos", uv));
	}
	
	public void updateCheckerBoardGeometry() {
		// TODO: Highly inefficient....
		//		- Share vertices between cells
		//		- Simplify by spanning faces over more faces
		// TODO: Howto handle ingredients
		//	    - Textures: need additional UV coordinates
		//      - Colors: need per vertex color attribute,
		//             would make sharing of vertices impossible in case of different colors in neighbor cell 
		
		// 4 vertices per cell with each having 3 components (x,y,z)
		double[] positions = new double[board.cells.size()*4*3];
		// use same normal for each vertex
		double[] normals = new double[] {board.normal.x, board.normal.y, board.normal.z};
		// use same normal for each cell
		double[] uv = new double[] {
				0.0, 0.0,
				0.0, 1.0,
				1.0, 1.0,
				1.0, 0.0
		};
		// 6 indices per cell (2 triangles)
		int[] indices = new int[board.cells.size()*6*3];
		
		Vector3d dx = new Vector3d(board.xVector); dx.scale(0.5*board.cellSize.x);
		Vector3d dy = new Vector3d(board.yVector); dy.scale(0.5*board.cellSize.y);
		
		int i=0, j=0, k=0;
		for(CheckerBoardCell cell : board.cells) {
			// quad points
			Vector3d p0 = new Vector3d(cell.position); p0.sub(dx); p0.sub(dy);
			Vector3d p1 = new Vector3d(cell.position); p1.sub(dx); p1.add(dy);
			Vector3d p2 = new Vector3d(cell.position); p2.add(dx); p2.add(dy);
			Vector3d p3 = new Vector3d(cell.position); p3.add(dx); p3.sub(dy);
			positions[i]=p0.x;   positions[i+1]=p0.y;  positions[i+2]=p0.z;
			positions[i+3]=p1.x; positions[i+4]=p1.y;  positions[i+5]=p1.z;
			positions[i+6]=p2.x; positions[i+7]=p2.y;  positions[i+8]=p2.z;
			positions[i+9]=p3.x; positions[i+10]=p3.y; positions[i+11]=p3.z;
			
			// 2 triangle faces (the 0 is for the normal)
			indices[j] = k; j+=1;
			indices[j] = 0; j+=1;
			indices[j] = 0; j+=1;
			indices[j] = k+1; j+=1;
			indices[j] = 0;   j+=1;
			indices[j] = 1;   j+=1;
			indices[j] = k+2; j+=1;
			indices[j] = 0;   j+=1;
			indices[j] = 3;   j+=1;
			
			indices[j] = k; j+=1;
			indices[j] = 0; j+=1;
			indices[j] = 1; j+=1;
			indices[j] = k+2; j+=1;
			indices[j] = 0;   j+=1;
			indices[j] = 2;   j+=1;
			indices[j] = k+3; j+=1;
			indices[j] = 0;   j+=1;
			indices[j] = 3;   j+=1;
			
			i += 12; k += 4;
		}
		
		setPositions(positions);
		setNormals(normals);
		setCellTexco(uv);
		
		Polylist polyList = setTrianglePolyList(indices);
		polyList.getInputs().add(createOffsetType(2, "TEXCOORD", "#cell-texcos", 0));
		
		InstanceMaterial.BindVertexInput uvVertexInput = new InstanceMaterial.BindVertexInput();
		uvVertexInput.setSemantic("UVMap");
		uvVertexInput.setInputSemantic("TEXCOORD");
		uvVertexInput.setInputSet(createBigInt(0));
		instanceMaterial.getBindVertexInputs().add(uvVertexInput);
	}
	
	//////////////////////////////
	///////////// Factory
	//////////////////////////////
	
	public static CheckerBoardMesh createCheckerBoardMesh(Designator objectDesignator) {
		
		/*
		Designator checkerBoard = objectDesignator.getDesignator("PIZZA");
		if(checkerBoard==null) {
			return null;
		}
		Object boardSize = checkerBoard.get("SIZE");
		Object cellSize = checkerBoard.get("FIELD_SIZE");
		Object boardPose = checkerBoard.get("POSE");
		System.err.println("boardSize " + boardSize.getClass());
		System.err.println("cellSize " + cellSize.getClass());
		System.err.println("boardPose " + boardPose.getClass());
		*/
		
		// TODO
		return createCheckerBoardMesh();
	}
	
	public static CheckerBoardMesh createCheckerBoardMesh() {
		Vector3d position = new Vector3d(0.0,0.0,0.0);
		Quat4d orientation = new Quat4d(1.0,0.0,0.0,0.0);
		Vector2d boardSize = new Vector2d(0.50, 0.50);
		Vector2d cellSize = new Vector2d(0.05, 0.05);
		CheckerBoardMesh mesh = new CheckerBoardMesh(position, orientation, boardSize, cellSize);
		
		double startX = boardSize.x*0.5 - cellSize.x*0.5;
		double startY = boardSize.y*0.5 - cellSize.y*0.5;
		
		Vector3d p = new Vector3d(position);
		for(p.x = -startX; p.x <= boardSize.x*0.5; p.x += cellSize.x) {
			for(p.y = -startY; p.y <= boardSize.y*0.5; p.y += cellSize.y) {
				//if(Math.abs(p.x)<0.2 && Math.abs(p.y)<0.2) continue;
				mesh.addCell(new Vector3d(p), new String[] {"Dough"});
			}
		}
		
		mesh.updateCheckerBoardGeometry();
		
		return mesh;
	}
	
	//////////////////////////////
	///////////// Testing
	//////////////////////////////

	public static void main(String[] args) {
		System.out.println("----------------------------------");
		System.out.println("------------ WRITE ----------------");
		System.out.println("----------------------------------");
		try {
			ColladaMesh m = createCheckerBoardMesh();
			
			ProfileCOMMON profile = m.setPhongMaterial(
					new double[] {0.0, 0.0, 0.0, 1.0},
					new double[] {0.137255, 0.403922, 0.870588, 1},
					new double[] {0.5, 0.5, 0.5, 1});
			
			String imgPath = "../kitchen/food-drinks/pizza/pizza_sauce_DIFF.png";
			m.addDiffuseTexturePhong(profile, "tomato-sauce-diff", "UVMap", imgPath);
			
			m.marshal(System.out, true);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
}
