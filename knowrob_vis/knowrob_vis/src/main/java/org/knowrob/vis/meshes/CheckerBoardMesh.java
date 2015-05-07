/*
 * Copyright (c) 2014-15 Daniel Beßler
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/
package org.knowrob.vis.meshes;

import java.util.LinkedList;
import java.util.List;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.knowrob.interfaces.mongo.types.Designator;
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
	
	public void updateCheckerBoardGeometry() {
		Vector3d nor = new Vector3d(board.normal.x, board.normal.y, board.normal.z);
		Vector3d dx = new Vector3d(board.xVector); dx.scale(0.5*board.cellSize.x);
		Vector3d dy = new Vector3d(board.yVector); dy.scale(0.5*board.cellSize.y);
		Vector2d uv[] = new Vector2d[] {
				new Vector2d(0.0, 0.0),
				new Vector2d(0.0, 1.0),
				new Vector2d(1.0, 1.0),
				new Vector2d(1.0, 0.0)
		};
		
		for(CheckerBoardCell cell : board.cells) {
			addQuad(cell.position, dx, dy, nor, uv);
		}
		
		updateGeometry();
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
		Vector2d boardSize = new Vector2d(0.10, 0.10);
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
			
			String imgPath = "pizza_sauce_DIFF.png";
			m.addDiffuseTexturePhong(profile, "tomato-sauce-diff", "UVMap", imgPath);
			
			m.marshal(System.out, true);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
}
