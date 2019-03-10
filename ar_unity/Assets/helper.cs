using UnityEngine;
using System.Collections;

public class helper
{
    public static GameObject CreateQuad(float width, float height, Material mat)
	{
		GameObject go = new GameObject ("Quad");
		MeshFilter mf = go.AddComponent (typeof(MeshFilter)) as MeshFilter;
		MeshRenderer mr = go.AddComponent (typeof(MeshRenderer)) as MeshRenderer;
		Mesh m = new Mesh ();
		m.vertices = new Vector3[]
		{
			new Vector3(0, 0, 0),
			new Vector3(width, 0, 0),
			new Vector3(width, height, 0),
			new Vector3(0, height, 0),
		};
		m.uv = new Vector2[] 
		{
			new Vector2(0,0),
			new Vector2(0,1),
			new Vector2(1,1),
			new Vector2(1,0),
		};
		m.triangles = new int[] { 0, 1, 2, 0, 2, 3 };//,2,1,0,3,2,0};
		mf.mesh = m;
		mr.material = mat;
		m.RecalculateBounds ();
		m.RecalculateNormals ();
		return go;
	}

	public static GameObject CreateCube(float width, float height, float depth, Material mat)
	{
		GameObject go = new GameObject ("Cube");
		MeshFilter mf = go.AddComponent (typeof(MeshFilter)) as MeshFilter;
		MeshRenderer mr = go.AddComponent (typeof(MeshRenderer)) as MeshRenderer;
		Mesh m = new Mesh ();
		/*m.vertices = new Vector3[]
		{
			new Vector3(0, 0, 0),
			new Vector3(width, 0, 0),
			new Vector3(width, height, 0),
			new Vector3(0, height, 0),

			new Vector3(0, 0, depth),
			new Vector3(width, 0, depth),
			new Vector3(width, height, depth),
			new Vector3(0, height, depth),
		};*/
		m.vertices = new Vector3[]{
			new Vector3 (0, 0, 0),
			new Vector3 (width, 0, 0),
			new Vector3 (width, height, 0),
			new Vector3 (0, height, 0),
			new Vector3 (0, height, depth),
			new Vector3 (width, height, depth),
			new Vector3 (width, 0, depth),
			new Vector3 (0, 0, depth),
		};
		m.uv = new Vector2[] 
		{
			new Vector2(0,0),
			new Vector2(0,1),
			new Vector2(1,1),
			new Vector2(1,0),
		};
		m.triangles = new int[] {
			0, 2, 1, //face front
			0, 3, 2,
			2, 3, 4, //face top
			2, 4, 5,
			1, 2, 5, //face right
			1, 5, 6,
			0, 7, 4, //face left
			0, 4, 3,
			5, 4, 7, //face back
			5, 7, 6,
			0, 6, 7, //face bottom
			0, 1, 6
		};
		/*m.triangles = new int[] { 0, 1, 2, 0, 2, 3, 
								  7, 6, 4, 6, 5, 4,
								  0, 3, 7, 0, 7, 4,
								  1, 5, 6, 1, 6, 2,
								  3, 2, 6, 3, 6, 7,
								  0, 4, 5, 0, 5, 1};*/
		//m.Optimize ();
		mf.mesh = m;
		mr.material = mat;
		m.RecalculateNormals ();
		return go;
	}
}
