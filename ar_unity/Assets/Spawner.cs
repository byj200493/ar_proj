using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class Spawner : MonoBehaviour {
	public GameObject prefab_wall;//prefab to instantiate walls
	public GameObject prefab_sofa;//prefab to instantiate sofas 
	public GameObject prefab_bin;//prefab to instantiate trash bins
    public GameObject mapparent;//parent gameobject for the whole scene

    public void flipscene()
    {
        mapparent.transform.localScale = new Vector3(1f, -1f, 1f);
    }
    public void fliptonormal()
    {
        mapparent.transform.localScale = new Vector3(1f, 1f, 1f);
    }

	public void buildWalls()
	{
		string path = "Assets/transform/wallData.txt";
		StreamReader sr = new StreamReader (path);
		string line = sr.ReadLine ();
		string[] entries = line.Split (':');
		int nWalls = int.Parse (entries[1]);
		Vector3 wall_position, wall_rotAngles, wall_scale;
		for (int i=0; i < nWalls; ++i) 
		{
			line = sr.ReadLine ();
			line = sr.ReadLine ();
			entries = line.Split (',');
			wall_position.x = float.Parse (entries [0]);
			wall_position.y = float.Parse (entries [1]);
			wall_position.z = float.Parse (entries [2]);
			line = sr.ReadLine ();
			entries = line.Split (',');
			wall_rotAngles.x = float.Parse (entries [0]);
			wall_rotAngles.y = float.Parse (entries [1]);
			wall_rotAngles.z = float.Parse (entries [2]);
			line = sr.ReadLine ();
			entries = line.Split (',');
			wall_scale.x = float.Parse (entries [0]);
			wall_scale.y = float.Parse (entries [1]);
			wall_scale.z = float.Parse (entries [2]);
			GameObject go = Instantiate (prefab_wall, wall_position, Quaternion.identity);
			go.transform.eulerAngles = wall_rotAngles;
			go.transform.localScale = wall_scale;
            go.tag = "maptag";
            //go.transform.parent = mapparent.transform;
		}
		sr.Close ();
	}

	public void buildSofas()
	{
		string path = "Assets/transform/objectData.txt";
		StreamReader sr = new StreamReader (path);
		string line = sr.ReadLine ();
		string[] entries = line.Split (':');
		int nObjects = int.Parse (entries[1]);
		Vector3 position, rotAngles, scale;
		for (int i=0; i < nObjects; ++i) 
		{
			line = sr.ReadLine ();
			line = sr.ReadLine ();
			entries = line.Split (',');
			position.x = float.Parse (entries [0]);
			position.y = float.Parse (entries [1]);
			position.z = float.Parse (entries [2]);
			line = sr.ReadLine ();
			entries = line.Split (',');
			rotAngles.x = float.Parse (entries [0]);
			rotAngles.y = float.Parse (entries [1]);
			rotAngles.z = float.Parse (entries [2]);
			line = sr.ReadLine ();
			entries = line.Split (',');
			scale.x = float.Parse (entries [0])*0.01f;
			scale.y = float.Parse (entries [1])*0.01f;
			scale.z = float.Parse (entries [2])*0.01f;
			GameObject go = Instantiate (prefab_sofa, position, Quaternion.identity);
			go.transform.eulerAngles = rotAngles;
			go.transform.localScale = scale;
            go.tag = "maptag";
            //go.transform.parent = mapparent.transform;

		}
		sr.Close ();
	}
}
