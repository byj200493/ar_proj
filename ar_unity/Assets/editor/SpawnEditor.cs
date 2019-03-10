using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
[CustomEditor(typeof(Spawner))]
public class SpawnEditor : Editor {

	public override void OnInspectorGUI()
	{
		DrawDefaultInspector ();
		Spawner mySpawner = (Spawner)target;
		if (GUILayout.Button ("spawn Objects")) 
		{
			mySpawner.buildWalls ();
			mySpawner.buildSofas ();
		}
	}
}
