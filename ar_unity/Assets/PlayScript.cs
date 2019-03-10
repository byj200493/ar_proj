using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class PlayScript : MonoBehaviour {

    public GameObject mapparent;
    public float playtime;
    float timer;
	Texture2D tex;

	Vector3[] cam_positions;
	Quaternion[] cam_quats;
	int nPoses = 0;
	int nLoops = 0;

	void loadPoses()
	{
		string path = "Assets/ros_data/poseData.txt";
		StreamReader theReader = new StreamReader (path);
		string line = theReader.ReadLine ();
		string[] str = line.Split (':');
		nPoses = int.Parse (str [1]);
		cam_positions = new Vector3[nPoses];
		cam_quats = new Quaternion[nPoses];
		for (int i=0; i < nPoses; ++i) 
		{
			line = theReader.ReadLine();
			line = theReader.ReadLine();
			string[] entries = line.Split(',');
			cam_positions [i].x = float.Parse (entries [0]);
			cam_positions [i].y = float.Parse (entries [1]);
			cam_positions [i].z = float.Parse (entries [2]);
			line = theReader.ReadLine();
			entries = line.Split(',');
			cam_quats[i].x = float.Parse (entries [0]);
			cam_quats[i].y = float.Parse (entries [1]);
			cam_quats[i].z = float.Parse (entries [2]);
			cam_quats[i].w = float.Parse (entries [3]);
		}
	}
	// Use this for initialization
	void Start ()
	{
        loadPoses();
		/*int childcount = mapparent.transform.childCount;
        for (int i = 0; i < childcount;i++ )
        {
            mapparent.transform.GetChild(0).parent = null;
        }*/
        timer = 0f;
	}
	
	// Update is called once per frame
	void Update () 
	{
        if (timer <= 0f)
        {
            timer = playtime;
            //transform.parent = null;
            //mapparent.transform.localScale = new Vector3(1f, 1f, 1f);
			if (nLoops > nPoses - 1)
                nLoops = 0;
            transform.position = cam_positions[nLoops];
            transform.rotation = cam_quats[nLoops];
            nLoops++;

            //transform.parent = mapparent.transform;
            //mapparent.transform.localScale = new Vector3(1f, -1f, 1f);
        }
        else {
            timer -= Time.deltaTime;
        }
		//showobj.GetComponent<Renderer> ().material.mainTexture = Resources.Load("pngs/"+nLoops.ToString()) as Texture;;
	}
}
