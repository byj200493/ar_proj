using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using System.IO;
using UnityEngine.UI;

//librosbagreadposeplugin.so
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class readrosbagdata : MonoBehaviour {
	[DllImport("librosbagreadposeplugin.so", CallingConvention = CallingConvention.Cdecl)]
	private static extern void initplugin(string str);
	[DllImport("librosbagreadposeplugin.so")]
	private static extern int getpose(float[] poses);
	[DllImport("librosbagreadposeplugin.so")]
	private static extern void getNewImage (int[] imgBuf);
	[DllImport("librosbagreadposeplugin.so")]
	private static extern void getWalls (float[] quadBuf, int[] numVertices);
	[DllImport("librosbagreadposeplugin.so")]
	private static extern void getBoxes(float[] boxBuf, int[] numVertices);
	[DllImport("librosbagreadposeplugin.so")]
	private static extern int getwidth();
	[DllImport("librosbagreadposeplugin.so")]
	private static extern int getheight();

	//rgb vars	
	int[] img;
	byte[] myImg;
	int width;
	int height;
	Texture2D tex;
	float[] quadBuf;
	float[] boxBuf;
	float[] poseBuf;
	int[] numVertices;

	public GameObject showobj;
	public Material objectMat;
	public Material wallMat;
	float []pose;
	private Vector3 campos;
	private Vector3 sp,ep,angles;
	private Quaternion camrot;
	Quaternion camrotation;
	Matrix4x4 mymat;
	Matrix4x4 mymatnew;

	Vector3[] quadVertices; 
	Vector3[] boxVertices; 
	Vector3 centerPos;
	Quaternion rot;

	Vector3 firstvalidpos;
	Quaternion firstvalidrot;
	Vector3 firstvalidpospropercoord;
	Quaternion firstvalidrotpropercoord;
	bool gotfirstvalidpose;
	public GameObject cubetoinstan;
	public GameObject groundplane;
	float iniYaw = 0.0f;
	float iniPitch = 0.0f; 
	int nLoops = 0;

	public GameObject prefab_wall;
	public GameObject prefab_furniture;
	Vector3[] camPositions;
	Quaternion[] camRots;

	void writeObjectData()
	{
		int nObjects = 0;
		GameObject[] GameObjects = (FindObjectsOfType<GameObject>() as GameObject[]);
		for (int i = 0; i < GameObjects.Length; ++i) 
		{
			if (GameObjects [i].name == "furniture") 
			{
				++nObjects;
			}
		}
		string path = "Assets/transform/objectData.txt";
		StreamWriter sw = File.CreateText(path);
		sw.WriteLine("object count:{0}", nObjects);
		int count = 0;
		for (int i = 0; i < GameObjects.Length; ++i) 
		{
			if (GameObjects [i].name == "furniture") 
			{
				sw.WriteLine ("object id:{0}", count);
				sw.WriteLine("{0}, {1}, {2}", GameObjects[i].transform.position.x, GameObjects[i].transform.position.y, GameObjects[i].transform.position.z);
				sw.WriteLine("{0}, {1}, {2}", GameObjects[i].transform.eulerAngles.x, GameObjects[i].transform.eulerAngles.y, GameObjects[i].transform.eulerAngles.z);
				sw.WriteLine("{0}, {1}, {2}", GameObjects[i].transform.localScale.x, GameObjects[i].transform.localScale.y, GameObjects[i].transform.localScale.z);
				++count;
			}
		}
		sw.Close ();
	}

	void writePoseData()
	{
		string path = "Assets/ros_data/poseData.txt";
		StreamWriter sw = File.CreateText(path);
		sw.WriteLine ("pose total count:{0}", nLoops);
		for (int i=0; i < nLoops; ++i) 
		{
			sw.WriteLine ("pose count:{0}", i);
			sw.WriteLine ("{0}, {1}, {2}", camPositions[i].x, camPositions[i].y, camPositions[i].z);
			sw.WriteLine ("{0}, {1}, {2}, {3}", camRots[i].x, camRots[i].y, camRots[i].z, camRots[i].w);
		}
		sw.Close ();
	}

	void writeWallData()
	{
		int nWalls = 0;
		GameObject[] GameObjects = (FindObjectsOfType<GameObject>() as GameObject[]);
		for (int i = 0; i < GameObjects.Length; ++i) 
		{
			if (GameObjects [i].name == "wall") 
			{
				++nWalls;
			}
		}
		StreamWriter sw = File.CreateText ("Assets/transform/wallData.txt");
		sw.WriteLine ("nWalls:{0}", nWalls);
		int count = 0;
		for (int i = 0; i < GameObjects.Length; ++i)
		{
			if (GameObjects [i].name == "wall")
			{
				sw.WriteLine ("wall index:{0}", count);
				sw.WriteLine ("{0}, {1}, {2}", GameObjects[i].transform.position.x,GameObjects[i].transform.position.y,GameObjects[i].transform.position.z);
				sw.WriteLine ("{0}, {1}, {2}", GameObjects[i].transform.eulerAngles.x,GameObjects[i].transform.eulerAngles.y,GameObjects[i].transform.eulerAngles.z);
				sw.WriteLine ("{0}, {1}, {2}", GameObjects[i].transform.localScale.x,GameObjects[i].transform.localScale.y,GameObjects[i].transform.localScale.z);
				++count;
			}
		}
		sw.Close ();
	}

	void Start () {
		gotfirstvalidpose=false;
		width = getheight ();
		height = getwidth ();
		img = new int[width*height*3];
		myImg = new byte[width*height*4];
		quadBuf = new float[3000];
		boxBuf = new float[3000];
		numVertices = new int[2];
		numVertices[0] = numVertices[1] = 0;
		camPositions = new Vector3[2000];
		camRots = new Quaternion[2000];
		pose=new float[9];//plane equation coefficients and plane position
		pose[0] = 0;
		pose[1] = 0;
		pose[2] = 0;
		pose[3] = 1;
		pose[4] = 0;
		pose[5] = 0;
		pose[6] = 0;
		pose[7] = 0;
		pose[8] = 0;
		camrotation = new Quaternion(0,0,0,1);
		mymat=new Matrix4x4();
		sp = new Vector3 (0, 0, 0);
		ep = new Vector3 (1, 1, 1);
		angles = new Vector3 (0, 0, 0);
		string path = Directory.GetCurrentDirectory();
		Debug.Log(path);
		initplugin(path);
		tex = new Texture2D(width,height, TextureFormat.RGBA32, false);
		centerPos.x = 0;
		centerPos.y = 0;
		centerPos.z = 0;
		rot.x = 0;
		rot.y = 0;
		rot.z = 0;
		rot.w = 1;
		quadVertices = new Vector3[100];
		boxVertices = new Vector3[1000];
		iniYaw = -80f;
		//iniPitch = 0.0f;
	}

	bool moveCamera()
	{
		if (getpose (pose) == 0)
			return false;
		campos.x= -pose[1];
		campos.y= pose[2];// - 0.5f;
		campos.z= pose[0];
		Vector3 camForward = new Vector3 (-pose [4], pose [5], pose [3]);
		camForward.Normalize ();
		camForward = Quaternion.Euler(0, iniYaw, 0) * camForward;
		//camForward = Quaternion.Euler(iniPitch, 0, 0) * camForward;
		camrot = new Quaternion (1f, 1f, 1f, 0f);
		Vector3 forward = new Vector3 (0.0f, 0.0f, 1.0f);
		camrot.SetFromToRotation (forward, camForward);
		transform.position = campos;
		transform.rotation = camrot;
		return true;
	}
	// Update is called once per frame
	void Update () 
	{
		if(Input.GetKeyDown(KeyCode.H)){
			writePoseData ();
			writeWallData ();
			writeObjectData ();
		}
		if (Input.GetKeyDown (KeyCode.A)) {
			iniPitch += 2.0f;
		}
		if (Input.GetKeyDown (KeyCode.S)) {
			iniPitch -= 2.0f;
		}
		if (moveCamera () == false)
			return;
		DestroyAllCubeGameObjects ();
		getQuadVertices ();
		getBoxVertices ();
		createWallCubes ();
		createObjectCubes ();
		getNewImage (img);
		convertdata ();
		tex.LoadRawTextureData(myImg);
		tex.Apply();
		showobj.GetComponent<Renderer>().material.mainTexture = tex;
		camPositions [nLoops] = transform.position;
		camRots [nLoops] = transform.rotation;
		nLoops++;
	}

	void OnGUI() 
	{
		GUI.contentColor = Color.green;
		GUI.Label (new Rect (20, 200, 50, 30), nLoops.ToString());
		GUI.Label (new Rect (20, 250, 50, 30), iniPitch.ToString());
	}

	void getQuadVertices()
	{
		getWalls (quadBuf, numVertices);
		centerPos.x = 0;
		centerPos.y = 0;
		centerPos.z = 0;
		for (int i=0; i < numVertices[0]; i++) {
			quadVertices [i].x = -quadBuf [3 * i + 1];
			quadVertices [i].y = quadBuf [3 * i + 2];
			quadVertices [i].z = quadBuf [3 * i];
			centerPos += quadVertices [i];
		}
		centerPos = centerPos / numVertices [0];
	}

	void getBoxVertices()
	{
		getBoxes (boxBuf, numVertices);
		for (int i=0; i < numVertices[1]; i++) 
		{
			boxVertices [i].x = -boxBuf [3 * i + 1];
			boxVertices [i].y = boxBuf [3 * i + 2];
			boxVertices [i].z = boxBuf [3 * i];
		}
	}

	public void DestroyAllCubeGameObjects()
	{
		GameObject[] GameObjects = (FindObjectsOfType<GameObject>() as GameObject[]);
		for (int i = 0; i < GameObjects.Length; i++)
		{
			if (GameObjects [i].name == "wall" || GameObjects [i].name == "furniture")
				Destroy(GameObjects[i]);
			if (GameObjects [i].name == "Cube")
				Destroy(GameObjects[i]);
		}
	}

	void createWallCubes()
	{
		int i = 0;
		Vector3[] vertices = new Vector3[4];
		while (i < numVertices[0]) 
		{
			vertices [0] = quadVertices [i];
			vertices [1] = quadVertices [i+1];
			vertices [2] = quadVertices [i+2];
			vertices [3] = quadVertices [i+3];
			i+=4;
			Vector3 w = vertices [1] - vertices[0];
			Vector3 h = vertices [3] - vertices[0];
			//quads[c] = helper.CreateQuad (3, 3, wallMat);
			float quad_width = w.magnitude;
			float quad_height = h.magnitude;// * 2f;
			float depth = 0.2f;
			w = w.normalized;
			h = h.normalized;
			Vector3 forward = Vector3.Cross (w,h).normalized;
			GameObject go=(GameObject) Instantiate(prefab_wall,Vector3.zero,Quaternion.identity);
			go.transform.localScale=new Vector3(quad_width,quad_height,depth);
			go.transform.rotation=Quaternion.LookRotation (forward, h);//go.transform.rotation;
			go.transform.position = vertices [0];//go.transform.position;
			go.name="wall";
			//go.transform.Translate(new Vector3(0f,-2f,0f));
			//go.transform.parent = transform.parent;
		}
	}

	void createObjectCubes()
	{
		Vector3[] vertices = new Vector3[5];
		for(int i=0; i < numVertices[1]/8; ++i)
		{
			for (int j=0; j < 5; ++j) 
			{
				vertices [j] = boxVertices [8 * i + j];
			}
			Vector3 w = vertices [1] - vertices[0];
			Vector3 h = vertices [3] - vertices[0];
			Vector3 d = vertices [4] - vertices[0];
			float object_width = w.magnitude;
			float object_height = h.magnitude;
			float depth = d.magnitude;
			GameObject go = (GameObject) Instantiate(prefab_furniture,Vector3.zero,Quaternion.identity);//helper.CreateCube (box_width, box_height, depth, objectMat);
			go.transform.localScale=new Vector3(object_width,object_height,depth);
			go.transform.rotation = Quaternion.LookRotation (d.normalized, h.normalized);
			go.transform.position = vertices [0];
			go.name = "furniture";
			//go.transform.parent = transform.parent;
		}
	}

	void convertdata()
	{
		//rgb
		int fillcounter=0;
		for(int i=0;i<width*height*4;i++)
		{
			if((i+1)%4==0)
			{
				myImg[i]=(byte)255;
				continue;
			}
			myImg[i]=(byte)img[fillcounter++];//0;
		}
	}

}
