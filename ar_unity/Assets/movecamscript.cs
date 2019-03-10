using UnityEngine;
using System.Collections;

public class movecamscript : MonoBehaviour {

float mousescrolledx;
Vector3 mousepositionnow;

public GameObject yaw3d;
public GameObject pitch3d;
	// Use this for initialization
	void Start () {
	mousescrolledx=0f;
mousepositionnow=Input.mousePosition;
	}
	
	// Update is called once per frame
	void Update () {
	moverotatecam();
	}
void OnGUI(){
		Event ev=Event.current;
		if (ev.type == EventType.ScrollWheel) {
			mousescrolledx = ev.delta.y;
		}
	}
	void moverotatecam(){
		//panning
		if(mousescrolledx!=0f){
			yaw3d.transform.position += transform.forward * Time.deltaTime * mousescrolledx;
			mousescrolledx = 0f;
		}
		if(Input.GetMouseButton(1)&&mousepositionnow!=Input.mousePosition){
			yaw3d.transform.position+=-transform.right*Time.deltaTime*(Input.mousePosition.x-mousepositionnow.x);
			yaw3d.transform.position+=-transform.up*Time.deltaTime*(Input.mousePosition.y-mousepositionnow.y);
		}

		//panning
		//rotating
		if(Input.GetMouseButton(0)&&mousepositionnow!=Input.mousePosition){
			pitch3d.transform.Rotate (-Vector3.right * Time.deltaTime * (Input.mousePosition.y-mousepositionnow.y));
			yaw3d.transform.Rotate (Vector3.up* Time.deltaTime * (Input.mousePosition.x-mousepositionnow.x));
		}
		
		//rotating
		mousepositionnow=Input.mousePosition;
	}
}
