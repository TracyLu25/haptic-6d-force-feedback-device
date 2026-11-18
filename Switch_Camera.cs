using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Switch_Camera : MonoBehaviour
{
    public Camera cam1;
    public Camera cam2;
    public Camera cam3;

    private Camera currentCam;

    // Tốc độ zoom
    public float zoomSpeed = 10f;
    public float minFOV = 15f;
    public float maxFOV = 90f;

    public float minOrthoSize = 2f;
    public float maxOrthoSize = 20f;

    //private float rotationX = 0f;
    //private float rotationY = 0f;
    public float lookSpeed = 2f;    // Tốc độ xoay camera
    // Start is called before the first frame update
    void Start()
    {
        SwitchTo(cam1);
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1))
            SwitchTo(cam1);
        else if (Input.GetKeyDown(KeyCode.Alpha2))
            SwitchTo(cam2);
        else if (Input.GetKeyDown(KeyCode.Alpha3))
            SwitchTo(cam3);

        // Zoom in/out theo cuộn chuột
        if (currentCam != null)
        {
            // Xử lý zoom khi scroll chuột
            float scroll = Input.GetAxis("Mouse ScrollWheel");

            if (scroll != 0f)
            {
                if (currentCam.orthographic)
                {
                    currentCam.orthographicSize -= scroll * zoomSpeed;
                    currentCam.orthographicSize = Mathf.Clamp(currentCam.orthographicSize, minOrthoSize, maxOrthoSize);
                }
                else
                {
                    currentCam.fieldOfView -= scroll * zoomSpeed;
                    currentCam.fieldOfView = Mathf.Clamp(currentCam.fieldOfView, minFOV, maxFOV);
                }
            }

            /*
            // Xử lý xoay camera 
            // Xử lý xoay camera dựa trên chuyển động chuột
            rotationX += Input.GetAxis("Mouse X") * lookSpeed;
            rotationY -= Input.GetAxis("Mouse Y") * lookSpeed;
            rotationY = Mathf.Clamp(rotationY, -90f, 90f); // Giới hạn góc xoay theo trục X

            currentCam.transform.localRotation = Quaternion.Euler(rotationY, rotationX, 0f);
            */
        }
    }

    void SwitchTo(Camera activeCam)
    {
        // t?t h?t
        cam1.enabled = false;
        cam2.enabled = false;
        cam3.enabled = false;
        // cam3.enabled = false;

        /*
        // Reset góc quay khi chuyển cam
        rotationX = 0f;
        rotationY = 0f;
        */
    // b?t camera mong mu?n
    activeCam.enabled = true;
        currentCam = activeCam;
    }
}
