using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wall_manner : MonoBehaviour
{
    public GameObject targetObject;
    
    // Start is called before the first frame update
    void Start()
    {
        targetObject.SetActive(false);
    }

    // Update is called once per frame
    void Update()
    {
        // Khi ng??i ch?i nh?n phím Q
        if (Input.GetKeyDown(KeyCode.Q))
        {
            // Toggle tr?ng thái active c?a object
            bool isActive = targetObject.activeSelf;
            targetObject.SetActive(!isActive);
        }
    }
}
