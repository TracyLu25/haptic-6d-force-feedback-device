using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class COMManager : MonoBehaviour
{
    public static COMManager Instance;

    public int comPortNumber = 1;

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
            //Debug.Log("DontDestroyOnLoad called!");
        }
        else
        {
            Destroy(gameObject);
            //Debug.Log("Duplicate COMManager destroyed!");
        }
    }
}
