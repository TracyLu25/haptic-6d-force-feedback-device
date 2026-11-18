using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class Main_menu_load : MonoBehaviour
{
    public TMP_InputField comInputField;
    public void Play()
    {
        // Đọc giá trị
        if (int.TryParse(comInputField.text, out int comNumber))
        {
            COMManager.Instance.comPortNumber = comNumber;
            //Debug.Log("COM number set to: " + comNumber);
        }
        else
        {
            //Debug.LogWarning("Invalid COM number input!");
            COMManager.Instance.comPortNumber = 1; // mặc định
        }

        // LOAD scene chính
        SceneManager.LoadSceneAsync(1);
    }
}
