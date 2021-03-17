using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class time3d : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject[] numbers;
    ArrayList created_numbers = new ArrayList();
    
    void Start()
    {
        
    }

    void createNumber(string number,Transform parent,float offset)
    {
       
        if (number == "0" )
            created_numbers.Add(Instantiate(numbers[0], parent.transform.position +  new Vector3(offset, 0.1f, 0), Quaternion.identity , parent));
        if (number == "1")
            created_numbers.Add(Instantiate(numbers[1], parent.transform.position +  new Vector3(offset, 0.1f, 0), Quaternion.identity, parent));
        if (number == "2")
            created_numbers.Add(Instantiate(numbers[2], parent.transform.position +  new Vector3(offset, 0.1f, 0), Quaternion.identity, parent));
        if (number == "3")
            created_numbers.Add(Instantiate(numbers[3], parent.transform.position +  new Vector3(offset, 0.1f, 0), Quaternion.identity, parent));
        if (number == "4")
            created_numbers.Add(Instantiate(numbers[4], parent.transform.position +  new Vector3(offset, 0.1f, 0), Quaternion.identity, parent));
        if (number == "5")
            created_numbers.Add(Instantiate(numbers[5], parent.transform.position + new Vector3(offset, 0.1f, 0), Quaternion.identity, parent));
        if (number == "6")
            created_numbers.Add(Instantiate(numbers[6], parent.transform.position +  new Vector3(offset, 0.1f, 0), Quaternion.identity, parent));
        if (number == "7")
            created_numbers.Add(Instantiate(numbers[7], parent.transform.position +  new Vector3(offset, 0.1f, 0), Quaternion.identity, parent));
        if (number == "8")
            created_numbers.Add(Instantiate(numbers[8], parent.transform.position +  new Vector3(offset, 0.1f, 0), Quaternion.identity, parent));
        if (number == "9")
            created_numbers.Add(Instantiate(numbers[9], parent.transform.position +  new Vector3(offset, 0.1f, 0), Quaternion.identity, parent));
        if (number == ":")
            created_numbers.Add(Instantiate(numbers[10], parent.transform.position +  new Vector3(offset, 0.1f, 0), Quaternion.identity, parent));

      
    }

    void renderTime()
    {
        for ( int i = 0; i < created_numbers.Count; i++)
        {
            Destroy(((GameObject)created_numbers[i]));
        }

        created_numbers.Clear();

        string current_time = System.DateTime.Now.Second + ":" + System.DateTime.Now.Millisecond;

        for ( int i = 0; i < current_time.Length; i++)
        {
            createNumber(current_time[i].ToString(), transform, -i * 0.5f);
        }

    }

    // Update is called once per frame
    void  Update()
    {
       renderTime();
    }
}
