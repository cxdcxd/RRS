using UnityEngine;
using System.Collections;

public class Orbit : MonoBehaviour
{
    public GameObject target;
    public Transform targetFocus;

    public float distance = 1.0f;
    [Range(1f, 8f)]
    public float ZoomWheelSpeed = 8.0f;
    public float minDistance = 0.5f;
    public float maxDistance = 10f;

    public float xSpeed = 250.0f;
    public float ySpeed = 120.0f;

    public float xObjSpeed = 250.0f;
    public float yObjSpeed = 120.0f;

    public float yMinLimit = -20;
    public float yMaxLimit = 80;

    private float x = 0.0f;
    private float y = 0.0f;

    private float normal_angle = 0.0f;


    private float cur_distance = 0;

    private float cur_xSpeed = 0;
    private float cur_ySpeed = 0;
    private float req_xSpeed = 0;
    private float req_ySpeed = 0;

    private float cur_ObjxSpeed = 0;
    private float cur_ObjySpeed = 0;
    private float req_ObjxSpeed = 0;
    private float req_ObjySpeed = 0;

    private bool DraggingObject = false;
    private bool lastLMBState = false;
    private Collider[] surfaceColliders;
    private float bounds_MaxSize = 20;

    [HideInInspector]
    public bool disableSteering = false;

    void Start()
    {
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;

        Reset();
    }

    public void DisableSteering(bool state)
    {
        disableSteering = state;
    }

    public void Reset()
    {
        lastLMBState = Input.GetMouseButton(0);

        disableSteering = false;

        cur_distance = distance;
        cur_xSpeed = 0;
        cur_ySpeed = 0;
        req_xSpeed = 0;
        req_ySpeed = 0;
        surfaceColliders = null;

        cur_ObjxSpeed = 0;
        cur_ObjySpeed = 0;
        req_ObjxSpeed = 0;
        req_ObjySpeed = 0;

        if (target)
        {
            Renderer[] renderers = target.GetComponentsInChildren<Renderer>();
            Bounds bounds = new Bounds();
            bool initedBounds = false;
            foreach (Renderer rend in renderers)
            {
                if (!initedBounds)
                {
                    initedBounds = true;
                    bounds = rend.bounds;
                }
                else
                {
                    bounds.Encapsulate(rend.bounds);
                }
            }
            Vector3 size = bounds.size;
            float dist = size.x > size.y ? size.x : size.y;
            dist = size.z > dist ? size.z : dist;
            bounds_MaxSize = dist;
            cur_distance += bounds_MaxSize * 1.2f;

            surfaceColliders = target.GetComponentsInChildren<Collider>();
        }
    }

    void LateUpdate()
    {
        if (target && targetFocus)
        {

            if (!lastLMBState && Input.GetMouseButton(0))
            {
                // mouse down
                DraggingObject = false;
                if (surfaceColliders != null)
                {
                    RaycastHit hitInfo = new RaycastHit();
                    Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
                    foreach (Collider col in surfaceColliders)
                    {
                        if (col.Raycast(ray, out hitInfo, Mathf.Infinity))
                        {
                            DraggingObject = true;
                            break;
                        }
                    }
                }
            }
            else if (lastLMBState && !Input.GetMouseButton(0))
            {
                // mouse up
                DraggingObject = false;
            }
            lastLMBState = Input.GetMouseButton(0);

            if (DraggingObject)
            {
                if (Input.GetMouseButton(0) && !disableSteering)
                {
                    req_ObjxSpeed += (Input.GetAxis("Mouse X") * xObjSpeed * 0.02f - req_ObjxSpeed) * Time.deltaTime * 10;
                    req_ObjySpeed += (Input.GetAxis("Mouse Y") * yObjSpeed * 0.02f - req_ObjySpeed) * Time.deltaTime * 10;
                }
                else
                {
                    req_ObjxSpeed += (0 - req_ObjxSpeed) * Time.deltaTime * 4;
                    req_ObjySpeed += (0 - req_ObjySpeed) * Time.deltaTime * 4;
                }

                req_xSpeed += (0 - req_xSpeed) * Time.deltaTime * 4;
                req_ySpeed += (0 - req_ySpeed) * Time.deltaTime * 4;
            }
            else
            {
                if (Input.GetMouseButton(0) && !disableSteering)
                {
                    req_xSpeed += (Input.GetAxis("Mouse X") * xSpeed * 0.02f - req_xSpeed) * Time.deltaTime * 10;
                    req_ySpeed += (Input.GetAxis("Mouse Y") * ySpeed * 0.02f - req_ySpeed) * Time.deltaTime * 10;
                }
                else
                {
                    req_xSpeed += (0 - req_xSpeed) * Time.deltaTime * 4;
                    req_ySpeed += (0 - req_ySpeed) * Time.deltaTime * 4;
                }

                req_ObjxSpeed += (0 - req_ObjxSpeed) * Time.deltaTime * 4;
                req_ObjySpeed += (0 - req_ObjySpeed) * Time.deltaTime * 4;
            }

            distance -= Input.GetAxis("Mouse ScrollWheel") * ZoomWheelSpeed;
            distance = Mathf.Clamp(distance, minDistance, maxDistance);

            cur_ObjxSpeed += (req_ObjxSpeed - cur_ObjxSpeed) * Time.deltaTime * 20;
            cur_ObjySpeed += (req_ObjySpeed - cur_ObjySpeed) * Time.deltaTime * 20;
            target.transform.RotateAround(targetFocus.position, Vector3.Cross((targetFocus.position - transform.position), transform.right), -cur_ObjxSpeed);
            target.transform.RotateAround(targetFocus.position, Vector3.Cross((targetFocus.position - transform.position), transform.up), -cur_ObjySpeed);

            cur_xSpeed += (req_xSpeed - cur_xSpeed) * Time.deltaTime * 20;
            cur_ySpeed += (req_ySpeed - cur_ySpeed) * Time.deltaTime * 20;
            x += cur_xSpeed;
            y -= cur_ySpeed;

            y = ClampAngle(y, yMinLimit + normal_angle, yMaxLimit + normal_angle);

            //if (surfaceColliders != null)
            //{
            //    RaycastHit hitInfo = new RaycastHit();
            //    Vector3 vdir = Vector3.Normalize(targetFocus.position - transform.position);
            //    float reqDistance = 0.01f;
            //    bool surfaceFound = false;
            //    foreach (Collider surfaceCollider in surfaceColliders)
            //    {
            //        if (surfaceCollider.Raycast(new Ray(transform.position - vdir * bounds_MaxSize, vdir), out hitInfo, Mathf.Infinity))
            //        {
            //            reqDistance = Mathf.Max(Vector3.Distance(hitInfo.point, targetFocus.position) + distance, reqDistance);
            //            surfaceFound = true;
            //        }
            //    }
            //    if (surfaceFound)
            //    {
            //        cur_distance += (reqDistance - cur_distance) * Time.deltaTime * 4;
            //    }
            //}

            Quaternion rotation = Quaternion.Euler(y, x, 0);
            Vector3 position = rotation * new Vector3(0.0f, 0.0f, -distance) + targetFocus.position;

            transform.rotation = rotation;
            transform.position = position;
        }
    }

    static float ClampAngle(float angle, float min, float max)
    {
        if (angle < -360)
            angle += 360;
        if (angle > 360)
            angle -= 360;
        return Mathf.Clamp(angle, min, max);
    }

    public void set_normal_angle(float a)
    {
        normal_angle = a;
    }
}