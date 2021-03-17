using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using Simulator.Plugins;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using Yangrc.OpenGLAsyncReadback;

public class ColorCameraOpenGL : MonoBehaviour
{
    [Header("Shader Setup")]
    public Shader uberReplacementShader;
    public Shader opticalFlowShader;
    public float opticalFlowSensitivity = 1.0f;

    // cached materials
    private Material opticalFlowMaterial;

    [Range(1, 1920)]
    public int Width = 800;

    [Range(1, 1080)]
    public int Height = 600;

    [Range(1, 100)]
    public int Frequency = 15;

    [Range(0, 100)]
    public int JpegQuality = 75;

    [Range(1.0f, 90.0f)]
    public float FieldOfView = 60.0f;

    [Range(0.01f, 1000.0f)]
    public float MinDistance = 0.1f;

    [Range(0.01f, 2000.0f)]
    public float MaxDistance = 1000.0f;

    public enum Mode
    {
        depth, flow, objectid, catid, normals, color
    }

    public Mode mode;

    public delegate void DelegateDataChanged(byte[] buffer);
    public event DelegateDataChanged delegateCameraDataChanged;
    ConcurrentBag<NativeArray<byte>> AvailableGpuDataArrays = new ConcurrentBag<NativeArray<byte>>();
    private ConcurrentBag<byte[]> JpegOutput = new ConcurrentBag<byte[]>();

    private Queue<Task> Tasks = new Queue<Task>();

    private struct CameraCapture
    {
        public NativeArray<byte> GpuData;
        public UniversalAsyncGPUReadbackRequest Request;
        public double CaptureTime;
    }

    private Queue<CameraCapture> CaptureQueue = new Queue<CameraCapture>();

    Queue<UniversalAsyncGPUReadbackRequest> _requests = new Queue<UniversalAsyncGPUReadbackRequest>();

    private RenderTexture rt;
    protected Camera SensorCamera;

    protected RenderTextureReadWrite CameraTargetTextureReadWriteType = RenderTextureReadWrite.sRGB;

    public int CubemapSize = 1024;

    const int MaxJpegSize = 4 * 1024 * 1024; // 4MB
    byte[] buffer;
    private float NextCaptureTime;

    uint Sequence;

    bool invalid = false;

    private void Start()
    {
        SensorCamera = GetComponent<Camera>();

        buffer = new byte[MaxJpegSize];
        // default fallbacks, if shaders are unspecified
        if (!uberReplacementShader)
            uberReplacementShader = Shader.Find("Hidden/UberReplacement");

        if (!opticalFlowShader)
            opticalFlowShader = Shader.Find("Hidden/OpticalFlow");

        OnCameraChange();
        OnSceneChange();
    }

    public void OnCameraChange()
    {
        // cache materials and setup material properties
        if (!opticalFlowMaterial || opticalFlowMaterial.shader != opticalFlowShader)
            opticalFlowMaterial = new Material(opticalFlowShader);
        opticalFlowMaterial.SetFloat("_Sensitivity", opticalFlowSensitivity);

        if (mode == Mode.objectid) SetupCameraWithReplacementShader(SensorCamera, uberReplacementShader, ReplacementMode.ObjectId);
        if (mode == Mode.catid) SetupCameraWithReplacementShader(SensorCamera, uberReplacementShader, ReplacementMode.CatergoryId);
        if (mode == Mode.depth) SetupCameraWithReplacementShader(SensorCamera, uberReplacementShader, ReplacementMode.DepthCompressed, Color.white);
        if (mode == Mode.normals) SetupCameraWithReplacementShader(SensorCamera, uberReplacementShader, ReplacementMode.Normals);
        if (mode == Mode.flow) SetupCameraWithPostShader(SensorCamera, opticalFlowMaterial, DepthTextureMode.Depth | DepthTextureMode.MotionVectors);

    }

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacementMode mode)
    {
        SetupCameraWithReplacementShader(cam, shader, mode, Color.black);
    }

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacementMode mode, Color clearColor)
    {
        var cb = new CommandBuffer();
        cb.SetGlobalFloat("_OutputMode", (int)mode); // @TODO: CommandBuffer is missing SetGlobalInt() method
        cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
        cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
        cam.SetReplacementShader(shader, "");
        cam.backgroundColor = clearColor;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.allowHDR = false;
        cam.allowMSAA = false;
    }

    static private void SetupCameraWithPostShader(Camera cam, Material material, DepthTextureMode depthTextureMode = DepthTextureMode.None)
    {
        var cb = new CommandBuffer();
        cb.Blit(null, BuiltinRenderTextureType.CurrentActive, material);
        cam.AddCommandBuffer(CameraEvent.AfterEverything, cb);
        cam.depthTextureMode = depthTextureMode;
    }

    public enum ReplacementMode
    {
        ObjectId = 0,
        CatergoryId = 1,
        DepthCompressed = 2,
        DepthMultichannel = 3,
        Normals = 4
    };

    public void OnSceneChange()
    {
        var renderers = UnityEngine.Object.FindObjectsOfType<Renderer>();
        var mpb = new MaterialPropertyBlock();
        foreach (var r in renderers)
        {
            var id = r.gameObject.GetInstanceID();
            var layer = r.gameObject.layer;
            var tag = r.gameObject.tag;

            mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
            mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer));
            r.SetPropertyBlock(mpb);
        }
    }

    void CheckTexture()
    {
        // if this is not first time
        if (SensorCamera.targetTexture != null)
        {
            if (Width != SensorCamera.targetTexture.width || Height != SensorCamera.targetTexture.height)
            {
                // if camera capture size has changed
                SensorCamera.targetTexture.Release();
                SensorCamera.targetTexture = null;
            }
            else if (!SensorCamera.targetTexture.IsCreated())
            {
                // if we have lost rendertexture due to Unity window resizing or otherwise
                SensorCamera.targetTexture.Release();
                SensorCamera.targetTexture = null;
            }
        }

        if (SensorCamera.targetTexture == null)
        {
            SensorCamera.targetTexture = new RenderTexture(Width, Height, 24,
                RenderTextureFormat.ARGB32, CameraTargetTextureReadWriteType)
            {
                dimension = TextureDimension.Tex2D,
                antiAliasing = 1,
                useMipMap = false,
                useDynamicScale = false,
                wrapMode = TextureWrapMode.Clamp,
                filterMode = FilterMode.Bilinear,
            };
        }

    }

    void Update()
    {
        if (invalid == false)
        {
            SensorCamera.fieldOfView = FieldOfView;
            SensorCamera.nearClipPlane = MinDistance;
            SensorCamera.farClipPlane = MaxDistance;

            while (Tasks.Count > 0 && Tasks.Peek().IsCompleted)
            {
                Tasks.Dequeue();
            }

            CheckTexture();
            CheckCapture();
            ProcessReadbackRequests();
        }
    }

    void CheckCapture()
    {
        if (Time.time >= NextCaptureTime)
        {
            SensorCamera.Render();

            NativeArray<byte> gpuData;
            while (AvailableGpuDataArrays.TryTake(out gpuData) && gpuData.Length != Width * Height * 4)
            {
                gpuData.Dispose();
            }
            if (!gpuData.IsCreated)
            {
                gpuData = new NativeArray<byte>(Width * Height * 4, Allocator.Persistent);
            }

            var capture = new CameraCapture()
            {
                GpuData = gpuData,
                CaptureTime = Time.time,
            };

            capture.Request = UniversalAsyncGPUReadbackRequest.Request(SensorCamera.targetTexture);
            CaptureQueue.Enqueue(capture);

            NextCaptureTime = Time.time + (1.0f / Frequency);
        }
    }

    void ProcessReadbackRequests()
    {
        while (CaptureQueue.Count > 0)
        {
            var capture = CaptureQueue.Peek();
            if (capture.Request.hasError)
            {
                CaptureQueue.Dequeue();
                AvailableGpuDataArrays.Add(capture.GpuData);
                Debug.Log("Failed to read GPU texture");
            }
            else if (capture.Request.done)
            {
                CaptureQueue.Dequeue();

                // TODO: Remove the following two lines of extra memory copy, when we can use 
                // AsyncGPUReadback.RequestIntoNativeArray.
                var data = capture.Request.GetData<byte>();
                NativeArray<byte>.Copy(data, capture.GpuData, data.Length);

                if (!JpegOutput.TryTake(out buffer))
                {
                    buffer = new byte[MaxJpegSize];
                }


                Tasks.Enqueue(Task.Run(() =>
                {
                    int size = JpegEncoder.Encode(capture.GpuData, Width, Height, 4, JpegQuality, buffer);
                    if (size > 0)
                    {
                        //print(size);

                        byte[] new_buffer = new byte[size];

                        Buffer.BlockCopy(buffer, 0, new_buffer, 0, size);

                        delegateCameraDataChanged?.Invoke(new_buffer);
                    }
                    else
                    {
                        Debug.Log("Compressed image is empty, length = 0");
                    }
                    JpegOutput.Add(buffer);
                    AvailableGpuDataArrays.Add(capture.GpuData);
                }));

                Sequence++;


            }
            else
            {
                break;
            }
        }
    }

    public void OnDestroy()
    {
        invalid = true;

        if (SensorCamera != null && SensorCamera.targetTexture != null)
        {
            SensorCamera.targetTexture.Release();
        }

        while (CaptureQueue.Count > 0)
        {
            var capture = CaptureQueue.Dequeue();
        }

        // Wait all tasks finished to gurantee all native arrays are in AvailableGpuDataArrays.
        Task.WaitAll(Tasks.ToArray());
        while (AvailableGpuDataArrays.TryTake(out var gpuData))
        {
            gpuData.Dispose();
        }

    }

}

