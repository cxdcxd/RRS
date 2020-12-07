
using RRS.Tools.Log;
using System;
using System.IO;
using System.Xml.Serialization;

namespace RRS.Tools.DataManagerXML
{
    public class DataManagerXML
    {
        public delegate void DelegateNewLog(string log_message, LogType log_type, string section);
        public event DelegateNewLog delegateNewLog;

        string section = "DataManagerXML";

        ProcessResult XMLSaveData(string path, Type type, object obj)
        {
            ProcessResult pr = new ProcessResult();

            XmlSerializer serializer = new XmlSerializer(type);
            try
            {
                TextWriter writer = new StreamWriter(path);
                serializer.Serialize(writer, obj);
                writer.Close();
                pr.Success = true;
            }
            catch (Exception e)
            {
                pr.Success = false;
                pr.Message = e.Message;
            }

            return pr;
        }

        ProcessResult XMLLoadData(string path, Type type)
        {
            ProcessResult pr = new ProcessResult();
            pr.ResultType = type;

            XmlSerializer serializer = new XmlSerializer(type);
            try
            {
                FileStream reader = new FileStream(path, FileMode.Open);
                pr.Result = serializer.Deserialize(reader);
                reader.Close();
                pr.Success = true;
            }
            catch (Exception e)
            {
                pr.Success = false;
                pr.Message = e.Message;
            }

            return pr;
        }

        public ProcessResult saveXML<T>(string path, T target) where T : class, new()
        {
            ProcessResult pr = new ProcessResult();
            pr.Message = "";

            try
            {
                XMLSaveData(path, typeof(T), target);

                delegateNewLog?.Invoke("Save xml data done", LogType.DEBUG, section);
                pr.Message = "done";
                pr.Success = true;

            }
            catch (Exception e)
            {
                delegateNewLog?.Invoke("Cant save xml data " + e.Message, LogType.ERROR, section);
                pr.Message = e.Message;
                pr.Success = false;

            }

            return pr;
        }

        public ProcessResult loadXML<T>(string path) where T : class, new()
        {
            ProcessResult pr = new ProcessResult();

            string dir = System.IO.Path.GetDirectoryName(path);

            if (System.IO.Directory.Exists(dir) == false)
            {
                System.IO.Directory.CreateDirectory(dir);
            }

            if (System.IO.File.Exists(path) == false)
            {
                pr.Message = "Cant load xml data, loading path is not found";
                delegateNewLog?.Invoke(pr.Message, LogType.ERROR, section);
                delegateNewLog?.Invoke("Saving default xml data ", LogType.DEBUG, section);
                pr.Success = false;
                pr.Result = null;
                saveXML(path, new T());
                return pr;
            }

            try
            {
                ProcessResult result = XMLLoadData(path, typeof(T));
                return result;
            }
            catch (Exception e)
            {
                pr.Message = "Cant load xml data " + e.Message;
                pr.Success = false;
                delegateNewLog?.Invoke(pr.Message, LogType.ERROR, section);

                delegateNewLog?.Invoke("Saving default xml data ", LogType.DEBUG, section);

                saveXML(path, new T());
            }

            return pr;
        }
    }
}
