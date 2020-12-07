using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RRS.Tools.Protobuf
{
    public interface IRawData
    {
        byte[] Data { get; set; }
        int Length { get; set; }
    }

    public interface IData
    {
        int Version { get; set; }
    }
}
