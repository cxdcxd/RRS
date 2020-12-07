using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RRS.Tools.Network
{
    public partial class Net2
    {
        public class Net2HandlerPublisher : Net2HandlerBase
        {
            public void Send(byte[] data, uint priority = 0)
            {
                if ((this as INet2Handler).Instance != null)
                {
                    ((this as INet2Handler).Instance as Net2Publisher).Send(data, priority);
                }
            }
        }
    }
}
