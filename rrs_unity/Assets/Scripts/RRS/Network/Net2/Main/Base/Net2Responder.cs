using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RRS.Tools.Network
{
    public class Net2Responder
    {
        ulong sequence;
        byte[] buffer;
        object @lock;
        uint timeout;

        public Net2Responder(Net2.Net2HandlerBase channel_handler, ulong sequence, uint timeout)
        {
            @lock = new object();
            this.timeout = timeout;
            this.sequence = sequence;
            buffer = null;
            channel_handler.delegateNewData += Channel_handler_delegateNewData;
        }

        private void Channel_handler_delegateNewData(ulong sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
        {
            if (this.sequence == sequence)
            {
                sender.delegateNewData -= Channel_handler_delegateNewData;
                lock (@lock)
                {
                    this.buffer = buffer;
                }
            }
        }

        public byte[] GetResponse()
        {
            byte[] result;
            lock (@lock)
            {
                result = buffer;
            }
            while (result == null && timeout > 0)
            {
                System.Threading.Thread.Sleep(100);
                if (timeout >= 100)
                {
                    timeout -= 100;
                }
                else
                {
                    timeout = 0;
                }

                lock (@lock)
                {
                    result = buffer;
                }
            }
            return buffer;
        }
    }
}
