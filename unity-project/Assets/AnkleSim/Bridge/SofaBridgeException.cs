using System;

namespace AnkleSim.Bridge
{
    public class SofaBridgeException : Exception
    {
        public SofaBridgeException(string message) : base(message) { }
        public SofaBridgeException(string message, Exception inner) : base(message, inner) { }
    }
}
