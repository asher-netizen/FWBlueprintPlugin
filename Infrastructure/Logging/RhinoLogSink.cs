using Rhino;

namespace FWBlueprintPlugin.Infrastructure.Logging
{
    internal sealed class RhinoLogSink : ILogSink
    {
        public void Write(LogLevel level, string message)
        {
            if (string.IsNullOrWhiteSpace(message))
            {
                return;
            }

            RhinoApp.WriteLine($"[{level}] {message}");
        }
    }
}
