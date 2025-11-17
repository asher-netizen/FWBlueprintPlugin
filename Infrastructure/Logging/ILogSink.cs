namespace FWBlueprintPlugin.Infrastructure.Logging
{
    internal interface ILogSink
    {
        void Write(LogLevel level, string message);
    }
}
