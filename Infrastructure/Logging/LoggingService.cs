using System;

namespace FWBlueprintPlugin.Infrastructure.Logging
{
    internal static class LoggingService
    {
        private static ILogSink _sink = new RhinoLogSink();

        public static LogLevel MinimumLevel { get; set; } = LogLevel.Warn;

        public static ILogSink Sink
        {
            get => _sink;
            set => _sink = value ?? throw new ArgumentNullException(nameof(value));
        }

        public static bool IsEnabled(LogLevel level) => level >= MinimumLevel;

        public static void Log(LogLevel level, string message)
        {
            if (!IsEnabled(level))
            {
                return;
            }

            _sink.Write(level, message);
        }

        public static void Info(string message) => Log(LogLevel.Info, message);

        public static void Warn(string message) => Log(LogLevel.Warn, message);

        public static void Error(string message) => Log(LogLevel.Error, message);

        public static void Debug(string message) => Log(LogLevel.Debug, message);
    }
}
