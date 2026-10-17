using System;
using System.Globalization;
using System.IO;
using System.Text;

internal static class FakeCubeProgrammer
{
    private static string Env(string name)
    {
        return Environment.GetEnvironmentVariable(name) ?? String.Empty;
    }

    private static int Main(string[] args)
    {
        AppDomain.CurrentDomain.UnhandledException += delegate(object sender, UnhandledExceptionEventArgs eventArgs)
        {
            Console.Error.WriteLine("FAKE CLI unhandled exception: " + eventArgs.ExceptionObject);
        };
        if (args.Length == 2 && args[0] == "-l" && args[1] == "st-link-only")
        {
            string probes = Env("RUNTIME_LOG_FAKE_PROBES");
            if (String.IsNullOrEmpty(probes)) probes = Env("RUNTIME_LOG_FAKE_EXPECTED_SN");
            string[] serials = probes.Split(new char[] { '|' }, StringSplitOptions.RemoveEmptyEntries);
            if (serials.Length == 0)
            {
                Console.Error.WriteLine("No ST-Link probe found");
                return 7;
            }
            foreach (string serial in serials) Console.WriteLine("ST-LINK SN : " + serial.Trim());
            return 0;
        }

        string argsPath = Env("RUNTIME_LOG_FAKE_ARGS_PATH");
        if (String.IsNullOrEmpty(argsPath))
        {
            Console.Error.WriteLine("fake CLI missing args log path");
            return 90;
        }
        File.AppendAllText(argsPath, String.Join("\t", args) + Environment.NewLine,
                           new UTF8Encoding(false));

        string expectedSerial = Env("RUNTIME_LOG_FAKE_EXPECTED_SN");
        string expectedFrequency = Env("RUNTIME_LOG_FAKE_FREQ");
        if (String.IsNullOrEmpty(expectedFrequency)) expectedFrequency = "1000";
        bool sharedExpected = Env("RUNTIME_LOG_FAKE_SHARED") != "0";
        string mode = Env("RUNTIME_LOG_FAKE_MODE");
        int index = 0;
        if (args.Length < 8 || args[index++] != "-c" ||
            args[index++] != "port=SWD" || args[index++] != "mode=HOTPLUG" ||
            (!String.Equals(args[index], "freq=" + expectedFrequency, StringComparison.OrdinalIgnoreCase) &&
             !(mode == "connect-error-1000" && (args[index] == "freq=1000" || args[index] == "freq=400"))))
        {
            Console.Error.WriteLine("fake CLI rejected connection arguments");
            return 91;
        }
        index++;
        if (sharedExpected && (index >= args.Length || args[index] != "shared"))
        {
            Console.Error.WriteLine("fake CLI rejected shared connection argument");
            return 92;
        }
        if (sharedExpected) index++;
        else if (index < args.Length && args[index] == "shared")
        {
            Console.Error.WriteLine("fake CLI received unexpected shared connection argument");
            return 92;
        }
        if (!String.IsNullOrEmpty(expectedSerial) &&
            (index >= args.Length || args[index++] != "sn=" + expectedSerial))
        {
            Console.Error.WriteLine("fake CLI rejected probe serial argument");
            return 93;
        }
        int uploadIndex = index;
        if (args.Length != uploadIndex + 4 || args[uploadIndex] != "--upload")
        {
            Console.Error.WriteLine("fake CLI rejected unexpected connection or command arguments");
            return 94;
        }
        string[] forbidden = { "port=JTAG", "mode=NORMAL", "-rst", "-halt",
                               "--download", "--erase", "-go", "--run" };
        for (int i = 0; i < args.Length; ++i)
        {
            for (int j = 0; j < forbidden.Length; ++j)
            {
                if (args[i] == forbidden[j])
                {
                    Console.Error.WriteLine("fake CLI rejected forbidden argument " + args[i]);
                    return 96;
                }
            }
        }

        string address = args[uploadIndex + 1];
        bool descriptorRead = address == Env("RUNTIME_LOG_FAKE_DESCRIPTOR_ADDRESS");
        bool scalarRead = address == Env("RUNTIME_LOG_FAKE_STATUS_SCALAR_ADDRESS");
        if (mode == "connect-error" ||
            (mode == "connect-error-1000" && args[3] == "freq=1000") ||
            (mode == "status-only-error" && scalarRead))
        {
            Console.Error.WriteLine("DEV_CONNECT_ERR: fake no RAM access");
            return 7;
        }
        if (mode == "fail-stderr")
        {
            Console.Error.WriteLine("fake CLI failure written to stderr");
            return 7;
        }
        if (mode == "warning-stderr") Console.Error.WriteLine("fake CLI warning written to stderr");

        int callNumber = 1;
        string callPath = Env("RUNTIME_LOG_FAKE_CALL_INDEX_PATH");
        if (!String.IsNullOrEmpty(callPath) && File.Exists(callPath))
        {
            Int32.TryParse(File.ReadAllText(callPath), out callNumber);
            callNumber++;
        }
        if (!String.IsNullOrEmpty(callPath))
            File.WriteAllText(callPath, callNumber.ToString(CultureInfo.InvariantCulture));

        string fixturePath = Env("RUNTIME_LOG_FAKE_BUFFER_PATH");
        string expectedAddress = Env("RUNTIME_LOG_FAKE_EXPECTED_ADDRESS");
        if (descriptorRead)
        {
            fixturePath = Env("RUNTIME_LOG_FAKE_DESCRIPTOR_PATH");
            expectedAddress = Env("RUNTIME_LOG_FAKE_DESCRIPTOR_ADDRESS");
        }
        else if (scalarRead)
        {
            fixturePath = Env("RUNTIME_LOG_FAKE_STATUS_SCALAR_PATH");
            expectedAddress = Env("RUNTIME_LOG_FAKE_STATUS_SCALAR_ADDRESS");
        }
        if (!String.IsNullOrEmpty(expectedAddress) && address != expectedAddress)
        {
            Console.Error.WriteLine("fake CLI rejected address " + address);
            return 97;
        }
        if (!File.Exists(fixturePath))
        {
            Console.Error.WriteLine("fake CLI fixture does not exist: " + fixturePath);
            return 98;
        }

        int requestedSize;
        if (!Int32.TryParse(args[uploadIndex + 2], NumberStyles.None,
                            CultureInfo.InvariantCulture, out requestedSize) || requestedSize < 0)
        {
            Console.Error.WriteLine("fake CLI rejected upload size");
            return 99;
        }
        string modeExpectedAddress = Env("RUNTIME_LOG_FAKE_MUTATE_FINAL_HEADER_ADDRESS");
        if (mode == "mutate-final-header" && callNumber == 4 &&
            (String.IsNullOrEmpty(modeExpectedAddress) || address == modeExpectedAddress))
        {
            if (requestedSize != 64)
            {
                Console.Error.WriteLine("fake CLI mutation was not a header read");
                return 100;
            }
            byte[] mutated = MutatedHeader(fixturePath);
            Directory.CreateDirectory(Path.GetDirectoryName(args[uploadIndex + 3]));
            File.WriteAllBytes(args[uploadIndex + 3], mutated);
        }
        else
        {
            byte[] fixture = File.ReadAllBytes(fixturePath);
            if (requestedSize > fixture.Length)
            {
                Console.Error.WriteLine("fake CLI fixture is shorter than requested upload");
                return 101;
            }
            byte[] content = new byte[requestedSize];
            Array.Copy(fixture, 0, content, 0, requestedSize);
            if (mode == "decode-error" && callNumber == 3 && content.Length > 127)
                content[64 + 63] = 0xFF;
            Directory.CreateDirectory(Path.GetDirectoryName(args[uploadIndex + 3]));
            File.WriteAllBytes(args[uploadIndex + 3], content);
        }
        string actualFrequency = Env("RUNTIME_LOG_FAKE_ACTUAL_FREQ");
        if (String.IsNullOrEmpty(actualFrequency)) actualFrequency = args[3].Substring(5);
        Console.WriteLine("SWD freq : " + actualFrequency + " KHz");
        Console.WriteLine("fake CLI uploaded " + requestedSize.ToString(CultureInfo.InvariantCulture) + " bytes");
        return 0;
    }

    private static byte[] MutatedHeader(string fixturePath)
    {
        byte[] header = new byte[64];
        using (FileStream stream = File.OpenRead(fixturePath))
        {
            if (stream.Read(header, 0, header.Length) != header.Length)
                throw new InvalidDataException("fake fixture has no complete header");
        }
        uint sessionId = (uint)header[20] | ((uint)header[21] << 8) |
                         ((uint)header[22] << 16) | ((uint)header[23] << 24);
        sessionId = sessionId == UInt32.MaxValue ? 1u : sessionId + 1u;
        header[20] = (byte)sessionId;
        header[21] = (byte)(sessionId >> 8);
        header[22] = (byte)(sessionId >> 16);
        header[23] = (byte)(sessionId >> 24);
        header[40] = header[41] = header[42] = header[43] = 0;
        uint crc = 0xFFFFFFFFu;
        for (int i = 0; i < header.Length; ++i)
        {
            crc ^= header[i];
            for (int bit = 0; bit < 8; ++bit)
            {
                uint mask = 0u - (crc & 1u);
                crc = (crc >> 1) ^ (0xEDB88320u & mask);
            }
        }
        crc ^= 0xFFFFFFFFu;
        header[40] = (byte)crc;
        header[41] = (byte)(crc >> 8);
        header[42] = (byte)(crc >> 16);
        header[43] = (byte)(crc >> 24);
        return header;
    }
}
