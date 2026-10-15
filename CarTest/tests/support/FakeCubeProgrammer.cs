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
        string argsPath = Env("RUNTIME_LOG_FAKE_ARGS_PATH");
        if (String.IsNullOrEmpty(argsPath))
        {
            Console.Error.WriteLine("fake CLI missing args log path");
            return 90;
        }
        File.AppendAllText(argsPath, String.Join("\t", args) + Environment.NewLine,
                           new UTF8Encoding(false));

        string expectedSerial = Env("RUNTIME_LOG_FAKE_EXPECTED_SN");
        bool serialSupplied = !String.IsNullOrEmpty(expectedSerial);
        string expectedFrequency = Env("RUNTIME_LOG_FAKE_FREQ");
        if (String.IsNullOrEmpty(expectedFrequency)) expectedFrequency = "4000";
        bool sharedExpected = Env("RUNTIME_LOG_FAKE_SHARED") == "1";
        int uploadIndex = 0;
        if (args.Length < 8 || args[uploadIndex++] != "-c" ||
            args[uploadIndex++] != "port=SWD" || args[uploadIndex++] != "mode=HOTPLUG" ||
            args[uploadIndex++] != "freq=" + expectedFrequency)
        {
            Console.Error.WriteLine("fake CLI rejected connection arguments");
            return 91;
        }
        if (sharedExpected && (uploadIndex >= args.Length || args[uploadIndex++] != "shared"))
        {
            Console.Error.WriteLine("fake CLI rejected shared connection argument");
            return 92;
        }
        if (serialSupplied && (uploadIndex >= args.Length || args[uploadIndex] != "sn=" + expectedSerial))
        {
            Console.Error.WriteLine("fake CLI rejected probe serial argument");
            return 93;
        }
        if (serialSupplied) uploadIndex++;
        int expectedCount = uploadIndex + 4;
        if (args.Length != expectedCount)
        {
            Console.Error.WriteLine("fake CLI rejected unexpected connection or command arguments");
            return 94;
        }
        if (args[uploadIndex] != "--upload")
        {
            Console.Error.WriteLine("fake CLI rejected non-upload operation");
            return 95;
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

        string expectedAddress = Env("RUNTIME_LOG_FAKE_EXPECTED_ADDRESS");
        if (!String.IsNullOrEmpty(expectedAddress) && args[uploadIndex + 1] != expectedAddress)
        {
            Console.Error.WriteLine("fake CLI rejected address " + args[uploadIndex + 1]);
            return 97;
        }

        int callNumber = 1;
        string callPath = Env("RUNTIME_LOG_FAKE_CALL_INDEX_PATH");
        if (!String.IsNullOrEmpty(callPath) && File.Exists(callPath))
        {
            Int32.TryParse(File.ReadAllText(callPath), out callNumber);
            callNumber++;
        }
        if (!String.IsNullOrEmpty(callPath))
        {
            File.WriteAllText(callPath, callNumber.ToString(CultureInfo.InvariantCulture));
        }

        string mode = Env("RUNTIME_LOG_FAKE_MODE");
        if (mode == "connect-error")
        {
            Console.Error.WriteLine("DEV_CONNECT_ERR: fake no RAM access");
            return 7;
        }
        if (mode == "fail-stderr")
        {
            Console.Error.WriteLine("fake CLI failure written to stderr");
            return 7;
        }
        if (mode == "warning-stderr")
        {
            Console.Error.WriteLine("fake CLI warning written to stderr");
        }

        string fixturePath = Env("RUNTIME_LOG_FAKE_BUFFER_PATH");
        if (!File.Exists(fixturePath))
        {
            Console.Error.WriteLine("fake CLI fixture does not exist");
            return 98;
        }
        int requestedSize;
        if (!Int32.TryParse(args[uploadIndex + 2], NumberStyles.None,
                            CultureInfo.InvariantCulture, out requestedSize) ||
            requestedSize < 0)
        {
            Console.Error.WriteLine("fake CLI rejected upload size");
            return 99;
        }
        string destination = args[uploadIndex + 3];
        byte[] fixture = File.ReadAllBytes(fixturePath);
        byte[] content;
        if (mode == "mutate-final-header" && callNumber == 3)
        {
            content = MutatedHeader();
            if (requestedSize != 64)
            {
                Console.Error.WriteLine("fake CLI mutation was not a header read");
                return 100;
            }
        }
        else
        {
            if (requestedSize > fixture.Length)
            {
                Console.Error.WriteLine("fake CLI fixture is shorter than requested upload");
                return 101;
            }
            content = new byte[requestedSize];
            Array.Copy(fixture, 0, content, 0, requestedSize);
            if (mode == "decode-error" && callNumber == 2 && content.Length > 127)
            {
                content[64 + 63] = 0xFF; /* invalid recovery_active byte */
            }
        }
        Directory.CreateDirectory(Path.GetDirectoryName(destination));
        File.WriteAllBytes(destination, content);
        Console.WriteLine("fake CLI uploaded " + requestedSize.ToString(CultureInfo.InvariantCulture) + " bytes");
        return 0;
    }

    private static byte[] MutatedHeader()
    {
        string fixturePath = Env("RUNTIME_LOG_FAKE_BUFFER_PATH");
        byte[] header = new byte[64];
        using (FileStream stream = File.OpenRead(fixturePath))
        {
            if (stream.Read(header, 0, header.Length) != header.Length)
            {
                throw new InvalidDataException("fake fixture has no complete header");
            }
        }
        uint sessionId = (uint)header[20] | ((uint)header[21] << 8) |
                         ((uint)header[22] << 16) | ((uint)header[23] << 24);
        sessionId++;
        if (sessionId == 0u)
        {
            sessionId = 1u;
        }
        header[20] = (byte)sessionId;
        header[21] = (byte)(sessionId >> 8);
        header[22] = (byte)(sessionId >> 16);
        header[23] = (byte)(sessionId >> 24);
        header[40] = 0;
        header[41] = 0;
        header[42] = 0;
        header[43] = 0;
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
