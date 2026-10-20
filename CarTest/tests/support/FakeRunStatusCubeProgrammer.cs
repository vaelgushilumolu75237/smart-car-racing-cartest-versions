using System;
using System.Globalization;
using System.IO;
using System.Text;

internal static class FakeRunStatusCubeProgrammer
{
    private static string Env(string name)
    {
        return Environment.GetEnvironmentVariable(name) ?? String.Empty;
    }

    private static int Main(string[] args)
    {
        if (args.Length == 2 && args[0] == "-l" && args[1] == "st-link-only")
        {
            string probes = Env("RUN_STATUS_FAKE_PROBES");
            if (String.IsNullOrEmpty(probes)) probes = Env("RUN_STATUS_FAKE_EXPECTED_SN");
            string[] serials = probes.Split(new char[] { '|' }, StringSplitOptions.RemoveEmptyEntries);
            if (serials.Length == 0)
            {
                Console.Error.WriteLine("No ST-Link probe found");
                return 7;
            }
            foreach (string serial in serials) Console.WriteLine("ST-LINK SN : " + serial.Trim());
            return 0;
        }

        string argsPath = Env("RUN_STATUS_FAKE_ARGS_PATH");
        if (String.IsNullOrEmpty(argsPath))
        {
            Console.Error.WriteLine("fake status CLI missing args log path");
            return 90;
        }
        File.AppendAllText(argsPath, String.Join("\t", args) + Environment.NewLine,
                           new UTF8Encoding(false));

        string expectedFrequency = Env("RUN_STATUS_FAKE_FREQ");
        if (String.IsNullOrEmpty(expectedFrequency)) expectedFrequency = "1000";
        bool sharedExpected = Env("RUN_STATUS_FAKE_SHARED") != "0";
        int index = 0;
        if (args.Length < 8 || args[index++] != "-c" || args[index++] != "port=SWD" ||
            args[index++] != "mode=HOTPLUG" ||
            (!String.Equals(args[index], "freq=" + expectedFrequency, StringComparison.OrdinalIgnoreCase) &&
             !(Env("RUN_STATUS_FAKE_MODE") == "connect-error-1000" &&
               (args[index] == "freq=1000" || args[index] == "freq=400"))))
        {
            Console.Error.WriteLine("fake status CLI rejected SWD HOTPLUG frequency arguments");
            return 91;
        }
        string requestedFrequency = args[index++].Substring(5);
        if (sharedExpected && (index >= args.Length || args[index++] != "shared"))
        {
            Console.Error.WriteLine("fake status CLI rejected shared connection argument");
            return 92;
        }
        if (!sharedExpected && index < args.Length && args[index] == "shared")
        {
            Console.Error.WriteLine("fake status CLI received unexpected shared argument");
            return 92;
        }
        string expectedSerial = Env("RUN_STATUS_FAKE_EXPECTED_SN");
        if (!String.IsNullOrEmpty(expectedSerial) &&
            (index >= args.Length || args[index++] != "sn=" + expectedSerial))
        {
            Console.Error.WriteLine("fake status CLI rejected probe serial");
            return 93;
        }
        if (args.Length != index + 4 || args[index++] != "--upload")
        {
            Console.Error.WriteLine("fake status CLI rejected operation or extra arguments");
            return 94;
        }

        string[] forbidden = { "port=JTAG", "mode=NORMAL", "-rst", "--reset", "-hardrst",
                               "-halt", "--halt", "--download", "--erase", "-go", "--run" };
        foreach (string value in args)
            foreach (string token in forbidden)
                if (String.Equals(value, token, StringComparison.OrdinalIgnoreCase))
                {
                    Console.Error.WriteLine("fake status CLI rejected forbidden argument " + value);
                    return 95;
                }

        string address = args[index];
        string expectedAddress;
        string expectedSizeText;
        string fixture;
        if (address == Env("RUN_STATUS_FAKE_DESCRIPTOR_ADDRESS"))
        {
            expectedAddress = Env("RUN_STATUS_FAKE_DESCRIPTOR_ADDRESS");
            expectedSizeText = "64";
            fixture = Env("RUN_STATUS_FAKE_DESCRIPTOR_FIXTURE");
        }
        else if (address == Env("RUN_STATUS_FAKE_SPEED_PI_DIAGNOSTIC_ADDRESS"))
        {
            expectedAddress = Env("RUN_STATUS_FAKE_SPEED_PI_DIAGNOSTIC_ADDRESS");
            expectedSizeText = "128";
            fixture = Env("RUN_STATUS_FAKE_SPEED_PI_DIAGNOSTIC_FIXTURE");
        }
        else if (address == Env("RUN_STATUS_FAKE_DIAGNOSTIC_ADDRESS"))
        {
            expectedAddress = Env("RUN_STATUS_FAKE_DIAGNOSTIC_ADDRESS");
            expectedSizeText = "64";
            fixture = Env("RUN_STATUS_FAKE_DIAGNOSTIC_FIXTURE");
        }
        else if (address == Env("RUN_STATUS_FAKE_SCALAR_ADDRESS"))
        {
            expectedAddress = Env("RUN_STATUS_FAKE_SCALAR_ADDRESS");
            expectedSizeText = Env("RUN_STATUS_FAKE_SCALAR_SIZE");
            fixture = Env("RUN_STATUS_FAKE_SCALAR_FIXTURE");
        }
        else
        {
            expectedAddress = Env("RUN_STATUS_FAKE_HEADER_ADDRESS");
            expectedSizeText = "64";
            fixture = Env("RUN_STATUS_FAKE_HEADER_FIXTURE");
        }
        int expectedSize;
        if (String.IsNullOrEmpty(expectedAddress) || address != expectedAddress ||
            !Int32.TryParse(expectedSizeText, NumberStyles.None, CultureInfo.InvariantCulture, out expectedSize) ||
            args[index + 1] != expectedSize.ToString(CultureInfo.InvariantCulture))
        {
            Console.Error.WriteLine("fake status CLI rejected address or exact bounded read size");
            return 96;
        }

        int callNumber = 1;
        string callIndexPath = Env("RUN_STATUS_FAKE_CALL_INDEX_PATH");
        if (!String.IsNullOrEmpty(callIndexPath) && File.Exists(callIndexPath))
        {
            Int32.TryParse(File.ReadAllText(callIndexPath), out callNumber);
            callNumber++;
        }
        if (!String.IsNullOrEmpty(callIndexPath))
            File.WriteAllText(callIndexPath, callNumber.ToString(CultureInfo.InvariantCulture));

        string[] uploadSequence = Env("RUN_STATUS_FAKE_UPLOAD_SEQUENCE").Split(
            new char[] { '|' }, StringSplitOptions.RemoveEmptyEntries);
        if (callNumber <= uploadSequence.Length)
            fixture = uploadSequence[callNumber - 1];

        string mode = Env("RUN_STATUS_FAKE_MODE");
        if (mode == "connect-error" || (mode == "connect-error-1000" && requestedFrequency == "1000"))
        {
            Console.Error.WriteLine("DEV_CONNECT_ERR: fake probe did not read RAM");
            return 7;
        }
        if (String.IsNullOrEmpty(fixture) || !File.Exists(fixture))
        {
            Console.Error.WriteLine("fake status CLI fixture is missing");
            return 97;
        }
        byte[] bytes = File.ReadAllBytes(fixture);
        if (bytes.Length != expectedSize)
        {
            Console.Error.WriteLine("fake status CLI fixture length does not match requested size");
            return 98;
        }
        Directory.CreateDirectory(Path.GetDirectoryName(args[index + 2]));
        File.WriteAllBytes(args[index + 2], bytes);
        string actualFrequency = Env("RUN_STATUS_FAKE_ACTUAL_FREQ");
        if (String.IsNullOrEmpty(actualFrequency)) actualFrequency = requestedFrequency;
        Console.WriteLine("SWD freq : " + actualFrequency + " KHz");
        Console.WriteLine("fake status CLI uploaded " + expectedSize.ToString(CultureInfo.InvariantCulture) + " bytes");
        return 0;
    }
}
