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
        string argsPath = Env("RUN_STATUS_FAKE_ARGS_PATH");
        if (String.IsNullOrEmpty(argsPath))
        {
            Console.Error.WriteLine("fake status CLI missing args log path");
            return 90;
        }
        File.AppendAllText(argsPath, String.Join("\t", args) + Environment.NewLine,
                           new UTF8Encoding(false));

        int callNumber = 1;
        string callPath = Env("RUN_STATUS_FAKE_CALL_INDEX_PATH");
        if (!String.IsNullOrEmpty(callPath) && File.Exists(callPath))
        {
            Int32.TryParse(File.ReadAllText(callPath), out callNumber);
            callNumber++;
        }
        if (!String.IsNullOrEmpty(callPath))
        {
            File.WriteAllText(callPath, callNumber.ToString(CultureInfo.InvariantCulture));
        }

        int index = 0;
        if (args.Length < 9 || args[index++] != "-c" ||
            args[index++] != "port=SWD" || args[index++] != "mode=HOTPLUG" ||
            args[index++] != "freq=1000" || args[index++] != "shared")
        {
            Console.Error.WriteLine("fake status CLI rejected SWD HOTPLUG/shared arguments");
            return 91;
        }
        string expectedSerial = Env("RUN_STATUS_FAKE_EXPECTED_SN");
        if (!String.IsNullOrEmpty(expectedSerial))
        {
            if (index >= args.Length || args[index++] != "sn=" + expectedSerial)
            {
                Console.Error.WriteLine("fake status CLI rejected probe serial");
                return 92;
            }
        }
        if (args.Length != index + 4 || args[index++] != "--upload")
        {
            Console.Error.WriteLine("fake status CLI rejected operation or extra arguments");
            return 93;
        }

        string[] forbidden = {
            "port=JTAG", "mode=NORMAL", "-rst", "--reset", "-hardrst",
            "-halt", "--halt", "--download", "--erase", "-go", "--run"
        };
        foreach (string value in args)
        {
            foreach (string token in forbidden)
            {
                if (String.Equals(value, token, StringComparison.OrdinalIgnoreCase))
                {
                    Console.Error.WriteLine("fake status CLI rejected forbidden argument " + value);
                    return 94;
                }
            }
        }

        string expectedAddress = callNumber == 1
            ? Env("RUN_STATUS_FAKE_SCALAR_ADDRESS")
            : Env("RUN_STATUS_FAKE_HEADER_ADDRESS");
        string expectedSizeText = callNumber == 1
            ? Env("RUN_STATUS_FAKE_SCALAR_SIZE")
            : "64";
        string expectedFixture = callNumber == 1
            ? Env("RUN_STATUS_FAKE_SCALAR_FIXTURE")
            : Env("RUN_STATUS_FAKE_HEADER_FIXTURE");
        int expectedSize;
        if (callNumber > 2 || String.IsNullOrEmpty(expectedAddress) ||
            args[index++] != expectedAddress ||
            !Int32.TryParse(expectedSizeText, NumberStyles.None,
                            CultureInfo.InvariantCulture, out expectedSize) ||
            args[index++] != expectedSize.ToString(CultureInfo.InvariantCulture))
        {
            Console.Error.WriteLine("fake status CLI rejected address or exact bounded read size");
            return 95;
        }

        string mode = Env("RUN_STATUS_FAKE_MODE");
        if (mode == "connect-error")
        {
            Console.Error.WriteLine("DEV_CONNECT_ERR: fake probe did not read RAM");
            return 7;
        }

        if (String.IsNullOrEmpty(expectedFixture) || !File.Exists(expectedFixture))
        {
            Console.Error.WriteLine("fake status CLI fixture is missing");
            return 96;
        }
        byte[] bytes = File.ReadAllBytes(expectedFixture);
        if (bytes.Length != expectedSize)
        {
            Console.Error.WriteLine("fake status CLI fixture length does not match requested size");
            return 97;
        }
        string destination = args[index];
        Directory.CreateDirectory(Path.GetDirectoryName(destination));
        File.WriteAllBytes(destination, bytes);
        Console.WriteLine("fake status CLI uploaded " + expectedSize.ToString(CultureInfo.InvariantCulture) + " bytes");
        return 0;
    }
}
