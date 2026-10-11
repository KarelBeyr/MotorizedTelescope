
using System.IO.Ports;

if (args.Length != 1 ||
    !int.TryParse(args[0], out int photosPerRow) ||
    photosPerRow < 1)
{
    Console.WriteLine("Usage: Panorama <steps-per-row>");
    return;
}


const int settlingDelayMs = 2000;
const int captureDelayMs = 700;

using var motor = new SerialPort("COM5", 115200)
{
    NewLine = "\n",
    ReadTimeout = 1000
};

using var shutter = new SerialPort("COM8", 115200);

using var cts = new CancellationTokenSource();

Console.CancelKeyPress += (_, e) =>
{
    e.Cancel = true;
    cts.Cancel();
};

try
{
    motor.Open();
    shutter.Open();

    // Arduino Nano may reset when serial port opens.
    Console.WriteLine("Connecting...");
    await Task.Delay(2500, cts.Token);
    motor.DiscardInBuffer();
    shutter.DiscardInBuffer();

    Console.WriteLine($"Ready. {photosPerRow} photos per row.");
    Console.WriteLine("Press Ctrl+C to stop.");

    Console.WriteLine(
    "Press ENTER for next row, Ctrl+C to quit.");

    Console.ReadLine();

    int row = 0;

    while (!cts.IsCancellationRequested)
    {
        char direction = row % 2 == 0 ? 'r' : 'l';

        Console.WriteLine();
        Console.WriteLine(
            $"Row {row + 1}, direction {direction}");

        for (int i = 0; i < photosPerRow; i++)
        {
            cts.Token.ThrowIfCancellationRequested();

            // Capture at the current position
            shutter.Write("c");

            Console.WriteLine(
                $"  Photo {i + 1}/{photosPerRow}");

            await Task.Delay(captureDelayMs, cts.Token);

            // Don't move after the last photograph!
            if (i == photosPerRow - 1)
                break;

            // Move to the next position
            motor.Write(direction.ToString());

            // Wait until motor finishes
            while (true)
            {
                cts.Token.ThrowIfCancellationRequested();

                try
                {
                    string response = motor.ReadLine().Trim();

                    if (response.StartsWith("DONE"))
                        break;

                    if (response.StartsWith("ERR"))
                        throw new Exception(response);
                }
                catch (TimeoutException)
                {
                    // Keep waiting, allowing Ctrl+C
                }
            }

            // Allow telescope vibrations to settle
            await Task.Delay(settlingDelayMs, cts.Token);
        }

        row++;

        Console.WriteLine();
        Console.WriteLine(
            "Row completed! Adjust telescope elevation.");
        Console.WriteLine(
            "Press ENTER for next row, Ctrl+C to quit.");

        Console.ReadLine();
        cts.Token.ThrowIfCancellationRequested();
    }
}
catch (OperationCanceledException)
{
    Console.WriteLine("Cancelled.");
}
catch (Exception ex)
{
    Console.WriteLine($"ERROR: {ex.Message}");
}
finally
{
    if (motor.IsOpen)
    {
        // Stop any movement still in progress.
        try { motor.Write("s"); }
        catch { }
    }

    Console.WriteLine("Finished.");
}
