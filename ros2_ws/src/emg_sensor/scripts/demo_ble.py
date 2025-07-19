import asyncio, bleak

ADDR = "F7:DC:B8:38:6D:F6"

UART_RX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"    # Nordic UART RX UUID

async def main():
    async with bleak.BleakClient(ADDR) as dev:
        await dev.start_notify(UART_RX, lambda h, d: print(d))
        print("Listening…  Ctrl+C to stop")
        while True:
            await asyncio.sleep(60)

if __name__ == "__main__":
    asyncio.run(main())

