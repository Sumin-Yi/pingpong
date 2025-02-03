import asyncio
from bleak import BleakClient

BLE_DEVICE_ADDRESS = "E5F088FF-EDFB-9C73-68E9-83CB9159FA51"

async def list_ble_services():
    try:
        ble_client = BleakClient(BLE_DEVICE_ADDRESS)
        await ble_client.connect()
        print("Connected to Ring device. Listing services and characteristics:")
        for service in ble_client.services:
            print(f"Service: {service.uuid}")
            for char in service.characteristics:
                print(f"  Characteristic: {char.uuid} | Properties: {char.properties}")
        await ble_client.disconnect()
    except Exception as e:
        print(f"Failed to list BLE services: {e}")

if __name__ == "__main__":
    asyncio.run(list_ble_services())
