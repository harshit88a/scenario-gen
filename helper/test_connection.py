import carla
import scenic
import sys

# Python 3.8+ standard way to check versions
from importlib.metadata import version 

print("Libraries imported successfully.")

try:
    # Check CARLA connection
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    
    # Check versions
    scenic_ver = version('scenic')
    carla_ver = client.get_client_version()
    
    print("-" * 30)
    print(f"✅ CARLA Server: Connected | Map: {world.get_map().name}")
    print(f"✅ CARLA Client: {carla_ver}")
    print(f"✅ Scenic Version: {scenic_ver}")
    print("-" * 30)
    
except Exception as e:
    print(f"❌ Error: {e}")