# super vibecoded test of lidar, this just prints the distance from straight forward


import asyncio
import time
from rplidarc1 import RPLidar
import serial

PORT = "/dev/ttyUSB0"
BAUD = 460800
MAX_DISTANCE_MM = 12000.0
MIN_DISTANCE_MM = 50.0


def prepare_serial_port(port, baud):
  """Stops any ongoing scan and flushes leftover bytes."""
  with serial.Serial(port, baud, timeout=0.5) as ser:
    ser.write(bytes([0xA5, 0x25]))
    time.sleep(0.1)
    ser.reset_input_buffer()


def get_front_distance(scan_dict, tolerance=2.0):
  """Finds the valid distance reading closest to 0° (within +/- tolerance)."""
  closest_angle = None
  closest_dist = None
  min_diff = tolerance

  for raw_angle, raw_dist in list(scan_dict.items()):
    # Skip None values
    if raw_angle is None or raw_dist is None:
      continue

    try:
      # Handle case where value is a tuple/list like (distance, quality)
      if isinstance(raw_dist, (tuple, list)):
        raw_dist = raw_dist

      angle = float(raw_angle)
      distance = float(raw_dist)
    except (ValueError, TypeError):
      continue

    if not (MIN_DISTANCE_MM <= distance <= MAX_DISTANCE_MM):
      continue

    diff = min(angle, 360.0 - angle)
    if diff <= min_diff:
      min_diff = diff
      closest_angle = angle
      closest_dist = distance

  return closest_angle, closest_dist


async def display_front(lidar):
  print("Reading forward distance... Press Ctrl+C to stop.")
  while not lidar.stop_event.is_set():
    await asyncio.sleep(0.2)
    try:
      if lidar.output_dict:
        angle, dist_mm = get_front_distance(lidar.output_dict)
        if dist_mm is not None:
          print(
              f"Front ({angle:6.2f}°): {dist_mm:7.1f} mm ({dist_mm / 1000:.2f} m)"
          )
        else:
          print("Front: Out of range (>12m or <5cm)")
    except Exception as e:
      # Print the actual error message if anything unexpected happens
      print(f"Warning: {type(e).__name__}: {e}")


async def main():
  prepare_serial_port(PORT, BAUD)
  lidar = RPLidar(PORT, BAUD)

  try:
    async with asyncio.TaskGroup() as tg:
      tg.create_task(lidar.simple_scan(make_return_dict=True))
      tg.create_task(display_front(lidar))
  except (KeyboardInterrupt, asyncio.CancelledError):
    pass
  finally:
    print("\nStopping LiDAR...")
    lidar.stop_event.set()
    try:
      lidar.reset()
    except Exception:
      pass


if __name__ == "__main__":
  try:
    asyncio.run(main())
  except KeyboardInterrupt:
    pass